//! The `stm32f407` board's I²C, end to end.
//!
//! A unit test can say "the register accepted the write". This says something
//! much stronger: a **Cortex-M4 running hand-assembled Thumb-2 out of flash**
//! drives the STM32's I²C peripheral through the exact event sequence RM0090
//! §25.3.3 documents — `EV5`, `EV6`, `EV8`, `EV8_2`, `EV7`, each with its
//! read-then-do clearing sequence — writes **a whole page to an AT24C02
//! EEPROM**, polls for the device's internally self-timed write cycle to finish
//! by watching it refuse its own address (datasheet §5.3), and then reads all
//! eight bytes back through a random read with a repeated START and a NACK on
//! the last byte. The bytes end up in SRAM, and this test compares them against
//! what the firmware sent *and* against the EEPROM's own snapshot chunk.
//!
//! It does that **twice**: once with `i2clink = "wired"`, where SCL and SDA are
//! two open-drain nets carrying one edge per half bit period, and once with
//! `i2clink = "transactional"`. The two runs must produce the same bytes and
//! the same guest-visible timeline, which is the claim
//! `docs/buses/low-speed.md` asks a machine file to make explicitly.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-stm32f407`.

#![cfg(feature = "machine-stm32f407")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::state::StateReader;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// `I2C1`'s base, per the machine file (RM0090 §2.3, Table 1).
const I2C1: u64 = 0x4000_5400;
/// Where SRAM1 starts. The firmware puts what it read back at the bottom of it.
const SRAM1: u64 = 0x2000_0000;
/// Where the firmware writes its "finished" marker.
const MARKER: u64 = 0x2000_0100;
/// The marker value.
const DONE_MARK: u64 = 0xa5;

/// The EEPROM's seven-bit address with `A2 A1 A0` tied low (datasheet §4.1).
const EEPROM_WRITE: u16 = 0xa0;
/// The same with the `R/W̅` bit set.
const EEPROM_READ: u16 = 0xa1;
/// Where in the array the firmware works: the start of page 2.
const WORD_ADDRESS: u16 = 0x10;

/// What the firmware writes and expects to read back.
const PAYLOAD: [u8; 8] = [0xde, 0xad, 0xbe, 0xef, 0x12, 0x34, 0x56, 0x78];

/// Where the payload sits in flash, which is also where it sits in the boot
/// alias the core fetches through.
const DATA_ADDR: u32 = 0x300;
/// Where the program's entry point is.
const ENTRY: u32 = 0x100;
/// The initial stack pointer: the top of SRAM2.
const STACK: u32 = 0x2002_0000;

// `SR1` bit masks, RM0090 §25.6.6.
const SB: u16 = 1 << 0;
const ADDR: u16 = 1 << 1;
const BTF: u16 = 1 << 2;
const RXNE: u16 = 1 << 6;
const TXE: u16 = 1 << 7;
const AF: u16 = 1 << 10;

// `CR1` bit masks, RM0090 §25.6.1.
const PE: u16 = 1 << 0;
const START: u16 = 1 << 8;
const STOP: u16 = 1 << 9;
const ACK: u16 = 1 << 10;

// Register offsets.
const CR1: u16 = 0x00;
const DR: u16 = 0x10;
const SR1: u16 = 0x14;
const SR2: u16 = 0x18;
const CCR: u16 = 0x1c;

/// `CCR` for 100 kHz standard mode on a 42 MHz APB1: `Thigh = Tlow = CCR ×
/// TPCLK1`, so `42_000_000 / (2 × 100_000) = 210` (RM0090 §25.6.8).
const CCR_100K: u16 = 210;

// ---------------------------------------------------------------------------
// A very small Thumb-2 assembler
// ---------------------------------------------------------------------------
//
// The same table `tests/stm32f407_board.rs` keeps, plus the three encodings an
// I²C driver needs that a USART one does not: `SUBS`, `STRB` and `BNE`. Not an
// assembler — a table, checkable by hand against the ARMv7-M ARM (DDI 0403,
// A7.7). What is computed rather than written down is the branch
// displacements, because those are the part a human gets wrong.

/// `MOVW Rd, #imm16` — encoding T3, DDI 0403 A7.7.76.
fn movw(d: u16, imm16: u16) -> [u16; 2] {
    let i = (imm16 >> 11) & 1;
    let imm4 = (imm16 >> 12) & 0xf;
    let imm3 = (imm16 >> 8) & 7;
    let imm8 = imm16 & 0xff;
    [0xf240 | (i << 10) | imm4, (imm3 << 12) | (d << 8) | imm8]
}

/// `MOVT Rd, #imm16` — encoding T1, A7.7.79. The same field layout.
fn movt(d: u16, imm16: u16) -> [u16; 2] {
    let [a, b] = movw(d, imm16);
    [a | 0x0080, b]
}

/// `STR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.158.
fn str_imm(t: u16, n: u16, off: u16) -> u16 {
    0x6000 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDR Rt, [Rn, #imm5*4]` — encoding T1, A7.7.42.
fn ldr_imm(t: u16, n: u16, off: u16) -> u16 {
    0x6800 | ((off / 4) << 6) | (n << 3) | t
}

/// `LDRB Rt, [Rn, #imm5]` — encoding T1, A7.7.45.
fn ldrb_imm(t: u16, n: u16, off: u16) -> u16 {
    0x7800 | (off << 6) | (n << 3) | t
}

/// `STRB Rt, [Rn, #imm5]` — encoding T1, A7.7.163.
fn strb_imm(t: u16, n: u16, off: u16) -> u16 {
    0x7000 | (off << 6) | (n << 3) | t
}

/// `MOVS Rd, #imm8` — encoding T1, A7.7.76.
fn movs(d: u16, imm8: u16) -> u16 {
    0x2000 | (d << 8) | imm8
}

/// `CMP Rn, #imm8` — encoding T1, A7.7.27.
fn cmp(n: u16, imm8: u16) -> u16 {
    0x2800 | (n << 8) | imm8
}

/// `ANDS Rdn, Rm` — encoding T1, A7.7.9.
fn ands(dn: u16, m: u16) -> u16 {
    0x4000 | (m << 3) | dn
}

/// `ADDS Rd, Rn, #imm3` — encoding T1, A7.7.3.
fn adds(d: u16, n: u16, imm3: u16) -> u16 {
    0x1c00 | (imm3 << 6) | (n << 3) | d
}

/// `SUBS Rd, Rn, #imm3` — encoding T1, A7.7.174.
fn subs(d: u16, n: u16, imm3: u16) -> u16 {
    0x1e00 | (imm3 << 6) | (n << 3) | d
}

/// A conditional branch's displacement: `imm8` is signed, doubled, and taken
/// from `PC`, which in Thumb is the instruction's address plus four.
fn bcond(cond: u16, at: usize, target: usize) -> u16 {
    let delta = (target as isize - (at as isize + 2)) as i16;
    assert!(
        (-128..128).contains(&delta),
        "a conditional branch of {delta} halfwords is out of range"
    );
    0xd000 | (cond << 8) | (delta as u16 & 0xff)
}

fn beq(at: usize, target: usize) -> u16 {
    bcond(0, at, target)
}

fn bne(at: usize, target: usize) -> u16 {
    bcond(1, at, target)
}

/// The same for an unconditional `B`, which has eleven bits of it.
fn b(at: usize, target: usize) -> u16 {
    let delta = (target as isize - (at as isize + 2)) as i16;
    assert!(
        (-1024..1024).contains(&delta),
        "a branch of {delta} halfwords is out of range"
    );
    0xe000 | (delta as u16 & 0x7ff)
}

/// The program under construction, with the label bookkeeping.
struct Asm {
    code: Vec<u16>,
}

impl Asm {
    fn new() -> Asm {
        Asm { code: Vec::new() }
    }

    fn at(&self) -> usize {
        self.code.len()
    }

    fn push(&mut self, half: u16) {
        self.code.push(half);
    }

    fn extend(&mut self, halves: [u16; 2]) {
        self.code.extend(halves);
    }

    /// `movw`+`movt` for a 32-bit constant.
    fn li(&mut self, r: u16, value: u32) {
        self.extend(movw(r, value as u16));
        self.extend(movt(r, (value >> 16) as u16));
    }

    /// Spin until every bit of `mask` is set in `SR1`.
    ///
    /// Every wait in an I²C driver has this shape, so it is written once. The
    /// mask is a `movs` immediate, which covers `SB`, `ADDR`, `BTF`, `RxNE` and
    /// `TxE` — everything this program waits on.
    fn wait_sr1(&mut self, mask: u16) {
        assert!(mask <= 0xff, "wait_sr1 masks are movs immediates");
        let top = self.at();
        self.push(ldr_imm(5, 0, SR1));
        self.push(movs(6, mask));
        self.push(ands(5, 6));
        self.push(cmp(5, 0));
        let at = self.at();
        self.push(beq(at, top));
    }

    /// The `EV5` sequence: wait for `SB`, read `SR1`, write the address into
    /// `DR` (RM0090 §25.3.3).
    fn start_and_address(&mut self, address: u16) {
        self.extend(movw(1, PE | ACK | START));
        self.push(str_imm(1, 0, CR1));
        self.wait_sr1(SB);
        self.push(ldr_imm(5, 0, SR1));
        self.push(movs(1, address));
        self.push(str_imm(1, 0, DR));
    }

    /// The `EV6` sequence: wait for `ADDR`, then read `SR1` and `SR2`.
    fn clear_addr(&mut self) {
        self.wait_sr1(ADDR);
        self.push(ldr_imm(5, 0, SR1));
        self.push(ldr_imm(5, 0, SR2));
    }

    /// Spin until `MSL` clears, which is how §25.6.7 says a STOP finishes.
    fn wait_idle(&mut self) {
        let top = self.at();
        self.push(ldr_imm(5, 0, SR2));
        self.push(movs(6, 1));
        self.push(ands(5, 6));
        self.push(cmp(5, 0));
        let at = self.at();
        self.push(bne(at, top));
    }
}

/// The firmware image: a vector table, the program, and the payload.
///
/// ```text
///   0x000: .word 0x20020000        ; initial SP, the top of SRAM2
///   0x004: .word 0x00000101        ; reset vector, entry|1
///
///   0x100: movw r0, #0x5400        ; r0 = I2C1
///          movt r0, #0x4000
///          movs r1, #210           ; CCR: 42 MHz / (2*210) = 100 kHz
///          str  r1, [r0, #0x1c]
///          movs r1, #1             ; CR1 = PE
///          str  r1, [r0, #0x00]
///
///          ; ---- page write of eight bytes at word address 0x10 ----
///          START; EV5; DR = 0xa0; EV6
///          wait TxE; DR = 0x10                     ; the word address
///          movw r2, #0x300         ; r2 = the payload
///          movs r4, #8
///   wloop: ldrb r3, [r2, #0]
///          wait TxE; DR = r3
///          adds r2, r2, #1
///          subs r4, r4, #1
///          bne  wloop
///          wait BTF                                ; EV8_2
///          CR1 = PE|ACK|STOP
///          wait MSL clear
///
///          ; ---- acknowledge polling, datasheet §5.3 ----
///   poll:  START; EV5; DR = 0xa0
///   pw:    ldr r5, [r0, #0x14]
///          tst ADDR -> bne ready                   ; the device answered
///          tst AF   -> beq pw
///          SR1 &= ~AF; CR1 = PE|STOP; wait MSL clear
///          b poll
///
///          ; ---- the write that answered is the random read's dummy write ----
///   ready: EV6
///          wait TxE; DR = 0x10
///          wait BTF
///          START; EV5; DR = 0xa1; EV6              ; repeated START, read
///          movw r7, #0; movt r7, #0x2000           ; r7 = SRAM1
///          movs r4, #8
///   rloop: cmp  r4, #1
///          bne  more
///          CR1 = PE|STOP                           ; ACK cleared for the last
///   more:  wait RxNE
///          ldr  r3, [r0, #0x10]
///          strb r3, [r7, #0]
///          adds r7, r7, #1
///          subs r4, r4, #1
///          bne  rloop
///
///          movs r1, #0xa5
///          movw r6, #0x0100; movt r6, #0x2000
///          strb r1, [r6, #0]                       ; the "finished" marker
///          b .
///
///   0x300: the eight payload bytes
/// ```
fn firmware() -> Vec<u8> {
    let mut a = Asm::new();
    a.li(0, I2C1 as u32);
    a.push(movs(1, CCR_100K));
    a.push(str_imm(1, 0, CCR));
    a.push(movs(1, PE));
    a.push(str_imm(1, 0, CR1));

    // ---- the page write ----
    a.start_and_address(EEPROM_WRITE);
    a.clear_addr();
    a.wait_sr1(TXE);
    a.push(movs(1, WORD_ADDRESS));
    a.push(str_imm(1, 0, DR));
    a.extend(movw(2, DATA_ADDR as u16));
    a.push(movs(4, PAYLOAD.len() as u16));
    let wloop = a.at();
    a.push(ldrb_imm(3, 2, 0));
    a.wait_sr1(TXE);
    a.push(str_imm(3, 0, DR));
    a.push(adds(2, 2, 1));
    a.push(subs(4, 4, 1));
    let at = a.at();
    a.push(bne(at, wloop));
    // EV8_2: TxE and BTF are both set, so the STOP goes out after the last byte.
    a.wait_sr1(BTF);
    a.extend(movw(1, PE | ACK | STOP));
    a.push(str_imm(1, 0, CR1));
    a.wait_idle();

    // ---- acknowledge polling ----
    let poll = a.at();
    a.start_and_address(EEPROM_WRITE);
    let pw = a.at();
    a.push(ldr_imm(5, 0, SR1));
    a.push(movs(6, ADDR));
    a.push(ands(6, 5));
    a.push(cmp(6, 0));
    let bne_ready = a.at();
    a.push(0); // patched to `bne ready`
    a.extend(movw(6, AF));
    a.push(ands(6, 5));
    a.push(cmp(6, 0));
    let at = a.at();
    a.push(beq(at, pw));
    // Still busy: clear `AF` (§25.6.6's `rc_w0`), release the bus, go round.
    a.extend(movw(1, !AF));
    a.push(str_imm(1, 0, SR1));
    a.extend(movw(1, PE | STOP));
    a.push(str_imm(1, 0, CR1));
    a.wait_idle();
    let at = a.at();
    a.push(b(at, poll));

    // ---- the random read (datasheet §6.2) ----
    let ready = a.at();
    a.code[bne_ready] = bne(bne_ready, ready);
    // The write that finally got an acknowledge *is* the dummy write, so all
    // that is left of it is the word address.
    a.clear_addr();
    a.wait_sr1(TXE);
    a.push(movs(1, WORD_ADDRESS));
    a.push(str_imm(1, 0, DR));
    a.wait_sr1(BTF);
    // The repeated START turns the transfer round.
    a.start_and_address(EEPROM_READ);
    a.clear_addr();
    a.li(7, SRAM1 as u32);
    a.push(movs(4, PAYLOAD.len() as u16));
    let rloop = a.at();
    a.push(cmp(4, 1));
    let bne_more = a.at();
    a.push(0); // patched to `bne more`
    // §25.3.3: "the ACK bit must be cleared just after reading the second last
    // data byte", and the STOP programmed at the same moment.
    a.extend(movw(1, PE | STOP));
    a.push(str_imm(1, 0, CR1));
    let more = a.at();
    a.code[bne_more] = bne(bne_more, more);
    a.wait_sr1(RXNE);
    a.push(ldr_imm(3, 0, DR));
    a.push(strb_imm(3, 7, 0));
    a.push(adds(7, 7, 1));
    a.push(subs(4, 4, 1));
    let at = a.at();
    a.push(bne(at, rloop));

    a.push(movs(1, DONE_MARK as u16));
    a.li(6, MARKER as u32);
    a.push(strb_imm(1, 6, 0));
    a.push(0xe7fe); // b .

    let mut image = vec![0u8; DATA_ADDR as usize + PAYLOAD.len()];
    image[0..4].copy_from_slice(&STACK.to_le_bytes());
    image[4..8].copy_from_slice(&(ENTRY | 1).to_le_bytes());
    let entry = ENTRY as usize;
    assert!(
        entry + a.code.len() * 2 <= DATA_ADDR as usize,
        "the program grew into its own payload"
    );
    for (i, half) in a.code.iter().enumerate() {
        image[entry + i * 2..entry + i * 2 + 2].copy_from_slice(&half.to_le_bytes());
    }
    let data = DATA_ADDR as usize;
    image[data..data + PAYLOAD.len()].copy_from_slice(&PAYLOAD);
    image
}

/// Build the board with `i2clink` set, boot it, and run until the firmware
/// leaves its marker.
///
/// Returns the machine and how much virtual time it took, so the two link
/// models can be compared on both counts.
fn run(link: &str) -> (Machine, u64) {
    let entry = catalog::machine("stm32f407").expect("this build ships stm32f407");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("i2clink"), String::from(link)));
    // A bus name per run, so two boards in one test binary do not meet.
    options
        .resolve
        .params
        .push((String::from("i2cbus"), format!("i2c-{link}")));
    let registry = catalog::registry().expect("a registry");
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize with i2clink={link}: {e}"),
    };

    // The whole sequence is a page write, a 5 ms write cycle and a random read
    // at 100 kHz, so a few tens of milliseconds of virtual time. The loop is a
    // condition rather than a fixed span because what bounds how much a core
    // gets through per millisecond is the scheduler's quantum budget, and a
    // hard-coded span would fail the day that default changes.
    let mut elapsed = 0u64;
    while peek(&machine, MARKER) != DONE_MARK && elapsed < 500_000_000 {
        machine
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        elapsed += 1_000_000;
    }
    assert_eq!(
        peek(&machine, MARKER),
        DONE_MARK,
        "the firmware never finished within {elapsed} ns of virtual time under i2clink={link}"
    );
    (machine, elapsed)
}

/// Read one byte of the guest's memory space, without side effects.
fn peek(m: &Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .expect("a mapped byte")
}

/// The EEPROM's array, out of its snapshot chunk.
///
/// There is no route from a `dyn Device` to an `At24c` — `core::device` keeps
/// `Any` out of the supertrait chain deliberately — so the way to see a
/// device's state from outside is the surface `ROADMAP.md` §4.5 already
/// promises. Reading it here doubles as a check that the chunk really is the
/// architectural state.
fn eeprom_array(m: &Machine) -> Vec<u8> {
    let bytes = m.save().expect("the machine snapshots");
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let (_class, _version, data) = reader.load_raw("eeprom").expect("the EEPROM has a chunk");
    // `ticks` (u64), then a length-prefixed byte array: the array itself.
    let len = u64::from_le_bytes(data[8..16].try_into().unwrap()) as usize;
    data[16..16 + len].to_vec()
}

// ---------------------------------------------------------------------------

#[test]
fn a_cortex_m4_writes_a_page_to_an_eeprom_and_reads_it_back() {
    for link in ["wired", "transactional"] {
        let (machine, _) = run(link);

        // What the firmware read back, byte for byte, out of guest RAM.
        let read_back: Vec<u8> = (0..PAYLOAD.len())
            .map(|i| peek(&machine, SRAM1 + i as u64) as u8)
            .collect();
        assert_eq!(
            read_back,
            PAYLOAD.to_vec(),
            "the guest did not read back what it wrote, under i2clink={link}"
        );

        // And what actually landed in the device, which is the stronger claim:
        // an address phase that gets an acknowledge proves far less than a page
        // that is *in the array*.
        let array = eeprom_array(&machine);
        assert_eq!(array.len(), 256, "an AT24C02 is 256 bytes");
        assert_eq!(
            &array[WORD_ADDRESS as usize..WORD_ADDRESS as usize + PAYLOAD.len()],
            &PAYLOAD,
            "the page is not in the EEPROM under i2clink={link}"
        );
        assert!(
            array[..WORD_ADDRESS as usize].iter().all(|b| *b == 0xff),
            "and nothing outside the page was touched"
        );
        assert!(
            array[WORD_ADDRESS as usize + PAYLOAD.len()..]
                .iter()
                .all(|b| *b == 0xff)
        );
    }
}

#[test]
fn the_two_link_models_leave_the_same_bytes_behind() {
    // The claim `docs/buses/low-speed.md` asks a machine file to make
    // explicitly, checked at machine level with a whole firmware image: the
    // same program, the same device, the same array.
    let (wired, _) = run("wired");
    let (transactional, _) = run("transactional");
    assert_eq!(eeprom_array(&wired), eeprom_array(&transactional));
    let bytes = |m: &Machine| -> Vec<u8> {
        (0..PAYLOAD.len())
            .map(|i| peek(m, SRAM1 + i as u64) as u8)
            .collect()
    };
    assert_eq!(bytes(&wired), bytes(&transactional));
}

#[test]
fn the_board_realizes_with_the_i2c_wired_to_the_core_and_to_the_eeprom() {
    let (machine, _) = run("wired");
    for path in ["i2c1", "eeprom"] {
        assert!(
            machine.device(path).is_some(),
            "the board has no `{path}` instance"
        );
    }
    // The register block answers where RM0090 §2.3 puts it.
    assert!(
        machine
            .space("mem")
            .expect("the memory space")
            .read(I2C1 + u64::from(CCR), Width::U32, MemAttrs::DEBUG)
            .is_ok(),
        "I2C1's registers are not mapped at 0x40005400"
    );
}
