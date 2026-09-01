//! The `ne2k-mini` board, end to end: a **real driver** moving a real frame.
//!
//! A unit test can say "a write to `CR` with `TXP` set produced a frame". This
//! says something stronger: Z80 firmware, executing out of ROM on the board's
//! own core, reaches the card only through `IN` and `OUT` on the machine's port
//! space; it runs the DP8390 data sheet's initialisation procedure, builds a
//! frame in card memory a byte at a time through the remote DMA window, pulls
//! `TXP`, and the exact bytes come out of the backend. Then a frame injected at
//! the backend crosses the receive ring, raises `/INT` **through the wire**, is
//! taken by an interrupt-mode-1 handler at `0x0038` that acknowledges `ISR`,
//! and is copied out of the ring by the driver into RAM where this test can
//! read it.
//!
//! Nothing here pokes a register on the device's behalf. Everything below the
//! `FIRMWARE` constant is assertions about memory.
//!
//! # And the frame arrives when the *machine* says
//!
//! [`NetPort::deliver_at`] names a tick of the card's clock domain, and the
//! card is a lazy device, so the scheduler stops the world exactly there. Run
//! the same firmware against the same `(tick, frame)` input twice and the state
//! hash is identical — which is the whole point of the seam being a pull rather
//! than `pktkit`'s push. `the_run_is_reproducible` asserts it.
//!
//! [`NetPort::deliver_at`]: rsemu::dev::net::link::NetPort::deliver_at

#![cfg(feature = "machine-ne2k-mini")]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::net::link::{NetPort, ports};
use rsemu::machine::{Machine, catalog};

/// The station address `machines/ne2k-mini.machine` gives the card by default,
/// and the one the firmware programs into `PAR0`-`PAR5`.
const STATION: [u8; 6] = [0x52, 0x54, 0x00, 0x12, 0x34, 0x56];

/// Where the driver leaves what it learned, in RAM:
///
/// ```text
///   $4000  the ISR value the interrupt handler read
///   $4001  how many interrupts have been taken
///   $4002  $a5 once a packet has been copied out of the ring
///   $4100  the 256-byte ring page the packet was in: four header bytes
///          (status, next page, count low, count high) then the frame
/// ```
const LAST_ISR: u64 = 0x4000;
const IRQ_COUNT: u64 = 0x4001;
const DONE: u64 = 0x4002;
const RX_PAGE: u64 = 0x4100;

/// `ISR.PRX`, the bit a received packet sets.
const ISR_PRX: u64 = 0x01;

/// The frame the firmware transmits: a 60-byte broadcast, the shortest 802.3
/// allows once the FCS is off. Built here so the assertion and the ROM cannot
/// drift apart.
fn outgoing() -> Vec<u8> {
    let mut f = Vec::with_capacity(60);
    f.extend_from_slice(&[0xff; 6]); // destination: broadcast
    f.extend_from_slice(&STATION); // source: this card
    f.extend_from_slice(&[0x08, 0x00]); // ether type: IPv4, so it looks real
    for i in f.len()..60 {
        f.push((i * 3 + 5) as u8);
    }
    f
}

/// The frame the far end sends back, addressed to this card so that the
/// DP8390's physical-address filter is what lets it in.
fn incoming() -> Vec<u8> {
    let mut f = Vec::with_capacity(96);
    f.extend_from_slice(&STATION); // destination: us
    f.extend_from_slice(&[0x02, 0x00, 0x00, 0x00, 0x00, 0x99]); // some peer
    f.extend_from_slice(&[0x08, 0x06]); // ether type: ARP
    for i in f.len()..96 {
        f.push((i * 11 + 7) as u8);
    }
    f
}

/// The tick of the card's clock domain the frame is injected at.
///
/// Deliberately in the **future** when it is queued: the first `run_for` below
/// stops at 21,000 T-states and this is 25,000, so the scheduler has to stop
/// the world at a tick the card named for the frame to arrive at all. A tick
/// already gone by would have proved only that `advance_to` filters.
///
/// It also leaves the driver room to finish: copying a 256-byte ring page out
/// through the data window costs it about 10,500 T-states, and the second
/// `run_for` gives it 17,000.
const ARRIVAL_TICK: u64 = 25_000;

/// The base port the board's decoder gives the card.
const NIC: u8 = 0x03;

/// A `(register, value)` table for the firmware's `apply` routine, terminated
/// by `0xff` — which is not a register, since the card decodes five address
/// lines.
fn table(pairs: &[(u8, u8)]) -> Vec<u8> {
    let mut out = Vec::with_capacity(pairs.len() * 2 + 1);
    for (reg, value) in pairs {
        out.push(*reg);
        out.push(*value);
    }
    out.push(0xff);
    out
}

/// The DP8390 data sheet's initialisation procedure, as a table.
///
/// *DP8390D*, "Initialization Procedures", in its own order: stop the chip,
/// program the data configuration, clear the remote byte count, park the
/// receiver in monitor mode and the transmitter in loopback while the ring and
/// the address are set up, then start and lift both.
fn init_table() -> Vec<u8> {
    let mut pairs = vec![
        (0x00, 0x21), // CR: STP, page 0, abort remote DMA
        (0x0e, 0x48), // DCR: byte-wide transfers, normal operation (LS = 1)
        (0x0a, 0x00), // RBCR0
        (0x0b, 0x00), // RBCR1
        (0x0c, 0x20), // RCR: monitor mode while we set up
        (0x0d, 0x02), // TCR: internal loopback while we set up
        (0x04, 0x40), // TPSR: the transmit buffer is page 0x40
        (0x01, 0x46), // PSTART: the receive ring starts above it
        (0x02, 0x80), // PSTOP
        (0x03, 0x46), // BNRY: the ring is empty
        (0x07, 0xff), // ISR: clear everything
        (0x0f, 0x01), // IMR: PRX only, so our own transmit does not interrupt
        (0x00, 0x61), // CR: page 1, still stopped
    ];
    for (i, byte) in STATION.iter().enumerate() {
        pairs.push((1 + i as u8, *byte)); // PAR0-PAR5
    }
    pairs.push((0x07, 0x46)); // CURR: an empty ring is CURR == BNRY
    for i in 0..8 {
        pairs.push((8 + i, 0x00)); // MAR0-MAR7: no multicast
    }
    pairs.push((0x00, 0x22)); // CR: page 0, START
    pairs.push((0x0d, 0x00)); // TCR: out of loopback
    pairs.push((0x0c, 0x04)); // RCR: accept broadcast and our own address
    table(&pairs)
}

/// The 16 KiB ROM image, hand-assembled from the Zilog user manual's opcode
/// table (UM0080, appendix "Z80 CPU Instruction Set").
///
/// ```text
///  0000  di / ld sp,$8000 / jp main
///  0038  the interrupt-mode-1 handler
///  0100  main: zero the mailbox, initialise the card, transmit, wait
///  0140  rx:   copy the packet out of the ring and park
///  0180  apply: walk a (register, value) table, `OUT` each pair
///  0200  the initialisation table
///  0240  the remote-DMA-write set-up table
///  0260  the transmit table
///  0280  the remote-DMA-read set-up table
///  0300  the 60 bytes to transmit
/// ```
///
/// Two Z80 facts shape it. The card is at port `0x300`, so every access is
/// `IN r,(C)` / `OUT (C),r` with `B = 0x03` — `IN A,(n)` puts the accumulator
/// on `A8`-`A15` and would land somewhere else entirely. And the byte loops
/// count in `D`, not `B`, because `B` *is* the high address byte and `OTIR`
/// would walk the port address as it counted.
fn firmware() -> Vec<u8> {
    let mut rom = vec![0u8; 16 * 1024];

    let put = |rom: &mut Vec<u8>, at: usize, bytes: &[u8]| {
        rom[at..at + bytes.len()].copy_from_slice(bytes);
    };

    // -- the reset vector --------------------------------------------------
    put(
        &mut rom,
        0x0000,
        &[
            0xf3, // di
            0x31, 0x00, 0x80, // ld sp,$8000
            0xc3, 0x00, 0x01, // jp main
        ],
    );

    // -- the interrupt handler, where mode 1 vectors ------------------------
    //
    // Read ISR, record it, acknowledge every maskable bit — which is what lets
    // the card release /INT — and count the interrupt. A handler that forgot
    // the acknowledgement would loop forever on a level-sensitive pin, exactly
    // as it would on real hardware.
    put(
        &mut rom,
        0x0038,
        &[
            0xf5, // push af
            0xc5, // push bc
            0x01, 0x07, NIC, // ld bc,$0307        ; ISR
            0xed, 0x78, // in a,(c)
            0x32, 0x00, 0x40, // ld ($4000),a
            0x3e, 0xff, // ld a,$ff
            0xed, 0x79, // out (c),a          ; write-1-to-clear
            0x3a, 0x01, 0x40, // ld a,($4001)
            0x3c, // inc a
            0x32, 0x01, 0x40, // ld ($4001),a
            0xc1, // pop bc
            0xf1, // pop af
            0xfb, // ei
            0xed, 0x4d, // reti
        ],
    );

    // -- main ---------------------------------------------------------------
    put(
        &mut rom,
        0x0100,
        &[
            0xaf, // xor a
            0x32, 0x00, 0x40, // ld ($4000),a
            0x32, 0x01, 0x40, // ld ($4001),a
            0x32, 0x02, 0x40, // ld ($4002),a
            0x21, 0x00, 0x02, // ld hl,$0200        ; the init table
            0xcd, 0x80, 0x01, // call apply
            0x21, 0x40, 0x02, // ld hl,$0240        ; arm a remote DMA write
            0xcd, 0x80, 0x01, // call apply
            0x21, 0x00, 0x03, // ld hl,$0300        ; the frame to send
            0x01, 0x10, NIC, // ld bc,$0310        ; the data window
            0x16, 0x3c, // ld d,60
            // txloop ($011e):
            0x7e, // ld a,(hl)
            0xed, 0x79, // out (c),a
            0x23, // inc hl
            0x15, // dec d
            0x20, 0xf9, // jr nz,txloop
            0x21, 0x60, 0x02, // ld hl,$0260        ; TPSR, TBCR, TXP
            0xcd, 0x80, 0x01, // call apply
            0xed, 0x56, // im 1
            0xfb, // ei
            // wait ($012e): spin until the handler has counted an interrupt
            0x3a, 0x01, 0x40, // ld a,($4001)
            0xb7, // or a
            0x28, 0xfa, // jr z,wait
            0xc3, 0x40, 0x01, // jp rx
        ],
    );

    // -- rx: copy the packet at BNRY out of the ring -------------------------
    //
    // A whole 256-byte page rather than the header's byte count, because a
    // packet always starts page-aligned and reading the rest of the page costs
    // one `D` register and no arithmetic. The header is the first four bytes.
    put(
        &mut rom,
        0x0140,
        &[
            0x01, 0x03, NIC, // ld bc,$0303        ; BNRY
            0xed, 0x78, // in a,(c)
            0x5f, // ld e,a
            0x21, 0x80, 0x02, // ld hl,$0280        ; RBCR = 256, RSAR0 = 0
            0xcd, 0x80, 0x01, // call apply
            0x01, 0x09, NIC,  // ld bc,$0309        ; RSAR1 = the boundary page
            0x7b, // ld a,e
            0xed, 0x79, // out (c),a
            0x01, 0x00, NIC, // ld bc,$0300        ; CR
            0x3e, 0x0a, // ld a,$0a           ; remote read, start
            0xed, 0x79, // out (c),a
            0x21, 0x00, 0x41, // ld hl,$4100
            0x01, 0x10, NIC, // ld bc,$0310        ; the data window
            0x16, 0x00, // ld d,0             ; 256 bytes
            // rxloop ($0161):
            0xed, 0x78, // in a,(c)
            0x77, // ld (hl),a
            0x23, // inc hl
            0x15, // dec d
            0x20, 0xf9, // jr nz,rxloop
            0x3a, 0x01, 0x41, // ld a,($4101)       ; the header's next page
            0x01, 0x03, NIC, // ld bc,$0303
            0xed, 0x79, // out (c),a          ; BNRY moves on
            0x01, 0x07, NIC, // ld bc,$0307
            0x3e, 0xff, // ld a,$ff
            0xed, 0x79, // out (c),a          ; and the DMA-complete bit goes
            0x3e, 0xa5, // ld a,$a5
            0x32, 0x02, 0x40, // ld ($4002),a       ; done
            0xf3, // di
            0x18, 0xfe, // jr $               ; park
        ],
    );

    // -- apply: OUT every (register, value) pair HL points at ----------------
    put(
        &mut rom,
        0x0180,
        &[
            0x7e, // ld a,(hl)
            0xfe, 0xff, // cp $ff
            0xc8, // ret z
            0x4f, // ld c,a
            0x06, NIC,  // ld b,$03
            0x23, // inc hl
            0x7e, // ld a,(hl)
            0xed, 0x79, // out (c),a
            0x23, // inc hl
            0x18, 0xf2, // jr apply
        ],
    );

    // -- the tables ----------------------------------------------------------
    let init = init_table();
    assert!(init.len() <= 0x40, "the init table must fit before $0240");
    put(&mut rom, 0x0200, &init);
    // Arm a remote DMA write of 60 bytes into the transmit page at $4000.
    put(
        &mut rom,
        0x0240,
        &table(&[
            (0x0a, 60),   // RBCR0
            (0x0b, 0x00), // RBCR1
            (0x08, 0x00), // RSAR0
            (0x09, 0x40), // RSAR1
            (0x00, 0x12), // CR: remote write, start
        ]),
    );
    // Transmit what is now in the page.
    put(
        &mut rom,
        0x0260,
        &table(&[
            (0x04, 0x40), // TPSR
            (0x05, 60),   // TBCR0
            (0x06, 0x00), // TBCR1
            (0x00, 0x26), // CR: TXP, start, abort remote DMA
        ]),
    );
    // Arm a remote DMA read of one whole 256-byte page. RSAR1 is written
    // separately, because it is the boundary pointer the driver just read.
    put(
        &mut rom,
        0x0280,
        &table(&[
            (0x0a, 0x00), // RBCR0
            (0x0b, 0x01), // RBCR1 -> 256 bytes
            (0x08, 0x00), // RSAR0
        ]),
    );

    put(&mut rom, 0x0300, &outgoing());
    rom
}

/// Build the board with the firmware in its `firmware` slot, and hand back the
/// far end of its wire.
fn boot() -> (Machine, Arc<NetPort>) {
    let entry = catalog::machine("ne2k-mini").expect("this build ships ne2k-mini");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    let registry = catalog::registry().expect("a registry");
    let machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    // The same name the machine file gave the card resolves, from out here, to
    // the same port object the card is holding.
    let port = ports::open(&options.realize.hosts, "net0").expect("the board opened a net port");
    (machine, port)
}

/// Read one byte of a named space.
fn peek(m: &Machine, space: &str, addr: u64) -> u64 {
    m.space(space)
        .unwrap_or_else(|| panic!("the machine has no space called `{space}`"))
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte")
}

/// Read `len` bytes of the memory space.
fn peek_bytes(m: &Machine, addr: u64, len: usize) -> Vec<u8> {
    (0..len as u64)
        .map(|i| peek(m, "mem", addr + i) as u8)
        .collect()
}

/// Six milliseconds of virtual time, which at 3.5 MHz is 21,000 T-states.
fn a_while() -> GlobalTime {
    GlobalTime::from_nanos(6_000_000)
}

#[test]
fn the_board_realizes_with_the_card_on_the_port_bus_and_its_pin_wired() {
    let (m, _port) = boot();
    assert_eq!(m.name(), "ne2k-mini");
    for path in ["cpu", "boot", "dram", "nic"] {
        assert!(
            m.device(path).is_some(),
            "the machine has no instance called `{path}`"
        );
    }
    // The card answers on the port space and nowhere else. Offset 0 is CR,
    // which after reset reads 0x21 (*DP8390D*, "Register Descriptions").
    assert_eq!(peek(&m, "port", 0x0300), 0x21);
    assert_eq!(
        peek(&m, "mem", 0x0300),
        u64::from(outgoing()[0]),
        "and it must not be in the memory space — 0x0300 there is ROM, holding \
         the frame the firmware is going to send, and an NE2000 is I/O mapped"
    );
}

#[test]
fn the_driver_transmits_a_frame_and_the_backend_gets_it_byte_for_byte() {
    let (mut m, port) = boot();
    m.run_for(a_while()).expect("it runs");

    let sent = port.drain();
    assert_eq!(sent.len(), 1, "exactly one frame went out");
    assert_eq!(
        sent[0],
        outgoing(),
        "the bytes the guest DMA'd into card memory are not the bytes on the wire"
    );
    // And the card told the far side who it is, from `realize` and again from
    // the driver's PAR0-PAR5 writes.
    assert_eq!(port.mac().octets(), STATION);
}

#[test]
fn a_frame_injected_at_the_backend_reaches_the_driver_through_the_ring() {
    let (mut m, port) = boot();
    m.run_for(a_while())
        .expect("the driver initialises the card");
    assert_eq!(peek(&m, "mem", IRQ_COUNT), 0, "nothing has interrupted yet");

    let frame = incoming();
    assert!(
        port.deliver_at(ARRIVAL_TICK, &frame),
        "the port took the frame"
    );
    m.run_for(a_while()).expect("it runs on");

    assert_eq!(peek(&m, "mem", DONE), 0xa5, "the driver never got that far");
    assert_eq!(
        peek(&m, "mem", IRQ_COUNT),
        1,
        "the card's pin should have interrupted the core exactly once"
    );
    assert_eq!(
        peek(&m, "mem", LAST_ISR) & ISR_PRX,
        ISR_PRX,
        "and the handler should have found ISR.PRX set"
    );

    // The four-byte packet header the NIC wrote in front of the frame
    // (*DP8390D*, "Packet Reception"): status, next page, and a byte count that
    // includes the header.
    let header = peek_bytes(&m, RX_PAGE, 4);
    assert_eq!(header[0] & 0x01, 0x01, "the receive status says PRX");
    let count = u64::from(header[2]) | (u64::from(header[3]) << 8);
    assert_eq!(
        count as usize,
        frame.len() + 4,
        "the byte count should include the header"
    );
    assert_eq!(
        peek_bytes(&m, RX_PAGE + 4, frame.len()),
        frame,
        "the frame did not survive the ring"
    );
}

#[test]
fn the_run_is_reproducible() {
    // The determinism claim, at machine level: the same firmware against the
    // same (tick, frame) input twice is the same machine, down to the hash.
    let hash_of = || {
        let (mut m, port) = boot();
        m.run_for(a_while()).expect("it runs");
        port.deliver_at(ARRIVAL_TICK, &incoming());
        m.run_for(a_while()).expect("it runs on");
        assert_eq!(peek(&m, "mem", DONE), 0xa5);
        m.state_hash().expect("a hash")
    };
    assert_eq!(hash_of(), hash_of());
}

#[test]
fn the_frame_arrives_at_the_tick_it_was_queued_for() {
    // Two runs that differ only in *when* the frame was queued must differ in
    // what the guest has seen at a fixed point in virtual time. If they did
    // not, the arrival tick would not be guest-visible and the seam would be
    // buying nothing.
    let done_at = |arrival: u64| {
        let (mut m, port) = boot();
        m.run_for(a_while()).expect("it runs");
        port.deliver_at(arrival, &incoming());
        // Long enough for the driver's 256-byte copy loop out of the ring, and
        // nowhere near the later arrival.
        m.run_for(a_while()).expect("runs on");
        peek(&m, "mem", DONE)
    };
    assert_eq!(
        done_at(ARRIVAL_TICK),
        0xa5,
        "queued for a tick just gone by"
    );
    assert_eq!(
        done_at(10_000_000),
        0x00,
        "a frame queued for a tick a long way off must not be visible yet"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let (mut m, port) = boot();
    m.run_for(a_while()).expect("it runs");
    port.deliver_at(ARRIVAL_TICK, &incoming());
    m.run_for(a_while()).expect("it runs on");

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let (mut other, _other_port) = boot();
    other.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "a save/load round trip changed the machine's state hash"
    );
    assert_eq!(
        peek_bytes(&other, RX_PAGE + 4, incoming().len()),
        incoming(),
        "and the packet is still where the driver left it"
    );
}
