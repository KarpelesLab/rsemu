//! An 8042 and an 8259A, four scan codes, and four interrupts — with no
//! firmware at all.
//!
//! `tests/pc_at_typing.rs` makes the same claim with the whole board running
//! rsemu's firmware, which is the honest end-to-end version and is also the
//! slowest possible way to find out which of the two chips is at fault. This
//! file is the small one: the keyboard corner of the board — two chips, one
//! wire and a scheduler — with the test itself playing the part of the
//! interrupt handler, reading the ports through the machine's I/O space and
//! taking each acknowledge cycle where a handler would.
//!
//! There **is** a processor on it, running two bytes of `jmp $` with
//! interrupts off, and it is not decoration: an 8259A's request latch is
//! cleared by an interrupt acknowledge cycle and by nothing else (Intel 8259A
//! data sheet), so without something to run one, the first edge latches and
//! every later edge is indistinguishable from it. It is the acknowledge
//! cycles that make "four separate interrupts" a measurable claim rather than
//! "one interrupt that never went away".
//!
//! # What is being claimed
//!
//! 1. Every byte the keyboard sends produces **its own rising edge** on IRQ1
//!    — four bytes, four rises and four falls, read off the net itself.
//! 2. The master 8259A **latches each one**: its interrupt request register
//!    has bit 1 set after each rise, and the read of the data port that
//!    empties the output buffer takes the line back down so the next byte has
//!    an edge to make. An 8259A input in edge-triggered mode latches a
//!    transition, not a level (Intel 8259A data sheet, edge-triggered mode),
//!    which is why a line that never fell would announce one keystroke and
//!    then nothing whatever the keyboard did.
//! 3. The refill is **a serial frame late**, not instantaneous: after the
//!    read, the status register says the output buffer is empty, and it is
//!    still empty until the controller's own clock domain ticks. An AT
//!    keyboard clocks eleven bits at 10-16.7 kHz (*IBM Personal Computer AT
//!    Technical Reference*, the keyboard interface), which is about a
//!    millisecond a byte, and `machines/pc-at.machine` divides the 8042's
//!    12 MHz crystal by 12,000 to give the device one tick per byte-time.
//! 4. Reading the **status** register between two bytes changes nothing: it
//!    is a pure function of the state (8042 data sheet), so polling it —
//!    which is exactly what a BIOS does while it waits — cannot eat a
//!    keystroke or drop an interrupt.
//! 5. A `MemAttrs::debug` read of the data port pops nothing: the byte stays,
//!    `OBF` stays set and the interrupt stays up, so a debugger looking at
//!    0x60 does not steal the guest's keystroke.
//!
//! # Sources
//!
//! Intel 8042 data sheet (the output buffer, `OBF`, the status register);
//! Intel 8259A data sheet (edge-triggered mode, the IRR, and OCW3's
//! read-register command); *IBM Personal Computer AT Technical Reference* for
//! the keyboard interface's bit rate and for which port is which. No emulator
//! source was consulted.

#![cfg(all(feature = "dev-pc", feature = "machine-pc-at"))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::Machine;

/// The two chips, the wire between them, and nothing else.
///
/// Inline rather than in `machines/`, because it models no product: it is the
/// keyboard corner of `machines/pc-at.machine`, with that file's own crystal,
/// divider, addresses and wiring.
const KBC_AND_PIC: &str = r#"
machine "kbc-irq" {
  # The 8042's crystal, divided to one byte-time a tick: an AT keyboard's
  # serial line runs at 10-16.7 kHz and a frame is eleven bits, so one tick is
  # one opportunity to move one byte.
  osc kbd = 12000000 Hz
  osc cpu = 25000000 Hz

  space mem  { width = 20, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }

  # A processor, because an 8259A's request register is cleared by an
  # *interrupt acknowledge cycle* and nothing else (8259A data sheet): without
  # something to run one, the first edge latches and every later one is
  # indistinguishable from it. It runs two bytes of its own -- `jmp $`, with
  # interrupts off -- so nothing it does can interfere; this file takes the
  # acknowledge cycles itself, exactly where an interrupt handler would.
  object cpu0 "cpu.x86" {
    clock   = cpu
    space   = mem
    iospace = "port"
    model   = "80486"
    engine  = "interp"
  }
  object dram "ram" { size = 0xf0000 }
  object boot "rom" { size = 64K, image = "firmware" }

  object kbc  "pc.kbc" { clock = kbd / 12000, port = "keyboard" }
  object pic1 "pc.pic" { mode = "master" }

  map mem  0x00000 size 0xf0000 = dram
  map mem  0xf0000 size 64K     = boot
  map port 0x0020 size 0x0002 = pic1.regs
  map port 0x0060 size 0x0001 = kbc.data
  map port 0x0064 size 0x0001 = kbc.cmd

  wire kbc.irq1 -> pic1.ir1
  wire pic1.int -> cpu0.intr
}
"#;

/// The controller command byte's "raise IRQ1 when the output buffer fills
/// from the keyboard" bit, which is what POST sets.
const CB_KBD_INT: u8 = 0x01;
/// Status bit 0: there is a byte for the CPU.
const ST_OBF: u8 = 0x01;
/// The master's IRQ1 bit in its interrupt request register.
const IRR_IRQ1: u8 = 0x02;

/// The vector IRQ1 fetches, with the master's base programmed to 0x08 as
/// every PC programs it: 0x08 + 1, which is `INT 09h`, the keyboard.
const IRQ1_VECTOR: u8 = 0x09;

/// The processor's whole program: `jmp $`, at the reset vector. It exists so
/// that the machine has something to run an interrupt acknowledge cycle with,
/// and it must never take one of its own -- `IF` is clear out of reset and
/// nothing here sets it.
fn firmware() -> Vec<u8> {
    let mut image = vec![0xffu8; 0x1_0000];
    image[0xfff0..0xfff2].copy_from_slice(&[0xeb, 0xfe]);
    image
}

/// One byte-time of the 8042's clock domain, in nanoseconds: 12 MHz / 12,000
/// is 1 kHz, so a tick is a millisecond. Two of them, so a test never turns
/// on which side of a tick boundary it started.
const BYTE_TIME: GlobalTime = GlobalTime::from_nanos(2_000_000);

struct Rig {
    machine: Machine,
    cpu: Arc<rsemu::cpu::x86::X86>,
    port: Arc<rsemu::host::chardev::CharPort>,
    /// Which net the 8042's `irq1` pin drives, so the *line* can be read as
    /// well as what the controller latched off it.
    irq1: usize,
}

impl Rig {
    fn new() -> Rig {
        let cpus: Arc<rsemu::core::Captured<rsemu::cpu::x86::X86>> =
            Arc::new(rsemu::core::Captured::new());
        let mut bindings = rsemu::machine::catalog::bindings().expect("this build's bindings");
        let kept = Arc::clone(&cpus);
        bindings.replace("cpu.x86", move |props| {
            let cpu = Arc::new(rsemu::cpu::x86::X86::from_props_defaulting(
                props,
                rsemu::cpu::x86::Variant::I80486,
            )?);
            kept.push(&cpu);
            Ok(cpu)
        });
        let mut options = rsemu::machine::BuildOptions::new()
            .with_classes(rsemu::machine::catalog::classes())
            .with_bindings(bindings);
        options.realize.media.insert("firmware", firmware());
        let registry = rsemu::machine::catalog::registry().expect("this build's registry");
        let mut machine =
            rsemu::machine::build("kbc-irq.machine", KBC_AND_PIC, &registry, &options)
                .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
        let port = rsemu::host::chardev::ports::open(&options.realize.hosts, "keyboard")
            .expect("the 8042 opened its port");
        let irq1 = machine
            .nets()
            .iter()
            .position(|net| {
                net.sources().iter().any(|pin| {
                    pin.port == "irq1" && machine.devices()[pin.device].path().ends_with("kbc")
                })
            })
            .expect("the machine file wires kbc.irq1 to the master");
        machine.reset(ResetKind::Cold);
        machine.sweep();

        let rig = Rig {
            machine,
            cpu: cpus.take().expect("the constructor kept a handle"),
            port,
            irq1,
        };
        // What POST does before it expects a keystroke: interrupts on, and the
        // master's IRQ1 input unmasked. ICW1/ICW2/ICW4 then OCW1 (8259A data
        // sheet, the initialization sequence).
        rig.outb(0x20, 0x11);
        rig.outb(0x21, 0x08);
        rig.outb(0x21, 0x04);
        rig.outb(0x21, 0x01);
        // OCW1, the mask: every line masked but IRQ1.
        rig.outb(0x21, !IRR_IRQ1);
        rig.outb(0x64, 0x60);
        rig.outb(0x60, CB_KBD_INT);
        rig
    }

    fn outb(&self, port: u64, value: u8) {
        self.machine
            .space("port")
            .expect("the I/O space")
            .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("a byte write");
    }

    fn inb(&self, port: u64) -> u8 {
        self.machine
            .space("port")
            .expect("the I/O space")
            .read(port, Width::U8, MemAttrs::DEFAULT)
            .expect("a byte read") as u8
    }

    /// A read as a *debugger* makes it: no side effects allowed.
    fn peek(&self, port: u64) -> u8 {
        self.machine
            .space("port")
            .expect("the I/O space")
            .read(port, Width::U8, MemAttrs::DEBUG)
            .expect("a byte read") as u8
    }

    /// The master's interrupt request register. OCW3 selects which register a
    /// read of 0x20 returns (8259A data sheet).
    fn irr(&self) -> u8 {
        self.outb(0x20, 0x0a);
        self.inb(0x20)
    }

    /// Whether the master is asking the processor for attention.
    fn intr(&self) -> bool {
        self.cpu.intr_asserted()
    }

    /// Take the interrupt, as a processor and its handler do: the acknowledge
    /// cycle, which is the only thing that clears an 8259A's request latch
    /// (8259A data sheet), and then the end-of-interrupt that releases the
    /// in-service bit. Returns the vector fetched.
    fn take_interrupt(&self) -> u8 {
        let vector = self.cpu.acknowledge();
        self.outb(0x20, 0x20);
        vector
    }

    /// What IRQ1 is sitting at, asked of the net rather than of either chip.
    fn irq1(&self) -> bool {
        self.machine.nets()[self.irq1]
            .wire()
            .resolve_net()
            .is_high()
    }

    /// Let one serial frame pass.
    fn byte_time(&mut self) {
        self.machine.run_for(BYTE_TIME).expect("the machine runs");
    }
}

/// Four scan codes, four interrupts — and the 8259A latches every one.
#[test]
fn four_scan_codes_make_four_edges_and_the_8259a_latches_each() {
    let mut rig = Rig::new();
    // Set 2, which is what an AT keyboard sends: `A`, `B`, `C`, `D` going
    // down. Translation is off — the command byte above turns on the
    // interrupt and nothing else — so these are the bytes the guest reads.
    let codes = [0x1cu8, 0x32, 0x21, 0x23];
    rig.port.feed(&codes);

    assert_eq!(rig.irr() & IRR_IRQ1, 0, "nothing is requesting yet");
    assert!(!rig.irq1(), "and the line is down");
    assert!(!rig.intr(), "and the processor has nothing to take");

    for (n, code) in codes.iter().enumerate() {
        // One tick of the 8042's clock domain moves one byte off the cable.
        rig.byte_time();

        assert_eq!(
            rig.inb(0x64) & ST_OBF,
            ST_OBF,
            "byte {n}: the output buffer never filled"
        );
        assert_eq!(
            rig.irr() & IRR_IRQ1,
            IRR_IRQ1,
            "byte {n}: the 8259A did not latch an edge for it"
        );
        assert!(rig.irq1(), "byte {n}: the line did not rise for it");
        assert!(rig.intr(), "byte {n}: the processor was never asked");

        // Polling the status register is what firmware does while it waits,
        // and it must cost nothing: three more reads, and the byte and the
        // interrupt are both still there.
        for _ in 0..3 {
            assert_eq!(rig.inb(0x64) & ST_OBF, ST_OBF);
        }
        assert_eq!(rig.irr() & IRR_IRQ1, IRR_IRQ1);

        // A debugger looking at 0x60 sees the byte and takes nothing.
        assert_eq!(rig.peek(0x60), *code, "byte {n}: through a debug read");
        assert_eq!(rig.inb(0x64) & ST_OBF, ST_OBF, "byte {n}: still waiting");

        // The handler: take the interrupt, then take the byte. The read is
        // what drops IRQ1, and it has to drop before the next byte arrives or
        // the next byte has no edge to make.
        assert_eq!(
            rig.take_interrupt(),
            IRQ1_VECTOR,
            "byte {n}: the acknowledge cycle fetched the wrong vector"
        );
        assert_eq!(rig.inb(0x60), *code, "byte {n}: the wrong byte came out");
        assert!(!rig.irq1(), "byte {n}: the read did not take IRQ1 down");
        assert_eq!(
            rig.inb(0x64) & ST_OBF,
            0,
            "byte {n}: the buffer refilled inside the read that emptied it, \
             so IRQ1 never fell and the next code would be silent"
        );
        assert_eq!(
            rig.irr() & IRR_IRQ1,
            0,
            "byte {n}: the request is still latched after the acknowledge"
        );
        assert!(
            !rig.intr(),
            "byte {n}: and the processor is still being asked"
        );
    }

    // Nothing is left over: the keyboard's queue is empty, and a tick with
    // nothing to move leaves the line where it is.
    rig.byte_time();
    assert_eq!(rig.inb(0x64) & ST_OBF, 0, "a fifth byte appeared");
    assert!(!rig.irq1(), "and the line rose again for nothing");
}

/// The refill is a serial frame late, not instantaneous.
///
/// The gap is the whole mechanism: it is what gives the edge-triggered input
/// its falling edge. This measures it rather than assuming it — the buffer is
/// empty for a byte-time after the read, and full again after one tick.
#[test]
fn the_next_byte_arrives_a_serial_frame_after_the_read() {
    let mut rig = Rig::new();
    rig.port.feed(&[0x1c, 0x32]);
    rig.byte_time();
    assert_eq!(rig.inb(0x64) & ST_OBF, ST_OBF);
    assert_eq!(rig.take_interrupt(), IRQ1_VECTOR);
    assert_eq!(rig.inb(0x60), 0x1c);

    // Immediately after the read: empty, whatever the keyboard still holds.
    assert_eq!(
        rig.inb(0x64) & ST_OBF,
        0,
        "the second byte was already in the buffer"
    );
    assert_eq!(rig.irr() & IRR_IRQ1, 0);
    assert!(!rig.intr());

    // And a frame later: there.
    rig.byte_time();
    assert_eq!(
        rig.inb(0x64) & ST_OBF,
        ST_OBF,
        "the second byte never came off the cable"
    );
    assert_eq!(rig.irr() & IRR_IRQ1, IRR_IRQ1, "and it interrupted");
    assert_eq!(rig.inb(0x60), 0x32);
}

/// A snapshot taken between two keystrokes restores to the same place.
///
/// There is no pending-refill event to serialize, and this is the test that
/// says so: the refill is the *scheduler's* tick of this device's clock
/// domain, which the machine's own scheduler state carries, and what the
/// device saves is the architectural state the data sheet describes — the
/// output buffer, `OBF`, the command byte and the keyboard's queue. So a
/// machine restored with a byte waiting and three more in the keyboard
/// delivers those three afterwards, one byte-time apart, exactly as the
/// original would have.
#[test]
fn a_snapshot_between_two_keystrokes_goes_on_delivering_afterwards() {
    let mut rig = Rig::new();
    rig.port.feed(&[0x1c, 0x32, 0x21]);
    rig.byte_time();
    assert_eq!(rig.inb(0x64) & ST_OBF, ST_OBF);

    let saved = rig.machine.save().expect("the machine snapshots");

    // Drain it on the original.
    let mut original = Vec::new();
    for _ in 0..3 {
        original.push(rig.inb(0x60));
        rig.byte_time();
    }
    assert_eq!(original, vec![0x1c, 0x32, 0x21]);

    // And on a machine restored from the snapshot, from the same point.
    let mut other = Rig::new();
    other.machine.load(&saved).expect("and restores");
    let mut restored = Vec::new();
    for _ in 0..3 {
        restored.push(other.inb(0x60));
        other.byte_time();
    }
    assert_eq!(
        restored, original,
        "the restored machine delivered a different sequence"
    );
    assert!(
        !other.irq1(),
        "the line is down again after the last byte was taken"
    );
}
