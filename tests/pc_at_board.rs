//! Does the PC/AT board actually assemble, and does an interrupt get all the
//! way from the timer to the processor's pin?
//!
//! Every chip in `src/dev/pc` has unit tests proving it works alone. This proves
//! they fit together the way `machines/pc-at.machine` says they do — which
//! isolated tests structurally cannot show, and which is where a memory map, an
//! I/O map or a wire graph goes wrong.
//!
//! # The processor
//!
//! The real `cpu.x86` core, under the class name the board uses. There used to
//! be a stub here, because `cpu.i8086` was registered but not **bound** — no
//! `Instance` impl, no `bind`, no input pins, no `schema` — so a machine file
//! could not hand it an address space or wire an interrupt to it. That stub was
//! written as a specification of what the core had to grow; the core has grown
//! it, so the stub is gone and the same assertions run against the real thing.
//!
//! The one thing this file still does for itself is build its own [`Bindings`]
//! rather than taking the catalog's, so that the constructor can keep an
//! `Arc<X86>`. `Device` keeps `Any` out of its supertrait chain on purpose, so
//! the only moment a concrete type exists is construction — the same seam
//! `host::display` uses, and for the same reason.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy"
))]

use std::sync::Arc;

use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

// ---------------------------------------------------------------------------
// reaching the processor
// ---------------------------------------------------------------------------

thread_local! {
    /// The core built most recently on this thread.
    ///
    /// There is no route from a `dyn Device` to a concrete type — `Device`
    /// keeps `Any` out of its supertrait chain on purpose — so the handle is
    /// taken at the one moment the concrete type exists: construction.
    static LAST_CPU: std::cell::RefCell<Option<Arc<X86>>> =
        const { std::cell::RefCell::new(None) };
}

/// Everything this board needs to construct, with a `cpu.i8086` that hands a
/// handle back.
fn bindings() -> Bindings {
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    b.bind("cpu.i8086", |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I8088)?.as_i8086());
        LAST_CPU.with(|slot| *slot.borrow_mut() = Some(Arc::clone(&cpu)));
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");
    b
}

// ---------------------------------------------------------------------------
// building the board
// ---------------------------------------------------------------------------

/// A firmware image that is not firmware: recognisable bytes, so a test can say
/// where the socket landed without needing anything to execute.
fn fake_bios(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 251) as u8).collect()
}

/// A video option ROM header, which is all a scan looks at.
fn fake_vgabios(len: usize) -> Vec<u8> {
    let mut v = vec![0u8; len];
    v[0] = 0x55;
    v[1] = 0xaa;
    v[2] = (len / 512) as u8;
    v
}

/// Serialises board construction.
///
/// Both handles a test needs — the stub processor's and the display's — are
/// taken from process-wide tables, because `Device` has no route back to a
/// concrete type. Those tables are documented as "build one machine at a time";
/// `cargo test` runs test functions in parallel, so this is the "at a time".
static BUILDING: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// The board, its stub processor, and its display — all three taken while the
/// construction lock is held, so two tests building at once cannot swap them.
/// A 1.44 MB image whose every sector says which sector it is, so a transfer
/// that lands one sector out is a failure rather than a coincidence.
fn fake_floppy() -> Vec<u8> {
    const SECTORS: usize = 2880;
    let mut image = vec![0u8; SECTORS * 512];
    for lba in 0..SECTORS {
        let at = lba * 512;
        image[at] = 0xa5;
        image[at + 1] = lba as u8;
        image[at + 2] = (lba >> 8) as u8;
        image[at + 511] = 0x5a;
    }
    // The boot signature, where firmware looks for it.
    image[510] = 0x55;
    image[511] = 0xaa;
    image
}

fn board_with_display() -> (
    rsemu::machine::Machine,
    Arc<X86>,
    rsemu::dev::pc::video::VideoScanout,
) {
    let guard = BUILDING.lock().unwrap_or_else(|e| e.into_inner());
    let machine = build_board();
    let cpu = LAST_CPU
        .with(|slot| slot.borrow().clone())
        .expect("the constructor kept a handle");
    let scanout =
        rsemu::host::display::pc::capture::take().expect("the board has a display adapter");
    drop(guard);
    (machine, cpu, scanout)
}

/// The board and its processor, for a test with no interest in the picture.
fn board() -> (rsemu::machine::Machine, Arc<X86>) {
    let (machine, cpu, _) = board_with_display();
    (machine, cpu)
}

fn build_board() -> rsemu::machine::Machine {
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings());
    options.realize.media.insert("bios", fake_bios(128 * 1024));
    options
        .realize
        .media
        .insert("vgabios", fake_vgabios(32 * 1024));
    // 1.44 MB of zeroes: a formatted-looking blank, enough for the controller
    // to infer a geometry and report a drive that is ready.
    options.realize.media.insert("floppy", fake_floppy());
    // Intercept the display's constructor so a test can look at the picture,
    // exactly as `rsemu run --screenshot` does.
    rsemu::host::display::pc::capture::clear();
    rsemu::host::display::pc::capture::install(&mut options).expect("one display class");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    match build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read one byte of the guest's memory space.
fn peek(m: &rsemu::machine::Machine, addr: u64) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte")
}

/// Write one byte to an I/O port, as an `OUT` would.
fn outb(m: &rsemu::machine::Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

/// Read one byte from an I/O port, as an `IN` would.
fn inb(m: &rsemu::machine::Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[cfg(feature = "machine-pc-at")]
#[test]
fn the_catalog_realizes_this_board_with_its_own_bindings() {
    // The rest of this file builds its own `Bindings` so it can keep a handle
    // on the core. This one does not: it goes through `catalog::build_options`,
    // which is what `rsemu run pc-at` uses — and which could only realize the
    // board once `cpu.i8086` was bound, `schema`'d and given its pins.
    let _guard = BUILDING.lock().unwrap_or_else(|e| e.into_inner());
    let entry = rsemu::machine::catalog::machine("pc-at").expect("this build ships pc-at");
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", fake_bios(128 * 1024));
    options
        .realize
        .media
        .insert("vgabios", fake_vgabios(32 * 1024));
    options.realize.media.insert("floppy", fake_floppy());
    rsemu::host::display::pc::capture::clear();
    rsemu::host::display::pc::capture::install(&mut options).expect("one display class");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = match build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the catalog cannot realize its own pc-at: {e}"),
    };
    m.reset(ResetKind::Cold);
    m.sweep();
    // And it runs: the core fetches from the reset vector at the top of the
    // ROM's high window and executes whatever is there. The image is not
    // firmware, so what it executes is nonsense — the assertion is that the
    // scheduler drove a bound processor at all, which a machine whose CPU has
    // no address space cannot do.
    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(200_000))
        .expect("the machine runs");
}

#[test]
fn the_board_realizes_with_every_chip_mapped_and_wired() {
    let (m, _cpu) = board();
    assert_eq!(m.name(), "pc-at");
    for path in [
        "cpu0", "ram_low", "ram_high", "pic1", "pic2", "pit0", "cmos", "kbc", "sysctl", "dma1",
        "dma2", "vga", "fdc", "bios", "vgarom",
    ] {
        assert!(
            m.device(path).is_some(),
            "no `{path}` in the realized board"
        );
    }
    // Two spaces, and they are genuinely separate: the 8086 family drives I/O
    // with its own status lines, so port 0x60 is not memory address 0x60.
    assert_eq!(m.spaces().len(), 2);
}

#[test]
fn the_system_rom_answers_in_both_of_its_windows() {
    // The one mapping on this board that looks like a mistake and is not: a 386
    // fetches its first instruction from just below 4 GiB and only reaches
    // 0xf0000 after firmware's own first far jump. Both windows are one chip.
    let (m, _cpu) = board();
    let image = fake_bios(128 * 1024);
    let low = 0x000e_0000u64;
    let high = 0x1_0000_0000u64 - 128 * 1024;
    for offset in [0u64, 1, 0x1_0000, 128 * 1024 - 16, 128 * 1024 - 1] {
        let expect = u64::from(image[offset as usize]);
        assert_eq!(peek(&m, low + offset), expect, "low window at {offset:#x}");
        assert_eq!(
            peek(&m, high + offset),
            expect,
            "high window at {offset:#x}"
        );
    }
    // And the reset vector itself, sixteen bytes below the top.
    assert_eq!(peek(&m, 0xffff_fff0), peek(&m, 0x000e_0000 + 0x1_fff0));
}

#[test]
fn the_video_option_rom_signature_is_where_a_scan_looks_for_it() {
    // Firmware finds a video BIOS by walking 0xc0000 upward on 2 KiB boundaries
    // looking for 0x55 0xaa. A top-aligned image would hide it.
    let (m, _cpu) = board();
    assert_eq!(peek(&m, 0x000c_0000), 0x55);
    assert_eq!(peek(&m, 0x000c_0001), 0xaa);
    assert_eq!(peek(&m, 0x000c_0002), 64, "32 KiB, in 512-byte blocks");
}

#[test]
fn base_memory_is_writable_and_the_video_hole_is_not_ram() {
    let (m, _cpu) = board();
    let mem = m.space("mem").expect("the memory space");
    mem.write(0x0004_1234, Width::U8, 0x5a, MemAttrs::DEFAULT)
        .expect("base memory");
    assert_eq!(peek(&m, 0x0004_1234), 0x5a);
    // Extended memory, above the 1 MiB line.
    mem.write(0x0020_0000, Width::U8, 0xa5, MemAttrs::DEFAULT)
        .expect("extended memory");
    assert_eq!(peek(&m, 0x0020_0000), 0xa5);
    // 0xa0000 is the video hole: nothing is mapped, and an ISA bus with nothing
    // driving it reads as ones. Firmware sizes memory by exactly this.
    assert_eq!(peek(&m, 0x000a_0000), 0xff);
}

#[test]
fn every_chip_answers_at_the_port_the_at_decodes_it_at() {
    let (mut m, _cpu) = board();
    m.reset(ResetKind::Cold);
    // The 8042's status port: bit 2, the system flag, is clear before the
    // self test and set after it, which is a reply only the chip can give.
    assert_eq!(inb(&m, 0x64) & 0x04, 0, "the system flag before self test");
    outb(&m, 0x64, 0xaa);
    assert_eq!(inb(&m, 0x60), 0x55, "the 8042 self test");

    // The RTC: seconds read back as the machine file's declared start time.
    outb(&m, 0x70, 0x00);
    assert_eq!(inb(&m, 0x71), 0x00, "the declared start second");
    outb(&m, 0x70, 0x09);
    assert_eq!(inb(&m, 0x71), 0x26, "the year, in BCD, from `time`");

    // The CMOS base-memory bytes, which firmware reads before it trusts
    // anything else.
    outb(&m, 0x70, 0x15);
    let lo = u32::from(inb(&m, 0x71));
    outb(&m, 0x70, 0x16);
    let hi = u32::from(inb(&m, 0x71));
    assert_eq!(lo | (hi << 8), 640, "640 KiB of base memory");

    // Port B's timer-2 gate reads back what was written, and port A's A20 bit
    // reaches the processor's pin.
    outb(&m, 0x61, 0x01);
    assert_eq!(inb(&m, 0x61) & 0x01, 0x01);
    outb(&m, 0x92, 0x02);

    // The floppy controller. Bit 2 of the digital output register is the
    // controller's reset, active low, so a board that has only been powered on
    // holds it in reset — which is why every BIOS's first floppy access is a
    // write here. Taking it out of reset makes the main status register say it
    // is ready for a command.
    outb(&m, 0x3f2, 0x0c);
    assert_eq!(inb(&m, 0x3f4) & 0xc0, 0x80, "RQM set, DIO clear");
}

#[test]
fn the_fast_a20_gate_reaches_the_processors_pin() {
    let (mut m, cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();
    assert!(!cpu.a20_open(), "A20 is shut at power on");
    outb(&m, 0x92, 0x02);
    assert!(cpu.a20_open(), "port 0x92 bit 1 opens it");
    // And the keyboard controller's path opens the same net — two drivers,
    // wire-ORed, which is why the pin keeps a `FanIn`.
    outb(&m, 0x92, 0x00);
    assert!(!cpu.a20_open());
    outb(&m, 0x64, 0xd1);
    outb(&m, 0x60, 0x03);
    assert!(cpu.a20_open(), "the 8042's output port opens it too");
}

#[test]
fn both_reset_paths_pulse_the_processors_reset_pin() {
    // Two drivers on one net again, and the reason every PC has two ways to
    // reboot: the keyboard controller's pulse command is the original, and the
    // chipset's port 0x92 bit 0 is the fast one firmware uses to leave
    // protected mode.
    let (mut m, cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();
    assert!(!cpu.reset_requested(), "nothing has pulsed it yet");
    outb(&m, 0x64, 0xfe);
    assert!(
        cpu.reset_requested(),
        "the 8042's pulse command must reach the pin"
    );

    // The latch is consumed by the sequence it asks for, which is what makes
    // the second path observable rather than indistinguishable from the first.
    cpu.step();
    assert!(!cpu.reset_requested());
    outb(&m, 0x92, 0x01);
    assert!(
        cpu.reset_requested(),
        "and so must the chipset's fast reset"
    );
}

#[test]
fn a_timer_tick_reaches_the_processor_and_acknowledges_to_a_vector() {
    // The whole point of the board: a crystal, through a counter, through two
    // interrupt controllers, onto a pin — and then the acknowledge cycle back
    // down the same wire to fetch the vector.
    let (mut m, cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();
    assert!(!cpu.intr_asserted(), "nothing is pending at power on");

    // Initialise the master exactly as a PC's firmware does: ICW1 with ICW4 to
    // follow, vector base 0x08, a slave on IR2, 8086 mode. Then unmask IR0
    // alone.
    outb(&m, 0x20, 0x11);
    outb(&m, 0x21, 0x08);
    outb(&m, 0x21, 0x04);
    outb(&m, 0x21, 0x01);
    outb(&m, 0x21, 0xfe);

    // Counter 0, low-then-high access, mode 2, binary. A divisor of 100 is
    // 83.8 microseconds of virtual time at 105/88 MHz.
    outb(&m, 0x43, 0x34);
    outb(&m, 0x40, 100);
    outb(&m, 0x40, 0);

    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");

    assert!(
        cpu.intr_asserted(),
        "the timer's output never reached the processor's pin"
    );
    assert_eq!(
        cpu.acknowledge(),
        0x08,
        "the acknowledge cycle must return the vector the controller was given"
    );
    // Having taken it, the line drops: the request moved from pending to in
    // service, which is the bookkeeping `IntAck` exists to make possible.
    assert!(!cpu.intr_asserted(), "the request is now in service");

    // And an end-of-interrupt lets the next tick through.
    outb(&m, 0x20, 0x20);
    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");
    assert!(cpu.intr_asserted(), "the next tick");
}

/// Save `m`, restore that snapshot into a freshly built board, and compare the
/// two one device chunk at a time.
///
/// The whole-machine hash is a boolean. It says a byte moved somewhere in
/// sixteen devices, which is a much worse bug report than the one the format
/// can give for free: chunks are keyed by device instance path (§4.5), so
/// walking them costs a loop and turns "the board did not round-trip" into
/// "pit0's byte 58". The hash is checked too, at the end, because it is the
/// thing §15's invariant is actually written in terms of.
///
/// `what` names the point in the board's life this is being asked at, because
/// a device only fails to round-trip in the state it was driven into.
#[track_caller]
fn assert_round_trips(m: &rsemu::machine::Machine, what: &str) {
    let image = m.save().expect("the board saves");

    let (mut other, _) = board();
    other.reset(ResetKind::Cold);
    other.load(&image).expect("the board loads");
    let again = other.save().expect("the restored board saves");

    let from = rsemu::core::state::StateReader::new(&image).expect("a snapshot we just wrote");
    let to = rsemu::core::state::StateReader::new(&again).expect("a snapshot we just wrote");
    let mut report = String::new();
    for chunk in from.chunks() {
        let (_, _, saved) = from.load_raw(chunk.path).expect("the chunk we just listed");
        let (_, _, restored) = to
            .load_raw(chunk.path)
            .expect("the same board, so the same chunk keys");
        if saved == restored {
            continue;
        }
        report.push_str(&format!(
            "\n  {} ({}): {} bytes saved, {} restored",
            chunk.path,
            chunk.class,
            saved.len(),
            restored.len()
        ));
        for (at, (a, b)) in saved.iter().zip(restored.iter()).enumerate() {
            if a != b {
                report.push_str(&format!(
                    "\n    byte {at}: saved {a:#04x}, restored {b:#04x}"
                ));
            }
        }
    }
    assert!(
        report.is_empty(),
        "{what}: a restored board must be indistinguishable from the one it \
         came from, and these chunks differ:{report}"
    );
    assert_eq!(
        other.state_hash().expect("a hash"),
        m.state_hash().expect("a hash"),
        "{what}: the chunks all match but the state hash does not, so something \
         outside a device chunk moved"
    );
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let (mut m, _cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();

    // Straight out of a cold reset, before the guest has touched anything.
    //
    // This is the state every level the chipset shares between two chips sits
    // at its power-on default in, and it is precisely the state a test that
    // programs the ports first can no longer reach: writing 0x03 to port 0x61
    // drives GATE2 high, and driving a shared level *repairs* a chip whose idea
    // of that pin had drifted from the board's. A round-trip test that only
    // ever looks after the pokes proves the least interesting case.
    assert_round_trips(&m, "at rest, out of a cold reset");

    // Driven, by an x86 rather than by this file. The image is not firmware, so
    // what it executes is nonsense — but it is nonsense on a real bus, and what
    // it pokes at is not up to us.
    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(500_000))
        .expect("the machine runs");
    assert_round_trips(&m, "after 500 us of the reset-vector image");

    // And programmed, the way firmware programs it: a periodic tick, the
    // speaker gate up, an RTC control register written.
    outb(&m, 0x43, 0x34);
    outb(&m, 0x40, 100);
    outb(&m, 0x40, 0);
    outb(&m, 0x61, 0x03);
    outb(&m, 0x70, 0x0b);
    outb(&m, 0x71, 0x42);
    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(500_000))
        .expect("the machine runs");
    assert_round_trips(&m, "after the chipset has been programmed");

    // Then the gate back down, so the shared level is exercised in both
    // directions rather than only the one a write happens to set.
    outb(&m, 0x61, 0x00);
    m.run_for(rsemu::core::clock::GlobalTime::from_nanos(500_000))
        .expect("the machine runs");
    assert_round_trips(&m, "after the speaker gate has been lowered again");
}

#[test]
fn counter_2_does_not_count_until_port_0x61_gates_it() {
    // The level port 0x61 bit 0 drives lives in two chips: the system control
    // port latches it and the 8254 sees it on GATE2. Only one of them owns it,
    // and this is the assertion that says which — because a timer that has
    // helped itself to a default for an input pin is indistinguishable from a
    // correct one until something asks it to stand still.
    let (mut m, _cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();

    // Counter 2, low-then-high, mode 3, binary, divided by 100 — about 84 us of
    // square wave, if it is allowed to run.
    outb(&m, 0x43, 0xb6);
    outb(&m, 0x42, 100);
    outb(&m, 0x42, 0);

    // Port 0x61 bit 0 is clear out of a cold reset, so GATE2 is low, so mode 3
    // holds OUT2 high and does not count. Bit 5 of the same port is that OUT2
    // pin brought back to the bus, which is how firmware watches it.
    assert_eq!(inb(&m, 0x61) & 0x01, 0x00, "the gate is down to begin with");
    for _ in 0..20 {
        m.run_for(rsemu::core::clock::GlobalTime::from_nanos(20_000))
            .expect("the machine runs");
        assert_eq!(
            inb(&m, 0x61) & 0x20,
            0x20,
            "counter 2 counted through a gate the board is holding low"
        );
    }

    // Raise it and the same counter runs.
    outb(&m, 0x61, 0x01);
    let mut went_low = false;
    for _ in 0..20 {
        m.run_for(rsemu::core::clock::GlobalTime::from_nanos(20_000))
            .expect("the machine runs");
        went_low |= inb(&m, 0x61) & 0x20 == 0;
    }
    assert!(went_low, "raising the gate never started counter 2");
}

// ---------------------------------------------------------------------------
// the picture
// ---------------------------------------------------------------------------

/// Put a string into the colour text page, the way firmware's teletype output
/// does: a character byte and an attribute byte per cell, starting at 0xb8000.
fn write_text(m: &rsemu::machine::Machine, row: u32, col: u32, text: &str, attr: u8) {
    let mem = m.space("mem").expect("the memory space");
    let mut at = 0x000b_8000u64 + u64::from(row * 80 + col) * 2;
    for byte in text.bytes() {
        mem.write(at, Width::U8, u64::from(byte), MemAttrs::DEFAULT)
            .expect("the text page");
        mem.write(at + 1, Width::U8, u64::from(attr), MemAttrs::DEFAULT)
            .expect("the text page");
        at += 2;
    }
}

#[test]
fn what_the_guest_writes_into_the_text_page_comes_out_of_the_scanout() {
    // The last link in the chain: guest memory, through the character
    // generator and the DAC, to host pixels. If this works, `--screenshot`
    // works, and so does the browser.
    use rsemu::host::display::{Scanout, Surface};

    let (mut m, _cpu, scanout) = board_with_display();
    m.reset(ResetKind::Cold);

    write_text(&m, 0, 0, "rsemu pc-at", 0x0f);
    write_text(
        &m,
        2,
        0,
        "no firmware is shipped; point --bios at your own",
        0x07,
    );

    let mut surface = Surface::for_scanout(&scanout);
    scanout.capture(&mut surface);

    // 80 columns of nine-pixel cells by 25 rows of sixteen: the shape a VGA
    // comes out of reset in.
    assert_eq!(surface.width(), 720);
    assert_eq!(surface.height(), 400);

    // Something was actually drawn: the first row is not uniform.
    let row = surface.row(0).expect("the top row").to_vec();
    let mut nonblank = 0usize;
    for y in 0..surface.height() {
        for x in 0..surface.width() {
            if surface.get(x, y) != Some([0, 0, 0]) {
                nonblank += 1;
            }
        }
    }
    assert!(
        nonblank > 200,
        "only {nonblank} lit pixels — the character generator drew nothing"
    );
    let _ = row;

    // Deterministic, because everything under it is: the same board, the same
    // bytes, the same picture, on any host (`ROADMAP.md` §0).
    let first = surface.hash();
    let mut again = Surface::for_scanout(&scanout);
    scanout.capture(&mut again);
    assert_eq!(again.hash(), first, "the same frame twice");

    // Written out only when asked, into a directory the caller names — the
    // convention the conformance corpora already use. `RSEMU_SCREENSHOT_DIR=…
    // cargo test --all-features pc_at_board` regenerates it.
    #[cfg(feature = "display-png")]
    if let Ok(dir) = std::env::var("RSEMU_SCREENSHOT_DIR") {
        let bytes = rsemu::host::display::png::encode(&surface).expect("a PNG");
        let path = std::path::Path::new(&dir).join("pc-at.png");
        std::fs::write(&path, &bytes).expect("the screenshot directory exists");
        println!("wrote {} ({} bytes)", path.display(), bytes.len());
    }
}

#[test]
fn a_sector_travels_from_the_floppy_through_the_dma_controller_into_memory() {
    // The other end-to-end path, and the one a boot depends on: a command to
    // the controller, a DRQ, an 8237 that builds the physical address the chip
    // cannot drive, and 512 bytes in guest memory. Nothing in this test knows
    // about either device's internals — it is written the way firmware writes
    // it, through the I/O ports.
    let (mut m, _cpu) = board();
    m.reset(ResetKind::Cold);
    m.sweep();

    // Where the sector is to land: 0x00500, well inside base memory.
    const DEST: u64 = 0x0000_0500;

    // Channel 2, single mode, write to memory, no autoinitialise, increment.
    outb(&m, 0x0a, 0x06); // mask channel 2 while it is programmed
    outb(&m, 0x0c, 0x00); // clear the byte-pointer flip-flop
    outb(&m, 0x0b, 0x46); // mode: single, write, increment, channel 2
    outb(&m, 0x04, (DEST & 0xff) as u8);
    outb(&m, 0x04, ((DEST >> 8) & 0xff) as u8);
    outb(&m, 0x81, (DEST >> 16) as u8); // the page latch for channel 2
    outb(&m, 0x0c, 0x00);
    outb(&m, 0x05, 0xff); // count is n-1, so 0x01ff is 512 bytes
    outb(&m, 0x05, 0x01);
    outb(&m, 0x0a, 0x02); // unmask channel 2

    // The controller: out of reset, DMA and interrupts enabled, motor on.
    outb(&m, 0x3f2, 0x1c);
    // Four SENSE INTERRUPT STATUS commands clear the post-reset state, which is
    // what every BIOS does before its first real command.
    for _ in 0..4 {
        outb(&m, 0x3f5, 0x08);
        let _st0 = inb(&m, 0x3f5);
        let _pcn = inb(&m, 0x3f5);
    }

    // READ DATA, MFM, drive 0, head 0, cylinder 0, sector 1.
    for byte in [0x46u8, 0x00, 0x00, 0x00, 0x01, 0x02, 0x12, 0x1b, 0xff] {
        outb(&m, 0x3f5, byte);
    }

    // The result phase: seven bytes, and ST0's top two bits clear means the
    // command finished normally.
    let st0 = inb(&m, 0x3f5);
    let st1 = inb(&m, 0x3f5);
    let _st2 = inb(&m, 0x3f5);
    let cylinder = inb(&m, 0x3f5);
    let head = inb(&m, 0x3f5);
    let sector = inb(&m, 0x3f5);
    let _n = inb(&m, 0x3f5);
    assert_eq!(st0 & 0xc0, 0x00, "ST0 says the read failed: {st0:#04x}");
    assert_eq!(st1, 0x00, "ST1 reports an error: {st1:#04x}");
    assert_eq!(
        (cylinder, head, sector),
        (0, 0, 2),
        "the next sector address"
    );

    // And the bytes are in memory, at the address the page latch and the
    // address register named between them.
    let image = fake_floppy();
    for offset in [0u64, 1, 2, 100, 510, 511] {
        assert_eq!(
            peek(&m, DEST + offset),
            u64::from(image[offset as usize]),
            "byte {offset} of the sector"
        );
    }
    // Sector one, not sector two: the classic off-by-one, caught.
    assert_eq!(peek(&m, DEST + 1), 0x00, "the sector number in the payload");
    // And the boot signature is where firmware looks.
    assert_eq!(peek(&m, DEST + 510), 0x55);
    assert_eq!(peek(&m, DEST + 511), 0xaa);
}
