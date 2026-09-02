//! Does the q35 board assemble, does its ECAM window answer, and are the ACPI
//! tables it generates the machine's own?
//!
//! Every part of the chipset has unit tests proving it works alone. This proves
//! they fit together the way `machines/q35.machine` says they do, which is
//! where a memory map, an I/O map or a wire graph goes wrong — and it proves the
//! one claim that cannot be made anywhere else: **the tables describe the
//! machine that was actually built.**
//!
//! The strongest assertion here is
//! [`the_tables_describe_the_machine_that_was_built`], which moves the ECAM
//! window with a guest configuration write, regenerates, and checks that the
//! MCFG followed. Nothing in the generator is told where the window is; if the
//! two agree after the window has moved, they agree because one read the other.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-q35",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
    feature = "dev-pc-video",
    feature = "dev-pc-ide",
    feature = "machine-q35"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::dev::q35::acpi;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

// ---------------------------------------------------------------------------
// building the board
// ---------------------------------------------------------------------------

/// Everything this board needs to construct, with a `cpu.x86` that pushes what
/// it builds into `cpus`.
///
/// The same shape `tests/pc_at_board.rs` uses, and for the same reason: there
/// is no route from a `dyn Device` to a concrete type, so the handle is taken
/// at the one moment the concrete type exists.
fn bindings(cpus: &Arc<Captured<X86>>, tables: &Arc<Captured<acpi::AcpiTables>>) -> Bindings {
    let mut b = rsemu::machine::catalog::bindings().expect("this build's bindings");
    let kept = Arc::clone(cpus);
    b.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    // The same capture, for the same reason: `Device` keeps `Any` out of its
    // supertrait chain on purpose, so construction is the one moment the
    // concrete type exists, and a test that wants to call `regenerate` has to
    // take its handle there.
    let kept = Arc::clone(tables);
    b.replace("q35.acpi", move |props| {
        let device = Arc::new(acpi::AcpiTables::new(props)?);
        kept.push(&device);
        Ok(device)
    });
    b
}

/// Build the board from its own machine file, with blank media in every slot.
///
/// A blank BIOS socket: this file is about the chipset, not about firmware, and
/// 64 KiB of zeroes realizes and executes open bus.
fn board() -> (Machine, Arc<X86>, Arc<acpi::AcpiTables>) {
    board_with_bios(vec![0u8; 64 * 1024])
}

/// The same board with `bios` filled from `image`.
fn board_with_bios(image: Vec<u8>) -> (Machine, Arc<X86>, Arc<acpi::AcpiTables>) {
    board_with_bios_and_disk(image, Vec::new())
}

/// The same board again, with a drive in the primary channel's master bay.
fn board_with_bios_and_disk(
    image: Vec<u8>,
    hd0: Vec<u8>,
) -> (Machine, Arc<X86>, Arc<acpi::AcpiTables>) {
    board_configured(image, hd0, &[])
}

/// The same board once more, with machine-file parameters overridden.
///
/// The one thing that needs it is a **128 KiB** third-party firmware: the
/// board's own socket is 64 KiB at `0xf0000` because `0xe0000`-`0xeffff` is
/// where the generated ACPI tables go, and a firmware that wants the whole
/// `0xe0000`-`0xfffff` band has to be given it — and has to be publishing its
/// own tables, which is the arrangement a real machine has anyway.
fn board_configured(
    image: Vec<u8>,
    hd0: Vec<u8>,
    params: &[(&str, &str)],
) -> (Machine, Arc<X86>, Arc<acpi::AcpiTables>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let tables: Arc<Captured<acpi::AcpiTables>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus, &tables));
    for (name, value) in params {
        options = options.with_param(*name, *value);
    }
    options.realize.media.insert("bios", image);
    options
        .realize
        .media
        .insert("vgabios", vec![0u8; 32 * 1024]);
    options.realize.media.insert("hd0", hd0);
    options.realize.media.insert("hd1", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = match build("q35.machine", rsemu::dev::q35::Q35, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (
        machine,
        cpus.take().expect("the constructor kept a handle"),
        tables.take().expect("the constructor kept a handle"),
    )
}

/// The board's memory space.
fn mem(machine: &Machine) -> Arc<rsemu::core::space::AddressSpace> {
    Arc::clone(machine.space("mem").expect("the board declares `mem`"))
}

/// The board's I/O space.
fn port(machine: &Machine) -> Arc<rsemu::core::space::AddressSpace> {
    Arc::clone(machine.space("port").expect("the board declares `port`"))
}

/// Read a dword out of a space the way a guest would.
fn read32(space: &rsemu::core::space::AddressSpace, at: u64) -> u32 {
    space
        .read(at, Width::U32, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read of {at:#x} faulted: {e:?}")) as u32
}

/// Read a run of bytes out of a space the way a guest would.
fn read_bytes(space: &rsemu::core::space::AddressSpace, at: u64, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    space
        .read_bytes(at, &mut out, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("read of {at:#x} faulted: {e:?}"));
    out
}

// ---------------------------------------------------------------------------
// the chipset
// ---------------------------------------------------------------------------

#[test]
fn the_board_realizes_and_the_host_bridge_answers_the_legacy_port_pair() {
    let (machine, _cpu, _tables) = board();
    let port = port(&machine);
    // Configuration mechanism #1, exactly as a 1996 firmware would: address
    // 00:00.0 register 0 and read the identification back.
    port.write(0xcf8, Width::U32, 0x8000_0000, MemAttrs::DEFAULT)
        .expect("CONFADD is a dword register");
    let id = read32(&port, 0xcfc);
    assert_eq!(
        id, 0x29c0_8086,
        "the board presents an 82G33/82P35 (G)MCH — datasheet 316966-002 \
         Table 5-1's own default value, and the one a current firmware \
         recognises; `machines/q35.machine` says why"
    );
    // And the south bridge at 00:1f.0, by class code rather than by its
    // identification — which is the only half of it this board can cite.
    port.write(0xcf8, Width::U32, 0x8000_f808, MemAttrs::DEFAULT)
        .expect("CONFADD");
    let class = read32(&port, 0xcfc);
    assert_eq!(
        class >> 8,
        0x0006_0100,
        "an ISA bridge is class 060100h (ICH9 §13.1.6-§13.1.8)"
    );
}

#[test]
fn the_ecam_window_reaches_the_same_functions_as_the_port_pair() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    // The board pre-programs PCIEXBAR to 0xe0000000 with the enable set, so
    // the window is decoded from the first instruction.
    const ECAM: u64 = 0xe000_0000;
    assert_eq!(read32(&mem, ECAM), 0x29c0_8086, "00:00.0 through ECAM");
    // 00:1f.0 is at base + device * 32 KiB.
    assert_eq!(
        read32(&mem, ECAM + 31 * 32 * 1024) & 0xffff,
        0x8086,
        "00:1f.0 through ECAM"
    );
    // 00:02.0, the VGA card, and its class code — 030000h, which is what a
    // firmware hunts for when it goes looking for the console.
    assert_eq!(
        read32(&mem, ECAM + 2 * 32 * 1024 + 0x08) >> 8,
        0x0003_0000,
        "the display adapter at 00:02.0"
    );
    // An address nothing answers at master-aborts and reads as ones.
    assert_eq!(read32(&mem, ECAM + 9 * 32 * 1024), 0xffff_ffff);
    // And extended configuration space is zero, which is a function with no
    // extended capabilities rather than a hole.
    assert_eq!(read32(&mem, ECAM + 0x100), 0);
}

#[test]
fn a_guest_can_move_the_ecam_window_and_it_follows() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    let port = port(&machine);
    assert_eq!(read32(&mem, 0xe000_0000), 0x29c0_8086);
    // Write PCIEXBAR's low dword through the port pair: base 0xd0000000, 256 MB,
    // enabled. The write arrives through the I/O space, so the memory space's
    // try-lock succeeds and the window moves immediately.
    port.write(0xcf8, Width::U32, 0x8000_0060, MemAttrs::DEFAULT)
        .expect("CONFADD");
    port.write(0xcfc, Width::U32, 0xd000_0001, MemAttrs::DEFAULT)
        .expect("CONFDATA");
    assert_eq!(
        read32(&mem, 0xd000_0000),
        0x29c0_8086,
        "the window moved to where PCIEXBAR now points"
    );
    assert_eq!(
        read32(&mem, 0xe000_0000),
        0xffff_ffff,
        "and stopped decoding where it was"
    );
    // Clearing the enable bit takes it out of the map entirely.
    port.write(0xcfc, Width::U32, 0xd000_0000, MemAttrs::DEFAULT)
        .expect("CONFDATA");
    assert_eq!(read32(&mem, 0xd000_0000), 0xffff_ffff);
}

#[test]
fn the_pam_registers_are_at_the_q35s_offsets_and_shadow_the_bios() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    let port = port(&machine);
    // The BIOS socket is a run of zeroes in this test, so a ROM read is 0 and
    // DRAM that has been written is not. Out of reset every PAM nibble is 0 and
    // the socket answers.
    assert_eq!(read32(&mem, 0xf_0000), 0);
    // PAM0[7:4] = 11b, read/write, written through the *port pair*. The window
    // it moves is in the memory space and the write travels through the I/O
    // space, so the order-exempt try-lock succeeds and the map moves at once.
    port.write(0xcf8, Width::U32, 0x8000_0090, MemAttrs::DEFAULT)
        .expect("CONFADD");
    port.write(0xcfc, Width::U8, 0x30, MemAttrs::DEFAULT)
        .expect("PAM0 is a byte register at 0x90");
    mem.write(0xf_0000, Width::U32, 0xdead_beef, MemAttrs::DEFAULT)
        .expect("the shadow answers writes now");
    assert_eq!(
        read32(&mem, 0xf_0000),
        0xdead_beef,
        "PAM0 at 0x90 switched DRAM into the f-segment"
    );
    // And 0x59, where a 440FX keeps PAM0, does nothing here — which is the
    // whole reason this board needs its own north bridge.
    port.write(0xcf8, Width::U32, 0x8000_0090, MemAttrs::DEFAULT)
        .expect("CONFADD");
    port.write(0xcfc, Width::U8, 0x00, MemAttrs::DEFAULT)
        .expect("PAM0");
    assert_eq!(read32(&mem, 0xf_0000), 0, "back to the ROM socket");
    port.write(0xcf8, Width::U32, 0x8000_0058, MemAttrs::DEFAULT)
        .expect("CONFADD");
    port.write(0xcfc + 1, Width::U8, 0x30, MemAttrs::DEFAULT)
        .expect("a reserved byte takes a write and drops it");
    assert_eq!(read32(&mem, 0xf_0000), 0, "0x59 is not a PAM register here");
}

#[test]
fn a_pam_write_through_ecam_lands_on_the_bridges_next_scheduler_tick() {
    // The case a 440FX cannot have, and the reason `q35.mch` takes a clock
    // domain. A configuration write can only retopologise a space the access is
    // not already travelling through; through ECAM the access *is* in the
    // memory space, so the PAM window cannot move under it. On a 440FX the
    // retry lands at the next configuration access, because there is only one
    // route to configuration space and it is in the other space. Here every
    // route a q35 firmware uses is in this space, so the retry would never
    // land — and instead the bridge asks the scheduler for a tick.
    //
    // This test is the proof that the mechanism is real rather than a comment:
    // the write does nothing until the machine is *run*, and then it lands.
    const ECAM: u64 = 0xe000_0000;
    let (mut machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    assert_eq!(read32(&mem, 0xf_0000), 0, "the ROM socket, out of reset");

    mem.write(ECAM + 0x90, Width::U8, 0x30, MemAttrs::DEFAULT)
        .expect("PAM0 through ECAM");
    assert_eq!(
        read32(&mem, 0xf_0000),
        0,
        "the window cannot move out from under the access that moved it"
    );
    // Not even another configuration access helps: that one is in this space
    // too. This assertion is the whole reason the clock domain exists.
    let _ = read32(&mem, ECAM + 0x90);
    assert_eq!(read32(&mem, 0xf_0000), 0);

    // One tick of the bridge's own domain is all it takes.
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(1_000))
        .expect("the board runs");
    mem.write(0xf_0000, Width::U32, 0xdead_beef, MemAttrs::DEFAULT)
        .expect("the shadow answers writes now");
    assert_eq!(
        read32(&mem, 0xf_0000),
        0xdead_beef,
        "the owed retopology did not land on the bridge's scheduler tick"
    );
}

#[test]
fn the_acpi_register_block_decodes_where_pmbase_says() {
    let (mut machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    let port = port(&machine);
    // The board pre-programs PMBASE to 0x600 with ACPI_EN set.
    const PM: u64 = 0x600;
    // The power-management timer counts. Two reads with the machine run in
    // between must differ, and neither may fault.
    let first = read32(&port, PM + 0x08);
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(1_000_000))
        .expect("the board runs");
    let second = read32(&port, PM + 0x08);
    assert!(
        second > first,
        "the PM timer did not advance: {first} then {second}"
    );
    // 3.579545 MHz for a millisecond is about 3579 counts. Loose bounds,
    // because the point is that the rate is the crystal's and not something
    // else.
    let ticks = second - first;
    assert!(
        (3000..4200).contains(&ticks),
        "a millisecond of a 3.579545 MHz clock is about 3579 counts, not {ticks}"
    );

    // Clearing ACPI_EN takes the window out of the I/O space — and it has to be
    // done through **ECAM**, because the window is in the I/O space and a
    // configuration write through 0xcfc is travelling through it. This is the
    // mirror image of the PAM case above, and between the two of them they say
    // exactly what the try-lock rule buys: a q35 can move either kind of window
    // promptly because it has two ways to reach configuration space.
    const ECAM: u64 = 0xe000_0000;
    const LPC: u64 = ECAM + 31 * 32 * 1024;
    mem.write(LPC + 0x44, Width::U8, 0x00, MemAttrs::DEFAULT)
        .expect("ACPI_CNTL is a byte register");
    assert_eq!(
        read32(&port, PM + 0x08),
        0xffff_ffff,
        "with ACPI_EN clear nothing decodes there"
    );
    // And moving PMBASE while it is disabled, then re-enabling, puts the block
    // somewhere else entirely.
    mem.write(LPC + 0x40, Width::U32, 0x0000_0480, MemAttrs::DEFAULT)
        .expect("PMBASE");
    mem.write(LPC + 0x44, Width::U8, 0x80, MemAttrs::DEFAULT)
        .expect("ACPI_CNTL");
    assert_ne!(
        read32(&port, 0x480 + 0x08),
        0xffff_ffff,
        "the block did not follow PMBASE to 0x480"
    );
    assert_eq!(read32(&port, PM + 0x08), 0xffff_ffff, "and left 0x600");
}

// ---------------------------------------------------------------------------
// the tables
// ---------------------------------------------------------------------------

/// Find a table by signature by walking the XSDT, the way an operating system
/// does: RSDP at 0xe0000, XSDT address out of it, then its entry list.
fn find_table(space: &rsemu::core::space::AddressSpace, signature: &[u8; 4]) -> Option<u64> {
    let rsdp = read_bytes(space, 0xe_0000, 36);
    assert_eq!(
        &rsdp[..8],
        b"RSD PTR ",
        "the RSDP is where the search looks"
    );
    assert_eq!(
        acpi::checksum(&rsdp[..20]),
        0,
        "the ACPI 1.0 checksum covers bytes 0-19"
    );
    assert_eq!(
        acpi::checksum(&rsdp),
        0,
        "and the extended one the whole 36"
    );
    let xsdt_at = u64::from_le_bytes(rsdp[24..32].try_into().expect("eight bytes"));
    let header = read_bytes(space, xsdt_at, 36);
    assert_eq!(&header[..4], b"XSDT");
    let len = u32::from_le_bytes(header[4..8].try_into().expect("four bytes")) as usize;
    let entries = read_bytes(space, xsdt_at, len);
    assert_eq!(acpi::checksum(&entries), 0, "the XSDT's checksum");
    for chunk in entries[36..].as_chunks::<8>().0 {
        let at = u64::from_le_bytes(*chunk);
        if &read_bytes(space, at, 4)[..] == signature {
            return Some(at);
        }
    }
    None
}

/// Read a whole table and check its checksum.
fn table(space: &rsemu::core::space::AddressSpace, signature: &[u8; 4]) -> Vec<u8> {
    let at = find_table(space, signature)
        .unwrap_or_else(|| panic!("no `{}` in the XSDT", String::from_utf8_lossy(signature)));
    let header = read_bytes(space, at, 36);
    let len = u32::from_le_bytes(header[4..8].try_into().expect("four bytes")) as usize;
    let bytes = read_bytes(space, at, len);
    assert_eq!(
        acpi::checksum(&bytes),
        0,
        "`{}` does not checksum",
        String::from_utf8_lossy(signature)
    );
    bytes
}

#[test]
fn an_operating_systems_own_search_finds_a_complete_table_set() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    // Every table the board should publish, found by walking the XSDT from the
    // RSDP and checksummed on the way. `table` panics if any step fails.
    let fadt = table(&mem, b"FACP");
    assert_eq!(fadt.len(), acpi::FADT_LEN);
    assert_eq!(fadt[8], acpi::FADT_REVISION);
    let _madt = table(&mem, b"APIC");
    let _mcfg = table(&mem, b"MCFG");
    let _hpet = table(&mem, b"HPET");
    // The FADT points at the DSDT and the FACS directly rather than through the
    // XSDT (§5.2.8), so they are checked from there.
    let dsdt_at = u64::from_le_bytes(fadt[140..148].try_into().expect("eight bytes"));
    assert_eq!(&read_bytes(&mem, dsdt_at, 4)[..], b"DSDT");
    let facs_at = u64::from_le_bytes(fadt[132..140].try_into().expect("eight bytes"));
    assert_eq!(&read_bytes(&mem, facs_at, 4)[..], b"FACS");
    assert_eq!(facs_at % 64, 0, "the FACS is 64-byte aligned (§5.2.10)");
}

#[test]
fn the_tables_describe_the_machine_that_was_built() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);

    // The MADT's local APIC address is where the machine file mapped it, and
    // its I/O APIC entry is where that one is.
    let madt = table(&mem, b"APIC");
    assert_eq!(
        u32::from_le_bytes(madt[36..40].try_into().expect("four bytes")),
        0xfee0_0000,
        "the local APIC's page, read out of the address space"
    );
    assert_eq!(
        u32::from_le_bytes(madt[40..44].try_into().expect("four bytes")) & 1,
        1,
        "PCAT_COMPAT: this board has the 8259A pair"
    );
    // Walk the interrupt controller structures.
    let mut at = 44;
    let mut ioapic_address = None;
    let mut overrides = Vec::new();
    let mut processors = 0;
    while at + 1 < madt.len() {
        let (kind, len) = (madt[at], madt[at + 1] as usize);
        assert!(len >= 2, "a zero-length MADT entry would loop for ever");
        match kind {
            0 => processors += 1,
            1 => {
                ioapic_address = Some(u32::from_le_bytes(
                    madt[at + 4..at + 8].try_into().expect("four bytes"),
                ));
            }
            2 => overrides.push((
                madt[at + 3],
                u32::from_le_bytes(madt[at + 4..at + 8].try_into().expect("four bytes")),
            )),
            _ => {}
        }
        at += len;
    }
    assert_eq!(processors, 1);
    assert_eq!(ioapic_address, Some(0xfec0_0000));
    // The board wires `pit0.out0 -> ioapic.irq2`, so IRQ0 is global system
    // interrupt 2 and the MADT has to say so or an operating system loses its
    // timer the moment it stops using the 8259A.
    assert!(
        overrides.contains(&(0, 2)),
        "no IRQ0 -> GSI2 override: {overrides:?}"
    );

    // The FADT's block pointers are where PMBASE actually put the window.
    let fadt = table(&mem, b"FACP");
    let pm1a_evt = u32::from_le_bytes(fadt[56..60].try_into().expect("four bytes"));
    let pm_tmr = u32::from_le_bytes(fadt[76..80].try_into().expect("four bytes"));
    assert_eq!(pm1a_evt, 0x600, "PM1a_EVT_BLK is PMBASE + 0");
    assert_eq!(pm_tmr, 0x608, "PMTMR_BLK is PMBASE + 8 (ICH9 Table 13-11)");
    // And the 64-bit forms agree with the 32-bit ones, as §5.2.9 requires.
    let x_pm_tmr = u64::from_le_bytes(fadt[212..220].try_into().expect("eight bytes"));
    assert_eq!(x_pm_tmr, u64::from(pm_tmr));

    // The MCFG names the window PCIEXBAR placed.
    let mcfg = table(&mem, b"MCFG");
    let base = u64::from_le_bytes(mcfg[44..52].try_into().expect("eight bytes"));
    assert_eq!(base, 0xe000_0000);
    assert_eq!(mcfg[54], 0, "start bus");
    assert_eq!(mcfg[55], 255, "a 256 MiB window covers buses 0-255");

    // The HPET table's address is where the machine file mapped the part, and
    // its block ID is that part's own capabilities register.
    let hpet = table(&mem, b"HPET");
    let address = u64::from_le_bytes(hpet[44..52].try_into().expect("eight bytes"));
    assert_eq!(address, 0xfed0_0000);
    let block_id = u32::from_le_bytes(hpet[36..40].try_into().expect("four bytes"));
    assert_eq!(
        block_id,
        read32(&mem, 0xfed0_0000),
        "the block ID is the part's General_Cap&ID register, read back"
    );
}

#[test]
fn moving_the_ecam_window_moves_what_the_mcfg_says() {
    // The strongest claim in this file: nothing tells the generator where the
    // window is, so if the table follows a guest's configuration write it is
    // because it read the machine.
    let (machine, _cpu, tables) = board();
    let mem = mem(&machine);
    let port = port(&machine);
    assert_eq!(
        u64::from_le_bytes(table(&mem, b"MCFG")[44..52].try_into().expect("eight")),
        0xe000_0000
    );

    // Move it to 0xd0000000 and shrink it to 64 MB — LENGTH = 10b, which also
    // changes the bus range the table reports.
    port.write(0xcf8, Width::U32, 0x8000_0060, MemAttrs::DEFAULT)
        .expect("CONFADD");
    port.write(0xcfc, Width::U32, 0xd000_0005, MemAttrs::DEFAULT)
        .expect("CONFDATA");
    // Regenerate the way a reset does. A warm reset would put PCIEXBAR back, so
    // the device's own entry point is used rather than a reset.
    tables.regenerate().expect("the machine still has a lapic");

    let mcfg = table(&mem, b"MCFG");
    assert_eq!(
        u64::from_le_bytes(mcfg[44..52].try_into().expect("eight")),
        0xd000_0000,
        "the MCFG followed the window"
    );
    assert_eq!(mcfg[55], 63, "a 64 MiB window covers buses 0-63");
}

#[test]
fn the_dsdt_names_a_pci_express_host_bridge_and_an_s5_package() {
    let (machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    let fadt = table(&mem, b"FACP");
    let dsdt_at = u64::from_le_bytes(fadt[140..148].try_into().expect("eight bytes"));
    let header = read_bytes(&mem, dsdt_at, 36);
    let len = u32::from_le_bytes(header[4..8].try_into().expect("four bytes")) as usize;
    let dsdt = read_bytes(&mem, dsdt_at, len);
    assert_eq!(acpi::checksum(&dsdt), 0, "the DSDT's checksum");
    // `_S5_` and `PCI0` both appear as name segments in the AML, and the host
    // bridge's `_HID` is `PNP0A08` — 0x080ad041 as a dword.
    let body = &dsdt[36..];
    assert!(
        body.windows(4).any(|w| w == b"_S5_"),
        "no _S5 package: an ACPI shutdown has nothing to write"
    );
    assert!(body.windows(4).any(|w| w == b"PCI0"));
    assert!(
        body.windows(4).any(|w| w == 0x080a_d041u32.to_le_bytes()),
        "no PNP0A08 host bridge in the namespace"
    );
}

#[test]
fn a_reset_regenerates_the_tables_from_the_machine_as_it_then_is() {
    let (mut machine, _cpu, _tables) = board();
    let mem = mem(&machine);
    let before = table(&mem, b"MCFG");
    // A cold reset puts every chipset register back, so the tables have to come
    // back identical — they are a pure function of the topology.
    machine.reset(ResetKind::Cold);
    let after = table(&mem, b"MCFG");
    assert_eq!(before, after);
}

#[test]
fn the_board_snapshots_and_restores_to_an_identical_state_hash() {
    let (mut machine, _cpu, _tables) = board();
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(2_000_000))
        .expect("the board runs");
    let snapshot = machine.save().expect("the board saves");
    let hash = machine.state_hash().expect("the board hashes");

    let (mut restored, _cpu, _tables) = board();
    restored.load(&snapshot).expect("the board loads");
    assert_eq!(
        restored.state_hash().expect("the board hashes"),
        hash,
        "a round trip changed the machine"
    );
}

// ---------------------------------------------------------------------------
// the furthest observable
// ---------------------------------------------------------------------------

/// **rsemu's own BIOS runs a complete POST on this board.**
///
/// The strongest claim this work can make today, and it is worth stating
/// precisely because it is easy to overclaim. `src/fw/pcbios` is a *legacy*
/// BIOS: it knows nothing about a q35, never enumerates the bus, never touches
/// `PCIEXBAR` or `PMBASE`, and never looks for an ACPI table. What this test
/// shows is therefore not that the chipset was *used* — it is that the chipset
/// is **transparent to software that predates it**, which is exactly the
/// property a board has to have before anything newer is worth trying on it.
///
/// Every legacy chip is at the address it is on `pc-at`, so the same POST
/// sequence has to come out the same: read the CMOS, size memory, program the
/// 8254 and the 8259As, find the drive, publish the BIOS Data Area. The
/// assertions below are `tests/pc_at_boot.rs`'s POST assertions, and they hold
/// on a machine whose north and south bridges are different silicon.
///
/// Two things it deliberately does **not** claim:
///
/// * It does not boot a guest. The IDE bays are empty here; `tests/pc_at_boot`
///   is where a boot sector runs, and pointing it at this board is the next
///   step rather than this one.
/// * It says nothing about a modern kernel. That needs long mode, and the x86
///   core does not have it.
#[cfg(feature = "fw-pcbios")]
#[test]
fn rsemus_own_bios_posts_on_this_board() {
    let (mut machine, cpu, _tables) = board_with_bios(rsemu::fw::pcbios::image().to_vec());
    // Long enough for a complete POST. `tests/pc_at_boot` measures the same
    // firmware needing well under this on `pc-at`.
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(300_000_000))
        .expect("the board runs");
    let mem = mem(&machine);
    let peek16 = |at: u64| {
        mem.read(at, Width::U16, MemAttrs::DEFAULT)
            .expect("low memory is RAM") as u16
    };
    let peek32 = |at: u64| {
        mem.read(at, Width::U32, MemAttrs::DEFAULT)
            .expect("low memory is RAM") as u32
    };
    let regs = cpu.regs();
    println!(
        "q35: stopped at {:04x}:{:08x}, halted={}, {} cycles",
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.cycles()
    );
    let (faults, last) = cpu.bus_faults();
    println!("q35: {faults} unanswered bus access(es), last at {last:08x}");

    // The BIOS Data Area, filled from the CMOS and from the hardware. 639 KiB
    // rather than 640: the last kilobyte is the EBDA.
    assert_eq!(
        peek16(0x413),
        639,
        "the BDA's memory size is not what POST read out of the CMOS"
    );
    assert_eq!(peek16(0x40e), 0x9fc0, "the EBDA segment is not published");
    assert_ne!(peek16(0x410), 0, "the equipment word is still zero");
    // And the machine is *running*: the 8254 is programmed, the master 8259A
    // is unmasked, IRQ0 reaches the processor, and the handler counts. Nothing
    // else on this board proves the interrupt path end to end.
    assert!(
        peek32(0x46c) > 0,
        "the tick count at 0040:006c never moved: the 8254 or the 8259A is not \
         programmed, or IRQ0 is masked"
    );

    // The tables are still where the generator put them: a legacy BIOS that
    // does not know about ACPI also does not stand on it.
    assert_eq!(&read_bytes(&mem, 0xe_0000, 8)[..], b"RSD PTR ");
    let _ = table(&mem, b"FACP");
}

/// Where the firmware loads a boot sector, and where this one leaves its marks.
#[cfg(feature = "fw-pcbios")]
const BOOT_ADDRESS: u16 = 0x7c00;
/// A scratch word in low memory the guest writes and this test reads.
#[cfg(feature = "fw-pcbios")]
const SCRATCH: u16 = 0x0500;
/// What the guest writes there once it has run to the end.
#[cfg(feature = "fw-pcbios")]
const DONE_MARKER: u16 = 0xb007;

/// A boot sector that calls back into the firmware and leaves what it learned
/// in low memory.
///
/// Deliberately small — `tests/pc_at_boot.rs` has the exhaustive one, and
/// duplicating it here would be testing the firmware twice rather than testing
/// this board. What it does is the minimum that proves the whole path: it uses
/// `INT 12h` and `INT 11h`, which read the BDA that POST filled, and `INT 13h
/// AH=08h`, which reports the geometry the firmware got out of the drive's own
/// `IDENTIFY DEVICE` over the IDE cable.
#[cfg(feature = "fw-pcbios")]
fn boot_sector() -> Vec<u8> {
    use rsemu::fw::asm16::{AH, AX, Asm, CX, DL, DS, DX, ES, Mem, SP, SS};
    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);

    // The firmware leaves DL holding the drive the sector came off, and nothing
    // else is promised. Segments and a stack first, because nothing else is.
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    a.movto8(Mem::abs(SCRATCH), DL);

    // INT 12h: base memory in kilobytes, straight out of the BDA.
    a.int(0x12);
    a.movto(Mem::abs(SCRATCH + 2), AX);

    // INT 11h: the equipment word.
    a.int(0x11);
    a.movto(Mem::abs(SCRATCH + 4), AX);

    // INT 13h AH=08h: the drive's geometry, which the firmware read out of
    // `IDENTIFY DEVICE` over the IDE cable during POST.
    a.movi8(AH, 0x08);
    a.mov8(DL, Mem::abs(SCRATCH));
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 6), CX);
    a.movto(Mem::abs(SCRATCH + 8), DX);

    // Reached the end.
    a.movmi(Mem::abs(SCRATCH + 10), DONE_MARKER);
    let spin = a.here_label();
    a.jmp(spin);

    let mut image = a.finish();
    // The signature the firmware checks before it jumps here.
    image[usize::from(BOOT_ADDRESS) + 510] = 0x55;
    image[usize::from(BOOT_ADDRESS) + 511] = 0xaa;
    image[usize::from(BOOT_ADDRESS)..].to_vec()
}

/// **A guest boots off the IDE drive on this board.**
///
/// The furthest observable this work reaches, and the ladder it is on is
/// `ROADMAP.md` phase 6b's: *the board builds and a firmware enumerates it* →
/// *a kernel's early console prints* → *a kernel finds its root device*. This
/// is the first rung, on a chipset a 6b guest would recognise.
///
/// Each step is checked rather than assumed:
///
/// 1. The processor fetches `0xfffffff0` out of the ROM alias and runs POST.
/// 2. POST finds the drive with `IDENTIFY DEVICE` over the IDE cable.
/// 3. `INT 19h` reads cylinder 0, head 0, sector 1 into `0000:7c00`, sees the
///    `0x55 0xaa` signature and jumps there.
/// 4. **The boot sector runs**, and calls back into the firmware through
///    `INT 11h`, `INT 12h` and `INT 13h`.
///
/// What it does **not** show: anything about a modern operating system. This is
/// a legacy BIOS booting a real-mode guest on a q35-shaped board, which is a
/// statement about the board rather than about the chipset being used — the
/// firmware never touches `PCIEXBAR`, `PMBASE` or an ACPI table.
#[cfg(feature = "fw-pcbios")]
#[test]
fn a_guest_boots_off_the_ide_drive_on_this_board() {
    // Four cylinders of sixteen heads of 63 sectors is what `ata.disk`'s
    // default translation covers.
    const SECTORS: usize = 4 * 16 * 63;
    let mut disk = vec![0u8; SECTORS * 512];
    disk[..512].copy_from_slice(&boot_sector());

    let (mut machine, cpu, _tables) =
        board_with_bios_and_disk(rsemu::fw::pcbios::image().to_vec(), disk);
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(300_000_000))
        .expect("the board runs");
    let mem = mem(&machine);
    let peek16 = |at: u64| {
        mem.read(at, Width::U16, MemAttrs::DEFAULT)
            .expect("low memory is RAM") as u16
    };
    let regs = cpu.regs();
    println!(
        "q35 boot: stopped at {:04x}:{:08x}, halted={}, {} cycles",
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.cycles()
    );

    assert_eq!(
        peek16(u64::from(SCRATCH) + 10),
        DONE_MARKER,
        "the boot sector never ran to the end: the firmware did not load it, \
         did not jump to it, or one of its callbacks did not return"
    );
    assert_eq!(
        peek16(u64::from(SCRATCH)) & 0xff,
        0x80,
        "the firmware booted from something other than the first fixed disk"
    );
    assert_eq!(
        peek16(u64::from(SCRATCH) + 2),
        639,
        "INT 12h disagreed with the BDA POST filled"
    );
    assert_ne!(
        peek16(u64::from(SCRATCH) + 4),
        0,
        "INT 11h reported nothing"
    );
    // The geometry `INT 13h AH=08h` reports is the drive's own, read over the
    // cable during POST: sectors in CL[5:0] and heads in DH.
    let (cx, dx) = (
        peek16(u64::from(SCRATCH) + 6),
        peek16(u64::from(SCRATCH) + 8),
    );
    assert_eq!(cx & 0x3f, 63, "sectors per track");
    assert_eq!(
        (dx >> 8) & 0xff,
        15,
        "the last head number, so sixteen heads"
    );
    assert_eq!(dx & 0xff, 1, "the drive count");
}

/// What the log port answers a read with.
///
/// A firmware built for an emulated machine **probes** this port before it
/// trusts it, and expects its own opcode-shaped signature back — the convention
/// is Bochs's debug console at `0xe9`, and the port at `0x402` answers the same
/// way. `tests/pc_at_firmware.rs` has the long version of why that matters.
const DEBUG_PORT_SIGNATURE: u8 = 0xe9;

/// A write-only port that keeps every byte written to it.
///
/// Mapped at `0x402` by [`listen`], which is **not** part of the board: the
/// firmwares built for emulated machines write their progress log there one
/// character at a time, and reading what a program prints is the most ordinary
/// black-box observation there is (`ROADMAP.md` §1).
#[derive(Debug, Default)]
struct DebugPort {
    text: std::sync::Mutex<Vec<u8>>,
}

impl rsemu::core::space::MemOps for DebugPort {
    fn read(&self, _: u64, dst: &mut [u8], _: MemAttrs) -> rsemu::core::space::MemResult {
        dst.fill(DEBUG_PORT_SIGNATURE);
        Ok(())
    }

    fn write(&self, _: u64, src: &[u8], attrs: MemAttrs) -> rsemu::core::space::MemResult {
        // A debugger write is not the guest's, and must not appear in the log.
        if !attrs.debug {
            self.text
                .lock()
                .expect("not poisoned")
                .extend_from_slice(src);
        }
        Ok(())
    }

    fn constraints(&self) -> rsemu::core::space::AccessConstraints {
        rsemu::core::space::AccessConstraints::ANY
    }
}

/// Map the log port into a realized machine's I/O space.
fn listen(m: &Machine) -> Arc<DebugPort> {
    let port = Arc::new(DebugPort::default());
    m.space("port")
        .expect("the I/O space")
        .topology()
        .map(
            rsemu::core::space::Region::io("debug-log", 1, Arc::clone(&port) as Arc<_>),
            0x402,
        )
        .expect("0x402 is a hole on this board");
    port
}

/// A **real PC firmware** running on this board, if the environment names one.
///
/// Gated on `RSEMU_BIOS` exactly as `tests/pc_at_firmware.rs` is, so
/// `cargo test` stays hermetic and needs nothing installed:
///
/// ```text
/// RSEMU_BIOS=/usr/share/qemu/bios.bin \
///   cargo test --release --all-features --test q35_board -- --nocapture third_party
/// ```
///
/// **Nothing is vendored**: the image is read from wherever the variable points
/// and never enters this repository. Running a program — including a copyleft
/// one — as an emulated guest is ordinary use and creates no derivative work
/// (`ROADMAP.md` §1); reading its source would be a different matter, and was
/// not done. Everything printed below is a byte the guest wrote.
///
/// The parameter overrides are worth understanding. A 128 KiB firmware wants
/// the whole `0xe0000`-`0xfffff` band, which is where this board stages its
/// generated ACPI tables — so the board is told to give it up and put them at
/// `0xd0000`. That is not a workaround: a firmware that fills the band is a
/// firmware that publishes its own tables, and this board's generated set
/// exists precisely because `src/fw/pcbios` does not yet.
///
/// # What this measured, at the time it was written
///
/// Against `/usr/share/qemu/bios.bin` — a current prebuilt PC firmware, run as
/// a guest and never read — the board reaches its **boot prompt**. The log it
/// printed, in its own words:
///
/// ```text
/// RamSize: 0x040c0000 [cmos]
/// Relocating init from 0x000e9620 to 0x030b5300 (size 44128)
/// === PCI bus & bridge init ===
/// === PCI device probing ===
/// Found 3 PCI devices (max PCI bus is 00)
/// PCIe: using q35 mmconfig at 0xb0000000
/// PCI: init bdf=00:00.0 id=8086:29c0
/// PCI: init bdf=00:02.0 id=1234:1111
/// PCI: init bdf=00:1f.0 id=8086:2918
/// Q35 LPC init: elcr=00 0c
/// PCI: Using 00:02.0 for primary VGA
/// Turning on vga text mode console
/// PS2 keyboard initialized
/// Press ESC for boot menu.
/// ```
///
/// Four claims in there are this work's, and each is asserted below rather than
/// admired:
///
/// * **`PCIe: using q35 mmconfig at 0xb0000000`.** The firmware recognised the
///   chipset, wrote `PCIEXBAR` *itself* — to an address neither this board nor
///   this test chose — and enumerated through the window that write created.
///   Everything after that line came through ECAM.
/// * **`Relocating init`** and a non-zero `PAM0`. It shadowed itself into the
///   DRAM the PAM registers at `0x90`-`0x96` switch into view, which is the
///   register file a 440FX keeps at `0x59`-`0x5f`.
/// * **`Q35 LPC init`.** It found the south bridge at `00:1f.0` and ran the
///   initialisation it keeps for that part.
/// * A complete POST: the BDA filled, a text console, a keyboard, and a timer
///   tick that is still counting.
///
/// It does **not** boot an operating system, and nothing here claims it would:
/// there is no bootable medium in this test, and a modern kernel would want
/// long mode from a core that has not got it.
#[test]
fn a_third_party_firmware_runs_on_this_board() {
    let Ok(path) = std::env::var("RSEMU_BIOS") else {
        println!("q35: RSEMU_BIOS is not set, so there is no firmware to run");
        return;
    };
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let size = format!("{}", image.len());
    let (mut machine, cpu, _tables) = board_configured(
        image,
        Vec::new(),
        &[
            ("biosbase", "0xe0000"),
            ("biossize", &size),
            ("acpibase", "0xd0000"),
            ("acpisize", "0x8000"),
        ],
    );
    let log = listen(&machine);
    machine.reset(ResetKind::Cold);
    machine
        .run_for(rsemu::core::clock::GlobalTime::from_nanos(1_500_000_000))
        .expect("the board runs");
    let mem = mem(&machine);
    let regs = cpu.regs();
    println!(
        "q35 third-party: stopped at {:04x}:{:08x}, halted={}, {} cycles",
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.cycles()
    );
    let (faults, last) = cpu.bus_faults();
    println!("q35 third-party: {faults} unanswered bus access(es), last at {last:08x}");

    // What the firmware printed. By far the most useful instrument here, and it
    // is nothing more than reading what a program wrote to a port.
    let text = log.text.lock().expect("not poisoned").clone();
    println!("q35 third-party: {} byte(s) of log:", text.len());
    for line in String::from_utf8_lossy(&text).lines() {
        println!("  |{line}|");
    }

    // What it left in the BIOS Data Area.
    let peek16 = |at: u64| {
        mem.read(at, Width::U16, MemAttrs::DEFAULT)
            .expect("low memory is RAM") as u16
    };
    let peek32 = |at: u64| {
        mem.read(at, Width::U32, MemAttrs::DEFAULT)
            .expect("low memory is RAM") as u32
    };
    println!(
        "q35 third-party: equipment {:#06x}, base memory {} KiB, EBDA {:#06x}, ticks {}",
        peek16(0x410),
        peek16(0x413),
        peek16(0x40e),
        peek32(0x46c),
    );

    // And what the chipset says about itself afterwards, which is the one thing
    // asserted: whatever the firmware did or did not manage, the bridges are
    // still answering. A run that ends with the host bridge master-aborting
    // would mean the chipset fell over rather than the firmware giving up, and
    // those are very different bugs.
    let port = port(&machine);
    port.write(0xcf8, Width::U32, 0x8000_0000, MemAttrs::DEFAULT)
        .expect("CONFADD");
    let id = read32(&port, 0xcfc);
    port.write(0xcf8, Width::U32, 0x8000_0090, MemAttrs::DEFAULT)
        .expect("CONFADD");
    let pam = read32(&port, 0xcfc);
    println!("q35 third-party: host bridge {id:#010x}, PAM0-3 {pam:#010x}");
    assert_eq!(
        id, 0x29c0_8086,
        "the host bridge stopped answering configuration cycles, so the chipset \
         fell over rather than the firmware giving up"
    );
    assert_eq!(
        faults, 0,
        "the firmware made an access nothing on this board answered"
    );

    // A complete POST. These are `tests/pc_at_boot.rs`'s POST assertions, and
    // they hold for a *third-party* firmware on a board whose north and south
    // bridges are both parts rsemu had never modelled before.
    assert_eq!(peek16(0x413), 639, "the BDA's base memory size");
    assert_eq!(peek16(0x40e), 0x9fc0, "the EBDA segment");
    assert_ne!(peek16(0x410), 0, "the equipment word");
    assert!(peek32(0x46c) > 0, "the timer tick is not counting");

    // It shadowed itself through the PAM registers at 0x90-0x96 — which is the
    // whole reason this board needs its own north bridge, and is a claim about
    // *this* work rather than about the firmware.
    assert_ne!(
        pam, 0,
        "PAM0-PAM3 are still at their reset values, so the firmware never \
         shadowed itself: it did not find the register file where a q35 keeps it"
    );

    // And it reprogrammed PCIEXBAR itself. The board comes up with the window
    // at 0xe0000000; anything else is the firmware's own choice, made through
    // a register this work added, and is the strongest evidence that ECAM is
    // being *used* rather than merely being present.
    port.write(0xcf8, Width::U32, 0x8000_0060, MemAttrs::DEFAULT)
        .expect("CONFADD");
    let pciexbar = read32(&port, 0xcfc);
    println!("q35 third-party: PCIEXBAR {pciexbar:#010x}");
    assert_ne!(pciexbar & 1, 0, "the firmware disabled the ECAM window");
    assert_ne!(
        pciexbar & 0xf000_0000,
        0xe000_0000,
        "PCIEXBAR is still where the board put it, so the firmware never moved \
         the window and this measures nothing"
    );

    // The log is the instrument, so a run that printed nothing is a run that
    // told us nothing, whatever else passed.
    assert!(
        !text.is_empty(),
        "the firmware wrote nothing to its log port: either it is not one of \
         the images this instrument suits, or it stopped before it opened the log"
    );
}
