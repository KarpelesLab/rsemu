//! **An application processor reads its *own* local APIC through the
//! architectural address** — the last thing between rsemu and a real
//! multiprocessor PC, and the reason `machines/pc-at-smp.machine` ships.
//!
//! # The defect this file is the proof against
//!
//! rsemu models each local APIC as a separate device, so a two-processor board
//! used to map each one's register page somewhere of its own —
//! `machines/pc-apic.machine` puts the second at `0xfef00000`. But both tables
//! that describe a multiprocessor PC carry **one** local-APIC address (*MP*
//! §4.2's `ADDRESS OF LOCAL APIC`, *ACPI* §5.2.12's Local Interrupt Controller
//! Address), because on silicon the register block is on the processor's own
//! die and its aperture never reaches the system bus. An operating system
//! therefore uses `0xfee00000` on *every* processor — and an application
//! processor that read its own APIC ID there read the bootstrap processor's.
//! Enumerating and starting the second processor worked; every per-processor
//! timer, self-IPI and task-priority write on it went to the wrong registers.
//!
//! `machines/pc-at-smp.machine` maps `lapic0.window` rather than `lapic0.regs`.
//! The window is an ordinary [`MemOps`] that demultiplexes on
//! [`MemAttrs::requester`], which the machine layer allocates per object,
//! `cpu.x86` stamps on every access, and KVM rebuilds on both exit paths — so
//! `core::space` needed no change and one mapping serves both processors.
//!
//! # What is asserted, and the control that makes it mean something
//!
//! The guest is a boot sector. It walks the firmware's own MP configuration
//! table for a processor entry whose `BP` flag is clear, sends that APIC ID the
//! *MultiProcessor Specification* §B.4 INIT/Start-Up pair, and the processor
//! that starts enters protected mode and reads **its own APIC ID register at
//! `0xfee00020`**. Both processors leave what they read in low memory.
//!
//! * on the shipped board the bootstrap processor reads 0 and the application
//!   processor reads **1**;
//! * on the same text with `lapic0.window` changed back to `lapic0.regs` — the
//!   model as it was — the application processor reads **0**, the bootstrap
//!   processor's. That is the negative control: it is the same guest, the same
//!   firmware and the same two processors, and the only difference is the one
//!   mapping.
//!
//! Both claims are made **twice**, because the requester id they turn on
//! travels two different code paths: the interpreter stamps it in
//! `cpu::x86::exec` and `accel::kvm` rebuilds it on both of its exit paths. The
//! interpreted tests run everywhere; the accelerated one is in the
//! `accelerated` module at the bottom and skips cleanly with no `/dev/kvm`.
//!
//! [`MemOps`]: rsemu::core::space::MemOps
//! [`MemAttrs::requester`]: rsemu::core::space::MemAttrs

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at-smp"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, build};

// ---------------------------------------------------------------------------
// the two boards
// ---------------------------------------------------------------------------

/// The shipped two-processor AT.
fn shipped() -> &'static str {
    rsemu::machine::catalog::PC_AT_SMP.source
}

/// The same board with the architectural page decoded the *old* way: straight
/// to the bootstrap processor's own register block, which is what every board
/// in the tree did before the window existed.
///
/// The second APIC keeps a page of its own at `0xfef00000`, exactly as
/// `machines/pc-apic.machine` gives it one, so the board is a working
/// two-processor machine in every respect but the one under test.
fn without_the_window() -> String {
    const WINDOW: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.window";
    let text = shipped();
    assert!(
        text.contains(WINDOW),
        "the shipped board no longer maps `lapic0.window`; this control has nothing to remove"
    );
    text.replace(
        WINDOW,
        "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
         map mem 0xfef00000 size 0x1000   = lapic1.regs",
    )
}

// ---------------------------------------------------------------------------
// where the guest leaves what it found
// ---------------------------------------------------------------------------

/// Where the boot sector lands, and what its labels are relative to.
const BOOT: u16 = 0x7c00;
/// Offsets inside the 512-byte sector, fixed so a far jump can name one.
const OFF_GDT: u16 = 0x0060;
const OFF_GDT_PTR: u16 = 0x0080;
const OFF_PM: u16 = 0x0090;
const OFF_TRAMPOLINE: u16 = 0x0180;

/// Where the application processor's trampoline is copied to, and the page a
/// Start-Up names to reach it.
const AP_TRAMPOLINE: u32 = 0x8000;
const AP_PAGE: u8 = 0x08;

/// The block at `0x0500` every PC has left free since 1981.
const BSP_MARKER: u32 = 0x0500;
const BSP_ALIVE: u32 = 0x0000_b005;
const AP_MARKER: u32 = 0x0504;
const AP_ALIVE: u32 = 0x0000_a55a;
/// The application processor's APIC ID **as the firmware's MP configuration
/// table named it**, which is what the Start-Up is addressed to.
const AP_ID_FROM_TABLE: u32 = 0x0508;
/// The APIC ID the application processor read out of *its own* ID register
/// through `0xfee00020`. The whole point of this file.
const AP_ID_READ: u32 = 0x050c;
/// The same, read by the bootstrap processor.
const BSP_ID_READ: u32 = 0x0510;
/// What an unwritten marker holds, so "never ran" is distinguishable from
/// "read zero".
const UNSET: u32 = 0xffff_ffff;

/// The architectural local-APIC page (*Intel SDM* Vol 3A §10.4.4).
const LAPIC: u32 = 0xfee0_0000;
/// The local APIC ID register, and the spurious-interrupt vector register
/// (§10.4.6 and §10.9).
const REG_ID: u32 = 0x0020;
const REG_SVR: u32 = 0x00f0;
/// The interrupt command register's two halves (§10.6.1).
const REG_ICR_LOW: u32 = 0x0300;
const REG_ICR_HIGH: u32 = 0x0310;

// ---------------------------------------------------------------------------
// hand assembly
// ---------------------------------------------------------------------------

/// Append a little-endian 32-bit word.
fn dw(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

/// `mov dword [edi+disp32], imm32`, for a register-relative device page.
fn store_at(out: &mut Vec<u8>, disp: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x87]);
    dw(out, disp);
    dw(out, value);
}

/// `mov dword [disp32], imm32` — an absolute store, for a flat data segment.
fn store_abs(out: &mut Vec<u8>, at: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x05]);
    dw(out, at);
    dw(out, value);
}

/// A hand-assembled fragment whose short jumps are patched afterwards.
///
/// The table walk below has six jumps, and every displacement would change
/// whenever an instruction above it did. Labels are numbered rather than named
/// because there are seven of them and this is a test.
#[derive(Default)]
struct Frag {
    out: Vec<u8>,
    marks: [Option<usize>; 8],
    fixups: Vec<(usize, usize)>,
}

impl Frag {
    fn emit(&mut self, bytes: &[u8]) -> &mut Frag {
        self.out.extend_from_slice(bytes);
        self
    }

    fn mark(&mut self, id: usize) -> &mut Frag {
        assert!(self.marks[id].is_none(), "label {id} was bound twice");
        self.marks[id] = Some(self.out.len());
        self
    }

    fn jump(&mut self, opcode: &[u8], id: usize) -> &mut Frag {
        self.out.extend_from_slice(opcode);
        self.fixups.push((self.out.len(), id));
        self.out.push(0);
        self
    }

    fn finish(mut self) -> Vec<u8> {
        for (at, id) in &self.fixups {
            let target = self.marks[*id].expect("a jump to an unbound label");
            // A `rel8` is measured from the end of the instruction, which is
            // the byte after the displacement (*Intel SDM* Vol 2A, `Jcc`).
            let rel = target as isize - (*at as isize + 1);
            self.out[*at] = i8::try_from(rel).expect("a short jump") as u8;
        }
        self.out
    }
}

/// Find the application processor's local APIC ID in the MP configuration
/// table, leave it at [`AP_ID_FROM_TABLE`], and answer with it shifted into the
/// interrupt command register's destination field in `EBX`.
///
/// *MultiProcessor Specification* §4.1: the floating pointer "must span a
/// minimum of 16 contiguous bytes, beginning on a 16-byte boundary", and may be
/// "in the BIOS ROM address space between 0F0000h and 0FFFFFh". §4.3: entries
/// are stepped through until `ENTRY COUNT` is reached, twenty bytes for a
/// processor entry and eight for every other type. §4.3.1: `CPU FLAGS` bit 1 is
/// `BP`, set on the bootstrap processor.
fn find_application_processor() -> Vec<u8> {
    const SCAN: usize = 0;
    const FOUND: usize = 1;
    const WALK: usize = 2;
    const OTHER: usize = 3;
    const STEP: usize = 4;
    const GOT: usize = 5;
    const HAVE: usize = 6;

    let mut f = Frag::default();
    f.emit(&[0xbe, 0x00, 0x00, 0x0f, 0x00]); // mov esi, 0xf0000
    f.mark(SCAN);
    f.emit(&[0x81, 0x3e, 0x5f, 0x4d, 0x50, 0x5f]); // cmp dword [esi], "_MP_"
    f.jump(&[0x74], FOUND); // je found
    f.emit(&[0x83, 0xc6, 0x10]); // add esi, 16
    f.emit(&[0x81, 0xfe, 0x00, 0x00, 0x10, 0x00]); // cmp esi, 0x100000
    f.jump(&[0x72], SCAN); // jb scan
    f.emit(&[0xb8, 0xff, 0x00, 0x00, 0x00]); // mov eax, 0xff
    f.jump(&[0xeb], HAVE);

    f.mark(FOUND);
    f.emit(&[0x8b, 0x76, 0x04]); // mov esi, [esi+4]
    f.emit(&[0x0f, 0xb7, 0x4e, 0x22]); // movzx ecx, word [esi+34]
    f.emit(&[0x83, 0xc6, 0x2c]); // add esi, 44
    f.mark(WALK);
    f.emit(&[0x80, 0x3e, 0x00]); // cmp byte [esi], 0
    f.jump(&[0x75], OTHER); // jne other
    f.emit(&[0xf6, 0x46, 0x03, 0x02]); // test byte [esi+3], 2
    f.jump(&[0x74], GOT); // jz got -- not the bootstrap processor
    f.emit(&[0x83, 0xc6, 0x14]); // add esi, 20
    f.jump(&[0xeb], STEP);
    f.mark(OTHER);
    f.emit(&[0x83, 0xc6, 0x08]); // add esi, 8
    f.mark(STEP);
    f.emit(&[0x49]); // dec ecx
    f.jump(&[0x75], WALK); // jnz walk
    f.emit(&[0xb8, 0xff, 0x00, 0x00, 0x00]); // mov eax, 0xff
    f.jump(&[0xeb], HAVE);

    f.mark(GOT);
    f.emit(&[0x0f, 0xb6, 0x46, 0x01]); // movzx eax, byte [esi+1]
    f.mark(HAVE);
    f.emit(&[0xa3]); // mov [AP_ID_FROM_TABLE], eax
    let mut out = f.finish();
    dw(&mut out, AP_ID_FROM_TABLE);
    out.extend_from_slice(&[0xc1, 0xe0, 0x18]); // shl eax, 24
    out.extend_from_slice(&[0x89, 0xc3]); // mov ebx, eax
    out
}

/// The application processor's trampoline: enter protected mode off the
/// bootstrap processor's own descriptor table, read **its own** APIC ID
/// register through the architectural page, and say it is alive.
///
/// Protected mode because `0xfee00000` is nowhere a real-mode segment can
/// reach, and a Start-Up leaves a processor in real mode (*Intel SDM* Vol 3A
/// §8.4.3). The descriptor table is the one the boot sector already built in
/// RAM at [`OFF_GDT`], which every processor on the board can see.
///
/// `at` is where this code will be copied to, because a far jump into the
/// 32-bit half has to name an absolute address.
fn trampoline(at: u32) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    out.push(0xfa); // cli
    out.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    // lgdt [BOOT + OFF_GDT_PTR], a 16-bit displacement off DS = 0.
    out.extend_from_slice(&[0x0f, 0x01, 0x16]);
    out.extend_from_slice(&(BOOT + OFF_GDT_PTR).to_le_bytes());
    out.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    out.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    out.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    // jmp far 0x08:<the 32-bit half>, which is eight bytes long, so the target
    // is the byte after it.
    out.extend_from_slice(&[0x66, 0xea]);
    let pm = at + out.len() as u32 + 6;
    dw(&mut out, pm);
    out.extend_from_slice(&[0x08, 0x00]);

    // -- 32 bits ------------------------------------------------------------
    out.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    out.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    out.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    out.push(0xbc); // mov esp, imm32
    dw(&mut out, 0x6000);
    out.push(0xbf); // mov edi, LAPIC
    dw(&mut out, LAPIC);
    // mov eax, [edi+REG_ID] — the whole reason this file exists.
    out.extend_from_slice(&[0x8b, 0x87]);
    dw(&mut out, REG_ID);
    out.extend_from_slice(&[0xc1, 0xe8, 0x18]); // shr eax, 24
    out.push(0xa3); // mov [AP_ID_READ], eax
    dw(&mut out, AP_ID_READ);
    store_abs(&mut out, AP_MARKER, AP_ALIVE);
    out.push(0xf4); // hlt
    out.extend_from_slice(&[0xeb, 0xfd]); // jmp back to the hlt
    out
}

/// A 1.44 MB diskette whose boot sector starts the other processor and has both
/// of them report the APIC ID they read at the architectural address.
fn bootable_diskette() -> Vec<u8> {
    let mut sector = vec![0u8; 512];

    // -- real mode ----------------------------------------------------------
    let mut entry: Vec<u8> = Vec::new();
    entry.push(0xfa); // cli
    entry.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    entry.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    entry.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    entry.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    entry.extend_from_slice(&[0xbc]); // mov sp, imm16
    entry.extend_from_slice(&BOOT.to_le_bytes());
    entry.extend_from_slice(&[0x0f, 0x01, 0x16]); // lgdt [BOOT + OFF_GDT_PTR]
    entry.extend_from_slice(&(BOOT + OFF_GDT_PTR).to_le_bytes());
    entry.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    entry.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    entry.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    entry.extend_from_slice(&[0x66, 0xea]); // jmp far 0x08:pm
    dw(&mut entry, u32::from(BOOT) + u32::from(OFF_PM));
    entry.extend_from_slice(&[0x08, 0x00]);
    assert!(
        entry.len() <= OFF_GDT as usize,
        "the real-mode entry ran into the GDT"
    );
    sector[..entry.len()].copy_from_slice(&entry);

    // -- the descriptor table -----------------------------------------------
    //
    // In RAM, so the accessed bit a descriptor load sets is a write a
    // hypervisor can land; set here anyway, so that a reader copying this into
    // a ROM-resident table does not inherit the `KVM_RUN` that never returns
    // (`accel::board`).
    let gdt: [u8; 24] = [
        0, 0, 0, 0, 0, 0, 0, 0, // the null descriptor
        0xff, 0xff, 0, 0, 0, 0x9b, 0xcf, 0, // a flat 4 GiB code segment, ring 0
        0xff, 0xff, 0, 0, 0, 0x93, 0xcf, 0, // and a flat data segment
    ];
    let at = OFF_GDT as usize;
    sector[at..at + gdt.len()].copy_from_slice(&gdt);

    let mut gdt_ptr = Vec::new();
    gdt_ptr.extend_from_slice(&(gdt.len() as u16 - 1).to_le_bytes());
    dw(&mut gdt_ptr, u32::from(BOOT) + u32::from(OFF_GDT));
    let at = OFF_GDT_PTR as usize;
    sector[at..at + gdt_ptr.len()].copy_from_slice(&gdt_ptr);

    // -- the trampoline, as data --------------------------------------------
    let tramp = trampoline(AP_TRAMPOLINE);
    let at = OFF_TRAMPOLINE as usize;
    assert!(
        at + tramp.len() <= 510,
        "the trampoline overran the boot signature"
    );
    sector[at..at + tramp.len()].copy_from_slice(&tramp);

    // -- protected mode -----------------------------------------------------
    let mut pm: Vec<u8> = Vec::new();
    pm.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    pm.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    pm.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    pm.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    pm.push(0xbc); // mov esp, imm32
    dw(&mut pm, 0x7000);

    // The trampoline, into the board's own RAM at a page a Start-Up can name.
    pm.push(0xbe); // mov esi, imm32
    dw(&mut pm, u32::from(BOOT) + u32::from(OFF_TRAMPOLINE));
    pm.push(0xbf); // mov edi, imm32
    dw(&mut pm, AP_TRAMPOLINE);
    pm.push(0xb9); // mov ecx, imm32
    dw(&mut pm, tramp.len() as u32);
    pm.extend_from_slice(&[0xf3, 0xa4]); // rep movsb

    // Who to start, out of the firmware's own MP configuration table rather
    // than out of this file. Leaves the destination half in `EBX`.
    pm.extend_from_slice(&find_application_processor());

    // Software-enable this processor's local APIC, with 0xff as the spurious
    // vector: nothing it delivers — an IPI it sends included — is reliable
    // until this is written (*Intel SDM* Vol 3A §10.4.7.2).
    pm.push(0xbf); // mov edi, LAPIC
    dw(&mut pm, LAPIC);
    store_at(&mut pm, REG_SVR, 0x1ff);

    // And what *this* processor reads at the same address, which is the other
    // half of the claim: the window is per-processor, not a redirection.
    pm.extend_from_slice(&[0x8b, 0x87]); // mov eax, [edi+REG_ID]
    dw(&mut pm, REG_ID);
    pm.extend_from_slice(&[0xc1, 0xe8, 0x18]); // shr eax, 24
    pm.push(0xa3); // mov [BSP_ID_READ], eax
    dw(&mut pm, BSP_ID_READ);

    // The *MultiProcessor Specification* §B.4 sequence: the destination half,
    // `INIT` assert, `INIT` de-assert, Start-Up carrying the page.
    pm.extend_from_slice(&[0x89, 0x9f]); // mov [edi+REG_ICR_HIGH], ebx
    dw(&mut pm, REG_ICR_HIGH);
    store_at(&mut pm, REG_ICR_LOW, 0x0000_c500);
    store_at(&mut pm, REG_ICR_LOW, 0x0000_8500);
    store_at(&mut pm, REG_ICR_LOW, 0x0000_0600 | u32::from(AP_PAGE));

    store_abs(&mut pm, BSP_MARKER, BSP_ALIVE);
    pm.push(0xf4); // hlt
    pm.extend_from_slice(&[0xeb, 0xfd]); // jmp back to the hlt
    let at = OFF_PM as usize;
    assert!(
        at + pm.len() <= OFF_TRAMPOLINE as usize,
        "the protected-mode half ran into the trampoline"
    );
    sector[at..at + pm.len()].copy_from_slice(&pm);

    // What `INT 19h` looks for, and the reason a blank diskette is not booted.
    sector[510] = 0x55;
    sector[511] = 0xaa;

    let mut image = vec![0u8; 1_474_560];
    image[..sector.len()].copy_from_slice(&sector);
    image
}

// ---------------------------------------------------------------------------
// running it
// ---------------------------------------------------------------------------

/// The build options: rsemu's own firmware, assembled **for this board**, and
/// that diskette.
fn options(text: &str) -> BuildOptions {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert(
        "bios",
        rsemu::fw::pcbios::image_for_machine("pc-at-smp.machine", text)
            .expect("the two-processor board resolves"),
    );
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("optionrom", vec![0u8; 65536]);
    options.realize.media.insert("floppy", bootable_diskette());
    for slot in ["disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    options
}

/// A dword of guest memory, read as a debugger would.
fn peek32(m: &Machine, addr: u32) -> u32 {
    let mem = m.space("mem").expect("the memory space");
    mem.read(u64::from(addr), Width::U32, MemAttrs::DEBUG)
        .unwrap_or(0) as u32
}

/// What the guest reported.
#[derive(Debug)]
struct Reported {
    bsp_alive: bool,
    ap_alive: bool,
    ap_id_from_table: u32,
    ap_id_read: u32,
    bsp_id_read: u32,
}

/// Boot `text`, run until both processors have reported, and read the markers.
fn run(text: &str) -> Reported {
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at-smp.machine", text, &registry, &options(text))
        .unwrap_or_else(|e| panic!("the two-processor AT does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    // The markers start at a value neither processor can write, so "never ran"
    // is not read as "read zero" — which is exactly the failure the negative
    // control is looking for.
    let mem = m.space("mem").expect("the memory space");
    for marker in [AP_ID_FROM_TABLE, AP_ID_READ, BSP_ID_READ] {
        mem.write(
            u64::from(marker),
            Width::U32,
            u64::from(UNSET),
            MemAttrs::DEBUG,
        )
        .expect("low memory is RAM");
    }

    for _ in 0..2000 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek32(&m, BSP_MARKER) == BSP_ALIVE && peek32(&m, AP_MARKER) == AP_ALIVE {
            break;
        }
    }

    // POST ran on the board's own firmware, whichever way the page decodes.
    assert_eq!(
        m.space("mem")
            .expect("the memory space")
            .read(0x413, Width::U16, MemAttrs::DEBUG)
            .expect("the BDA"),
        639,
        "POST did not fill the BIOS data area"
    );

    Reported {
        bsp_alive: peek32(&m, BSP_MARKER) == BSP_ALIVE,
        ap_alive: peek32(&m, AP_MARKER) == AP_ALIVE,
        ap_id_from_table: peek32(&m, AP_ID_FROM_TABLE),
        ap_id_read: peek32(&m, AP_ID_READ),
        bsp_id_read: peek32(&m, BSP_ID_READ),
    }
}

// ---------------------------------------------------------------------------
// the gate
// ---------------------------------------------------------------------------

/// **The claim.** On the shipped board, an application processor that reads
/// `0xfee00020` reads *its own* APIC ID.
#[test]
fn an_application_processor_reads_its_own_apic_id_at_the_architectural_address() {
    let found = run(shipped());

    assert!(
        found.bsp_alive,
        "the boot sector never reached protected mode"
    );
    assert_eq!(
        found.ap_id_from_table, 1,
        "the boot sector did not find an application processor entry in the firmware's MP \
         configuration table"
    );
    assert!(
        found.ap_alive,
        "the application processor never started: {found:?}"
    );

    assert_eq!(
        found.bsp_id_read, 0,
        "the bootstrap processor read the wrong APIC through the architectural page"
    );
    assert_eq!(
        found.ap_id_read, 1,
        "the application processor read APIC ID {} through 0xfee00000 — its own is 1, and the \
         bootstrap processor's is 0",
        found.ap_id_read
    );
}

/// **The negative control.** The same guest, the same firmware and the same two
/// processors on a board that maps `lapic0.regs` at the architectural address —
/// the model as it was — and the application processor reads the bootstrap
/// processor's APIC ID.
///
/// This is what makes the test above mean something: it fails against the model
/// this change replaced, and it fails for the reason claimed rather than by not
/// booting.
#[test]
fn without_the_window_an_application_processor_reads_the_bootstrap_processors_apic() {
    let found = run(&without_the_window());

    // Everything else about the board still works, which is the point of a
    // control: it enumerates and it starts the second processor.
    assert!(
        found.bsp_alive,
        "the boot sector never reached protected mode"
    );
    assert_eq!(found.ap_id_from_table, 1, "the tables still name it");
    assert!(
        found.ap_alive,
        "the application processor never started: {found:?}"
    );

    assert_eq!(found.bsp_id_read, 0);
    assert_eq!(
        found.ap_id_read, 0,
        "without the window the application processor should read the bootstrap processor's APIC \
         ID; reading {} means this control no longer controls for anything",
        found.ap_id_read
    );
}

/// A board that maps the architectural page and then leaves a local APIC not
/// saying whose it is **fails to build**, naming it.
///
/// The failure mode the window exists to remove is silent, so the one way to
/// reach it must not be: an APIC with no `cpu` on a bus with a window would
/// answer that processor's accesses with somebody else's registers, which is
/// exactly the defect. It is a configuration error, at build time, before
/// anything has run.
#[test]
fn a_local_apic_that_does_not_say_whose_it_is_fails_the_build() {
    let text = shipped().replace(
        r#"object lapic1 "pc.lapic"  { clock = bus, id = 1, bus = "apic", cpu = cpu1 }"#,
        r#"object lapic1 "pc.lapic"  { clock = bus, id = 1, bus = "apic" }"#,
    );
    assert_ne!(text, shipped(), "the `lapic1` object moved");
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let message = match build("pc-at-smp.machine", &text, &registry, &options(&text)) {
        Err(e) => format!("{e}"),
        Ok(_) => panic!("a board with an unclaimed local APIC must not realize"),
    };
    assert!(
        message.contains("cpu ="),
        "the error should say what to add: {message}"
    );
}

/// The firmware's tables follow the shipped board: two processors, and the one
/// local-APIC address they have room for is the architectural one.
#[test]
fn the_tables_describe_two_processors_at_one_address() {
    let platform = rsemu::fw::pcbios::Platform::from_machine("pc-at-smp.machine", shipped())
        .expect("the shipped board resolves");
    assert_eq!(platform.processor_count(), 2);
    assert!(platform.processors[0].bootstrap);
    assert!(!platform.processors[1].bootstrap);
    assert_eq!(platform.processors[0].apic_id, 0);
    assert_eq!(platform.processors[1].apic_id, 1);
    // Read off `map mem 0xfee00000 … = lapic0.window`, not written down here:
    // a board that moved the page would move the table with it.
    assert_eq!(platform.lapic, 0xfee0_0000);
    // The I/O APIC's ID moved out of the second local APIC's way, and the
    // table follows the file rather than restating 1.
    assert_eq!(platform.ioapic.expect("an I/O APIC").id, 2);
}

/// The same claim **in hardware**.
///
/// The requester id the window decodes on is stamped by the interpreter in
/// `cpu::x86::exec` and rebuilt by `accel::kvm` on both of its exit paths, so
/// "an application processor reads its own APIC" is two separate facts about
/// two code paths. This is the second one: the same board, the same diskette,
/// two vCPUs, and the APIC ID read through a `KVM_EXIT_MMIO` that rsemu routes
/// back into its own address space.
///
/// Skips cleanly with no usable `/dev/kvm`.
#[cfg(all(feature = "accel-kvm", target_os = "linux", target_arch = "x86_64"))]
mod accelerated {
    use super::*;
    use rsemu::accel::cpu::AccelCpus;
    use rsemu::accel::kvm::Kvm;
    use rsemu::core::sched::ThreadingMode;

    #[test]
    fn an_application_processor_reads_its_own_apic_id_in_hardware() {
        if !Kvm::is_available() {
            println!("pc-at-smp kvm: no usable /dev/kvm on this host, skipping");
            return;
        }
        let accel = match AccelCpus::open(ThreadingMode::Parallel) {
            Ok(accel) => accel,
            Err(e) if e.is_unavailable() => return,
            Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
        };

        let text = shipped();
        let mut opts = options(text);
        opts.realize.scheduler.mode = ThreadingMode::Parallel;
        accel.install(&mut opts.bindings);
        let registry = rsemu::machine::catalog::registry().expect("this build's registry");
        let mut m = build("pc-at-smp.machine", text, &registry, &opts)
            .unwrap_or_else(|e| panic!("the shipped two-processor AT does not realize: {e}"));
        m.reset(ResetKind::Cold);
        m.sweep();

        let cpus = accel.cpus();
        assert_eq!(cpus.len(), 2, "the board declares two processors");
        assert_eq!(
            cpus[1].entries(),
            0,
            "the application processor ran before the guest started it"
        );

        {
            let mem = m.space("mem").expect("the memory space");
            for marker in [AP_ID_FROM_TABLE, AP_ID_READ, BSP_ID_READ] {
                mem.write(
                    u64::from(marker),
                    Width::U32,
                    u64::from(UNSET),
                    MemAttrs::DEBUG,
                )
                .expect("low memory is RAM");
            }
        }
        for _ in 0..400 {
            m.run_for(GlobalTime::from_nanos(1_000_000))
                .expect("the board runs");
            if peek32(&m, BSP_MARKER) == BSP_ALIVE && peek32(&m, AP_MARKER) == AP_ALIVE {
                break;
            }
        }

        assert!(
            !cpus[0].is_stopped(),
            "cpu0 stopped: {:?}",
            cpus[0].failure()
        );
        assert!(
            !cpus[1].is_stopped(),
            "cpu1 stopped: {:?}",
            cpus[1].failure()
        );
        assert_eq!(
            peek32(&m, AP_MARKER),
            AP_ALIVE,
            "the second processor never ran"
        );
        assert!(
            cpus[1].entries() > 0,
            "the marker is there but the application processor entered no guest"
        );
        assert_eq!(peek32(&m, BSP_ID_READ), 0);
        assert_eq!(
            peek32(&m, AP_ID_READ),
            1,
            "the application processor read the wrong APIC through a KVM MMIO exit"
        );
    }
}

/// Both processors' APICs survive a snapshot, together, to an identical state
/// hash — with the machine stopped somewhere neither of them is idle.
#[test]
fn a_running_two_processor_board_round_trips_through_a_snapshot() {
    let text = shipped();
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let opts = options(text);
    let mut m = build("pc-at-smp.machine", text, &registry, &opts).expect("it realizes");
    m.reset(ResetKind::Cold);
    m.sweep();
    // Long enough for the boot sector to have started the second processor, so
    // the chunk being round-tripped is two *live* APICs rather than two reset
    // ones: one software-enabled with an in-service interrupt history, one that
    // has just left wait-for-SIPI.
    for _ in 0..400 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek32(&m, AP_MARKER) == AP_ALIVE {
            break;
        }
    }
    assert_eq!(
        peek32(&m, AP_MARKER),
        AP_ALIVE,
        "the second processor had not started, so this snapshots the wrong thing"
    );

    let bytes = m.save().expect("the machine snapshots");
    let before = m.state_hash().expect("a hash");

    let mut other = build("pc-at-smp.machine", text, &registry, &options(text)).expect("it builds");
    other.reset(ResetKind::Cold);
    other.load(&bytes).expect("the snapshot restores");
    assert_eq!(
        other.state_hash().expect("a hash"),
        before,
        "the restored two-processor board is not the machine that was saved"
    );
}
