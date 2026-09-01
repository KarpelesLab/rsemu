//! Run a **real PC firmware image** on the whole assembled `pc-at` board.
//!
//! [`cpu::x86::firmware`] runs one on a bare core against flat RAM, with no
//! chipset at all, and says so: every I/O port there reads as ones, so it can
//! show the processor executing firmware correctly and nothing about a boot.
//! This is the other half — the same image on the board `machines/pc-at.machine`
//! describes, with its two 8259As, its 8254, its 146818, its 8237 pair, its
//! 8042, its VGA, its floppy controller and its IDE channel all answering.
//!
//! # Running it
//!
//! Gated on an environment variable naming an image, exactly as the corpus
//! runners are, so `cargo test` stays hermetic and needs nothing installed:
//!
//! ```text
//! RSEMU_BIOS=/usr/share/qemu/bios.bin \
//! RSEMU_VGABIOS=/usr/share/qemu/vgabios.bin \
//!   cargo test --release --all-features --test pc_at_firmware -- --nocapture
//! ```
//!
//! **Nothing is vendored**: both images are read from wherever the variables
//! point and neither enters this repository. Running a program — including a
//! copyleft one — as an emulated guest is ordinary use and creates no
//! derivative work (`ROADMAP.md` §1); reading its source would be a different
//! matter, and was not done.
//!
//! # What it reports
//!
//! Everything it prints is an observation, and the point of the file is that
//! "it got further" is a claim somebody else can check:
//!
//! - **The log port at 0x402.** Firmware built for emulated machines writes
//!   its progress there one character at a time. This test maps a sink over
//!   that hole — the board does not, and must not — and prints what arrived.
//!   It is by far the most useful instrument here, and it is nothing more than
//!   reading what a program printed.
//! - **The BIOS data area**, hex-dumped and with the fields a boot fills in
//!   called out: the equipment word at `0x410`, the base memory size at
//!   `0x413`, the tick count at `0x46c`, the video mode at `0x449`.
//! - **The CMOS through ports 0x70/0x71**, because the memory sizes there are
//!   what the firmware sizes RAM from.
//! - **The 8259A pair and the 8254**, read back through their own commands:
//!   what is pending, what is masked, what mode the counters are in.
//! - **The host bridge's configuration space through 0xcf8/0xcfc**, and its
//!   PAM registers in particular: they say which of the thirteen windows
//!   between `0xc0000` and `0xfffff` is decoding RAM rather than ROM, and in
//!   which direction. Beside them, a byte-for-byte comparison of the shadowed
//!   f-segment against the high ROM alias — which is always the chip — so a
//!   shadow that was never filled is distinguishable from one the firmware
//!   filled and then wrote its own variables into.
//! - **`0x7c00`**, where a boot sector lands, and the text page as characters.
//!
//! # The knobs
//!
//! | Variable | What it does |
//! | --- | --- |
//! | `RSEMU_BIOS` | The system firmware image. Without it the test skips. |
//! | `RSEMU_VGABIOS` | A video option ROM. Without it the socket is empty. |
//! | `RSEMU_FLOPPY` | A diskette image, padded to 1.44 MB. Blank without it. |
//! | `RSEMU_BIOS_MS` | How long to run, in virtual milliseconds. |
//! | `RSEMU_TRACE` | How many instructions of tail to print. |
//! | `RSEMU_TRACE_BACK_US` | Where the fine pass starts, before the stop. |
//! | `RSEMU_TRACE_HEAD` | Instructions to print from reset, stepped by hand. |
//! | `RSEMU_TRACE_SKIP` | How many of those to skip first. |
//! | `RSEMU_TRACE_IVT` | Poke each real-mode vector with its own number, so a |
//! | | fault into a zeroed table says which vector it was. |
//!
//! The run is deterministic, so the interesting instant can be found twice: a
//! coarse pass runs until the machine stops making progress and records *when*
//! in virtual time that happened, and a fine pass rebuilds the same board and
//! single-steps the last stretch.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

/// How long to let the board run, in virtual milliseconds, unless
/// `RSEMU_BIOS_MS` says otherwise.
const DEFAULT_MS: u64 = 200;

/// How many instructions of tail the fine pass keeps, unless `RSEMU_TRACE`
/// says otherwise.
const DEFAULT_TRACE: usize = 64;

/// The board, with the user's firmware in its sockets.
///
/// Nothing is laid over the ROM window. There used to be a switch that did —
/// writable RAM over `0xe0000`, to measure what shadowing would be worth before
/// the mechanism existed — and it is gone because the mechanism exists: the
/// board carries a `pc.pmc` whose PAM registers do the real thing.
fn board(bios: Vec<u8>, vgabios: Option<Vec<u8>>) -> (Machine, Arc<X86>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::dev::ata::bind(&mut b).expect("the hard disks");
    let kept = Arc::clone(&cpus);
    b.bind("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");

    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(b);
    options.realize.media.insert("bios", bios);
    // A blank option ROM socket if the user named no video BIOS: 64 KiB of
    // zeroes has no `0x55 0xaa` signature, which is exactly what an empty
    // socket looks like to the scan.
    options
        .realize
        .media
        .insert("vgabios", vgabios.unwrap_or_default());
    // The diskette: whatever `RSEMU_FLOPPY` points at, padded to 1.44 MB, or a
    // blank one. Blank is the honest default — a PC with no disk in the drive
    // is an ordinary PC — but a *bootable* one is the only way to measure the
    // last step of a boot, which is the firmware handing control to 0x7c00.
    let mut floppy = std::env::var("RSEMU_FLOPPY")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
        .unwrap_or_default();
    floppy.resize(1_474_560, 0);
    options.realize.media.insert("floppy", floppy);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("hd1", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = match build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu)
}

/// One byte of the guest's memory space, read as a debugger reads.
fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

/// A run of guest memory, for a hex dump.
fn peek_bytes(m: &Machine, addr: u64, len: u64) -> Vec<u8> {
    (0..len).map(|i| peek(m, addr + i)).collect()
}

/// What the log port answers a read with.
///
/// A firmware built for an emulated machine **probes** this port before it
/// trusts it: it reads, and expects its own opcode-shaped signature back. The
/// convention is Bochs's, whose debug console at 0xe9 reads back `0xe9`, and
/// the port at 0x402 answers the same way.
///
/// That mattered the moment RAM shadowing started working, and the way it
/// mattered is worth writing down. The variable holding "which port is the log
/// port" lives in the firmware's own f-segment. With the f-segment a ROM
/// socket, a firmware that probed, disliked the answer and stored a zero had
/// its store *swallowed* — so the log kept working by accident. With the
/// f-segment shadowed into RAM the store sticks, and an unrecognised port
/// silently turns the log off. The first run after shadowing landed showed
/// exactly that: the banner, then nothing.
const DEBUG_PORT_SIGNATURE: u8 = 0xe9;

/// A write-only port that keeps every byte written to it.
///
/// Mapped at 0x402 by [`listen`], which is **not** part of the board: the
/// firmwares built for emulated machines write their progress log there one
/// character at a time, and reading what a program prints is the most ordinary
/// black-box observation there is (`ROADMAP.md` §1). It answers reads with
/// ones, exactly as the unmapped hole it replaces did, so a firmware that
/// probes for it finds what it would have found anyway.
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

/// One trace line: where the core is, what is there, and what it holds.
fn one(cpu: &X86, regs: &rsemu::cpu::x86::Regs) -> String {
    let text = cpu
        .disassemble(regs.cs, regs.eip, 1)
        .first()
        .map_or_else(|| "??".to_string(), |d| format!("{d}"));
    format!(
        "{:04x}:{:08x}  {:<34}  eax={:08x} ebx={:08x} ecx={:08x} edx={:08x} esi={:08x} \
         edi={:08x} ebp={:08x} esp={:08x} ds={:04x} es={:04x} ss={:04x} fl={:08x}",
        regs.cs,
        regs.eip,
        text,
        regs.eax,
        regs.ebx,
        regs.ecx,
        regs.edx,
        regs.esi,
        regs.edi,
        regs.ebp,
        regs.esp,
        regs.ds,
        regs.es,
        regs.ss,
        regs.eflags,
    )
}

/// Where a coarse pass stopped, and why.
struct Stop {
    /// Virtual time the machine had reached.
    at: GlobalTime,
    /// Whether `CS:EIP` had stopped moving.
    stuck: bool,
    /// Whether `CR0.PE` was ever set during the run.
    ///
    /// Sampled as it goes rather than read at the end, because firmware for
    /// this machine goes in and out of protected mode and usually stops in
    /// real mode. "Did it ever get there" is the question; "is it there now"
    /// is not.
    protected: bool,
}

/// Run the board until it stops making progress or `limit` runs out.
fn run_until_stuck(m: &mut Machine, cpu: &X86, limit: GlobalTime) -> Stop {
    // 100 microseconds is 2,500 cycles of the 25 MHz clock: fine enough that a
    // tight loop is caught within a millisecond of entering it, coarse enough
    // that two hundred milliseconds is two thousand slices.
    let slice = GlobalTime::from_nanos(100_000);
    let mut last = (0u16, 0u32, 0u64);
    let mut same = 0u32;
    let mut protected = false;
    while m.now() < limit {
        m.run_for(slice).expect("the machine runs");
        protected |= cpu.sys().protected();
        let regs = cpu.regs();
        // The cycle counter as well as the instruction pointer, because one
        // enormous `REP MOVSB` charges hundreds of thousands of clocks in a
        // single step and the core then sits at the *next* address for
        // milliseconds of virtual time repaying the debt. That looks exactly
        // like a wedge and is the opposite of one.
        let here = (regs.cs, regs.eip, cpu.cycles());
        if cpu.cycle_debt() > 0 {
            // Still repaying: not wedged, just expensive.
            same = 0;
            last = here;
            continue;
        }
        if cpu.is_halted() && regs.eflags & 0x200 != 0 {
            // Halted with `IF` set is a `HLT` waiting for an interrupt, which
            // is not a wedge: the BIOS's own idle loop does exactly this, and
            // the tick that wakes it is 55 ms away. Counting those slices as
            // "no progress" was what made a working timer look like a dead
            // one. A halt with `IF` *clear* is the other thing entirely, and
            // falls through to the counter below.
            same = 0;
            last = here;
            continue;
        }
        if here == last {
            same += 1;
            // A thousand slices — a tenth of a second of virtual time — with
            // the instruction pointer and the cycle count both frozen. Long
            // enough that no periodic device this board carries could still be
            // owed a wake-up, short enough to notice inside a run.
            if same >= 1_000 {
                return Stop {
                    at: m.now(),
                    stuck: true,
                    protected,
                };
            }
        } else {
            same = 0;
            last = here;
        }
    }
    Stop {
        at: m.now(),
        stuck: false,
        protected,
    }
}

#[test]
fn a_pc_firmware_image_runs_on_the_assembled_board() {
    let Ok(path) = std::env::var("RSEMU_BIOS") else {
        println!(
            "pc-at firmware: set RSEMU_BIOS to a legacy PC BIOS image to run one on \
             this board; see the module docs"
        );
        return;
    };
    let bios = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let vgabios = std::env::var("RSEMU_VGABIOS")
        .ok()
        .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")));
    println!(
        "pc-at firmware: {} bytes of system BIOS, {} bytes of video BIOS",
        bios.len(),
        vgabios.as_ref().map_or(0, Vec::len)
    );

    let ms: u64 = std::env::var("RSEMU_BIOS_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_MS);
    let limit = GlobalTime::from_nanos(ms * 1_000_000);

    let (mut m, cpu) = board(bios.clone(), vgabios.clone());
    // The two ends of the ROM's two windows, before anything runs. The first
    // is where the processor fetches its first instruction and the second is
    // the same byte through the low alias; a board that got the aliasing wrong
    // shows it here rather than as an unexplained fault a million cycles in.
    for at in [0xffff_fff0u64, 0xf_fff0, 0xf_0000, 0xe_0000] {
        let hex: Vec<String> = peek_bytes(&m, at, 8)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        println!("pc-at firmware: [{at:08x}] {}", hex.join(" "));
    }
    println!(
        "pc-at firmware: at reset intr={} nmi={} vector={:02x} reset_pending={}",
        cpu.intr_asserted(),
        cpu.nmi_pending(),
        cpu.intr_vector(),
        cpu.reset_pending()
    );
    let log = listen(&m);
    let stop = run_until_stuck(&mut m, &cpu, limit);
    let text = String::from_utf8_lossy(&log.text.lock().expect("not poisoned")).into_owned();
    println!("pc-at firmware: what it wrote to the log port at 0x402:");
    if text.is_empty() {
        println!("  (nothing)");
    }
    for line in text.lines() {
        println!("  | {line}");
    }
    // The CMOS as the guest sees it, through the ports rather than through the
    // device: the memory sizes it reports are what a firmware sizes RAM from.
    //
    // These reads are the guest's own, not a debugger's — an index latch has
    // to be written for the data port to mean anything, and a read-back
    // command is a write. That is why every probe from here down happens
    // *after* the run and nothing is executed afterwards.
    {
        let port = m.space("port").expect("the I/O space");
        let mut cmos = [0u8; 0x40];
        for (i, byte) in cmos.iter_mut().enumerate() {
            port.write(0x70, Width::U8, i as u64, MemAttrs::DEFAULT)
                .ok();
            *byte = port.read(0x71, Width::U8, MemAttrs::DEFAULT).unwrap_or(0) as u8;
        }
        println!("pc-at firmware: CMOS 0x00..0x40 through ports 0x70/0x71:");
        for (i, row) in cmos.chunks(16).enumerate() {
            let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
            println!("  {:02x}  {}", i * 16, hex.join(" "));
        }
        println!(
            "pc-at firmware: CMOS base 0x15/16={} KiB, ext 0x17/18={} KiB, ext 0x30/31={} KiB, \
             high 0x34/35={} x 64K",
            u16::from(cmos[0x15]) | u16::from(cmos[0x16]) << 8,
            u16::from(cmos[0x17]) | u16::from(cmos[0x18]) << 8,
            u16::from(cmos[0x30]) | u16::from(cmos[0x31]) << 8,
            u16::from(cmos[0x34]) | u16::from(cmos[0x35]) << 8,
        );
    }

    // The host bridge's configuration space, read the way the firmware reads
    // it, and the PAM registers in particular: they say which of the thirteen
    // windows between 0xc0000 and 0xfffff is decoding RAM rather than ROM, and
    // in which direction (Intel 82441FX datasheet §3.2.18).
    {
        let port = m.space("port").expect("the I/O space");
        let mut cfg = [0u8; 0x60];
        for (i, byte) in cfg.iter_mut().enumerate() {
            let addr = 0x8000_0000u64 | (i as u64 & 0xfc);
            port.write(0xcf8, Width::U32, addr, MemAttrs::DEFAULT).ok();
            *byte = port
                .read(0xcfc + (i as u64 & 3), Width::U8, MemAttrs::DEFAULT)
                .unwrap_or(0xff) as u8;
        }
        println!("pc-at firmware: PCI 00:00.0 config 0x00..0x60:");
        for (i, row) in cfg.chunks(16).enumerate() {
            let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
            println!("  {:02x}  {}", i * 16, hex.join(" "));
        }
        println!(
            "pc-at firmware: host bridge {:04x}:{:04x} class {:02x}{:02x}{:02x}, \
             PAM0-6 = {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
            u16::from(cfg[0]) | u16::from(cfg[1]) << 8,
            u16::from(cfg[2]) | u16::from(cfg[3]) << 8,
            cfg[0x0b],
            cfg[0x0a],
            cfg[0x09],
            cfg[0x59],
            cfg[0x5a],
            cfg[0x5b],
            cfg[0x5c],
            cfg[0x5d],
            cfg[0x5e],
            cfg[0x5f],
        );
        // Whether the shadow actually holds the firmware. The high ROM alias is
        // always the chip, so comparing the two windows says whether the copy
        // happened and whether it is intact.
        let low = peek_bytes(&m, 0xf_0000, 0x1_0000);
        let high = peek_bytes(&m, 0xffff_0000, 0x1_0000);
        let same = low.iter().zip(&high).filter(|(a, b)| a == b).count();
        println!(
            "pc-at firmware: f-segment vs the high ROM alias: {same}/{} bytes agree, \
             low[0..8]={:02x?}",
            low.len(),
            &low[..8]
        );
        // Where they differ, as runs. A shadow the firmware filled from the ROM
        // and then wrote its own variables into shows a handful of short runs;
        // one that was never filled shows one enormous run.
        let mut runs: Vec<(u64, u64)> = Vec::new();
        let mut start: Option<u64> = None;
        for i in 0..=low.len() {
            let differs = i < low.len() && low[i] != high[i];
            match (differs, start) {
                (true, None) => start = Some(i as u64),
                (false, Some(s)) => {
                    runs.push((s, i as u64 - s));
                    start = None;
                }
                _ => {}
            }
        }
        println!(
            "pc-at firmware: {} differing run(s); the ten longest:",
            runs.len()
        );
        runs.sort_by_key(|(_, len)| core::cmp::Reverse(*len));
        for (at, len) in runs.iter().take(10) {
            println!("  0xf{:04x} +{len:#x}", at);
        }
    }

    // The interrupt controllers, through their own OCW3 read-back: which
    // requests are pending, which are in service, and which are masked. A
    // firmware that stops at a `HLT` with `IF` set is waiting for one of them.
    {
        let port = m.space("port").expect("the I/O space");
        let show = |name: &str, cmd: u64, data: u64| {
            let imr = port.read(data, Width::U8, MemAttrs::DEFAULT).unwrap_or(0);
            port.write(cmd, Width::U8, 0x0a, MemAttrs::DEFAULT).ok();
            let irr = port.read(cmd, Width::U8, MemAttrs::DEFAULT).unwrap_or(0);
            port.write(cmd, Width::U8, 0x0b, MemAttrs::DEFAULT).ok();
            let isr = port.read(cmd, Width::U8, MemAttrs::DEFAULT).unwrap_or(0);
            println!("pc-at firmware: {name} irr={irr:02x} isr={isr:02x} imr={imr:02x}");
        };
        show("pic1", 0x20, 0x21);
        show("pic2", 0xa0, 0xa1);

        // And the 8254, through its read-back command: the status byte carries
        // the mode the counter was programmed in and the level its `OUT` pin
        // is at, and the latched count says whether it is still counting.
        for c in 0..3u64 {
            // Read-back, status only: bits 7-6 select it, bit 5 says "do not
            // latch the count", bits 3-1 pick the counters.
            let sel = 1u64 << (c + 1);
            port.write(0x43, Width::U8, 0xe0 | sel, MemAttrs::DEFAULT)
                .ok();
            let status = port
                .read(0x40 + c, Width::U8, MemAttrs::DEFAULT)
                .unwrap_or(0);
            // And a plain counter-latch command for the count itself.
            port.write(0x43, Width::U8, c << 6, MemAttrs::DEFAULT).ok();
            let lo = port
                .read(0x40 + c, Width::U8, MemAttrs::DEFAULT)
                .unwrap_or(0);
            let hi = port
                .read(0x40 + c, Width::U8, MemAttrs::DEFAULT)
                .unwrap_or(0);
            println!(
                "pc-at firmware: pit counter {c} status={status:02x} (mode {}, out {}) count={:04x}",
                (status >> 1) & 7,
                (status >> 7) & 1,
                lo | (hi << 8),
            );
        }
    }

    let regs = cpu.regs();
    let sys = cpu.sys();
    println!(
        "pc-at firmware: stopped at {:04x}:{:08x} after {} ms of virtual time \
         ({} cycles); stuck={} halted={} reset_pending={}",
        regs.cs,
        regs.eip,
        stop.at.as_nanos() / 1_000_000,
        cpu.cycles(),
        stop.stuck,
        cpu.is_halted(),
        cpu.reset_pending(),
    );
    println!("pc-at firmware: {regs}");
    println!(
        "pc-at firmware: cr0={:08x} gdtr={:08x}+{:x} idtr={:08x}+{:x}",
        sys.cr0, sys.gdtr.base, sys.gdtr.limit, sys.idtr.base, sys.idtr.limit
    );
    let (faults, last) = cpu.bus_faults();
    println!("pc-at firmware: {faults} unanswered bus access(es), last at {last:08x}");

    // The BIOS data area. Firmware fills these in as it finds the hardware, so
    // which of them is still zero says how far it got.
    let bda = peek_bytes(&m, 0x400, 0x100);
    println!("pc-at firmware: BDA 0x400..0x500:");
    for (i, row) in bda.chunks(16).enumerate() {
        let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
        println!("  {:04x}  {}", 0x400 + i * 16, hex.join(" "));
    }
    println!(
        "pc-at firmware: BDA equipment word 0x410={:02x}{:02x}, memory size 0x413={} KiB, \
         tick count 0x46c={:08x}, CRT mode 0x449={:02x}, video mode set 0x475(drives)={:02x}",
        peek(&m, 0x411),
        peek(&m, 0x410),
        u16::from(peek(&m, 0x413)) | (u16::from(peek(&m, 0x414)) << 8),
        u32::from_le_bytes([
            peek(&m, 0x46c),
            peek(&m, 0x46d),
            peek(&m, 0x46e),
            peek(&m, 0x46f)
        ]),
        peek(&m, 0x449),
        peek(&m, 0x475),
    );

    // The stack the core stopped on. A firmware waiting inside a `HLT` got
    // there through a call chain, and the return addresses are the only clue
    // to which routine is doing the waiting.
    {
        // Through the cached descriptor base, not `selector << 4`: in
        // protected mode the selector is an index into a table and shifting it
        // names an address the machine never drove.
        let base = u64::from(sys.segs[rsemu::cpu::x86::isa::seg::SS as usize].base)
            + u64::from(regs.esp & 0xffff);
        let words = peek_bytes(&m, base, 64);
        println!("pc-at firmware: stack at {:04x}:{:04x}:", regs.ss, regs.esp);
        for (i, row) in words.chunks(16).enumerate() {
            let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
            println!("  +{:02x}  {}", i * 16, hex.join(" "));
        }
    }

    // The boot sector's landing pad.
    let boot = peek_bytes(&m, 0x7c00, 16);
    let hex: Vec<String> = boot.iter().map(|b| format!("{b:02x}")).collect();
    println!("pc-at firmware: 0x7c00: {}", hex.join(" "));

    // The colour text page, as characters. A firmware that printed anything at
    // all is the cheapest proof that it got past its own early POST.
    println!("pc-at firmware: text page:");
    let mut blank = true;
    for row in 0..25u64 {
        let mut line = String::new();
        for col in 0..80u64 {
            let ch = peek(&m, 0xb8000 + (row * 80 + col) * 2);
            line.push(match ch {
                0x20..=0x7e => ch as char,
                _ => ' ',
            });
        }
        if !line.trim().is_empty() {
            blank = false;
            println!("  |{}|", line.trim_end());
        }
    }
    if blank {
        println!("  (nothing)");
    }

    // The head: the first instructions out of reset, which is where a board
    // that never gets going goes wrong.
    let head: usize = std::env::var("RSEMU_TRACE_HEAD")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    if head > 0 {
        let skip: usize = std::env::var("RSEMU_TRACE_SKIP")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        println!(
            "pc-at firmware: instructions {skip}..{} from reset:",
            skip + head
        );
        // Stepped through the core rather than through the scheduler, because
        // one instruction is the granularity the question needs and a
        // scheduler tick is not: several instructions can retire inside one,
        // and the one that faults is exactly the one that would be skipped
        // over. Virtual time does not advance, so a device that answers on a
        // timer will not — which is correct for the first stretch out of
        // reset, and the reason this is a separate mode.
        let (m2, cpu2) = board(bios.clone(), vgabios.clone());
        if std::env::var("RSEMU_TRACE_IVT").is_ok() {
            // Every real-mode interrupt vector poked with its own number, so a
            // fault that lands in the (all-zero) table says which vector it
            // was instead of vanishing into `0000:0000`. Diagnostic only, and
            // only in this mode: it is a write the firmware did not make.
            let mem = m2.space("mem").expect("the memory space");
            for v in 0..256u64 {
                mem.write(v * 4, Width::U16, v, MemAttrs::DEFAULT).ok();
                mem.write(v * 4 + 2, Width::U16, 0xdead, MemAttrs::DEFAULT)
                    .ok();
            }
        }
        for n in 0..skip + head {
            let regs = cpu2.regs();
            if n >= skip {
                println!("  {}", one(&cpu2, &regs));
            }
            if cpu2.step() == 0 {
                println!("  (the core stopped: halted or shut down)");
                break;
            }
        }
    }

    // The tail. Rebuilt rather than rewound, because the run is deterministic
    // and there is no way to step backwards.
    let keep: usize = std::env::var("RSEMU_TRACE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_TRACE);
    if keep > 0 {
        println!("pc-at firmware: last {keep} instructions before the stop:");
        let (mut m2, cpu2) = board(bios, vgabios);
        // How much of the tail to single-step. Zero — the default — steps the
        // whole run, which is affordable while the firmware stops in a few
        // milliseconds and is the only setting that is certain to cover the
        // stop: `run_until_stuck` notices a wedged core more than a
        // millisecond after it wedged, so a short window can miss it entirely.
        //
        // The fine pass cuts scheduling rounds where the coarse one does not
        // (`Machine::step_until` against `run_until`, §11.6), so the two are
        // not obliged to agree instant for instant. They have agreed on where
        // the firmware stops every time so far, and a disagreement would show
        // as a trace that ends somewhere else.
        let back: u64 = std::env::var("RSEMU_TRACE_BACK_US")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);
        let coarse = stop.at.saturating_sub(GlobalTime::from_nanos(back * 1_000));
        if back > 0 && coarse > GlobalTime::ZERO {
            m2.run_until(coarse).expect("the machine runs");
        }
        let mut ring: Vec<String> = Vec::new();
        let tick = GlobalTime::from_nanos(40);
        let mut last = (0u16, 0u32);
        while m2.now() < stop.at.saturating_add(GlobalTime::from_nanos(200_000)) {
            let regs = cpu2.regs();
            let here = (regs.cs, regs.eip);
            if here != last {
                last = here;
                ring.push(one(&cpu2, &regs));
                if ring.len() > keep * 4 {
                    ring.drain(..keep * 2);
                }
            }
            m2.step_until(m2.now().saturating_add(tick))
                .expect("the machine steps");
        }
        for line in ring.iter().rev().take(keep).rev() {
            println!("  {line}");
        }
    }

    // The gate. Everything above is a report; these four are the claims, and
    // each of them was false on this board at some point.
    //
    // The first is the board's rather than the firmware's: a Dword write to
    // 0xcf8 followed by a Dword read of 0xcfc reaches the host bridge, and it
    // answers with its own identity. That one is checked here because the
    // firmware's whole memory map depends on it, and because 0xcf9 sits inside
    // `CONFADD` — a board that mapped the reset control register there as well
    // would split every one of those writes into three pieces and this read
    // would come back as ones.
    //
    // A firmware that never leaves real mode did not fetch its reset vector —
    // the board's A20 gate was masking bit 20 out from under it — and one that
    // never reaches the log port did not get through its own entry code. The
    // bus-fault counter is the third: an AT reads ones off an unterminated bus
    // and never faults, so a climbing count means the memory map has a hole
    // the firmware fell into.
    {
        let port = m.space("port").expect("the I/O space");
        port.write(0xcf8, Width::U32, 0x8000_0000, MemAttrs::DEFAULT)
            .expect("a Dword write to CONFADD");
        assert_eq!(
            port.read(0xcfc, Width::U32, MemAttrs::DEFAULT),
            Ok(0x1237_8086),
            "00:00.0 does not answer with the host bridge's vendor and device id"
        );
    }
    assert!(
        stop.protected,
        "the image never set CR0.PE on this board — see the trace above"
    );
    assert!(
        !text.is_empty(),
        "the image reached protected mode but printed nothing: it did not get \
         as far as its own banner"
    );
    assert_eq!(
        faults, 0,
        "the memory map refused an access, last at {last:08x}"
    );
}
