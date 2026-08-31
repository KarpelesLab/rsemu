//! The interface a 6502 core must expose for these runners to drive it.
//!
//! **This file is the contract**, and [`adapter`] is the one place where
//! `rsemu::cpu::mos6502` is bolted onto it. The contract came first on purpose:
//! it is the smallest interface that can drive every runner here, rather than
//! whatever shape the core happened to have.
//!
//! Deliberately small. Three required methods, one optional one. Anything a
//! runner can compute for itself — elapsed cycles, instruction bytes,
//! disassembly text — is not in the trait.
//!
//! A build without [`CPU_FEATURES`] has no core it can drive, and
//! [`require_cpu`] reports that as a skip. A build *with* them and no core is a
//! defect and is asserted, not reported — see [`require_cpu`] for why the two
//! must never look alike.
//!
//! See `docs/testing/cpu-interface.md` for the prose version, including the
//! semantics of each method and the traps.

use std::fmt;

/// One bus access. Exactly one per CPU cycle, no exceptions.
///
/// A 6502 has no idle cycles: every cycle is a read or a write, including the
/// dummy reads that page crossings and read-modify-write instructions perform
/// (`docs/cpu/6502.md`). The SingleStepTests corpus checks that access by
/// access, so a core that batches memory traffic at the end of an instruction
/// cannot pass this suite no matter how correct its results are.
pub(crate) trait Bus6502 {
    /// Read one byte. Called once per read cycle.
    fn read(&mut self, addr: u16) -> u8;
    /// Write one byte. Called once per write cycle.
    fn write(&mut self, addr: u16, value: u8);
}

/// The architectural register file, as the vector format models it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct Regs {
    /// Program counter.
    pub(crate) pc: u16,
    /// Stack pointer (the low byte of `$01xx`).
    pub(crate) s: u8,
    /// Accumulator.
    pub(crate) a: u8,
    /// Index X.
    pub(crate) x: u8,
    /// Index Y.
    pub(crate) y: u8,
    /// Processor status, `NV1BDIZC`. Bit 5 reads as 1 on real silicon and the
    /// corpus encodes it that way, so a core that stores it as 0 fails every
    /// vector for a reason that has nothing to do with the instruction.
    pub(crate) p: u8,
}

/// Status-flag bit names, most significant first, for readable diffs.
pub(crate) const FLAG_NAMES: [(u8, char); 8] = [
    (0x80, 'N'),
    (0x40, 'V'),
    (0x20, '1'),
    (0x10, 'B'),
    (0x08, 'D'),
    (0x04, 'I'),
    (0x02, 'Z'),
    (0x01, 'C'),
];

/// Render a status byte as flag letters, lower-case where clear.
pub(crate) fn flags_str(p: u8) -> String {
    FLAG_NAMES
        .iter()
        .map(|&(bit, ch)| {
            if p & bit != 0 {
                ch
            } else {
                ch.to_ascii_lowercase()
            }
        })
        .collect()
}

impl fmt::Display for Regs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PC:{:04X} A:{:02X} X:{:02X} Y:{:02X} P:{:02X}[{}] S:{:02X}",
            self.pc,
            self.a,
            self.x,
            self.y,
            self.p,
            flags_str(self.p),
            self.s
        )
    }
}

/// A 6502 interpreter, driven one instruction at a time.
///
/// `Send` because the vector runner shards the corpus across threads and gives
/// each thread its own core. Nothing is shared between them.
pub(crate) trait Cpu6502: Send {
    /// Overwrite the architectural state **and discard all microarchitectural
    /// state**: any half-executed instruction, any latched interrupt, any
    /// pipelined operand fetch. After this call the core must behave exactly as
    /// if it had been in this state at an instruction boundary with no
    /// interrupt pending. Getting this wrong shows up as a handful of vectors
    /// failing per opcode with no pattern, which is a miserable thing to debug.
    fn set_regs(&mut self, regs: Regs);

    /// Read the architectural state back.
    fn regs(&self) -> Regs;

    /// Execute exactly one instruction, driving `bus` once per cycle in cycle
    /// order, and return the number of cycles consumed.
    ///
    /// The returned count must equal the number of bus calls made. It is
    /// checked, so a core that returns a table-driven count while making a
    /// different number of accesses is caught immediately rather than passing
    /// on a technicality.
    fn step(&mut self, bus: &mut dyn Bus6502) -> u32;

    /// Disassemble the instruction at `pc`, given the bytes starting there.
    ///
    /// Optional. `nestest` compares disassembly text only when asked to
    /// (`RSEMU_NESTEST_DISASM=1`), because the reference log's text is one
    /// emulator's formatting convention rather than anything the hardware
    /// specifies. `ROADMAP.md` §6 wants the disassembler generated from the
    /// same table as the decoder, so it should exist; returning `None` just
    /// means this runner will not check it.
    fn disassemble(&self, pc: u16, bytes: &[u8]) -> Option<String> {
        let _ = (pc, bytes);
        None
    }
}

/// Which member of the family is under test.
///
/// The corpus is split the same way, and picking the wrong one produces a
/// confident, uniform, entirely wrong failure across the arithmetic opcodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Variant {
    /// The original NMOS 6502, with a working decimal mode.
    Nmos6502,
    /// The Ricoh RP2A03 in the NES: decimal mode disabled in ADC/SBC, but the
    /// D flag itself still sets, clears, and pushes normally.
    Ricoh2A03,
}

impl Variant {
    /// The corpus subdirectory holding this variant's vectors.
    pub(crate) fn corpus_dir(self) -> &'static str {
        match self {
            Variant::Nmos6502 => "6502",
            Variant::Ricoh2A03 => "nes6502",
        }
    }
}

// ---------------------------------------------------------------------------
// The seam.
// ---------------------------------------------------------------------------

/// The Cargo features that let this harness drive a 6502.
///
/// `std` is not decoration. Without it `core::sync` selects the `single`
/// backend, whose `unsafe impl Sync` is sound "only on a target that cannot
/// create a second thread" and whose lock-order tracker is a process-global
/// rather than a thread-local. Both halves of this harness create threads —
/// the vector runner shards opcode files across them, and [`adapter`] gives
/// each core one — so a `no_std` build of this binary would be outside that
/// backend's contract, and reports lock-order violations that are artefacts of
/// the tracker rather than of the core. The conformance binary is a `std`
/// program; this says so where it is checked.
pub(crate) const CPU_FEATURES: &str = "cpu-mos6502,std";

/// Are those features on for this build?
///
/// The distinction this draws is the whole point of [`require_cpu`]: a build
/// without them genuinely cannot drive a core and must skip, while a build
/// *with* them and no core is a harness nobody wired up — and those two must
/// never print the same thing.
pub(crate) fn cpu_is_built() -> bool {
    cfg!(all(feature = "cpu-mos6502", feature = "std"))
}

/// Construct a core at a defined post-reset state, or `None` if this build has
/// no 6502.
///
/// The adapter stays in the test tree on purpose: the shape the corpus wants —
/// a `&mut dyn Bus6502` and a whole instruction per call — is a testing shape,
/// not the shape the scheduler drives the core with, and baking it into `src/`
/// would be letting the tests design the core. [`adapter`] is how the two
/// shapes are bridged.
#[cfg(all(feature = "cpu-mos6502", feature = "std"))]
pub(crate) fn new_cpu(variant: Variant) -> Option<Box<dyn Cpu6502>> {
    Some(Box::new(adapter::Adapter::new(variant)))
}

/// No 6502 in this build.
#[cfg(not(all(feature = "cpu-mos6502", feature = "std")))]
pub(crate) fn new_cpu(variant: Variant) -> Option<Box<dyn Cpu6502>> {
    let _ = variant;
    None
}

/// Is a core available at all?
pub(crate) fn have_cpu() -> bool {
    new_cpu(Variant::Ricoh2A03).is_some()
}

/// A core, or the *reason* there is none — and only one reason is allowed.
///
/// `nestest` sat behind an unwired `new_cpu` for months without anyone
/// noticing, because "the corpus was not fetched" and "nobody implemented the
/// adapter" both came out as `SKIP` and both passed. They are not the same
/// thing. A missing corpus is a fact about the machine the suite is running
/// on, and it must stay a skip — corpora are downloaded, never committed
/// (`CLAUDE.md`, Testing). A missing *binding* is a defect in this directory,
/// and the build already knows whether the component exists, so it is asserted
/// here rather than reported.
///
/// # Panics
///
/// If `cpu-mos6502` is compiled in and [`new_cpu`] still returns `None`.
pub(crate) fn require_cpu(variant: Variant) -> Result<Box<dyn Cpu6502>, crate::harness::Skip> {
    match new_cpu(variant) {
        Some(core) => Ok(core),
        None => {
            assert!(
                !cpu_is_built(),
                "`{CPU_FEATURES}` are on but tests/conformance/cpu.rs binds no core: \
                 every 6502 suite would skip and pass while measuring nothing. \
                 Implement `new_cpu` — see docs/testing/cpu-interface.md."
            );
            Err(crate::harness::Skip::NotBuilt {
                component: "a 6502 core",
                feature: CPU_FEATURES,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// The adapter
// ---------------------------------------------------------------------------

/// Bridging `rsemu::cpu::mos6502::Mos6502` onto [`Cpu6502`].
///
/// # Why this is not four forwarding calls
///
/// [`Cpu6502::set_regs`], [`Cpu6502::regs`] and [`Cpu6502::disassemble`] *are*
/// forwarding calls. [`Cpu6502::step`] is not, and the reason is a lifetime.
///
/// The real core reaches memory through an
/// [`AddressSpace`](rsemu::core::space::AddressSpace) built from
/// `Arc<dyn MemOps>` — shared, `Send + Sync`, `'static`. The corpus hands the
/// runner a `&mut dyn Bus6502` that lives only for the duration of one `step`
/// call. There is no safe way to put that borrow inside the `'static` `Arc`
/// the space needs, and `unsafe` is not on the table here: the six sanctioned
/// sites are listed in `CLAUDE.md` and a test harness is not one of them.
///
/// So the borrow stays where it is and the *core* moves. The core runs on its
/// own thread, owning the space and an `Arc<Proxy>` whose `MemOps` impl turns
/// every access into a message; `step` on the calling thread services those
/// messages against the borrowed bus and returns when the instruction reports
/// done. Ownership is never in question, no lifetime is erased, and the
/// `&mut dyn Bus6502` is touched only by the thread that was handed it.
///
/// The cost is a channel round trip per *read* (writes need no reply, so the
/// channel's ordering carries them). Cheap enough not to matter: the whole
/// 2 560 000-vector corpus runs in under five seconds on a 32-thread machine,
/// and the suite is opt-in.
///
/// # Reset
///
/// A freshly built `Mos6502` owes a reset sequence, and the vector corpus
/// starts mid-program with no reset pending (`docs/testing/cpu-interface.md`).
/// The public API has no "clear the reset" call — deliberately, a reset is a
/// signal — so the worker *runs* the sequence at start-up with the proxy
/// detached: the seven vector-fetch cycles read zeroes, reach no bus, and
/// leave the core at a genuine instruction boundary with no interrupt pending,
/// which is exactly what [`Cpu6502::set_regs`] promises its callers. The core's
/// class version and snapshot layout do not move for any of this.
#[cfg(all(feature = "cpu-mos6502", feature = "std"))]
pub(crate) mod adapter {
    use std::panic::AssertUnwindSafe;
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::mpsc::{Receiver, Sender, channel};

    use rsemu::core::space::{
        AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
    };
    use rsemu::cpu::mos6502::{Config, Mos6502, Regs as CoreRegs, disasm};

    use super::{Bus6502, Cpu6502, Regs, Variant};

    /// What the core thread asks the calling thread to do.
    #[derive(Debug)]
    enum Ev {
        /// Read one byte and send it back on the data channel.
        Read(u16),
        /// Write one byte. No reply — the channel already orders it.
        Write(u16, u8),
        /// The instruction finished, having charged this many cycles.
        Done { cycles: u32, regs: CoreRegs },
        /// The core panicked. The worker exits after sending this.
        Panicked(String),
    }

    /// What the calling thread asks the core thread to do.
    #[derive(Debug)]
    enum Cmd {
        SetRegs(CoreRegs),
        Step,
    }

    /// How many accesses a detached proxy answers before giving up.
    ///
    /// Reached only when the calling thread unwound out of the middle of an
    /// instruction — the runner's own cycle-overrun guard is the realistic
    /// cause. The core cannot be stopped mid-instruction, so it is let run to
    /// the end against zeroes; this bounds "to the end" so a genuinely runaway
    /// core cannot leak a spinning thread.
    const DETACHED_LIMIT: u32 = 1024;

    /// The `MemOps` the core executes against: every access becomes a message.
    #[derive(Debug)]
    struct Proxy {
        /// Accesses out to the calling thread.
        out: Mutex<Sender<Ev>>,
        /// Read data back from it.
        data: Mutex<Receiver<u8>>,
        /// Is a `step` being serviced right now?
        ///
        /// False during the start-up reset, when there is no bus to reach.
        live: AtomicBool,
        /// Accesses answered since the calling thread went away.
        detached: Mutex<u32>,
    }

    impl Proxy {
        /// One byte, from the calling thread's bus if there is one.
        fn fetch(&self, addr: u16, attrs: MemAttrs) -> u8 {
            // A debug read must not pop a FIFO or advance a pointer
            // (`CLAUDE.md`, Devices), and `Bus6502` has no side-effect-free
            // read to forward one to — so it is answered here and never
            // reaches the runner. Nothing in this harness makes one; the core
            // only does inside `disassemble`, which this adapter answers
            // without the core.
            if attrs.debug || !self.live.load(Ordering::Relaxed) {
                return 0;
            }
            if self
                .out
                .lock()
                .expect("proxy")
                .send(Ev::Read(addr))
                .is_err()
            {
                return self.detached();
            }
            match self.data.lock().expect("proxy").recv() {
                Ok(value) => value,
                Err(_) => self.detached(),
            }
        }

        /// The calling thread is gone; answer zero until the instruction ends.
        fn detached(&self) -> u8 {
            let mut n = self.detached.lock().expect("proxy");
            *n += 1;
            assert!(
                *n < DETACHED_LIMIT,
                "the core made {DETACHED_LIMIT} accesses after the driving thread \
                 unwound, without finishing its instruction"
            );
            0
        }
    }

    impl MemOps for Proxy {
        fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            for (i, slot) in dst.iter_mut().enumerate() {
                // Guest arithmetic wraps: the 6502's address bus is 16 bits and
                // an access at $FFFF continues at $0000.
                *slot = self.fetch((offset as u16).wrapping_add(i as u16), attrs);
            }
            Ok(())
        }

        fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
            for (i, byte) in src.iter().enumerate() {
                let addr = (offset as u16).wrapping_add(i as u16);
                if attrs.debug || !self.live.load(Ordering::Relaxed) {
                    continue;
                }
                if self
                    .out
                    .lock()
                    .expect("proxy")
                    .send(Ev::Write(addr, *byte))
                    .is_err()
                {
                    self.detached();
                }
            }
            Ok(())
        }

        fn constraints(&self) -> AccessConstraints {
            AccessConstraints::ANY
        }
    }

    /// The core's half of the conversation.
    fn worker(variant: Variant, cmds: Receiver<Cmd>, out: Sender<Ev>, data: Receiver<u8>) {
        let proxy = std::sync::Arc::new(Proxy {
            out: Mutex::new(out.clone()),
            data: Mutex::new(data),
            live: AtomicBool::new(false),
            detached: Mutex::new(0),
        });

        let space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::io("bus", 0x1_0000, proxy.clone()), 0)
            .expect("64 KiB fits a 16-bit space");

        let cpu = Mos6502::new(match variant {
            Variant::Nmos6502 => Config::NMOS_6502,
            Variant::Ricoh2A03 => Config::RP2A03,
        });
        cpu.attach_space(std::sync::Arc::new(space));
        // Consume the power-on reset with the proxy detached — see the module
        // header. Registers are overwritten before the first vector anyway.
        cpu.step();
        debug_assert!(!cpu.reset_pending());

        while let Ok(cmd) = cmds.recv() {
            match cmd {
                Cmd::SetRegs(regs) => {
                    // `set_regs` must discard microarchitectural state as well
                    // as architectural — the trait says so, and it is not
                    // pedantry: one core serves all 10 000 vectors of an
                    // opcode file, so the `JAM` in vector 1 of `02.json` would
                    // otherwise freeze it for the other 9 999. A jammed 6502
                    // is cleared by exactly one thing, and it is not a method
                    // call, so the reset sequence is run for real — with the
                    // proxy detached, so its seven vector-fetch cycles read
                    // zeroes and reach no bus. A `WAI` stall and a latched
                    // interrupt go the same way.
                    if cpu.is_halted() || cpu.is_waiting() || cpu.pending_interrupt().is_some() {
                        cpu.request_reset();
                        cpu.step();
                    }
                    cpu.set_regs(regs);
                }
                Cmd::Step => {
                    proxy.live.store(true, Ordering::Relaxed);
                    let stepped = crate::harness::catching(|| cpu.step());
                    proxy.live.store(false, Ordering::Relaxed);
                    let ev = match stepped {
                        Ok(cycles) => Ev::Done {
                            cycles: cycles as u32,
                            regs: cpu.regs(),
                        },
                        // A panic may have left the core's own lock in an
                        // undefined state, so this thread does not run another
                        // instruction; the adapter starts a fresh one.
                        Err(message) => {
                            let _ = out.send(Ev::Panicked(message));
                            return;
                        }
                    };
                    if out.send(ev).is_err() {
                        return;
                    }
                }
            }
        }
    }

    /// The live channels to one core thread.
    #[derive(Debug)]
    struct Link {
        cmds: Sender<Cmd>,
        events: Receiver<Ev>,
        data: Sender<u8>,
    }

    impl Link {
        fn spawn(variant: Variant) -> Link {
            let (cmds, cmd_rx) = channel();
            let (ev_tx, events) = channel();
            let (data, data_rx) = channel();
            std::thread::Builder::new()
                .name(format!("rsemu-conformance-6502-{variant:?}"))
                .spawn(move || worker(variant, cmd_rx, ev_tx, data_rx))
                .expect("a thread for the core under test");
            Link { cmds, events, data }
        }
    }

    /// A `Mos6502` driven one instruction at a time against a borrowed bus.
    #[derive(Debug)]
    pub(crate) struct Adapter {
        variant: Variant,
        link: Link,
        /// The register file as of the last `set_regs` or completed `step`.
        ///
        /// Kept here rather than fetched, so `regs()` — which the runners call
        /// once per vector and once per traced instruction — costs no round
        /// trip. It is exact except after a panic, where the core's real state
        /// is not knowable anyway.
        regs: Regs,
    }

    impl Adapter {
        pub(crate) fn new(variant: Variant) -> Adapter {
            Adapter {
                variant,
                link: Link::spawn(variant),
                regs: Regs::default(),
            }
        }

        /// Replace a core thread that will not be talked to again.
        fn respawn(&mut self) {
            self.link = Link::spawn(self.variant);
            let _ = self.link.cmds.send(Cmd::SetRegs(to_core(self.regs)));
        }
    }

    /// Drive one instruction, servicing its bus accesses against `bus`.
    ///
    /// `Err` is a panic *inside the core*, reported rather than propagated
    /// from the worker thread; a panic inside `bus` unwinds out of here, which
    /// is what the runner's cycle-overrun guard relies on.
    fn pump(link: &Link, bus: &mut dyn Bus6502) -> Result<(u32, CoreRegs), String> {
        if link.cmds.send(Cmd::Step).is_err() {
            return Err("the core thread is gone".to_string());
        }
        loop {
            match link.events.recv() {
                Ok(Ev::Read(addr)) => {
                    let value = bus.read(addr);
                    if link.data.send(value).is_err() {
                        return Err("the core thread is gone".to_string());
                    }
                }
                Ok(Ev::Write(addr, value)) => bus.write(addr, value),
                Ok(Ev::Done { cycles, regs }) => return Ok((cycles, regs)),
                Ok(Ev::Panicked(message)) => return Err(message),
                Err(_) => return Err("the core thread died without reporting why".to_string()),
            }
        }
    }

    fn to_core(regs: Regs) -> CoreRegs {
        CoreRegs {
            a: regs.a,
            x: regs.x,
            y: regs.y,
            s: regs.s,
            p: regs.p,
            pc: regs.pc,
        }
    }

    fn from_core(regs: CoreRegs) -> Regs {
        Regs {
            pc: regs.pc,
            s: regs.s,
            a: regs.a,
            x: regs.x,
            y: regs.y,
            p: regs.p,
        }
    }

    impl Cpu6502 for Adapter {
        fn set_regs(&mut self, regs: Regs) {
            self.regs = regs;
            if self.link.cmds.send(Cmd::SetRegs(to_core(regs))).is_err() {
                self.respawn();
            }
        }

        fn regs(&self) -> Regs {
            self.regs
        }

        fn step(&mut self, bus: &mut dyn Bus6502) -> u32 {
            let link = &self.link;
            // Two failure modes, and they unwind in opposite directions. A
            // panic in `bus` (the runner's cycle-overrun guard) unwinds
            // through here and must be re-raised unchanged, because that is
            // the message the runner matches on. A panic in the core arrives
            // as a value and is raised here so it lands inside the runner's
            // own `catching`. Either way the core thread is finished with.
            match std::panic::catch_unwind(AssertUnwindSafe(|| pump(link, bus))) {
                Ok(Ok((cycles, regs))) => {
                    self.regs = from_core(regs);
                    cycles
                }
                Ok(Err(message)) => {
                    self.respawn();
                    panic!("{message}");
                }
                Err(payload) => {
                    self.respawn();
                    std::panic::resume_unwind(payload);
                }
            }
        }

        /// Straight from the table the interpreter decodes with — no bus, no
        /// round trip, and no second opcode list (`CLAUDE.md`, CPU cores).
        ///
        /// # `RSEMU_NESTEST_DISASM=1` fails, and should
        ///
        /// Strict mode compares this text against `nestest.log`'s column
        /// character for character, and rsemu does not write 6502 the way
        /// Nintendulator does. Measured over all 8 991 instructions of the
        /// reference trace, 5 370 lines differ and every one of them is a
        /// naming or presentation convention:
        ///
        /// * **5 344** — lower-case hex (`JMP $c5f5` against `JMP $C5F5`),
        ///   Nintendulator's leading `*` on an undocumented encoding, and its
        ///   resolved-operand annotations (`STX $00 = 00`, `LDA ($33),Y =
        ///   0400 @ 0400 = 5B`). Those annotations are *execution* results
        ///   printed by a tracer; a disassembler cannot produce them and
        ///   should not try.
        /// * **21** — `ISC` against Nintendulator's `ISB`. Two established
        ///   names for `$e3`/`$e7`/`$ef`/`$f3`/`$f7`/`$fb`/`$ff`.
        /// * **5** — `USBC` against `*SBC` for `$eb`, where rsemu names the
        ///   alias apart from the documented `$e9`.
        ///
        /// Not one line differs in mnemonic-plus-addressing-mode: the decode
        /// this disassembler prints agrees with the reference on all 8 991.
        /// So the disassembler is reported here rather than reshaped, and
        /// strict mode stays what it is documented to be — an exact-text
        /// check for a core that has chosen Nintendulator's convention.
        fn disassemble(&self, pc: u16, bytes: &[u8]) -> Option<String> {
            let variant = match self.variant {
                Variant::Nmos6502 => rsemu::cpu::mos6502::Variant::Nmos6502,
                Variant::Ricoh2A03 => rsemu::cpu::mos6502::Variant::Ricoh2A03,
            };
            Some(disasm::disassemble_as(variant, pc, bytes).to_string())
        }
    }
}
