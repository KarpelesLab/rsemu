//! The protocol itself: one packet in, one reply out.
//!
//! [`Stub`] is a pure state machine over a [`DebugTarget`]. It owns no socket
//! and no machine, which is what makes the whole protocol testable without
//! either — see the tests at the bottom of this file, which drive a fake target
//! through a complete session.
//!
//! # What is implemented
//!
//! | Packet | Meaning |
//! | --- | --- |
//! | `?` | why the target is stopped |
//! | `g` `G` | read and write the whole register file |
//! | `p` `P` | one register |
//! | `m` `M` `X` | read and write memory, hex and binary |
//! | `c` `C` `s` `S` `vCont` | continue and step, per thread |
//! | `Z0` `z0` `Z1` `z1` | breakpoints, software and hardware |
//! | `Z2` `z2` | write watchpoints, on the selected thread's space |
//! | `H` `qC` `qfThreadInfo` `qsThreadInfo` `T` `qThreadExtraInfo` | CPUs as threads |
//! | `qSupported` `qXfer:features:read` | negotiation and the target description |
//! | `QStartNoAckMode` | drop the `+`/`-` handshake |
//! | `qRcmd` | the `monitor` command, answered for the selected thread |
//! | `qAttached` `qSymbol` `!` | the attach handshake |
//! | `k` `D` | kill and detach |
//!
//! Anything else gets an empty reply, which is the protocol's "I do not know
//! that packet" and is what GDB expects for everything it probes. That list is
//! not a guess: `RSEMU_GDB_DEBUG_REMOTE=1 cargo test --test gdb_real_client`
//! prints every packet a real GDB sends over a whole session, and the only two
//! it sends that end up here are `vMustReplyEmpty` and `qTStatus`, both of
//! which are *supposed* to be answered this way.
//!
//! # A stop reply says only what the client asked to hear
//!
//! `swbreak` and `hwbreak` post-date the protocol, so a stop reply may name one
//! **only** when the client offered it in its own `qSupported` — the manual is
//! explicit, and a client that never asked is entitled to treat an unknown stop
//! reason as a malformed packet. What we advertise says we *can* report them;
//! what the client advertises says it will understand them, and the two are
//! tracked separately.
//!
//! # Sources
//!
//! The GDB manual's "Remote Protocol" appendix: Packets, Stop Reply Packets,
//! General Query Packets, and Tracepoint/`vCont` sections.

use super::packet::{
    ACK, Event, NAK, frame, hex_decode, parse_hex_u64, parse_hex_usize, push_hex, push_hex_u8,
    push_hex_u64,
};
use super::target::{DebugTarget, Stop, StopKind, TargetError};

/// What the caller should do once the stub has processed an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// Carry on.
    Continue,
    /// The client sent `D`: it is finished, but the machine should keep going.
    Detach,
    /// The client sent `k`: shut the machine down.
    Kill,
}

/// Which thread a `c`/`s`/`g` applies to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ThreadSel {
    /// `0` — any thread; the stub picks the first.
    Any,
    /// `-1` — all threads.
    All,
    /// A one-based thread id, as CPU index.
    One(usize),
}

impl ThreadSel {
    /// Parse a thread id as it appears after `H` or in a `vCont` action.
    fn parse(text: &[u8]) -> Option<ThreadSel> {
        // Multiprocess syntax (`p<pid>.<tid>`) is not advertised, but GDB will
        // still send a bare `p1.1` in some configurations; take the tid.
        let text = match text.split(|b| *b == b'.').next_back() {
            Some(tail) if text.first() == Some(&b'p') => tail,
            _ => text,
        };
        if text == b"-1" {
            return Some(ThreadSel::All);
        }
        let id = parse_hex_u64(text)?;
        if id == 0 {
            return Some(ThreadSel::Any);
        }
        usize::try_from(id - 1).ok().map(ThreadSel::One)
    }

    /// The CPU index this selects, given a fallback for "any" and "all".
    fn cpu(self, fallback: usize) -> usize {
        match self {
            ThreadSel::One(i) => i,
            _ => fallback,
        }
    }
}

/// The GDB remote protocol, as a state machine.
#[derive(Debug)]
pub struct Stub {
    /// Whether the `+`/`-` handshake is still in force.
    no_ack: bool,
    /// Set once `QStartNoAckMode` has been answered, so the flag flips *after*
    /// that reply's own acknowledgement.
    no_ack_pending: bool,
    /// Whether the machine should be advancing.
    running: bool,
    /// The thread `g`, `p`, `m` and friends apply to.
    query_thread: usize,
    /// The thread `c` and `s` apply to when the packet does not say.
    cont_thread: ThreadSel,
    /// Ctrl-C arrived while the target was running.
    interrupt_pending: bool,
    /// Whether the client said `swbreak+` in its `qSupported`.
    ///
    /// The GDB manual is explicit that a stop reply may only carry `swbreak`
    /// or `hwbreak` when the client asked for it ("Stop Reply Packets"): the
    /// reasons post-date the protocol, and a client that has not asked for one
    /// is entitled to treat an unknown reason as a malformed packet. Real GDB
    /// always asks; an older one, or a client of our own, does not have to.
    client_swbreak: bool,
    /// Whether the client said `hwbreak+`.
    client_hwbreak: bool,
    /// Why the target last stopped, for `?`.
    last_stop: Stop,
    /// The last packet sent, for a `-` retransmission.
    last_sent: Vec<u8>,
}

impl Default for Stub {
    fn default() -> Self {
        Stub::new()
    }
}

impl Stub {
    /// A stub for a freshly accepted connection: acknowledgements on, target
    /// halted, thread 1 selected.
    #[must_use]
    pub fn new() -> Stub {
        Stub {
            no_ack: false,
            no_ack_pending: false,
            running: false,
            query_thread: 0,
            cont_thread: ThreadSel::Any,
            interrupt_pending: false,
            client_swbreak: false,
            client_hwbreak: false,
            last_stop: Stop {
                cpu: 0,
                kind: StopKind::Trap,
            },
            last_sent: Vec::new(),
        }
    }

    /// Whether the client has asked the machine to run.
    #[must_use]
    pub const fn is_running(&self) -> bool {
        self.running
    }

    /// Whether the `+`/`-` handshake has been turned off.
    #[must_use]
    pub const fn acks_disabled(&self) -> bool {
        self.no_ack
    }

    /// Frame `payload` into `out` and remember it in case of a `-`.
    fn send(&mut self, payload: &[u8], out: &mut Vec<u8>) {
        self.last_sent.clear();
        frame(payload, &mut self.last_sent);
        out.extend_from_slice(&self.last_sent);
    }

    fn send_ok(&mut self, out: &mut Vec<u8>) {
        self.send(b"OK", out);
    }

    fn send_empty(&mut self, out: &mut Vec<u8>) {
        self.send(b"", out);
    }

    fn send_error(&mut self, error: &TargetError, out: &mut Vec<u8>) {
        let mut payload = vec![b'E'];
        push_hex_u8(&mut payload, error.code());
        self.send(&payload, out);
    }

    /// An `O` packet: text for the user's GDB console.
    fn send_console(&mut self, text: &str, out: &mut Vec<u8>) {
        let mut payload = vec![b'O'];
        push_hex(&mut payload, text.as_bytes());
        self.send(&payload, out);
    }

    /// The stop reply for [`Stub::last_stop`].
    fn send_stop(&mut self, out: &mut Vec<u8>) {
        let stop = self.last_stop;
        let mut payload = vec![b'T'];
        push_hex_u8(&mut payload, stop.signal());
        payload.extend_from_slice(b"thread:");
        push_hex_u64(&mut payload, stop.cpu as u64 + 1);
        payload.push(b';');
        match stop.kind {
            StopKind::Breakpoint { hardware: false } if self.client_swbreak => {
                payload.extend_from_slice(b"swbreak:;");
            }
            StopKind::Breakpoint { hardware: true } if self.client_hwbreak => {
                payload.extend_from_slice(b"hwbreak:;");
            }
            StopKind::Breakpoint { .. } => {}
            StopKind::Watchpoint { addr } => {
                payload.extend_from_slice(b"watch:");
                push_hex_u64(&mut payload, addr);
                payload.push(b';');
            }
            StopKind::Trap | StopKind::Interrupt => {}
        }
        self.send(&payload, out);
    }

    /// Process one framer event.
    pub fn on_event(
        &mut self,
        event: Event,
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        match event {
            Event::Corrupt => {
                if !self.no_ack {
                    out.push(NAK);
                }
                Outcome::Continue
            }
            Event::Ack => Outcome::Continue,
            Event::Nak => {
                // The peer did not understand the last reply. Send it again;
                // this is the whole point of keeping a copy.
                let again = self.last_sent.clone();
                out.extend_from_slice(&again);
                Outcome::Continue
            }
            Event::Interrupt => {
                if self.running {
                    self.interrupt_pending = true;
                } else {
                    // Already stopped: GDB still wants to hear about it.
                    self.last_stop = Stop {
                        cpu: self.query_thread,
                        kind: StopKind::Interrupt,
                    };
                    self.send_stop(out);
                }
                Outcome::Continue
            }
            Event::Packet(payload) => {
                if !self.no_ack {
                    out.push(ACK);
                }
                let outcome = self.dispatch(&payload, target, out);
                if self.no_ack_pending {
                    self.no_ack_pending = false;
                    self.no_ack = true;
                }
                outcome
            }
        }
    }

    /// Advance the machine one slice, if the client asked it to run.
    ///
    /// Separate from [`Stub::on_event`] because the caller has to interleave
    /// the two: this is what lets a Ctrl-C arrive while the guest is running.
    pub fn drive(&mut self, target: &mut dyn DebugTarget, out: &mut Vec<u8>) -> Outcome {
        if !self.running {
            return Outcome::Continue;
        }
        if self.interrupt_pending {
            self.interrupt_pending = false;
            self.running = false;
            self.last_stop = Stop {
                cpu: self.cont_thread.cpu(self.query_thread),
                kind: StopKind::Interrupt,
            };
            self.send_stop(out);
            return Outcome::Continue;
        }
        match target.resume() {
            Ok(None) => {}
            Ok(Some(stop)) => {
                self.running = false;
                self.query_thread = stop.cpu;
                self.last_stop = stop;
                self.send_stop(out);
            }
            Err(e) => {
                // The machine itself failed — a scheduler that cannot advance,
                // a device that refused to save. Stopping and saying so beats
                // spinning on the same error every slice.
                self.running = false;
                let text = format!("rsemu: the machine stopped: {e}\n");
                self.send_console(&text, out);
                self.last_stop = Stop {
                    cpu: self.query_thread,
                    kind: StopKind::Trap,
                };
                self.send_stop(out);
            }
        }
        Outcome::Continue
    }

    // -- dispatch ----------------------------------------------------------

    fn dispatch(
        &mut self,
        packet: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some((&head, rest)) = packet.split_first() else {
            // An empty packet is not a request; GDB never sends one.
            self.send_empty(out);
            return Outcome::Continue;
        };
        match head {
            b'?' => {
                self.send_stop(out);
                Outcome::Continue
            }
            b'!' => {
                // Extended mode: accepted, though `R` (restart) is not offered.
                self.send_ok(out);
                Outcome::Continue
            }
            b'g' => self.read_registers(target, out),
            b'G' => self.write_registers(rest, target, out),
            b'p' => self.read_one_register(rest, target, out),
            b'P' => self.write_one_register(rest, target, out),
            b'm' => self.read_memory(rest, target, out),
            b'M' => self.write_memory_hex(rest, target, out),
            b'X' => self.write_memory_binary(rest, target, out),
            b'c' | b'C' => self.resume_packet(head, rest, target, out),
            b's' | b'S' => self.step_packet(head, rest, target, out),
            b'v' => self.v_packet(rest, target, out),
            b'H' => self.set_thread(rest, target, out),
            b'T' => self.thread_alive(rest, target, out),
            b'Z' => self.insert_point(rest, target, out),
            b'z' => self.remove_point(rest, target, out),
            b'q' | b'Q' => self.query(head, rest, target, out),
            b'D' => {
                self.running = false;
                self.send_ok(out);
                Outcome::Detach
            }
            b'k' => {
                self.running = false;
                // `k` has no reply, by specification.
                Outcome::Kill
            }
            _ => {
                self.send_empty(out);
                Outcome::Continue
            }
        }
    }

    // -- registers ---------------------------------------------------------

    fn read_registers(&mut self, target: &mut dyn DebugTarget, out: &mut Vec<u8>) -> Outcome {
        match target.read_registers(self.query_thread) {
            Ok(bytes) => {
                let mut payload = Vec::with_capacity(bytes.len() * 2);
                push_hex(&mut payload, &bytes);
                self.send(&payload, out);
            }
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn write_registers(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some(bytes) = hex_decode(rest) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        match target.write_registers(self.query_thread, &bytes) {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn read_one_register(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some(index) = parse_hex_usize(rest) else {
            self.send_error(&TargetError::NoSuchRegister, out);
            return Outcome::Continue;
        };
        match target.read_register(self.query_thread, index) {
            Ok(bytes) => {
                let mut payload = Vec::with_capacity(bytes.len() * 2);
                push_hex(&mut payload, &bytes);
                self.send(&payload, out);
            }
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn write_one_register(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some(split) = rest.iter().position(|b| *b == b'=') else {
            self.send_error(&TargetError::NoSuchRegister, out);
            return Outcome::Continue;
        };
        let (number, value) = rest.split_at(split);
        let value = value.get(1..).unwrap_or(&[]);
        let Some(index) = parse_hex_usize(number) else {
            self.send_error(&TargetError::NoSuchRegister, out);
            return Outcome::Continue;
        };
        let Some(bytes) = hex_decode(value) else {
            self.send_error(&TargetError::NoSuchRegister, out);
            return Outcome::Continue;
        };
        match target.write_register(self.query_thread, index, &bytes) {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    // -- memory ------------------------------------------------------------

    /// Split `addr,len` and reject anything that would allocate absurdly.
    fn parse_range(rest: &[u8]) -> Option<(u64, usize)> {
        let comma = rest.iter().position(|b| *b == b',')?;
        let addr = parse_hex_u64(rest.get(..comma)?)?;
        let len = parse_hex_usize(rest.get(comma + 1..)?)?;
        // The advertised PacketSize bounds a reply; anything larger is either a
        // confused client or a hostile one.
        if len > super::packet::MAX_PACKET / 2 {
            return None;
        }
        Some((addr, len))
    }

    fn read_memory(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some((addr, len)) = Self::parse_range(rest) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let mut buf = vec![0u8; len];
        match target.read_memory(self.query_thread, addr, &mut buf) {
            Ok(()) => {
                let mut payload = Vec::with_capacity(len * 2);
                push_hex(&mut payload, &buf);
                self.send(&payload, out);
            }
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn write_memory_hex(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some(colon) = rest.iter().position(|b| *b == b':') else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let (range, data) = rest.split_at(colon);
        let data = data.get(1..).unwrap_or(&[]);
        let (Some((addr, len)), Some(bytes)) = (Self::parse_range(range), hex_decode(data)) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        if bytes.len() != len {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        }
        match target.write_memory(self.query_thread, addr, &bytes) {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn write_memory_binary(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some(colon) = rest.iter().position(|b| *b == b':') else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let (range, data) = rest.split_at(colon);
        let data = data.get(1..).unwrap_or(&[]);
        let Some((addr, len)) = Self::parse_range(range) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        // `X<addr>,0:` is how GDB asks whether binary writes work at all.
        if len == 0 {
            self.send_ok(out);
            return Outcome::Continue;
        }
        if data.len() != len {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        }
        match target.write_memory(self.query_thread, addr, data) {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    // -- execution ---------------------------------------------------------

    /// The address a `c`/`s` packet optionally carries, and where it starts.
    fn resume_address(head: u8, rest: &[u8]) -> Option<u64> {
        // `c[addr]`, `s[addr]`, `C sig[;addr]`, `S sig[;addr]`.
        let tail = if head == b'C' || head == b'S' {
            let semi = rest.iter().position(|b| *b == b';')?;
            rest.get(semi + 1..)?
        } else {
            rest
        };
        if tail.is_empty() {
            None
        } else {
            parse_hex_u64(tail)
        }
    }

    /// Move a CPU's program counter, for the `c addr` form.
    fn set_pc(target: &mut dyn DebugTarget, cpu: usize, addr: u64) -> Result<(), TargetError> {
        let arch = target.arch(cpu)?;
        let reg = *arch.regs.get(arch.pc).ok_or(TargetError::NoSuchRegister)?;
        let bytes = addr.to_le_bytes();
        let value = bytes.get(..reg.bytes).ok_or(TargetError::NoSuchRegister)?;
        target.write_register(cpu, arch.pc, value)
    }

    fn resume_packet(
        &mut self,
        head: u8,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let cpu = self.cont_thread.cpu(self.query_thread);
        if let Some(addr) = Self::resume_address(head, rest)
            && let Err(e) = Self::set_pc(target, cpu, addr)
        {
            self.send_error(&e, out);
            return Outcome::Continue;
        }
        target.begin_resume();
        self.running = true;
        self.interrupt_pending = false;
        // No reply: the next thing this connection sends is a stop reply.
        Outcome::Continue
    }

    fn step_packet(
        &mut self,
        head: u8,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let cpu = self.cont_thread.cpu(self.query_thread);
        if let Some(addr) = Self::resume_address(head, rest)
            && let Err(e) = Self::set_pc(target, cpu, addr)
        {
            self.send_error(&e, out);
            return Outcome::Continue;
        }
        self.do_step(cpu, target, out);
        Outcome::Continue
    }

    fn do_step(&mut self, cpu: usize, target: &mut dyn DebugTarget, out: &mut Vec<u8>) {
        match target.step(cpu) {
            Ok(stop) => {
                self.running = false;
                self.query_thread = stop.cpu;
                self.last_stop = stop;
                self.send_stop(out);
            }
            Err(e) => self.send_error(&e, out),
        }
    }

    fn v_packet(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        if rest == b"Cont?" {
            self.send(b"vCont;c;C;s;S;t", out);
            return Outcome::Continue;
        }
        let Some(actions) = rest.strip_prefix(b"Cont;") else {
            // `vMustReplyEmpty`, `vFile:…`, `vRun`, everything else.
            self.send_empty(out);
            return Outcome::Continue;
        };
        // Actions are tried in order and the first that names this thread wins.
        // With one action and no thread it applies to everything, which is what
        // `vCont;c` means.
        let mut chosen: Option<(u8, ThreadSel)> = None;
        for action in actions.split(|b| *b == b';') {
            let Some((&kind, tail)) = action.split_first() else {
                continue;
            };
            let tail = match kind {
                // `C sig[:thread]` and `S sig[:thread]` carry a signal first.
                b'C' | b'S' => tail.get(2..).unwrap_or(&[]),
                _ => tail,
            };
            let sel = match tail.strip_prefix(b":") {
                Some(id) => ThreadSel::parse(id).unwrap_or(ThreadSel::Any),
                None => ThreadSel::Any,
            };
            if chosen.is_none() {
                chosen = Some((kind, sel));
            }
        }
        match chosen {
            Some((b's' | b'S', sel)) => {
                let cpu = sel.cpu(self.query_thread);
                self.do_step(cpu, target, out);
            }
            Some((b'c' | b'C', sel)) => {
                self.cont_thread = sel;
                target.begin_resume();
                self.running = true;
                self.interrupt_pending = false;
            }
            Some((b't', _)) => {
                self.running = false;
                self.last_stop = Stop {
                    cpu: self.query_thread,
                    kind: StopKind::Interrupt,
                };
                self.send_stop(out);
            }
            _ => self.send_empty(out),
        }
        Outcome::Continue
    }

    // -- threads -----------------------------------------------------------

    fn set_thread(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some((&op, id)) = rest.split_first() else {
            self.send_error(&TargetError::NoSuchCpu, out);
            return Outcome::Continue;
        };
        let Some(sel) = ThreadSel::parse(id) else {
            self.send_error(&TargetError::NoSuchCpu, out);
            return Outcome::Continue;
        };
        match op {
            b'c' => self.cont_thread = sel,
            b'g' => {
                let cpu = sel.cpu(0);
                if cpu >= target.cpu_count() {
                    self.send_error(&TargetError::NoSuchCpu, out);
                    return Outcome::Continue;
                }
                self.query_thread = cpu;
            }
            _ => {
                self.send_empty(out);
                return Outcome::Continue;
            }
        }
        self.send_ok(out);
        Outcome::Continue
    }

    fn thread_alive(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        match ThreadSel::parse(rest) {
            Some(ThreadSel::One(cpu)) if cpu < target.cpu_count() => self.send_ok(out),
            Some(ThreadSel::Any | ThreadSel::All) if target.cpu_count() > 0 => self.send_ok(out),
            _ => self.send_error(&TargetError::NoSuchCpu, out),
        }
        Outcome::Continue
    }

    // -- breakpoints -------------------------------------------------------

    /// `<type>,<addr>,<kind>` — the body shared by `Z` and `z`.
    fn parse_point(rest: &[u8]) -> Option<(u8, u64, u64)> {
        // `Z0,addr,kind[;cond_list][;cmds]`. The tail only appears when the stub
        // advertised `ConditionalBreakpoints` or `BreakpointCommands`, which
        // this one does not — but cutting it off costs one line and turns a
        // future `E22` into a working breakpoint.
        let rest = match rest.iter().position(|b| *b == b';') {
            Some(semi) => rest.get(..semi)?,
            None => rest,
        };
        let mut parts = rest.split(|b| *b == b',');
        let ty = parts.next()?;
        let addr = parse_hex_u64(parts.next()?)?;
        let kind = parse_hex_u64(parts.next()?)?;
        // The type is one digit; anything else is not a point packet.
        let &[ty] = ty else { return None };
        Some((ty, addr, kind))
    }

    fn insert_point(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some((ty, addr, kind)) = Self::parse_point(rest) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let result = match ty {
            // Software and hardware breakpoints are the same mechanism here —
            // a program-counter comparison — so `hbreak` works too. Which one
            // was asked for still has to be remembered: the stop reply that
            // reports it is a different packet.
            b'0' | b'1' => target.add_breakpoint(addr, ty == b'1'),
            b'2' if target.watch_support().write => {
                target.add_watchpoint(self.query_thread, addr, kind.max(1))
            }
            _ => {
                // An empty reply means "not supported", which is how GDB learns
                // that `rwatch` and `awatch` are not available here. An `E`
                // would make it think the watchpoint failed rather than that
                // the kind does not exist.
                self.send_empty(out);
                return Outcome::Continue;
            }
        };
        match result {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    fn remove_point(
        &mut self,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        let Some((ty, addr, kind)) = Self::parse_point(rest) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let result = match ty {
            b'0' | b'1' => target.remove_breakpoint(addr, ty == b'1'),
            b'2' if target.watch_support().write => {
                target.remove_watchpoint(self.query_thread, addr, kind.max(1))
            }
            _ => {
                self.send_empty(out);
                return Outcome::Continue;
            }
        };
        match result {
            Ok(()) => self.send_ok(out),
            Err(e) => self.send_error(&e, out),
        }
        Outcome::Continue
    }

    // -- queries -----------------------------------------------------------

    fn query(
        &mut self,
        head: u8,
        rest: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        if head == b'Q' {
            if rest == b"StartNoAckMode" {
                self.send_ok(out);
                self.no_ack_pending = true;
            } else {
                self.send_empty(out);
            }
            return Outcome::Continue;
        }
        if let Some(offer) = rest.strip_prefix(b"Supported") {
            // The client's own list, which is what decides whether a stop reply
            // may name a breakpoint kind. `swbreak+` in *our* reply says we can
            // report it; `swbreak+` in *theirs* says it will understand it.
            let offer = offer.strip_prefix(b":").unwrap_or(offer);
            for feature in offer.split(|b| *b == b';') {
                match feature {
                    b"swbreak+" => self.client_swbreak = true,
                    b"hwbreak+" => self.client_hwbreak = true,
                    _ => {}
                }
            }
            // `PacketSize` is hex, and is the size of a *packet*, so a reply
            // never needs splitting below it.
            self.send(
                b"PacketSize=1000;qXfer:features:read+;QStartNoAckMode+;swbreak+;hwbreak+;\
                  vContSupported+",
                out,
            );
            return Outcome::Continue;
        }
        if rest == b"C" {
            let mut payload = b"QC".to_vec();
            push_hex_u64(&mut payload, self.query_thread as u64 + 1);
            self.send(&payload, out);
            return Outcome::Continue;
        }
        if rest == b"fThreadInfo" {
            if target.cpu_count() == 0 {
                self.send(b"l", out);
                return Outcome::Continue;
            }
            let mut payload = vec![b'm'];
            for cpu in 0..target.cpu_count() {
                if cpu > 0 {
                    payload.push(b',');
                }
                push_hex_u64(&mut payload, cpu as u64 + 1);
            }
            self.send(&payload, out);
            return Outcome::Continue;
        }
        if rest == b"sThreadInfo" {
            self.send(b"l", out);
            return Outcome::Continue;
        }
        if let Some(id) = rest.strip_prefix(b"ThreadExtraInfo,") {
            let cpu = ThreadSel::parse(id).unwrap_or(ThreadSel::Any).cpu(0);
            let text = match (target.cpu_path(cpu), target.arch(cpu)) {
                (Ok(path), Ok(arch)) => format!("{path} ({})", arch.class.name),
                _ => {
                    self.send_error(&TargetError::NoSuchCpu, out);
                    return Outcome::Continue;
                }
            };
            let mut payload = Vec::new();
            push_hex(&mut payload, text.as_bytes());
            self.send(&payload, out);
            return Outcome::Continue;
        }
        if rest == b"Attached" {
            // `1`: the machine existed before the debugger did, so detaching
            // leaves it running rather than killing it.
            self.send(b"1", out);
            return Outcome::Continue;
        }
        if rest.starts_with(b"Symbol:") {
            self.send_ok(out);
            return Outcome::Continue;
        }
        if let Some(args) = rest.strip_prefix(b"Rcmd,") {
            return self.monitor(args, target, out);
        }
        if let Some(args) = rest.strip_prefix(b"Xfer:features:read:") {
            return self.features(args, target, out);
        }
        self.send_empty(out);
        Outcome::Continue
    }

    fn monitor(&mut self, args: &[u8], target: &mut dyn DebugTarget, out: &mut Vec<u8>) -> Outcome {
        let Some(bytes) = hex_decode(args) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let Ok(command) = String::from_utf8(bytes) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        match target.monitor(self.query_thread, &command) {
            // The reply to `qRcmd` is either `OK`, `E<xx>`, or `O`-packet
            // output followed by `OK`. Text comes back as the latter.
            Some(text) => {
                self.send_console(&text, out);
                self.send_ok(out);
            }
            None => self.send_empty(out),
        }
        Outcome::Continue
    }

    fn features(
        &mut self,
        args: &[u8],
        target: &mut dyn DebugTarget,
        out: &mut Vec<u8>,
    ) -> Outcome {
        // `<annex>:<offset>,<length>`
        let Some(colon) = args.iter().position(|b| *b == b':') else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        let (annex, range) = args.split_at(colon);
        let range = range.get(1..).unwrap_or(&[]);
        if annex != b"target.xml" {
            // Only one annex exists; anything else is genuinely absent.
            self.send(b"E00", out);
            return Outcome::Continue;
        }
        let Some((offset, length)) = Self::parse_range(range) else {
            self.send_error(&TargetError::Unsupported, out);
            return Outcome::Continue;
        };
        // The description belongs to the thread GDB is asking about. A machine
        // with two different architectures can only be described once this way;
        // the second one's registers would need a second inferior.
        let Ok(arch) = target.arch(self.query_thread) else {
            self.send_error(&TargetError::NoSuchCpu, out);
            return Outcome::Continue;
        };
        let xml = arch.target_xml();
        let bytes = xml.as_bytes();
        let offset = usize::try_from(offset).unwrap_or(usize::MAX);
        let slice = bytes.get(offset..).unwrap_or(&[]);
        let take = slice.len().min(length);
        let mut payload = Vec::with_capacity(take + 1);
        payload.push(if take < slice.len() { b'm' } else { b'l' });
        payload.extend_from_slice(slice.get(..take).unwrap_or(&[]));
        self.send(&payload, out);
        Outcome::Continue
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::gdb::arch::{Arch, RegDesc, RegType};
    use crate::host::gdb::packet::Framer;
    use crate::host::gdb::target::{TargetResult, WatchSupport};

    /// A target with no machine behind it: two "CPUs" with four bytes of state
    /// each and a 256-byte memory. Enough to drive every packet, and it makes
    /// the protocol tests independent of any CPU feature being enabled.
    #[derive(Debug)]
    struct FakeTarget {
        regs: [[u8; 4]; 2],
        mem: [u8; 256],
        breakpoints: Vec<(u64, bool)>,
        watchpoints: Vec<(usize, u64, u64)>,
        steps: usize,
        resumed: usize,
        began: usize,
    }

    static FAKE_REGS: &[RegDesc] = &[
        RegDesc {
            name: "r0",
            bytes: 2,
            offset: 0,
            ty: RegType::Int,
        },
        RegDesc {
            name: "pc",
            bytes: 2,
            offset: 2,
            ty: RegType::CodePtr,
        },
    ];

    static FAKE_CLASS: crate::core::device::DeviceClass = crate::core::device::DeviceClass {
        name: "cpu.fake",
        version: 1,
        summary: "a test target",
        properties: &[],
        construct: |_| Err(crate::Error::State(String::from("not constructible"))),
    };

    static FAKE_ARCH: Arch = Arch {
        class: &FAKE_CLASS,
        verified_version: 1,
        feature: "org.rsemu.fake",
        architecture: None,
        regs: FAKE_REGS,
        pc: 1,
        retire: None,
        computed: None,
    };

    impl FakeTarget {
        fn new() -> FakeTarget {
            let mut mem = [0u8; 256];
            for (i, byte) in mem.iter_mut().enumerate() {
                *byte = i as u8;
            }
            FakeTarget {
                regs: [[0x11, 0x22, 0x00, 0xc0], [0x33, 0x44, 0x00, 0xd0]],
                mem,
                breakpoints: Vec::new(),
                watchpoints: Vec::new(),
                steps: 0,
                resumed: 0,
                began: 0,
            }
        }
    }

    impl DebugTarget for FakeTarget {
        fn cpu_count(&self) -> usize {
            2
        }
        fn cpu_path(&self, cpu: usize) -> TargetResult<&str> {
            match cpu {
                0 => Ok("cpu0"),
                1 => Ok("cpu1"),
                _ => Err(TargetError::NoSuchCpu),
            }
        }
        fn arch(&self, cpu: usize) -> TargetResult<&'static Arch> {
            if cpu < 2 {
                Ok(&FAKE_ARCH)
            } else {
                Err(TargetError::NoSuchCpu)
            }
        }
        fn read_registers(&self, cpu: usize) -> TargetResult<Vec<u8>> {
            self.regs
                .get(cpu)
                .map(|r| r.to_vec())
                .ok_or(TargetError::NoSuchCpu)
        }
        fn write_registers(&mut self, cpu: usize, data: &[u8]) -> TargetResult<()> {
            let slot = self.regs.get_mut(cpu).ok_or(TargetError::NoSuchCpu)?;
            if data.len() != 4 {
                return Err(TargetError::NoSuchRegister);
            }
            slot.copy_from_slice(data);
            Ok(())
        }
        fn read_register(&self, cpu: usize, index: usize) -> TargetResult<Vec<u8>> {
            let slot = self.regs.get(cpu).ok_or(TargetError::NoSuchCpu)?;
            let reg = FAKE_REGS.get(index).ok_or(TargetError::NoSuchRegister)?;
            Ok(slot[reg.offset..reg.offset + reg.bytes].to_vec())
        }
        fn write_register(&mut self, cpu: usize, index: usize, data: &[u8]) -> TargetResult<()> {
            let slot = self.regs.get_mut(cpu).ok_or(TargetError::NoSuchCpu)?;
            let reg = FAKE_REGS.get(index).ok_or(TargetError::NoSuchRegister)?;
            if data.len() != reg.bytes {
                return Err(TargetError::NoSuchRegister);
            }
            slot[reg.offset..reg.offset + reg.bytes].copy_from_slice(data);
            Ok(())
        }
        fn read_memory(&self, _cpu: usize, addr: u64, dst: &mut [u8]) -> TargetResult<()> {
            let start = usize::try_from(addr).map_err(|_| TargetError::Fault)?;
            let end = start.checked_add(dst.len()).ok_or(TargetError::Fault)?;
            let src = self.mem.get(start..end).ok_or(TargetError::Fault)?;
            dst.copy_from_slice(src);
            Ok(())
        }
        fn write_memory(&mut self, _cpu: usize, addr: u64, src: &[u8]) -> TargetResult<()> {
            let start = usize::try_from(addr).map_err(|_| TargetError::Fault)?;
            let end = start.checked_add(src.len()).ok_or(TargetError::Fault)?;
            let dst = self.mem.get_mut(start..end).ok_or(TargetError::Fault)?;
            dst.copy_from_slice(src);
            Ok(())
        }
        fn add_breakpoint(&mut self, addr: u64, hardware: bool) -> TargetResult<()> {
            self.breakpoints.push((addr, hardware));
            Ok(())
        }
        fn remove_breakpoint(&mut self, addr: u64, hardware: bool) -> TargetResult<()> {
            self.breakpoints.retain(|b| *b != (addr, hardware));
            Ok(())
        }
        fn watch_support(&self) -> WatchSupport {
            WatchSupport {
                write: true,
                read: false,
                access: false,
            }
        }
        fn add_watchpoint(&mut self, cpu: usize, addr: u64, len: u64) -> TargetResult<()> {
            self.watchpoints.push((cpu, addr, len));
            Ok(())
        }
        fn remove_watchpoint(&mut self, cpu: usize, addr: u64, len: u64) -> TargetResult<()> {
            self.watchpoints.retain(|w| *w != (cpu, addr, len));
            Ok(())
        }
        fn step(&mut self, cpu: usize) -> TargetResult<Stop> {
            self.steps += 1;
            let slot = self.regs.get_mut(cpu).ok_or(TargetError::NoSuchCpu)?;
            let pc = u16::from_le_bytes([slot[2], slot[3]]).wrapping_add(1);
            slot[2..4].copy_from_slice(&pc.to_le_bytes());
            Ok(Stop {
                cpu,
                kind: StopKind::Trap,
            })
        }
        fn begin_resume(&mut self) {
            self.began += 1;
        }
        fn resume(&mut self) -> TargetResult<Option<Stop>> {
            self.resumed += 1;
            if self.resumed >= 3 {
                Ok(Some(Stop {
                    cpu: 0,
                    kind: StopKind::Breakpoint { hardware: false },
                }))
            } else {
                Ok(None)
            }
        }
        fn monitor(&mut self, cpu: usize, command: &str) -> Option<String> {
            // The CPU is echoed, because which thread a monitor command was
            // typed on is the whole reason it is a parameter.
            (command == "ping").then(|| format!("pong from {cpu}\n"))
        }
    }

    /// Send one packet and return everything the stub wrote, as text.
    fn ask(stub: &mut Stub, target: &mut FakeTarget, packet: &[u8]) -> String {
        let mut wire = Vec::new();
        frame(packet, &mut wire);
        let mut framer = Framer::new();
        let mut out = Vec::new();
        for byte in wire {
            if let Some(event) = framer.push(byte) {
                stub.on_event(event, target, &mut out);
            }
        }
        String::from_utf8_lossy(&out).into_owned()
    }

    /// The payload of the last packet in a reply, acknowledgements stripped and
    /// the wire encoding undone — so a test asserts on what GDB would see, not
    /// on how many spaces the run-length encoder folded away.
    fn payload(reply: &str) -> String {
        let start = reply.rfind('$').unwrap_or(0) + 1;
        let end = reply.rfind('#').unwrap_or(reply.len());
        let body = &reply.as_bytes()[start..end];
        let decoded = super::super::packet::decode_body(body).expect("a well-formed body");
        String::from_utf8_lossy(&decoded).into_owned()
    }

    #[test]
    fn a_packet_is_acknowledged_before_it_is_answered() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let reply = ask(&mut stub, &mut target, b"?");
        assert!(reply.starts_with('+'), "{reply}");
        assert_eq!(payload(&reply), "T05thread:1;");
    }

    #[test]
    fn negotiation_advertises_only_what_is_implemented() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let reply = payload(&ask(
            &mut stub,
            &mut target,
            b"qSupported:multiprocess+;swbreak+",
        ));
        for feature in [
            "PacketSize=",
            "qXfer:features:read+",
            "QStartNoAckMode+",
            "swbreak+",
            "vContSupported+",
        ] {
            assert!(reply.contains(feature), "{feature} missing from {reply}");
        }
        assert!(!reply.contains("multiprocess+"), "{reply}");
    }

    #[test]
    fn no_ack_mode_takes_effect_after_its_own_reply() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let reply = ask(&mut stub, &mut target, b"QStartNoAckMode");
        assert!(reply.starts_with('+'), "the request itself is still acked");
        assert_eq!(payload(&reply), "OK");
        assert!(stub.acks_disabled());
        let next = ask(&mut stub, &mut target, b"?");
        assert!(!next.starts_with('+'), "{next}");
    }

    #[test]
    fn registers_read_and_write_in_target_description_order() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(payload(&ask(&mut stub, &mut target, b"g")), "112200c0");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"p1")), "00c0");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"P1=34d0")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"g")), "112234d0");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Gdeadbeef")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"g")), "deadbeef");
        // A register that does not exist is an error, not a panic.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"p9")), "E16");
    }

    #[test]
    fn memory_reads_and_both_kinds_of_write() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(payload(&ask(&mut stub, &mut target, b"m10,4")), "10111213");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"M10,2:aabb")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"m10,4")), "aabb1213");
        // The binary form, with a byte that has to be escaped on the wire.
        let mut packet = b"X20,2:".to_vec();
        packet.extend_from_slice(&[b'#', 0x7f]);
        assert_eq!(payload(&ask(&mut stub, &mut target, &packet)), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"m20,2")), "237f");
        // The probe GDB uses to find out whether `X` works at all.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"X0,0:")), "OK");
        // Off the end of memory: an error reply, and the session survives.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"mfff0,20")), "E05");
    }

    #[test]
    fn cpus_are_threads_and_h_selects_between_them() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"qfThreadInfo")),
            "m1,2"
        );
        assert_eq!(payload(&ask(&mut stub, &mut target, b"qsThreadInfo")), "l");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"qC")), "QC1");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Hg2")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"qC")), "QC2");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"g")), "334400d0");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"T2")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"T9")), "E03");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Hg9")), "E03");
        let extra = payload(&ask(&mut stub, &mut target, b"qThreadExtraInfo,2"));
        let decoded = hex_decode(extra.as_bytes()).expect("hex");
        assert_eq!(String::from_utf8_lossy(&decoded), "cpu1 (cpu.fake)");
    }

    #[test]
    fn the_target_description_is_served_in_pieces() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let xml = FAKE_ARCH.target_xml();
        let first = payload(&ask(
            &mut stub,
            &mut target,
            b"qXfer:features:read:target.xml:0,10",
        ));
        assert!(first.starts_with('m'), "{first}");
        assert_eq!(&first[1..], &xml[..0x10]);
        let mut whole = String::new();
        let mut offset = 0usize;
        loop {
            let request = format!("qXfer:features:read:target.xml:{offset:x},20");
            let reply = payload(&ask(&mut stub, &mut target, request.as_bytes()));
            let (tag, body) = reply.split_at(1);
            whole.push_str(body);
            offset += body.len();
            if tag == "l" {
                break;
            }
        }
        assert_eq!(whole, xml);
        // An annex that does not exist is an error, not an empty document.
        assert_eq!(
            payload(&ask(
                &mut stub,
                &mut target,
                b"qXfer:features:read:threads:0,10"
            )),
            "E00"
        );
    }

    #[test]
    fn breakpoints_and_watchpoints_land_where_they_are_supported() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z0,c000,1")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z1,c010,1")), "OK");
        // `Z0` and `Z1` are the same mechanism and different stop replies, so
        // which one was asked for reaches the target.
        assert_eq!(target.breakpoints, vec![(0xc000, false), (0xc010, true)]);
        assert_eq!(payload(&ask(&mut stub, &mut target, b"z0,c000,1")), "OK");
        assert_eq!(target.breakpoints, vec![(0xc010, true)]);
        // A watchpoint carries the thread `H g` selected: its address is read
        // back through that CPU's space for as long as it is armed.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Hg2")), "OK");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z2,20,4")), "OK");
        assert_eq!(target.watchpoints, vec![(1, 0x20, 4)]);
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Hg1")), "OK");
        // Read and access watchpoints are not supported, and say so the way
        // the protocol says so: an empty reply, not an error.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z3,20,4")), "");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z4,20,4")), "");
        // Malformed point packets are refused rather than parsed halfway.
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z0,zz,1")), "E16");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z0")), "E16");
        // A condition list this stub never advertised is ignored rather than
        // refused: the breakpoint is still a breakpoint at that address.
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"Z0,c020,1;X3,010203")),
            "OK"
        );
        assert!(target.breakpoints.contains(&(0xc020, false)));
    }

    #[test]
    fn a_stop_reply_names_a_breakpoint_only_to_a_client_that_asked() {
        // The GDB manual's "Stop Reply Packets": `swbreak` and `hwbreak` are
        // sent only when the client offered them in its own `qSupported`.
        // Sending one unasked is a reason an older client may reject.
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(payload(&ask(&mut stub, &mut target, b"Z0,c000,1")), "OK");
        stub.last_stop = Stop {
            cpu: 0,
            kind: StopKind::Breakpoint { hardware: false },
        };
        assert_eq!(payload(&ask(&mut stub, &mut target, b"?")), "T05thread:1;");

        // Now negotiate, and the same stop says why it happened.
        let mut stub = Stub::new();
        assert!(
            payload(&ask(
                &mut stub,
                &mut target,
                b"qSupported:multiprocess+;swbreak+;hwbreak+"
            ))
            .contains("swbreak+")
        );
        stub.last_stop = Stop {
            cpu: 0,
            kind: StopKind::Breakpoint { hardware: false },
        };
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"?")),
            "T05thread:1;swbreak:;"
        );
        stub.last_stop = Stop {
            cpu: 0,
            kind: StopKind::Breakpoint { hardware: true },
        };
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"?")),
            "T05thread:1;hwbreak:;",
            "a `Z1` is reported as the hardware breakpoint it was"
        );
    }

    #[test]
    fn continue_runs_until_the_target_reports_a_stop() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        // Negotiate first, so the stop reply is allowed to say *why* it
        // stopped — see `a_stop_reply_names_a_breakpoint_only_to_a_client_that_asked`.
        ask(&mut stub, &mut target, b"qSupported:swbreak+;hwbreak+");
        // `c` has no reply of its own.
        assert_eq!(ask(&mut stub, &mut target, b"c"), "+");
        assert!(stub.is_running());
        assert_eq!(target.began, 1);
        let mut out = Vec::new();
        stub.drive(&mut target, &mut out);
        stub.drive(&mut target, &mut out);
        assert!(out.is_empty(), "no stop yet");
        stub.drive(&mut target, &mut out);
        assert_eq!(
            payload(&String::from_utf8_lossy(&out)),
            "T05thread:1;swbreak:;"
        );
        assert!(!stub.is_running());
    }

    #[test]
    fn ctrl_c_while_running_stops_with_sigint() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        ask(&mut stub, &mut target, b"c");
        let mut out = Vec::new();
        stub.on_event(Event::Interrupt, &mut target, &mut out);
        assert!(out.is_empty(), "the interrupt itself is silent");
        stub.drive(&mut target, &mut out);
        assert_eq!(payload(&String::from_utf8_lossy(&out)), "T02thread:1;");
        assert!(!stub.is_running());
    }

    #[test]
    fn stepping_works_through_both_spellings() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        assert_eq!(payload(&ask(&mut stub, &mut target, b"s")), "T05thread:1;");
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"vCont;s:1")),
            "T05thread:1;"
        );
        assert_eq!(target.steps, 2);
        assert_eq!(payload(&ask(&mut stub, &mut target, b"g")), "112202c0");
        assert_eq!(
            payload(&ask(&mut stub, &mut target, b"vCont?")),
            "vCont;c;C;s;S;t"
        );
        assert_eq!(ask(&mut stub, &mut target, b"vCont;c"), "+");
        assert!(stub.is_running());
    }

    #[test]
    fn a_continue_with_an_address_moves_the_program_counter() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        ask(&mut stub, &mut target, b"cbeef");
        assert_eq!(payload(&ask(&mut stub, &mut target, b"p1")), "efbe");
    }

    #[test]
    fn monitor_commands_reach_the_target() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let mut request = b"qRcmd,".to_vec();
        push_hex(&mut request, b"ping");
        let reply = ask(&mut stub, &mut target, &request);
        // "pong from 0\n": the reply carries the thread `H g` had selected.
        assert!(reply.contains("$O706f6e672066726f6d20300a#"), "{reply}");
        assert_eq!(payload(&reply), "OK");
        let mut unknown = b"qRcmd,".to_vec();
        push_hex(&mut unknown, b"nope");
        assert_eq!(payload(&ask(&mut stub, &mut target, &unknown)), "");
    }

    #[test]
    fn detach_and_kill_are_distinguishable() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let mut wire = Vec::new();
        frame(b"D", &mut wire);
        let mut framer = Framer::new();
        let mut out = Vec::new();
        let mut outcome = Outcome::Continue;
        for byte in wire {
            if let Some(event) = framer.push(byte) {
                outcome = stub.on_event(event, &mut target, &mut out);
            }
        }
        assert_eq!(outcome, Outcome::Detach);
        assert_eq!(payload(&String::from_utf8_lossy(&out)), "OK");

        let mut kill = Vec::new();
        frame(b"k", &mut kill);
        let mut out = Vec::new();
        let mut outcome = Outcome::Continue;
        for byte in kill {
            if let Some(event) = framer.push(byte) {
                outcome = stub.on_event(event, &mut target, &mut out);
            }
        }
        assert_eq!(outcome, Outcome::Kill);
        assert_eq!(out, b"+", "`k` has no reply");
    }

    #[test]
    fn an_unknown_packet_gets_the_empty_reply() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        for packet in [
            &b"vFile:open:2f746d70,0,0"[..],
            b"qTStatus",
            b"qOffsets",
            b"\x7f\xff",
        ] {
            assert_eq!(
                payload(&ask(&mut stub, &mut target, packet)),
                "",
                "{packet:?}"
            );
        }
    }

    #[test]
    fn a_corrupt_packet_is_refused_and_the_good_one_after_it_is_answered() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let mut framer = Framer::new();
        let mut out = Vec::new();
        for byte in b"$?#00" {
            if let Some(event) = framer.push(*byte) {
                stub.on_event(event, &mut target, &mut out);
            }
        }
        assert_eq!(out, b"-");
        out.clear();
        let mut good = Vec::new();
        frame(b"?", &mut good);
        for byte in good {
            if let Some(event) = framer.push(byte) {
                stub.on_event(event, &mut target, &mut out);
            }
        }
        assert_eq!(payload(&String::from_utf8_lossy(&out)), "T05thread:1;");
    }

    #[test]
    fn arbitrary_bytes_from_the_wire_never_panic() {
        // The socket is untrusted input and this is the parser on it. A
        // deterministic pseudo-random stream stands in for a `fuzz/` target,
        // which cannot live in the same file as the code it fuzzes: the point
        // is that no sequence of bytes reaches an index, a slice or a
        // conversion that can abort.
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let mut framer = Framer::new();
        let mut out = Vec::new();
        // xorshift64*, seeded so a failure is reproducible.
        let mut state: u64 = 0x2545_f491_4f6c_dd1d;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        // A mixture of pure noise and plausible packet fragments, because pure
        // noise almost never produces a valid checksum and would only ever
        // exercise the rejection path.
        let seeds: &[&[u8]] = &[
            b"$m",
            b"$M",
            b"$X",
            b"$Z",
            b"$z",
            b"$q",
            b"$Q",
            b"$v",
            b"$H",
            b"$p",
            b"$P",
            b"$G",
            b"#",
            b",",
            b":",
            b";",
            b"*",
            b"}",
            b"ffffffffffffffffffff",
        ];
        for i in 0..200_000u32 {
            let r = next();
            if i % 5 == 0 {
                let seed = seeds[(r as usize) % seeds.len()];
                for byte in seed {
                    if let Some(event) = framer.push(*byte) {
                        stub.on_event(event, &mut target, &mut out);
                    }
                }
            } else if let Some(event) = framer.push(r as u8) {
                stub.on_event(event, &mut target, &mut out);
            }
            out.clear();
        }
    }

    #[test]
    fn a_negative_acknowledgement_resends_the_last_reply() {
        let (mut stub, mut target) = (Stub::new(), FakeTarget::new());
        let first = ask(&mut stub, &mut target, b"?");
        let mut out = Vec::new();
        stub.on_event(Event::Nak, &mut target, &mut out);
        assert_eq!(
            String::from_utf8_lossy(&out),
            first.trim_start_matches('+').to_string()
        );
    }
}
