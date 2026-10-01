//! A debugging tap that gives a Linux kernel with a stubbed-out `printk` its
//! log back, on a character port.
//!
//! Production kernels are sometimes built with `printk` reduced to a stub that
//! returns without formatting anything — the Alphard navi's two kernels both
//! are, and neither registers a console either, so the unit prints nothing
//! anywhere. The formatting code is still there: `vprintk` is intact and still
//! fills the kernel's log ring. This device puts the two back together.
//!
//! **It changes the guest, on purpose.** Once the stub appears in guest
//! memory (and, optionally, once a "ready" word says the kernel's data is in
//! place), it writes a short trampoline into a function the board never calls
//! and points the stub at it; the trampoline turns `printk(fmt, ...)` into
//! `vprintk(fmt, args)`. From then on it copies whatever the kernel appends to
//! its log ring to `port`. It is a bring-up aid, not a model of anything on
//! the board, and a machine that wants the firmware untouched leaves it out.
//!
//! # Configuration
//!
//! Every address is the kernel image's own, read off its symbol table:
//! `stub` is `printk`, `vprintk` its worker, `tramp` the scratch site (a
//! function this board never calls, at least 32 bytes long), `log-buf` and
//! `log-end` the ring and its write index, `log-len` the ring's size. Physical
//! addresses are what the device writes through; the `-va` forms are where
//! the kernel runs them, which the branch offsets need, and default to the
//! physical ones for a kernel mapped one to one.
//!
//! `ready`, when given, holds the patch back until that word is nonzero — or,
//! with `ready-value`, until it holds that value. A kernel that is checked
//! before it is started — the navi's U-Boot hashes the image it boots — must
//! not be touched until it is running, and a word in its data section that
//! only the running kernel writes (its log's write index: a few things call
//! `vprintk` directly even with `printk` stubbed) is the signal.
//!
//! Any reset disarms the tap. After a warm reboot into a different kernel the
//! old one's addresses belong to someone else; the tap re-arms only when its
//! own stub appears again.
//!
//! # The stub this recognises
//!
//! `push {r0-r3}; mvn r0, #0x80000000; ...`: the first two words of an ARM
//! `printk` compiled to return without printing, as both of the navi's kernels
//! have it. A `printk` that does not start that way is left alone.
//!
//! # Sources
//!
//! The ARM Architecture Reference Manual for the encodings, and the guest's own
//! symbol tables for the addresses. No kernel source.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "linux.printk";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The first two words of the stubbed `printk`.
const STUB: [u32; 2] = [0xe92d_000f, 0xe3e0_0102];

/// An ARM `B`/`BL` from `from` to `to`.
fn branch(from: u32, to: u32, link: bool) -> u32 {
    let off = (to.wrapping_sub(from.wrapping_add(8)) as i32) >> 2;
    (if link { 0xeb00_0000 } else { 0xea00_0000 }) | (off as u32 & 0x00ff_ffff)
}

/// The trampoline: `printk(fmt, ...)` as `vprintk(fmt, va_list)`.
fn trampoline(at: u32, vprintk: u32) -> [u32; 8] {
    [
        0xe92d_000f, // push {r0-r3}: the variadic arguments, contiguous
        0xe52d_e004, // push {lr}
        0xe28d_1008, // add r1, sp, #8: va_list = &arguments after fmt
        0xe59d_0004, // ldr r0, [sp, #4]: fmt
        branch(at + 16, vprintk, true),
        0xe49d_e004, // pop {lr}
        0xe28d_d010, // add sp, sp, #16
        0xe12f_ff1e, // bx lr
    ]
}

#[derive(Debug, Clone, Copy)]
struct Layout {
    stub: u64,
    stub_va: u32,
    vprintk: u32,
    tramp: u64,
    tramp_va: u32,
    log_buf: u64,
    log_end: u64,
    log_len: u32,
    /// A word to wait for, and the value it must hold (`None`: nonzero).
    ready: Option<(u64, Option<u32>)>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct State {
    patched: bool,
    /// The log index already copied out.
    pos: u32,
}

/// The tap.
pub struct LinuxPrintk {
    layout: Layout,
    port: Arc<dyn CharDevice>,
    space: Mutex<Option<Weak<AddressSpace>>>,
    state: Mutex<State>,
}

impl fmt::Debug for LinuxPrintk {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LinuxPrintk")
            .field("stub", &self.layout.stub)
            .finish_non_exhaustive()
    }
}

impl LinuxPrintk {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property.
    pub fn new(props: &Props) -> Result<LinuxPrintk> {
        let mut r = props.reader();
        let port = r.require_str("port")?.to_string();
        let word = |v: u64, name: &str| -> Result<u32> {
            u32::try_from(v).map_err(|_| {
                Error::Property(alloc::format!("linux.printk `{name}` is a 32-bit address"))
            })
        };
        let stub = r.require_range("stub", 0..=u64::from(u32::MAX))?;
        let stub_va = r.or_range("stub-va", stub, 0..=u64::from(u32::MAX))?;
        let vprintk = r.require_range("vprintk", 0..=u64::from(u32::MAX))?;
        let tramp = r.require_range("tramp", 0..=u64::from(u32::MAX))?;
        let tramp_va = r.or_range("tramp-va", tramp, 0..=u64::from(u32::MAX))?;
        let log_buf = r.require_range("log-buf", 0..=u64::from(u32::MAX))?;
        let log_end = r.require_range("log-end", 0..=u64::from(u32::MAX))?;
        let log_len: u64 = r.or_range("log-len", 0x4000, 1..=0x0100_0000)?;
        let ready: Option<u64> = r.optional("ready")?;
        let ready_value: Option<u64> = r.optional("ready-value")?;
        r.finish()?;
        if !log_len.is_power_of_two() {
            return Err(Error::Property(String::from(
                "linux.printk `log-len` is the kernel's ring size, a power of two",
            )));
        }
        let ready = match (ready, ready_value) {
            (Some(a), Some(v)) => Some((a, Some(word(v, "ready-value")?))),
            (Some(a), None) => Some((a, None)),
            (None, None) => None,
            (None, Some(_)) => {
                return Err(Error::Property(String::from(
                    "linux.printk `ready-value` needs a `ready` address to compare",
                )));
            }
        };
        let layout = Layout {
            stub,
            stub_va: word(stub_va, "stub-va")?,
            vprintk: word(vprintk, "vprintk")?,
            tramp,
            tramp_va: word(tramp_va, "tramp-va")?,
            log_buf,
            log_end,
            log_len: word(log_len, "log-len")?,
            ready,
        };
        let port = ports::attach(props, &port)? as Arc<dyn CharDevice>;
        Ok(LinuxPrintk::with_port(layout, port))
    }

    fn with_port(layout: Layout, port: Arc<dyn CharDevice>) -> LinuxPrintk {
        LinuxPrintk {
            layout,
            port,
            space: Mutex::with_rank(LockRank::LEAF, None),
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
        }
    }

    fn space(&self) -> Option<Arc<AddressSpace>> {
        self.space.lock().as_ref().and_then(Weak::upgrade)
    }

    /// One look: patch if the stub has appeared, re-arm if it has gone or
    /// come back, and copy out what the kernel logged since the last look.
    pub fn poll(&self) {
        let Some(space) = self.space() else {
            return;
        };
        let rd = |a: u64| -> u32 {
            space
                .read(a, Width::U32, MemAttrs::DEBUG)
                .map_or(0, |v| v as u32)
        };
        let wr = |a: u64, v: u32| {
            let _ = space.write(a, Width::U32, u64::from(v), MemAttrs::DEBUG);
        };
        let l = self.layout;
        let ours = branch(l.stub_va, l.tramp_va, false);
        let at_stub = rd(l.stub);
        let mut s = self.state.lock();
        if s.patched && at_stub != ours {
            // The kernel went away: a reset, or another kernel in this RAM.
            *s = State::default();
        }
        if !s.patched {
            let stub_ok = at_stub == STUB[0] && rd(l.stub + 4) == STUB[1];
            let ready = l.ready.is_none_or(|(a, v)| match v {
                Some(v) => rd(a) == v,
                None => rd(a) != 0,
            });
            if !(stub_ok && ready) {
                return;
            }
            for (i, w) in trampoline(l.tramp_va, l.vprintk).iter().enumerate() {
                wr(l.tramp + 4 * i as u64, *w);
            }
            wr(l.stub, ours);
            s.patched = true;
            // From the oldest byte still in the ring: a few things call
            // vprintk directly, so a stubbed kernel has usually logged
            // something already.
            let end = rd(l.log_end);
            s.pos = end.wrapping_sub(end.min(l.log_len));
        }
        let end = rd(l.log_end);
        if end.wrapping_sub(s.pos) > l.log_len {
            s.pos = end.wrapping_sub(l.log_len);
        }
        let mut out: Vec<u8> = Vec::new();
        while s.pos != end {
            let a = l.log_buf + u64::from(s.pos & (l.log_len - 1));
            let byte = space
                .read(a, Width::U8, MemAttrs::DEBUG)
                .map_or(0, |v| v as u8);
            if byte != 0 {
                out.push(byte);
            }
            s.pos = s.pos.wrapping_add(1);
        }
        drop(s);
        if !out.is_empty() {
            self.port.write(&out);
        }
    }
}

/// The `linux.printk` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a debugging tap: un-stubs a Linux kernel's printk and streams its log ring to a port",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: true,
            summary: "the character port the log goes to",
        },
        PropertySpec {
            name: "stub",
            kind: ValueKind::Uint,
            required: true,
            summary: "physical address of the stubbed printk",
        },
        PropertySpec {
            name: "stub-va",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the kernel runs printk (default: stub)",
        },
        PropertySpec {
            name: "vprintk",
            kind: ValueKind::Uint,
            required: true,
            summary: "virtual address of vprintk",
        },
        PropertySpec {
            name: "tramp",
            kind: ValueKind::Uint,
            required: true,
            summary: "physical address of 32 bytes the kernel never runs",
        },
        PropertySpec {
            name: "tramp-va",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the kernel would run them (default: tramp)",
        },
        PropertySpec {
            name: "log-buf",
            kind: ValueKind::Uint,
            required: true,
            summary: "physical address of the log ring",
        },
        PropertySpec {
            name: "log-end",
            kind: ValueKind::Uint,
            required: true,
            summary: "physical address of the ring's write index",
        },
        PropertySpec {
            name: "log-len",
            kind: ValueKind::Uint,
            required: false,
            summary: "the ring's size (default 0x4000)",
        },
        PropertySpec {
            name: "ready",
            kind: ValueKind::Uint,
            required: false,
            summary: "a word that must be nonzero before patching",
        },
        PropertySpec {
            name: "ready-value",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value `ready` must hold instead",
        },
    ],
    construct: |props| Ok(Box::new(LinuxPrintk::new(props)?)),
};

impl Device for LinuxPrintk {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Disarm on any reset: a warm reboot may bring a different kernel,
        // and the old one's log ring is then somebody else's memory. The
        // stub reappearing is what re-arms the tap.
        *self.state.lock() = State::default();
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.poll();
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.state.lock();
        w.write_bool(s.patched)?;
        w.write_u32(s.pos)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let s = State {
            patched: r.read_bool()?,
            pos: r.read_u32()?,
        };
        *self.state.lock() = s;
        Ok(())
    }
}

impl Instance for LinuxPrintk {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "linux.printk patches and reads guest memory and needs `space = mem`",
            ),
        })?;
        *self.space.lock() = Some(Arc::downgrade(space));
        Ok(())
    }
}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(LinuxPrintk::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    let mut s =
        ClassSchema::new(CLASS_NAME).prop(PropSchema::new("port", ValueKind::Str).required());
    for (name, required) in [
        ("stub", true),
        ("stub-va", false),
        ("vprintk", true),
        ("tramp", true),
        ("tramp-va", false),
        ("log-buf", true),
        ("log-end", true),
        ("log-len", false),
        ("ready", false),
        ("ready-value", false),
    ] {
        let p = PropSchema::new(name, ValueKind::Uint);
        s = s.prop(if required { p.required() } else { p });
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::host::chardev::CharPort;

    fn rig(ready: Option<(u64, Option<u32>)>) -> (LinuxPrintk, Arc<AddressSpace>, Arc<CharPort>) {
        let space = Arc::new(AddressSpace::new("mem", 32));
        let ram = Arc::new(RamStore::new(0x10000));
        space.topology().map(Region::ram("ram", ram), 0).unwrap();
        let port = Arc::new(CharPort::new());
        let layout = Layout {
            stub: 0x1000,
            stub_va: 0x1000,
            vprintk: 0x2000,
            tramp: 0x3000,
            tramp_va: 0x3000,
            log_buf: 0x8000,
            log_end: 0x7000,
            log_len: 0x100,
            ready,
        };
        let tap = LinuxPrintk::with_port(layout, Arc::clone(&port) as Arc<dyn CharDevice>);
        *tap.space.lock() = Some(Arc::downgrade(&space));
        (tap, space, port)
    }

    fn w(space: &AddressSpace, a: u64, v: u32) {
        space
            .write(a, Width::U32, u64::from(v), MemAttrs::DEBUG)
            .unwrap();
    }

    fn r(space: &AddressSpace, a: u64) -> u32 {
        space.read(a, Width::U32, MemAttrs::DEBUG).unwrap() as u32
    }

    fn log(space: &AddressSpace, end: &mut u32, text: &[u8]) {
        for b in text {
            space
                .write(
                    0x8000 + u64::from(*end & 0xff),
                    Width::U8,
                    u64::from(*b),
                    MemAttrs::DEBUG,
                )
                .unwrap();
            *end += 1;
        }
        w(space, 0x7000, *end);
    }

    #[test]
    fn the_stub_is_patched_once_it_appears_and_the_ring_streams_out() {
        let (tap, space, port) = rig(None);
        tap.poll();
        assert_eq!(r(&space, 0x1000), 0, "nothing to patch yet");
        w(&space, 0x1000, STUB[0]);
        w(&space, 0x1004, STUB[1]);
        tap.poll();
        assert_eq!(r(&space, 0x1000), branch(0x1000, 0x3000, false));
        assert_eq!(
            r(&space, 0x3010),
            branch(0x3010, 0x2000, true),
            "bl vprintk"
        );
        let mut end = 0;
        log(&space, &mut end, b"<6>hello\n");
        tap.poll();
        assert_eq!(port.drain(), b"<6>hello\n");
        tap.poll();
        assert!(port.drain().is_empty(), "nothing new, nothing sent");
    }

    #[test]
    fn a_ready_word_holds_the_patch_back() {
        let (tap, space, _) = rig(Some((0x7100, Some(0x4000))));
        w(&space, 0x1000, STUB[0]);
        w(&space, 0x1004, STUB[1]);
        tap.poll();
        assert_eq!(r(&space, 0x1000), STUB[0], "not ready");
        w(&space, 0x7100, 0x4000);
        tap.poll();
        assert_eq!(r(&space, 0x1000), branch(0x1000, 0x3000, false));
    }

    #[test]
    fn a_reloaded_stub_is_patched_again() {
        let (tap, space, _) = rig(None);
        w(&space, 0x1000, STUB[0]);
        w(&space, 0x1004, STUB[1]);
        tap.poll();
        assert!(tap.state.lock().patched);
        w(&space, 0x1000, STUB[0]);
        tap.poll();
        assert_eq!(
            r(&space, 0x1000),
            branch(0x1000, 0x3000, false),
            "the kernel came back"
        );
    }

    #[test]
    fn a_ready_word_alone_waits_for_nonzero_and_a_reset_disarms() {
        let (tap, space, _) = rig(Some((0x7000, None)));
        w(&space, 0x1000, STUB[0]);
        w(&space, 0x1004, STUB[1]);
        tap.poll();
        assert_eq!(
            r(&space, 0x1000),
            STUB[0],
            "the kernel has logged nothing yet"
        );
        w(&space, 0x7000, 3);
        tap.poll();
        assert!(tap.state.lock().patched);
        tap.reset(ResetKind::Warm);
        tap.poll();
        assert!(
            !tap.state.lock().patched,
            "the patched branch is not the stub: stays down"
        );
    }
}
