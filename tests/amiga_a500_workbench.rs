//! A person at a real Workbench: pointer, double-click, Shell, typing.
//!
//! # Why this file exists
//!
//! `tests/amiga_a500_kickstart.rs` proves that Kickstart 2.04 and 1.3 boot
//! their Workbench disks to a desktop. That is a picture, not a computer
//! anybody can use. This file uses it: it moves the pointer onto the disk
//! icon, double-clicks it, double-clicks the Shell in the window that opens,
//! types a command and reads the answer off the screen.
//!
//! # The seam, and nothing past it
//!
//! Every input goes in where a VNC client's does: an [`InputEvent`] — an X11
//! keysym, or an absolute pointer position in framebuffer pixels with RFB's
//! button mask — posted on the frontend's `input:vnc` channel of the machine's
//! [`Recorder`], delivered at a round boundary into a [`Feed`] that fans it
//! out to [`AmigaKeyboardSink`] and [`AmigaMouseSink`]. That is `VncSession`'s
//! own wiring in `rsemu run --vnc`, minus the socket. From there the keymap,
//! the keyboard's KCLK/KDAT protocol, CIA-A's shift register, the mouse's
//! quadrature pins, Denise's counters and CIA-A's `PA6` all stand between the
//! person and Kickstart. Nothing here writes guest memory or a register; the
//! only things read back are the pictures Denise draws.
//!
//! # Time
//!
//! Every event is posted between two `run_for` calls whose lengths are
//! constants, so it lands at a fixed virtual instant, and the 2.04 test
//! replays its own recording to prove it: the same log, no live input, the
//! same pictures and the same state hash.
//!
//! # Putting the pointer somewhere
//!
//! A mouse is relative and a VNC pointer is absolute; [`AmigaMouseSink`] sends
//! the difference between two positions. Black-box, both Kickstarts move the
//! pointer one high-resolution pixel across and one interlaced line down per
//! count at their default preferences, with no acceleration at the rates the
//! mouse model produces — which is one framebuffer pixel each way, so host and
//! guest agree on distance. What they cannot agree on is *where*: the sink's
//! first event only establishes a position, and Intuition stops the pointer
//! at the screen's edges (and, dragging a window, wherever the window would
//! leave the screen) while the host goes on. So each test first **homes** the
//! pointer: it establishes the host at the framebuffer's bottom right and
//! sweeps to the screen's top-left corner, further than the screen is big, so
//! Intuition pins the pointer there wherever it was. From then on a host
//! position *is* the guest pointer's position, and a placement is checked by
//! finding the pointer's own red in the picture where it was sent.
//!
//! # What is in this file, and what is not
//!
//! **No byte of any ROM or disk.** Both are the user's, read in place from
//! `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_ADF_DIR`; without them every test
//! prints why and passes. What is asserted is this emulator's rendering: frame
//! hashes at fixed instants, each looked at before it was accepted and
//! described beside its constant, and one check that needs no golden — the
//! glyphs `echo` printed are the glyphs that were typed. `RSEMU_AMIGA_FRAME_DIR`
//! receives a PNG of every frame looked at, in a build with `display-png`. Run
//! with `--release`: each test is a minute or more of virtual time.
//!
//! No Amiga emulator source and no AROS source was consulted (`ROADMAP.md`
//! §1); the input path is from the *Amiga Hardware Reference Manual*, cited in
//! `src/dev/amiga/keyboard.rs`, `mouse.rs` and `src/host/input/amiga.rs`.

#![cfg(all(feature = "machine-amiga-a500", feature = "media-kickstart"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::record::{Channel, InputLog, Mode, Recorder};
use rsemu::cpu::m68k::M68k;
use rsemu::host::display::amiga::{DeniseScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::host::input::amiga::{AmigaKeyboardSink, AmigaMouseSink};
use rsemu::host::input::{self, Feed, InputEvent, Keysym};
use rsemu::machine::{Machine, catalog};

/// A booted A500 and a person's hands on it.
struct Desk {
    machine: Machine,
    cpu: Arc<M68k>,
    scanout: DeniseScanout,
    recorder: Arc<Recorder>,
    /// The frontend's channel, `input:vnc`.
    channel: Channel,
    /// For file names and messages: which run.
    tag: &'static str,
}

/// The user's file `name` from the directory `var` names, or `None` having
/// said why.
fn user_file(var: &str, name: &str, what: &str) -> Option<std::path::PathBuf> {
    let Ok(dir) = std::env::var(var) else {
        println!("amiga-a500: set {var} to an Amiga Forever `{what}` directory to run this.");
        return None;
    };
    let path = std::path::Path::new(&dir).join(name);
    if !path.exists() {
        println!("amiga-a500: {} is not there; skipped", path.display());
        return None;
    }
    Some(path)
}

/// The shipped A500 around the user's Kickstart `rom` with `adf` in DF0,
/// sealed against `recorder` — recording, or replaying a log.
fn desk(rom: &str, adf: &str, tag: &'static str, recorder: Arc<Recorder>) -> Option<Desk> {
    let rom_path = user_file("RSEMU_AMIGA_ROM_DIR", rom, "Shared/rom")?;
    let adf_path = user_file("RSEMU_AMIGA_ADF_DIR", adf, "Shared/adf")?;
    let image = rsemu::host::media::kickstart::open(&rom_path.to_string_lossy())
        .unwrap_or_else(|e| panic!("{}: {e}", rom_path.display()));
    let disk = std::fs::read(&adf_path).unwrap_or_else(|e| panic!("{}: {e}", adf_path.display()));
    let size = image.bytes.len();

    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    capture::install(&mut options).expect("a capture table");
    options
        .resolve
        .params
        .push(("kickstart-size".to_string(), format!("{}K", size / 1024)));
    options.realize.media.insert("kickstart", image.bytes);
    options.realize.media.insert("df0", disk);
    // What `rsemu run --record-input` does: the recorder goes in before the
    // build and realize wires the keyboard's and mouse's doors to it.
    options.realize.recorder = Some(Arc::clone(&recorder));
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a500")
        .expect("this build ships amiga-a500")
        .source;
    let machine = rsemu::machine::build("amiga-a500", source, &registry, &options)
        .unwrap_or_else(|e| panic!("{rom}: the board does not realize: {e}"));

    // And what `vnc_session` does once the machine exists: a feed with a sink
    // for each input the board opened, on the frontend's own channel.
    let hosts = &options.realize.hosts;
    let feed = Arc::new(Feed::new());
    feed.attach(Arc::new(
        AmigaKeyboardSink::open(hosts).expect("the board has a keyboard"),
    ));
    feed.attach(Arc::new(
        AmigaMouseSink::open(hosts).expect("the board has a mouse"),
    ));
    let channel = input::channel(input::DEFAULT_STREAM);
    recorder
        .register(channel.clone(), input::sink(&feed))
        .expect("the channel list is open until the first round");

    let cpu = cores.last().expect("the binding captured the processor");
    let scanout = capture::take(hosts, &machine).expect("a Denise");
    Some(Desk {
        machine,
        cpu,
        scanout,
        recorder,
        channel,
        tag,
    })
}

/// FNV-1a over the captured pixels, as `amiga_a500_kickstart.rs` hashes.
fn frame_hash(surface: &Surface) -> u64 {
    surface
        .pixels()
        .iter()
        .fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        })
}

/// How long a finger stays on a key or a button, and off it between two:
/// three PAL fields. Whatever reads a button samples it at least once a
/// field, and both Kickstarts' double-click and key-repeat delays are far
/// longer.
const HOLD_MS: u64 = 60;

/// Where the screen's top-left pixel is in the framebuffer: Denise's picture
/// starts 64 low-resolution pixels left of the standard `DIWSTRT` and a line
/// above it, two framebuffer pixels a low-resolution pixel and two rows a
/// line (`src/dev/amiga/denise.rs`).
const SCREEN_LEFT: u32 = 130;
const SCREEN_TOP: u32 = 30;

impl Desk {
    /// Let `ms` of virtual time pass.
    fn run_ms(&mut self, ms: u64) {
        self.machine
            .run_for(GlobalTime::from_nanos(ms * 1_000_000))
            .expect("it runs");
    }

    /// Post `event` as a VNC session would. Replaying, the log has it.
    fn post(&self, event: InputEvent) {
        if self.recorder.mode() == Mode::Record {
            self.recorder
                .post(&self.channel, &event.encode())
                .expect("a registered channel");
        }
    }

    fn pointer(&self, x: u32, y: u32, buttons: u8) {
        self.post(InputEvent::Pointer { x, y, buttons });
    }

    /// Pin the guest's pointer to the screen's top-left corner and leave the
    /// host there too. See the module docs.
    fn home(&mut self) {
        self.pointer(799, 567, 0);
        self.run_ms(20);
        self.pointer(SCREEN_LEFT, SCREEN_TOP, 0);
        // 669 and 537 counts at 5 000 a second.
        self.run_ms(300);
    }

    /// Move to `(x, y)` and give the pointer time to get there.
    fn move_to(&mut self, (x, y): (u32, u32)) {
        self.pointer(x, y, 0);
        self.run_ms(300);
    }

    /// Click the left button at `(x, y)`, where the pointer already is.
    fn click(&mut self, (x, y): (u32, u32)) {
        self.pointer(x, y, 1);
        self.run_ms(HOLD_MS);
        self.pointer(x, y, 0);
        self.run_ms(HOLD_MS);
    }

    fn double_click(&mut self, at: (u32, u32)) {
        self.click(at);
        self.click(at);
    }

    /// Type `text` a key at a time; `\n` is Return. A shifted character is
    /// sent as its keysym alone, as a client that does not report its shift
    /// does, so the keymap has to supply the shift.
    fn type_text(&mut self, text: &str) {
        for b in text.bytes() {
            let keysym = if b == b'\n' {
                Keysym::RETURN
            } else {
                Keysym::from_ascii(b)
            };
            self.post(InputEvent::Key { keysym, down: true });
            self.run_ms(HOLD_MS);
            self.post(InputEvent::Key {
                keysym,
                down: false,
            });
            self.run_ms(HOLD_MS);
        }
    }

    /// The picture now, saved as `<tag>-<name>.png` if frames are wanted.
    fn look(&self, name: &str) -> Surface {
        let info = self.scanout.info();
        let mut surface = Surface::new(PixelFormat::RGB888, info.width, info.height);
        self.scanout.capture(&mut surface);
        if let Ok(dir) = std::env::var("RSEMU_AMIGA_FRAME_DIR") {
            #[cfg(feature = "display-png")]
            {
                let png = rsemu::host::display::png::encode(&surface).expect("a PNG");
                let file = format!("{}-{name}.png", self.tag);
                std::fs::write(std::path::Path::new(&dir).join(file), png)
                    .expect("the frame directory is writable");
            }
            #[cfg(not(feature = "display-png"))]
            let _ = dir;
        }
        println!("{} {name}: frame {:#018x}", self.tag, frame_hash(&surface));
        surface
    }

    /// Look, and hold the picture to its golden.
    fn expect(&self, name: &str, golden: u64) -> Surface {
        let surface = self.look(name);
        assert_eq!(
            frame_hash(&surface),
            golden,
            "{} {name}: the picture moved; look at it (RSEMU_AMIGA_FRAME_DIR) before \
             accepting the new hash",
            self.tag
        );
        surface
    }

    /// Where the pointer's `red` starts near `at`, which is where it should
    /// be `offset` from the hot spot when the pointer is at `at`.
    fn pointer_is_at(&self, name: &str, red: [u8; 3], offset: (i32, i32), at: (u32, u32)) {
        let surface = self.look(name);
        let want = (
            at.0.wrapping_add_signed(offset.0),
            at.1.wrapping_add_signed(offset.1),
        );
        assert_eq!(
            find_near(&surface, red, at),
            Some(want),
            "{} {name}: the pointer is where it was sent",
            self.tag
        );
    }

    /// The machine is still a machine: running, and nothing faulted.
    fn healthy(&self) {
        assert!(
            !self.cpu.is_halted(),
            "{}: the 68000 double-faulted",
            self.tag
        );
        assert_eq!(
            self.cpu.bus_faults().0,
            0,
            "{}: an access faulted",
            self.tag
        );
    }
}

/// The top-left-most pixel of `rgb` in a 48-pixel square around `at`: where
/// the pointer's red starts, if the pointer is there.
fn find_near(surface: &Surface, rgb: [u8; 3], at: (u32, u32)) -> Option<(u32, u32)> {
    for y in at.1.saturating_sub(8)..at.1 + 40 {
        for x in at.0.saturating_sub(8)..at.0 + 40 {
            if surface.get(x, y) == Some(rgb) {
                return Some((x, y));
            }
        }
    }
    None
}

/// Topaz 8 in a console window: eight pixels a character across, sixteen
/// rows (eight lines) a line of text.
const CHAR_W: u32 = 8;
const LINE_H: u32 = 16;

/// Whether the `len` characters at text position `a` — `(column, line)` —
/// are the same pixels as those at `b`, in a console whose first character
/// cell is at framebuffer `origin`, and are not blank. Nothing is assumed
/// about the font but its cell size.
fn same_text(
    surface: &Surface,
    origin: (u32, u32),
    a: (u32, u32),
    b: (u32, u32),
    len: u32,
) -> bool {
    let pixel = |(col, line): (u32, u32), dx: u32, dy: u32| {
        surface.get(origin.0 + col * CHAR_W + dx, origin.1 + line * LINE_H + dy)
    };
    let paper = pixel(a, 0, 0);
    let mut ink = false;
    for dy in 0..LINE_H {
        for dx in 0..len * CHAR_W {
            let pa = pixel(a, dx, dy);
            if pa != pixel(b, dx, dy) {
                return false;
            }
            ink |= pa != paper;
        }
    }
    ink
}

// ---------------------------------------------------------------------------
// Kickstart 2.04 + Workbench 2.04
// ---------------------------------------------------------------------------

/// The Workbench2.0 disk icon on the 2.04 desktop, and the Shell icon in the
/// window it opens, in framebuffer pixels.
const WB2_DISK_ICON: (u32, u32) = (188, 178);
const WB2_SHELL_ICON: (u32, u32) = (254, 112);
/// The 2.04 pointer's red, and where the red starts from the hot spot.
const WB2_POINTER_RED: [u8; 3] = [0xee, 0x44, 0x44];
const WB2_POINTER_RED_AT: (i32, i32) = (-2, 0);
/// The AmigaShell window's first character cell.
const WB2_SHELL_TEXT: (u32, u32) = (134, 152);

/// The whole 2.04 session, live or replayed. Returns the state hash at its
/// end.
fn workbench_2_04_session(d: &mut Desk) -> u64 {
    // 1. The desktop, as `amiga_a500_kickstart.rs` has it at the same instant.
    d.run_ms(45_000);
    d.healthy();
    d.expect("1-desktop", GOLDEN_204_DESKTOP);

    // 2. The pointer onto the disk icon, where it lands, and a double-click.
    d.home();
    d.move_to(WB2_DISK_ICON);
    d.pointer_is_at(
        "2-on-disk-icon",
        WB2_POINTER_RED,
        WB2_POINTER_RED_AT,
        WB2_DISK_ICON,
    );
    d.double_click(WB2_DISK_ICON);
    d.run_ms(5_000);
    d.expect("2-disk-window", GOLDEN_204_DISK_WINDOW);

    // 3. The Shell, out of that window.
    d.move_to(WB2_SHELL_ICON);
    d.double_click(WB2_SHELL_ICON);
    d.run_ms(6_000);
    d.expect("3-shell", GOLDEN_204_SHELL);

    // 4. Typing into it.
    d.type_text("echo hello\n");
    d.run_ms(2_000);
    let echoed = d.expect("4-echo-hello", GOLDEN_204_ECHO);
    // `1.Workbench2.0:> echo ` is twenty-two characters: the five after them
    // on the first line are the five on the second.
    assert!(
        same_text(&echoed, WB2_SHELL_TEXT, (22, 0), (0, 1), 5),
        "the shell printed `hello` under the command that asked for it"
    );
    d.healthy();
    d.machine.state_hash().expect("deterministic mode hashes")
}

/// Steps 1-4 on Kickstart 2.04, then the recording replayed with nobody at
/// the keyboard: the same pictures and the same machine.
#[test]
fn kickstart_2_04_a_person_opens_the_disk_starts_a_shell_and_types() {
    let rom = "amiga-os-204.rom";
    let adf = "amiga-os-204-workbench.adf";
    let Some(mut d) = desk(rom, adf, "wb204", Arc::new(Recorder::recording())) else {
        return;
    };
    let live = workbench_2_04_session(&mut d);
    let at = d.machine.now();
    let bytes = d.recorder.log().encode().expect("a recording encodes");
    drop(d);

    let log = InputLog::decode(&bytes).expect("and decodes");
    let replay = Arc::new(Recorder::replaying(log));
    let mut again = desk(rom, adf, "wb204-replay", Arc::clone(&replay))
        .expect("the files were there a moment ago");
    let replayed = workbench_2_04_session(&mut again);
    assert_eq!(again.machine.now(), at, "the same instant");
    assert_eq!(replayed, live, "the same machine, bit for bit");
}

// ---------------------------------------------------------------------------
// Kickstart 1.3 + Workbench 1.3
// ---------------------------------------------------------------------------

/// The Workbench1.3 disk icon at the right-hand edge, and the Shell icon in
/// its window.
const WB13_DISK_ICON: (u32, u32) = (722, 130);
const WB13_SHELL_ICON: (u32, u32) = (198, 268);
/// The 1.3 pointer's red, and where the red starts from the hot spot.
const WB13_POINTER_RED: [u8; 3] = [0xdd, 0x22, 0x22];
const WB13_POINTER_RED_AT: (i32, i32) = (-2, 2);
/// The AmigaShell window's first character cell.
const WB13_SHELL_TEXT: (u32, u32) = (134, 152);

/// Steps 1-4 on Kickstart 1.3.
#[test]
fn kickstart_1_3_a_person_opens_the_disk_starts_a_shell_and_types() {
    let recorder = Arc::new(Recorder::recording());
    let Some(mut d) = desk(
        "amiga-os-130.rom",
        "amiga-os-134-workbench.adf",
        "wb13",
        recorder,
    ) else {
        return;
    };
    // 1. The desktop, as `amiga_a500_kickstart.rs` has it.
    d.run_ms(72_000);
    d.healthy();
    d.expect("1-desktop", GOLDEN_130_DESKTOP);

    // 2. The disk icon, double-clicked.
    d.home();
    d.move_to(WB13_DISK_ICON);
    d.pointer_is_at(
        "2-on-disk-icon",
        WB13_POINTER_RED,
        WB13_POINTER_RED_AT,
        WB13_DISK_ICON,
    );
    d.double_click(WB13_DISK_ICON);
    d.run_ms(5_000);
    d.expect("2-disk-window", GOLDEN_130_DISK_WINDOW);

    // 3. The Shell. 1.3 reads its way to a prompt for about eleven seconds.
    d.move_to(WB13_SHELL_ICON);
    d.double_click(WB13_SHELL_ICON);
    d.run_ms(15_000);
    d.expect("3-shell", GOLDEN_130_SHELL);

    // 4. A line with shifted characters, whose shift the keymap supplies.
    d.type_text("echo \"Hi, A500!\"\n");
    d.run_ms(2_000);
    let echoed = d.expect("4-echo", GOLDEN_130_ECHO);
    // `1.SYS:> echo "` is fourteen characters.
    assert!(
        same_text(&echoed, WB13_SHELL_TEXT, (14, 0), (0, 1), 9),
        "the shell printed `Hi, A500!` under the command that asked for it"
    );
    d.healthy();
}

/// At 45 s: the desktop — the blue-framed "Workbench" window holding the Ram
/// Disk and Workbench2.0 icons, the copyright in the screen's title bar, the
/// pointer at the top left.
const GOLDEN_204_DESKTOP: u64 = 0x95a5_9a12_c942_e139;
/// Five seconds after the double-click: the "Workbench2.0 94% full, 47K
/// free, 790K in use" window open over the desktop with Shell, System,
/// WBStartup, Monitors, Prefs, Utilities, Expansion and Trashcan, the screen
/// title now "Amiga Workbench 287248 graphics mem 0 other mem".
const GOLDEN_204_DISK_WINDOW: u64 = 0xb068_7a77_ca8e_2cad;
/// Six seconds after the Shell's double-click: an "AmigaShell" window across
/// the lower screen with the prompt `1.Workbench2.0:>` and a cursor, the
/// screen title "Workbench Screen", the pointer on the Shell icon behind it.
const GOLDEN_204_SHELL: u64 = 0x4bfa_84a2_6951_f705;
/// Two seconds after Return: `1.Workbench2.0:> echo hello`, then `hello`,
/// then a fresh prompt and cursor.
const GOLDEN_204_ECHO: u64 = 0x48ae_3353_da46_78f9;
/// At 72 s: the 1.3 desktop — blue, "Workbench release. 365000 free memory"
/// in the title bar, RAM DISK and Workbench1.3 down the right-hand edge.
const GOLDEN_130_DESKTOP: u64 = 0xb491_89ae_fb75_bd01;
/// Five seconds after the double-click: the "Workbench1.3" window with
/// Utilities, System, Expansion, Empty, Shell, Prefs and Trashcan, its icon
/// drawn open, the free memory down to 353672.
const GOLDEN_130_DISK_WINDOW: u64 = 0x9364_9d58_4213_09b1;
/// Fifteen seconds after the Shell's double-click: an "AmigaShell" window over
/// the disk window with the prompt `1.SYS:>` and an orange cursor, the screen
/// title "Workbench Screen".
const GOLDEN_130_SHELL: u64 = 0x9ed2_663d_c7e5_92e9;
/// Two seconds after Return: `1.SYS:> echo "Hi, A500!"`, then `Hi, A500!`,
/// then a fresh prompt.
const GOLDEN_130_ECHO: u64 = 0x604e_ab17_531d_6919;
