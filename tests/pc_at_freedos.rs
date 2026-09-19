//! **FreeDOS 1.3 installed onto a hard disk, and then booted off it.**
//!
//! `tests/pc_at_boot.rs` boots the FreeDOS 1.3 Floppy Edition's boot diskette
//! on rsemu's own BIOS and stops at the installer's first question, because
//! answering it takes a *conversation*: six diskettes, a partition, a format,
//! a reboot, and three quarters of an hour of guest time. This file has that
//! conversation.
//!
//! # What a pass means
//!
//! A person sits at the machine, and the only things that cross into it are
//! keystrokes and diskette swaps:
//!
//! 1. The Floppy Edition's boot diskette boots and its installer starts.
//! 2. `FDISK` partitions the blank IDE drive, and the machine **reboots** —
//!    off the diskette, which is the AT's boot order and was not this
//!    firmware's until this test needed it.
//! 3. `FORMAT` writes a FAT16 filesystem onto the partition, `SYS` writes the
//!    kernel and the boot sector, and the installer unpacks 114 archive
//!    volumes off five diskettes onto the hard disk, asking for each diskette
//!    by the name of a file on it.
//! 4. The diskette comes out, the machine is rebuilt around **the disk image
//!    the install wrote**, and it boots to `C:\>` with nothing else attached.
//! 5. `VER` and `DIR` are typed at that prompt and their answers read off the
//!    screen.
//!
//! Every input goes in where a VNC client's does — an X11 keysym on the
//! machine's [`Recorder`], through [`KeyboardSink`] onto the 8042's character
//! port as set-2 scan codes — and every answer comes out of the guest's own
//! text page at `0xb8000`. Nothing here writes guest memory, and nothing
//! reaches around the machine to the filesystem it is building.
//!
//! # Nothing is vendored, and no FreeDOS source was read
//!
//! FreeDOS is GPL-2.0. Running it as an emulated guest is ordinary use;
//! shipping it here would be redistribution (`ROADMAP.md` §1). The diskette
//! images are fetched by `scripts/fetch-testdata.sh freedos` into the ignored
//! corpus directory and found through `RSEMU_FREEDOS_DIR`; without them this
//! file prints why and passes. The installer is driven entirely black-box,
//! off what it prints on the screen — its source was not read, and no
//! emulator's was either.
//!
//! # Determinism
//!
//! The person is a state machine over the text page, sampled every 500 ms of
//! **virtual** time: it reads what the screen asks for and answers it at that
//! sample's instant. Two runs see the same screens at the same instants,
//! because nothing outside the machine is consulted — no wall clock, no host
//! randomness, and the RTC's date comes from the machine file.
//!
//! # Cost
//!
//! Measured: the install completes at 2,779.6 seconds of guest time — about
//! forty-six minutes — for 993 seconds of host time in `--release`, and the
//! hard-disk boot and the two commands add another 76 guest seconds. That is
//! what unpacking 114 archive volumes off five diskettes onto a 64 MiB disk
//! costs.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use std::path::{Path, PathBuf};
use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::record::{Channel, Recorder};
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::host::input::{self, Feed, InputEvent, KeyboardSink, Keysym};
use rsemu::machine::realize::Bindings;
use rsemu::machine::{Machine, build};

/// How big a disk the installer is given: 64 MiB, which FreeDOS formats as
/// FAT16 and which holds the Floppy Edition's "plain DOS system" (20 MB by
/// its own README) with room to spare. It costs its whole length in host
/// memory while the machine is up, which is why it is not larger.
const DISK_BYTES: usize = 64 << 20;

/// How often the person looks at the screen: 500 ms of guest time. Long
/// enough that the sampling costs nothing, short enough that a prompt is
/// answered inside a second.
const LOOK_MS: u64 = 500;

/// How long to let a prompt settle before answering it. A program that
/// prints "press a key" and then flushes the type-ahead ring throws away a
/// key pressed in the same instant it asked, and then waits for ever.
const SETTLE_MS: u64 = 2_000;

/// How long a key is held, and the gap before the next one. A DOS keyboard
/// handler reads the BIOS type-ahead ring, so anything longer than one 8042
/// byte-time works; 50 ms is a person's.
const HOLD_MS: u64 = 50;

/// The whole conversation's budget in guest time. A wall against a machine
/// that has stopped making progress rather than a measurement: the install
/// takes about forty-six guest minutes, most of it unpacking, and a
/// forty-five minute budget was short enough to stop it a few volumes from
/// the end.
const BUDGET_MS: u64 = 75 * 60 * 1000;

// ---------------------------------------------------------------------------
// the machine, and the person at it
// ---------------------------------------------------------------------------

/// A `pc-at` with rsemu's own firmware, a diskette drive and one IDE disk,
/// and somebody typing at the keyboard.
struct Pc {
    machine: Machine,
    cpu: Arc<X86>,
    recorder: Arc<Recorder>,
    channel: Channel,
    hosts: Arc<rsemu::core::hosts::HostObjects>,
}

impl Pc {
    /// Build the board with `floppy` in the drive and `hd0` on the primary
    /// channel, and open the keyboard the way `rsemu run --vnc` opens it.
    fn new(floppy: Vec<u8>, hd0: Vec<u8>) -> Pc {
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
        options
            .realize
            .media
            .insert("bios", rsemu::fw::pcbios::image());
        // No option ROM: this firmware draws its own text page.
        options.realize.media.insert("vgabios", Vec::new());
        options.realize.media.insert("floppy", floppy);
        options.realize.media.insert("hd0", hd0);
        options.realize.media.insert("hd1", Vec::new());
        // The record/replay seam, as `rsemu run --record-input` engages it:
        // every keystroke below crosses into the machine through it.
        let recorder = Arc::new(Recorder::recording());
        options.realize.recorder = Some(Arc::clone(&recorder));

        let registry = rsemu::machine::catalog::registry().expect("this build's registry");
        let mut machine = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
            .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
        let cpu = cpus.take().expect("the constructor kept a handle");
        let hosts = Arc::clone(&options.realize.hosts);

        let feed = Arc::new(Feed::new());
        let port = rsemu::host::chardev::ports::get(&hosts, "keyboard")
            .expect("no other kind of host object claims the name")
            .expect("the 8042 opened its port");
        feed.attach(Arc::new(KeyboardSink::new(port)));
        let channel = input::channel(input::DEFAULT_STREAM);
        recorder
            .register(channel.clone(), input::sink(&feed))
            .expect("the channel list is open until the first round");

        machine.reset(ResetKind::Cold);
        machine.sweep();
        Pc {
            machine,
            cpu,
            recorder,
            channel,
            hosts,
        }
    }

    /// Guest milliseconds.
    fn run_ms(&mut self, ms: u64) {
        self.machine
            .run_for(GlobalTime::from_nanos(ms * 1_000_000))
            .expect("the machine runs");
    }

    /// Where the machine is, in guest milliseconds since the cold reset.
    fn now_ms(&self) -> u64 {
        self.machine.now().as_nanos() / 1_000_000
    }

    /// Post an event on the recorder's input channel, as a VNC session does.
    fn post(&self, event: InputEvent) {
        self.recorder
            .post(&self.channel, &event.encode())
            .expect("a registered channel");
    }

    /// One key, pressed and released.
    fn key(&mut self, keysym: Keysym) {
        self.post(InputEvent::Key { keysym, down: true });
        self.run_ms(HOLD_MS);
        self.post(InputEvent::Key {
            keysym,
            down: false,
        });
        self.run_ms(HOLD_MS);
    }

    /// Type `text`; `\n` is Return.
    fn type_text(&mut self, text: &str) {
        for b in text.bytes() {
            self.key(if b == b'\n' {
                Keysym::RETURN
            } else {
                Keysym::from_ascii(b)
            });
        }
    }

    /// One byte of guest memory, as a debugger reads it.
    fn peek(&self, addr: u64) -> u8 {
        self.machine
            .space("mem")
            .expect("the memory space")
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .unwrap_or(0xff) as u8
    }

    /// The colour text page, as 25 lines of characters.
    fn screen(&self) -> Vec<String> {
        (0..25u64)
            .map(|row| {
                (0..80u64)
                    .map(|col| {
                        let ch = self.peek(0xb8000 + (row * 80 + col) * 2);
                        match ch {
                            0x20..=0x7e => ch as char,
                            _ => ' ',
                        }
                    })
                    .collect::<String>()
                    .trim_end()
                    .to_string()
            })
            .collect()
    }

    /// The screen as one string, which is how the person reads it.
    fn text(&self) -> String {
        self.screen().join("\n")
    }

    /// Print the screen under `what`, which is what this test reports.
    fn show(&self, what: &str) {
        println!("---- {what} @ {} ms ----", self.now_ms());
        for line in self.screen() {
            if !line.trim().is_empty() {
                println!("  |{line}|");
            }
        }
    }

    /// Swap the diskette, as a person swaps it: the door opens, the medium
    /// changes, and `DSKCHG` stays active until the guest next steps.
    fn insert(&self, name: &str, image: Vec<u8>) {
        rsemu::dev::pc::fdc::drives::get(&self.hosts, "fd0")
            .expect("no other kind of host object claims the name")
            .expect("the controller filed itself in a drive")
            .insert(name, image)
            .expect("a 1.44 MB diskette image");
    }

    /// What the hard disk holds now, read out of the drive's own medium
    /// through its bay — the host's way in, not a back door in the adapter.
    fn disk(&self) -> Vec<u8> {
        rsemu::dev::ata::bays::get(&self.hosts, "ide0-master")
            .expect("no other kind of host object claims the name")
            .expect("the channel and the drive both opened it")
            .drive()
            .expect("the master bay is populated")
            .contents()
            .expect("the medium reads back")
    }

    /// The machine is still a machine: nothing has faulted on the bus.
    fn healthy(&self, what: &str) {
        let (faults, last) = self.cpu.bus_faults();
        assert_eq!(
            faults, 0,
            "{what}: {faults} unanswered bus access(es), last at {last:08x}"
        );
    }
}

// ---------------------------------------------------------------------------
// the diskettes
// ---------------------------------------------------------------------------

/// The Floppy Edition's six 1.44 MB images: the boot diskette, and the five
/// that carry the archive.
struct Diskettes {
    boot: Vec<u8>,
    /// `(name, image, the files in its root directory)`.
    set: Vec<(String, Vec<u8>, Vec<String>)>,
}

impl Diskettes {
    /// Read them, or say why not.
    fn open() -> Option<Diskettes> {
        let dir = match std::env::var("RSEMU_FREEDOS_DIR") {
            Ok(dir) => PathBuf::from(dir),
            // The boot diskette's own variable, which `tests/pc_at_boot.rs`
            // takes: the rest of the set is beside it.
            Err(_) => match std::env::var("RSEMU_FREEDOS_FLOPPY")
                .ok()
                .and_then(|file| Path::new(&file).parent().map(Path::to_path_buf))
            {
                Some(dir) => dir,
                None => {
                    println!(
                        "pc-at freedos: RSEMU_FREEDOS_DIR is unset, so there is no \
                         installation set to install. `scripts/fetch-testdata.sh freedos` \
                         fetches one; nothing is committed here, because it is GPL-2.0."
                    );
                    return None;
                }
            },
        };
        let boot = match std::fs::read(dir.join("x86BOOT.img")) {
            Ok(image) => image,
            Err(e) => {
                println!("pc-at freedos: {}/x86BOOT.img: {e}; skipped", dir.display());
                return None;
            }
        };
        let mut set = Vec::new();
        for n in 1..=5 {
            let name = format!("x86DSK{n:02}.img");
            let Ok(image) = std::fs::read(dir.join(&name)) else {
                println!(
                    "pc-at freedos: {}/{name} is not there, so the installer has only its \
                     boot diskette and cannot install anything; \
                     `scripts/fetch-testdata.sh freedos` fetches the whole set",
                    dir.display()
                );
                return None;
            };
            let files = root_directory(&image);
            println!("pc-at freedos: {name}: {} files", files.len());
            set.push((name, image, files));
        }
        Some(Diskettes { boot, set })
    }

    /// The diskette whose root directory holds `file`, which is how the
    /// installer asks for one — "insert the diskette containing A:\FREEDOS.024".
    /// A person reads the label; this reads the filesystem, which is the same
    /// claim without depending on what the labels say.
    fn carrying(&self, file: &str) -> Option<(&str, &[u8])> {
        self.set
            .iter()
            .find(|(_, _, files)| files.iter().any(|f| f == file))
            .map(|(name, image, _)| (name.as_str(), image.as_slice()))
    }

    /// The diskette the screen asks for by label — "insert diskette #2
    /// (x86-DSK1) in A:\", where the label written on the medium itself is
    /// `FD13DSK01`. The prompt drops the leading zero, so the match is on the
    /// number rather than on the text.
    fn labelled(&self, text: &str) -> Option<(&str, &[u8])> {
        self.set
            .iter()
            .enumerate()
            .find(|(n, _)| text.contains(&format!("x86-DSK{}", n + 1)))
            .map(|(_, (name, image, _))| (name.as_str(), image.as_slice()))
    }
}

/// The 8.3 names in a FAT12 diskette's root directory, the volume label
/// first.
///
/// Enough of the FAT to read a directory and no more: the BIOS parameter
/// block at offset 11 gives the sector size, the reserved and FAT counts, the
/// root entry count and the sectors per FAT (Microsoft, *FAT: General
/// Overview of On-Disk Format*, the BPB); the root directory follows the
/// FATs; and each 32-byte entry is a name unless it is free (`0x00`/`0xE5`)
/// or a long-name fragment (attribute `0x0F`). This is the *host* reading the
/// label on a diskette, which is what tells a person which one to put in.
fn root_directory(image: &[u8]) -> Vec<String> {
    if image.len() < 512 {
        return Vec::new();
    }
    let word = |at: usize| u16::from_le_bytes([image[at], image[at + 1]]) as usize;
    let bytes_per_sector = word(11);
    let reserved = word(14);
    let fats = image[16] as usize;
    let root_entries = word(17);
    let sectors_per_fat = word(22);
    let start = (reserved + fats * sectors_per_fat) * bytes_per_sector;
    let mut names = Vec::new();
    for i in 0..root_entries {
        let at = start + i * 32;
        if at + 32 > image.len() {
            break;
        }
        let entry = &image[at..at + 32];
        if entry[0] == 0x00 || entry[0] == 0xe5 || entry[11] & 0x0f == 0x0f {
            continue;
        }
        let stem = String::from_utf8_lossy(&entry[0..8]).trim_end().to_string();
        let ext = String::from_utf8_lossy(&entry[8..11])
            .trim_end()
            .to_string();
        names.push(if ext.is_empty() {
            stem
        } else {
            format!("{stem}.{ext}")
        });
    }
    names
}

/// Whether `line` is a DOS command prompt — `A:\>`, `C:\>`, or `C:\FDOS\BIN>`
/// once an `AUTOEXEC.BAT` has moved somewhere. It is a drive, a path and a
/// `>`, and what it means here is "the installer has stopped talking".
fn is_dos_prompt(line: &str) -> bool {
    let line = line.trim();
    line.ends_with('>') && line.len() >= 4 && line[1..].starts_with(":\\")
}

/// The file name out of an "insert the diskette containing file A:\\NAME"
/// prompt.
fn wanted_file(line: &str) -> Option<String> {
    const LEAD: &str = "containing file A:\\";
    let rest = &line[line.find(LEAD)? + LEAD.len()..];
    let end = rest.find(char::is_whitespace).unwrap_or(rest.len());
    Some(rest[..end].to_string())
}

// ---------------------------------------------------------------------------
// the conversation
// ---------------------------------------------------------------------------

/// Answer whatever the installer has put on the screen, until it is finished
/// or the budget runs out. Reports whether it finished.
///
/// Nothing here knows what order the questions come in — the installer
/// reboots the machine in the middle and asks a different set afterwards — so
/// the person answers what is in front of them, which is also the only way to
/// drive a program whose source is off limits.
///
/// **What is being asked is the last line on the screen.** Everything above
/// it is history: a question already answered, or a prompt already dealt
/// with, scrolling up. Answering one of those again puts the wrong diskette
/// in the drive and types into whatever asks next.
fn drive_the_installer(pc: &mut Pc, disks: &Diskettes) -> bool {
    let mut last = String::new();
    let mut in_drive = String::from("x86BOOT.img");
    let deadline = pc.now_ms() + BUDGET_MS;
    while pc.now_ms() < deadline {
        pc.run_ms(LOOK_MS);
        let screen = pc.screen();
        // Nothing has changed since the last look, so nothing new is being
        // asked: the program is working, and an answer typed twice would be
        // read by whatever asks next.
        let text = screen.join("\n");
        if text == last {
            continue;
        }
        last = text.clone();
        if text.contains("has been aborted") {
            pc.show("the installer gave up");
            return false;
        }
        // The installer says so itself, and it says so *before* it offers to
        // reboot — which is the question that must not be answered `Y` here.
        // The diskette in the drive is the last of the archive set, the
        // firmware tries the diskette first as an AT does, and that diskette
        // says "this is not a bootable disk" and waits. The hard disk is what
        // this test wants to boot, and it boots it in a machine of its own.
        if text.contains("installation of FreeDOS") && text.contains("has completed") {
            pc.show("the installation has completed");
            return true;
        }

        let Some(bottom) = screen.iter().rev().find(|line| !line.trim().is_empty()) else {
            continue;
        };
        let bottom = bottom.clone();

        // The yes/no questions: proceed, format, reboot. The answer is echoed
        // on the same line, so a question with a letter after it has been
        // answered already.
        if bottom.contains("[Y,N]?") && !bottom.ends_with(['Y', 'N']) {
            pc.show(&bottom);
            pc.type_text("y\n");
            last = pc.text();
            continue;
        }

        // A diskette, asked for in one of two ways:
        //
        //   "Insert diskette #2 (x86-DSK1) in A:\"          — by label
        //   "Insert the diskette containing file A:\FREEDOS.024"
        //
        // The second is the unpacker's: the archive is 114 volumes spread
        // over five diskettes, and that is how it crosses from one to the
        // next. Either way the line under it is "press a key", which is what
        // says the prompt is live.
        if bottom.contains("Press a key to continue")
            || bottom.contains("Press any key to continue")
        {
            // The prompt is printed *and then* the type-ahead ring is
            // flushed, so a key pressed in the same breath as the prompt
            // appears is thrown away and the machine waits for ever.
            pc.run_ms(SETTLE_MS);
            let asked = screen
                .iter()
                .rev()
                .find(|line| line.contains("Insert"))
                .cloned()
                .unwrap_or_default();
            let wanted = match wanted_file(&asked) {
                Some(file) => {
                    let Some((name, image)) = disks.carrying(&file) else {
                        pc.show("a file that is on no diskette");
                        panic!(
                            "the installer wants {file}, which is on none of the five diskettes"
                        );
                    };
                    println!("pc-at freedos: {} ms: {file} is on {name}", pc.now_ms());
                    Some((name, image))
                }
                None => {
                    let found = disks.labelled(&asked);
                    if let Some((name, _)) = found {
                        println!("pc-at freedos: {} ms: it asks for {name}", pc.now_ms());
                    }
                    found
                }
            };
            if let Some((name, image)) = wanted.filter(|(name, _)| *name != in_drive) {
                in_drive = name.to_string();
                let image = image.to_vec();
                pc.insert(name, image);
            }
            pc.key(Keysym::RETURN);
            last = pc.text();
            continue;
        }

        // The installer's last act is to leave DOS at a prompt.
        if is_dos_prompt(&bottom) {
            pc.show("the installer has finished");
            return true;
        }
    }
    pc.show("out of time");
    false
}

// ---------------------------------------------------------------------------
// the test
// ---------------------------------------------------------------------------

/// Install FreeDOS onto a blank IDE disk, then boot that disk with nothing in
/// the diskette drive and use what booted.
#[test]
fn freedos_installs_onto_a_hard_disk_and_boots_off_it() {
    let Some(disks) = Diskettes::open() else {
        return;
    };

    // The disk the installer is given: a blank file in a temporary directory,
    // created here and removed at the end. A *file*, because that is what a
    // person points `--media hd0=` at.
    let dir = std::env::temp_dir().join(format!("rsemu-freedos-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("a temporary directory");
    let image_path = dir.join("hd0.img");
    std::fs::write(&image_path, vec![0u8; DISK_BYTES]).expect("a blank disk image");
    println!(
        "pc-at freedos: {} is {DISK_BYTES} bytes of nothing",
        image_path.display()
    );

    let started = std::time::Instant::now();
    let mut pc = Pc::new(
        disks.boot.clone(),
        std::fs::read(&image_path).expect("the blank image"),
    );

    // The boot diskette, as far as the installer's first question. Sixty
    // seconds is measured; `tests/pc_at_boot.rs` reports the same one.
    pc.run_ms(60_000);
    pc.show("the installer's first question");
    assert!(
        pc.text().contains("Do you want to proceed"),
        "the Floppy Edition's installer never asked its first question"
    );

    let installed = drive_the_installer(&mut pc, &disks);
    println!(
        "pc-at freedos: the install took {} ms of guest time and {:?} of host time",
        pc.now_ms(),
        started.elapsed()
    );
    pc.healthy("the install");
    assert!(installed, "the installer did not finish");

    // What it wrote, back into the file it came from.
    let written = pc.disk();
    drop(pc);
    std::fs::write(&image_path, &written).expect("the installed image is writable");
    assert_eq!(
        &written[510..512],
        &[0x55, 0xaa],
        "no master boot record was written to the disk"
    );

    // -- and now boot it, with no diskette at all --------------------------
    let mut pc = Pc::new(Vec::new(), std::fs::read(&image_path).expect("the image"));
    pc.run_ms(60_000);
    pc.show("booted off the hard disk");
    let prompt = pc
        .screen()
        .iter()
        .rev()
        .find(|line| !line.trim().is_empty())
        .cloned()
        .unwrap_or_default();
    assert!(
        is_dos_prompt(&prompt) && prompt.trim_start().starts_with("C:"),
        "the machine did not reach a C: prompt off the hard disk; the last line \
         is {prompt:?}"
    );
    // What the installed system says for itself on the way up. `FDCONFIG.SYS`
    // and `FDAUTO.BAT` are the two files the installer wrote and named; the
    // banner is `COMMAND.COM` running off the hard disk rather than the
    // diskette, which is no longer in the drive.
    let booted = pc.text();
    assert!(
        booted.contains("C:\\FDCONFIG.SYS") && booted.contains("C:\\FDAUTO.BAT"),
        "the startup files the installer wrote were not the ones processed:\n{booted}"
    );
    assert!(
        booted.contains("Welcome to the FreeDOS 1.3 operating system"),
        "nothing on the screen says which operating system came up:\n{booted}"
    );

    pc.type_text("ver\n");
    pc.run_ms(5_000);
    pc.show("VER");
    assert!(
        pc.text().contains("FreeCom version"),
        "`VER` was not answered by FreeDOS's own shell"
    );

    pc.type_text("dir c:\\\n");
    pc.run_ms(10_000);
    pc.show("DIR");
    let listing = pc.text();
    for expected in [
        "Volume in drive C",
        "Directory of C:",
        "KERNEL",
        "COMMAND",
        "FDCONFIG",
    ] {
        assert!(
            listing.contains(expected),
            "`DIR` does not show an installed FreeDOS: {expected:?} is missing \
             from\n{listing}"
        );
    }
    pc.healthy("the hard-disk boot");

    let _ = std::fs::remove_dir_all(&dir);
}
