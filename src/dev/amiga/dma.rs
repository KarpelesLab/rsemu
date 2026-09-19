//! Chip-RAM direct memory access: Agnus's pointers, `DMACON`, and chip RAM
//! behind one lock-free handle.
//!
//! # Why Agnus is in the middle
//!
//! Every DMA pointer on an Amiga is an **Agnus register**. Appendix B assigns
//! `DSKPTH`/`DSKPTL`, `AUDxLCH`/`AUDxLCL`, `BPLxPTH`/`BPLxPTL` and
//! `SPRxPTH`/`SPRxPTL` to Agnus alone, while the data registers those channels
//! land in — `DSKDAT`, `AUDxDAT`, `BPLxDAT`, `SPRxDATA` — are Paula's and
//! Denise's, marked `&` or `%` ("used by DMA channel"). And `DMACON`, which
//! enables each channel, is one register Agnus gates the transfers with.
//! Chapter 6, *Blitter Operations and System DMA*, describes the arbitration:
//! disk, audio, display and sprite DMA each get time slots on every line, the
//! copper has the next priority, and the blitter and the 68000 share what is
//! left.
//!
//! So neither Paula nor Denise reaches chip RAM. **Agnus pushes, both ways:**
//!
//! | channel | what Agnus does | where |
//! | --- | --- | --- |
//! | bitplanes | fetches each line's words and hands the finished line to the video chip | `agnus::display` |
//! | sprites | fetches control and data words and writes them into Denise through the bus | `agnus::display` |
//! | disk, audio | asks Paula in each line's slot what it wants, and moves the words | `agnus::slots` |
//!
//! Denise has no memory bus (Appendix J) and no vertical counter; Paula knows
//! when a channel needs a word but not where it is. Each was written to be
//! driven, and the two contracts are theirs.
//!
//! # What this handle is, then
//!
//! [`ChipDma`] is where Agnus keeps the part of that machinery that has to be
//! reachable **without Agnus's state lock**: chip RAM, `DMACON`, `DSKPT`, the
//! four `AUDxLC` locations and their running pointers, and the beam position.
//! A slot is served with Paula's lock free to be taken, so it cannot run under
//! Agnus's; everything it touches is here, in atomics.
//!
//! It is also published, as [`ExportId::CHIP_DMA`](crate::core::device::ExportId::CHIP_DMA),
//! for a test, a debugger or a monitor that wants to look at what Agnus's DMA
//! sees:
//!
//! | call | what it does |
//! | --- | --- |
//! | [`audio_restart`](ChipDma::audio_restart), [`audio_fetch`](ChipDma::audio_fetch) | reload channel *x*'s pointer from `AUDxLC`; read a word and add two |
//! | [`disk_fetch`](ChipDma::disk_fetch), [`disk_store`](ChipDma::disk_store) | read or store a word at `DSKPT` and add two |
//! | [`read_word`](ChipDma::read_word), [`write_word`](ChipDma::write_word) | any channel, any address |
//! | [`peek`](ChipDma::peek), [`poke`](ChipDma::poke) | chip RAM, ungated |
//! | [`beam`](ChipDma::beam) | where Agnus's counters were at the last count it simulated |
//!
//! # The chip data bus
//!
//! Every one of those cycles leaves its word on `D15`–`D0`. [`ChipDataBus`] is
//! that word, shared with `amiga.custom`, which drives it from the register
//! side and answers a read of a write-only register out of it — the one thing
//! on the board that can see an undriven bus. Its documentation has the whole
//! rule, and marks what is inference and what is choice.
//!
//! Each gated call answers `None` (or `false`) when `DMACON` has the channel's
//! enable bit or the master `DMAEN` clear — a disabled channel moves no data
//! and **does not advance its pointer**.
//!
//! The beam is published before every write Agnus makes through the custom
//! bus — a copper `MOVE`, a sprite word — so a chip receiving one can ask which
//! count it arrived on, from inside the write.
//!
//! # Locks
//!
//! **Nothing here takes a blocking lock.** The pointers and `DMACON` are
//! atomics, and chip RAM is a private address space whose read guard is a
//! non-blocking try-lock over a lock-free RAM store. The handle to the space
//! sits behind a `LEAF` mutex held for one `Arc` clone.
//!
//! # What is not modelled
//!
//! * **Contention.** A slot nobody used is not lent to the blitter, and the
//!   blitter never steals a 68000 cycle, so `BLTPRI` is stored and read back
//!   and does nothing.

use alloc::sync::Arc;
use core::fmt;

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};

/// One DMA channel, as its `DMACON` enable bit.
///
/// The `EtherType` pattern (`CLAUDE.md`, *Type conventions*): an open set of
/// bit masks rather than an enum, because the value *is* the bit a guest writes.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DmaChannel(pub u16);

impl DmaChannel {
    /// `AUD0EN`, bit 0.
    pub const AUD0: DmaChannel = DmaChannel(1 << 0);
    /// `AUD1EN`, bit 1.
    pub const AUD1: DmaChannel = DmaChannel(1 << 1);
    /// `AUD2EN`, bit 2.
    pub const AUD2: DmaChannel = DmaChannel(1 << 2);
    /// `AUD3EN`, bit 3.
    pub const AUD3: DmaChannel = DmaChannel(1 << 3);
    /// `DSKEN`, bit 4.
    pub const DISK: DmaChannel = DmaChannel(1 << 4);
    /// `SPREN`, bit 5.
    pub const SPRITE: DmaChannel = DmaChannel(1 << 5);
    /// `BLTEN`, bit 6.
    pub const BLITTER: DmaChannel = DmaChannel(1 << 6);
    /// `COPEN`, bit 7.
    pub const COPPER: DmaChannel = DmaChannel(1 << 7);
    /// `BPLEN`, bit 8.
    pub const BITPLANE: DmaChannel = DmaChannel(1 << 8);

    /// The audio channel `n`'s enable.
    ///
    /// # Panics
    ///
    /// If `n` is not 0–3.
    #[must_use]
    pub const fn audio(n: usize) -> DmaChannel {
        assert!(n < 4, "Paula has four audio channels");
        DmaChannel(1 << n)
    }
}

/// `DMACON` bit 9, `DMAEN`: "a master DMA enable bit. It enables the DMA for
/// all of the channels at bits 8-0" (chapter 7, table 7-6).
pub const DMAEN: u16 = 1 << 9;

/// `DMACON` bit 10, `BLTPRI` — "blitter nasty". Stored, not acted on.
pub const BLTPRI: u16 = 1 << 10;

/// `DMACON` bit 15: the set/clear control bit.
pub const SETCLR: u16 = 1 << 15;

/// The bits a `DMACON` write can change: 10 through 0. Bits 14 and 13 are
/// read-only status, 12 and 11 are unassigned.
pub const DMACON_WRITABLE: u16 = 0x07ff;

/// Where Agnus's beam was at one count.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BeamPosition {
    /// The count of Agnus's clock domain this position was reached on.
    pub tick: u64,
    /// Fields completed since power-on.
    pub field: u64,
    /// The vertical counter.
    pub vpos: u16,
    /// The horizontal counter, one count per colour clock.
    pub hpos: u16,
    /// `LOF`: the field is a long one.
    pub lof: bool,
}

/// `D15`–`D0`, the sixteen lines every chip on the board shares.
///
/// # What it is for
///
/// Two hundred of the custom registers are write-only, and Appendix B does not
/// say what a read of one answers. On the board it is not a register that
/// answers at all: no chip drives `D15`–`D0` during that cycle, so the
/// processor latches the lines as they are. Appendix C (p. 299), explaining
/// how to tell an 8362 from an 8373 by the `DENISEID` an 8362 does not have,
/// calls that "whatever value is left over on the bus from the last cycle".
///
/// So this is the word the last cycle left, and the rule is which cycles
/// leave one:
///
/// | cycle | drives `D15`–`D0`? |
/// | --- | --- |
/// | Agnus's DMA — bitplane, sprite, copper, blitter, disk, audio | **yes**, the word transferred ([`ChipDma::peek`], [`ChipDma::poke`]) |
/// | a processor or copper **write** into `$DFF000` | **yes**, the word written |
/// | a **read** of a register a chip answers | **yes**, the word that chip drove |
/// | a **refresh** slot | no, and it leaves the word alone |
/// | a **read nothing answers** | no, and the word does not survive it |
///
/// The first three are the board: chapter 6, *Blitter Operations and System
/// DMA*, gives Agnus every chip-memory slot and hands the 68000 the rest, and
/// the A500's schematic (#312511-02 sheet 2) puts the processor's own data on
/// these same sixteen lines through `U10`/`U12` — `src/dev/amiga/custom.rs`
/// cites both in full.
///
/// # What the manual does not say, and what is chosen here
///
/// It describes the arbitration, not the bus's electrical state, and it gives
/// no value for an undriven bus. Three things below are therefore inference
/// or choice, and are marked as such:
///
/// * **A refresh slot leaves the previous word.** Chapter 6 gives memory
///   refresh its own slots at the start of every line and Appendix B gives it
///   `REFPTR`, but a DRAM refresh is a `RAS`-only cycle: `CAS` stays high and
///   the DRAMs never enable their data outputs (any 41256-class data sheet's
///   "RAS-only refresh"). No data crosses the bus, so nothing there changes.
///   In this model that is not a rule at all — a refresh slot simply never
///   calls [`drive`](Self::drive).
/// * **A cycle nothing drives does not leave the word behind.** This is the
///   part that matters, and it is an *inference from the detection method
///   Appendix C documents*: if an absent register answered with a dependable
///   value, reading it twice would be indistinguishable from reading a real
///   one, and Commodore's own test — which Kickstart 2.04 makes, seventeen
///   reads of `$DFF07C` in a row, watched black-box — could not work. A read
///   nothing answers opens the board's buffers onto sixteen undriven lines;
///   the processor latches what was there and the lines are left floating.
///   So [`take`](Self::take) answers with the word and clears it. This is
///   different from a refresh slot, which never selects the data path at all.
/// * **An undriven bus reads zero.** No manual gives a value, and a real one
///   decays to something nobody specified. Zero is a choice, the same one
///   `tests/amiga_a500_unassigned.rs` records for an empty address.
///
/// A board with no DMA engine at all has no [`ChipDataBus`] and reads zero
/// throughout;
/// [`CustomBus::unclaimed`](crate::dev::amiga::custom::CustomBus::unclaimed)
/// counts every read that fell back to any of this.
///
/// # What is not modelled
///
/// **A processor access to chip RAM.** On the board it is a chip-bus cycle
/// like any other — Agnus arbitrates it and the A500's `U10`/`U12` buffers put
/// the word on these same lines — so on real hardware `MOVE.W $40000,D0`
/// leaves that word here. In this model chip RAM is reached through the
/// ordinary address space (`amiga.gary` and a `ram` object), which knows
/// nothing about the custom chips, so those cycles pass without driving. The
/// consequence is narrow: a guest that reads a write-only register straight
/// after touching chip RAM gets the previous *register or DMA* word instead of
/// that one. Closing it means a store on the hottest path in the machine —
/// every processor access to chip RAM — for a value only an unanswered
/// register read can see.
#[derive(Debug, Default)]
pub struct ChipDataBus {
    word: AtomicU64,
}

impl ChipDataBus {
    /// A bus nothing has driven yet.
    #[must_use]
    pub const fn new() -> ChipDataBus {
        ChipDataBus {
            word: AtomicU64::new(0),
        }
    }

    /// A chip-bus cycle drove `word` onto `D15`–`D0`.
    #[inline]
    pub fn drive(&self, word: u16) {
        self.word.store(u64::from(word), Ordering::Relaxed);
    }

    /// What is on the lines, without touching them: a debugger's look, and
    /// what a test asserts on.
    #[must_use]
    #[inline]
    pub fn word(&self) -> u16 {
        self.word.load(Ordering::Relaxed) as u16
    }

    /// A cycle in which nothing drove: answer with what was on the lines and
    /// leave them floating. See the type's documentation for why this is not
    /// the same as a refresh slot.
    #[must_use]
    #[inline]
    pub fn take(&self) -> u16 {
        self.word.swap(0, Ordering::Relaxed) as u16
    }

    /// Back to power-on: nothing has been driven.
    pub fn clear(&self) {
        self.word.store(0, Ordering::Relaxed);
    }
}

/// An audio channel's two addresses.
#[derive(Debug, Default)]
struct Audio {
    /// `AUDxLC`, as the processor or copper last wrote it.
    location: AtomicU64,
    /// The running pointer the channel's DMA reads through.
    pointer: AtomicU64,
}

/// The chip-RAM access Agnus grants the other chips.
///
/// See the module documentation for which chip calls what, and why nothing
/// here blocks.
pub struct ChipDma {
    /// Chip RAM, in a private space at address zero. `None` until Agnus binds.
    ram: Mutex<Option<Arc<AddressSpace>>>,
    /// The address bits Agnus drives: chip RAM's size rounded up to a power of
    /// two, less one, with the low bit clear — "the least significant bit of
    /// the address is ignored" (chapter 6, *DMA Channels*).
    mask: AtomicU64,
    /// `DMACON`'s bits 10–0.
    dmacon: AtomicU64,
    audio: [Audio; 4],
    /// `DSKPT`.
    disk: AtomicU64,
    /// The beam, packed: `vpos << 17 | hpos << 1 | lof`.
    beam: AtomicU64,
    /// The count that packed position was reached on.
    tick: AtomicU64,
    /// Fields since power-on.
    field: AtomicU64,
    /// Transfers refused because the channel was disabled.
    refused: AtomicU64,
    /// Transfers that addressed something chip RAM did not answer.
    faults: AtomicU64,
    /// `D15`–`D0`, as the last cycle left them. Shared with
    /// [`CustomBus`](crate::dev::amiga::custom::CustomBus), which answers a
    /// read of a write-only register out of it.
    data: Arc<ChipDataBus>,
}

impl fmt::Debug for ChipDma {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ChipDma")
            .field("mask", &self.mask.load(Ordering::Relaxed))
            .field("dmacon", &self.dmacon())
            .field("disk", &self.disk_pointer())
            .field("beam", &self.beam())
            .field("refused", &self.refused())
            .field("data", &self.data.word())
            .finish_non_exhaustive()
    }
}

impl Default for ChipDma {
    fn default() -> ChipDma {
        ChipDma::new()
    }
}

impl ChipDma {
    /// Nothing attached, every channel disabled.
    #[must_use]
    pub fn new() -> ChipDma {
        ChipDma {
            // LEAF: held for the length of an `Arc` clone and nothing else.
            ram: Mutex::with_rank(LockRank::LEAF, None),
            mask: AtomicU64::new(0),
            dmacon: AtomicU64::new(0),
            audio: Default::default(),
            disk: AtomicU64::new(0),
            beam: AtomicU64::new(u64::from(true)),
            tick: AtomicU64::new(0),
            field: AtomicU64::new(0),
            refused: AtomicU64::new(0),
            faults: AtomicU64::new(0),
            data: Arc::new(ChipDataBus::new()),
        }
    }

    /// The chip data bus these channels drive, for `amiga.custom` to answer a
    /// read of a write-only register out of. Agnus hands it over at bind.
    #[must_use]
    pub fn data_bus(&self) -> Arc<ChipDataBus> {
        Arc::clone(&self.data)
    }

    /// Point the channels at chip RAM. Agnus calls this from its bind.
    pub fn attach_ram(&self, space: Arc<AddressSpace>, len: u64) {
        *self.ram.lock() = Some(space);
        let mask = len.max(2).next_power_of_two().wrapping_sub(1) & !1;
        self.mask.store(mask, Ordering::Relaxed);
    }

    /// The chip-RAM address mask Agnus drives.
    #[must_use]
    pub fn address_mask(&self) -> u32 {
        self.mask.load(Ordering::Relaxed) as u32
    }

    /// The space chip RAM is reached through, for Agnus's own engine.
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.ram.lock().clone()
    }

    // -- DMACON ---------------------------------------------------------------

    /// `DMACON`'s bits 10–0, as Agnus holds them.
    #[must_use]
    pub fn dmacon(&self) -> u16 {
        self.dmacon.load(Ordering::Relaxed) as u16
    }

    /// Agnus's half: publish a new `DMACON`. Public so a Paula test can stand in
    /// for Agnus with a bare handle.
    pub fn set_dmacon(&self, value: u16) {
        self.dmacon
            .store(u64::from(value & DMACON_WRITABLE), Ordering::Relaxed);
    }

    /// Whether `channel` may transfer: its own bit **and** `DMAEN`.
    #[must_use]
    #[inline]
    pub fn enabled(&self, channel: DmaChannel) -> bool {
        let dmacon = self.dmacon();
        dmacon & DMAEN != 0 && dmacon & channel.0 == channel.0
    }

    // -- raw access -----------------------------------------------------------

    /// Read one big-endian word of chip RAM at `addr`, masked to the bits Agnus
    /// drives. Unmapped chip addresses read as zero and are counted.
    ///
    /// This **is** a chip-bus cycle: the word read is left on
    /// [`ChipDataBus`], where a read of a write-only custom register finds it.
    /// A cycle that faulted transferred nothing and leaves the bus alone. A
    /// debugger wanting to look at chip RAM without driving the bus reads the
    /// address space with [`MemAttrs::DEBUG`] instead.
    #[must_use]
    pub fn peek(&self, space: Option<&AddressSpace>, addr: u32) -> u16 {
        let Some(space) = space else {
            return 0;
        };
        let addr = addr & self.address_mask();
        let mut word = [0u8; 2];
        if space
            .read_bytes(u64::from(addr), &mut word, MemAttrs::DEFAULT)
            .is_err()
        {
            self.faults.fetch_add(1, Ordering::Relaxed);
            return 0;
        }
        let word = u16::from_be_bytes(word);
        self.data.drive(word);
        word
    }

    /// Write one big-endian word of chip RAM, masked like [`peek`](Self::peek),
    /// and leave it on the chip data bus.
    pub fn poke(&self, space: Option<&AddressSpace>, addr: u32, value: u16) {
        let Some(space) = space else {
            return;
        };
        let addr = addr & self.address_mask();
        if space
            .write_bytes(u64::from(addr), &value.to_be_bytes(), MemAttrs::DEFAULT)
            .is_err()
        {
            self.faults.fetch_add(1, Ordering::Relaxed);
            return;
        }
        self.data.drive(value);
    }

    /// Read a word for `channel`, or `None` if `DMACON` has it disabled.
    #[must_use]
    pub fn read_word(&self, channel: DmaChannel, addr: u32) -> Option<u16> {
        if !self.enabled(channel) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return None;
        }
        Some(self.peek(self.space().as_deref(), addr))
    }

    /// Write a word for `channel`; `false` if `DMACON` has it disabled.
    pub fn write_word(&self, channel: DmaChannel, addr: u32, value: u16) -> bool {
        if !self.enabled(channel) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        self.poke(self.space().as_deref(), addr, value);
        true
    }

    // -- audio ---------------------------------------------------------------

    /// Agnus's half: `AUDxLCH` or `AUDxLCL` was written.
    pub fn set_audio_location(&self, ch: usize, high: bool, value: u16) {
        set_half(&self.audio[ch].location, high, value);
    }

    /// `AUDxLC` for channel `ch`, as last written.
    #[must_use]
    pub fn audio_location(&self, ch: usize) -> u32 {
        self.audio[ch].location.load(Ordering::Relaxed) as u32
    }

    /// Channel `ch`'s running pointer.
    #[must_use]
    pub fn audio_pointer(&self, ch: usize) -> u32 {
        self.audio[ch].pointer.load(Ordering::Relaxed) as u32
    }

    /// Reload channel `ch`'s pointer from `AUDxLC`.
    ///
    /// Paula calls this when the channel starts and whenever its length
    /// counter runs out: `LCL,LCH` is the "chip memory location (starting
    /// address) of DMA data. Used to automatically restart pointers, such as
    /// ... the audio sample counter (whenever the audio length count is
    /// finished)" (Appendix B's legend).
    ///
    /// # Panics
    ///
    /// If `ch` is not 0–3.
    pub fn audio_restart(&self, ch: usize) {
        let location = self.audio[ch].location.load(Ordering::Relaxed);
        self.audio[ch].pointer.store(location, Ordering::Relaxed);
    }

    /// The next sample word of channel `ch`, advancing its pointer by two.
    ///
    /// `None`, with the pointer left where it is, if `AUDxEN` or `DMAEN` is
    /// clear.
    ///
    /// # Panics
    ///
    /// If `ch` is not 0–3.
    #[must_use]
    pub fn audio_fetch(&self, ch: usize) -> Option<u16> {
        if !self.enabled(DmaChannel::audio(ch)) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return None;
        }
        let pointer = self.audio[ch].pointer.load(Ordering::Relaxed) as u32;
        let word = self.peek(self.space().as_deref(), pointer);
        self.audio[ch]
            .pointer
            .store(u64::from(pointer.wrapping_add(2)), Ordering::Relaxed);
        Some(word)
    }

    // -- disk ----------------------------------------------------------------

    /// Agnus's half: `DSKPTH` or `DSKPTL` was written.
    pub fn set_disk_pointer(&self, high: bool, value: u16) {
        set_half(&self.disk, high, value);
    }

    /// `DSKPT`: where the next disk word is read from or stored to.
    #[must_use]
    pub fn disk_pointer(&self) -> u32 {
        self.disk.load(Ordering::Relaxed) as u32
    }

    /// The next word to write to the disk, advancing `DSKPT` by two.
    ///
    /// `None`, pointer unmoved, if `DSKEN` or `DMAEN` is clear.
    #[must_use]
    pub fn disk_fetch(&self) -> Option<u16> {
        if !self.enabled(DmaChannel::DISK) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return None;
        }
        let pointer = self.disk_pointer();
        let word = self.peek(self.space().as_deref(), pointer);
        self.disk
            .store(u64::from(pointer.wrapping_add(2)), Ordering::Relaxed);
        Some(word)
    }

    /// Store a word read from the disk at `DSKPT` and advance it by two — the
    /// `DSKDATR` "early read" transfer.
    ///
    /// `false`, pointer unmoved and nothing written, if `DSKEN` or `DMAEN` is
    /// clear.
    pub fn disk_store(&self, word: u16) -> bool {
        if !self.enabled(DmaChannel::DISK) {
            self.refused.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let pointer = self.disk_pointer();
        self.poke(self.space().as_deref(), pointer, word);
        self.disk
            .store(u64::from(pointer.wrapping_add(2)), Ordering::Relaxed);
        true
    }

    // -- the beam ------------------------------------------------------------

    /// Where Agnus's beam counters were at the last count it simulated.
    #[must_use]
    pub fn beam(&self) -> BeamPosition {
        let packed = self.beam.load(Ordering::Relaxed);
        BeamPosition {
            tick: self.tick.load(Ordering::Relaxed),
            field: self.field.load(Ordering::Relaxed),
            vpos: (packed >> 17) as u16,
            hpos: ((packed >> 1) & 0xffff) as u16,
            lof: packed & 1 != 0,
        }
    }

    /// Agnus's half: publish the beam.
    pub fn set_beam(&self, at: BeamPosition) {
        let packed = (u64::from(at.vpos) << 17) | (u64::from(at.hpos) << 1) | u64::from(at.lof);
        self.beam.store(packed, Ordering::Relaxed);
        self.tick.store(at.tick, Ordering::Relaxed);
        self.field.store(at.field, Ordering::Relaxed);
    }

    // -- diagnostics ---------------------------------------------------------

    /// How many transfers were refused because their channel was disabled.
    #[must_use]
    pub fn refused(&self) -> u64 {
        self.refused.load(Ordering::Relaxed)
    }

    /// How many transfers addressed a chip address nothing answered.
    #[must_use]
    pub fn faults(&self) -> u64 {
        self.faults.load(Ordering::Relaxed)
    }

    // -- snapshot ------------------------------------------------------------

    /// The guest-visible part, for Agnus's snapshot: `DMACON`, the four audio
    /// locations and pointers, `DSKPT`, and the word on the chip data bus.
    ///
    /// The bus word is **state and not derived**: a guest can read it back
    /// through any write-only custom register, and nothing else in the
    /// snapshot determines it — it is a fact about a cycle that has already
    /// happened.
    pub fn state(&self) -> [u32; 11] {
        let mut out = [0u32; 11];
        out[0] = self.dmacon() as u32;
        for (ch, audio) in self.audio.iter().enumerate() {
            out[1 + ch * 2] = audio.location.load(Ordering::Relaxed) as u32;
            out[2 + ch * 2] = audio.pointer.load(Ordering::Relaxed) as u32;
        }
        out[9] = self.disk_pointer();
        out[10] = u32::from(self.data.word());
        out
    }

    /// Put back what [`state`](Self::state) took.
    pub fn restore(&self, state: [u32; 11]) {
        self.set_dmacon(state[0] as u16);
        for (ch, audio) in self.audio.iter().enumerate() {
            audio
                .location
                .store(u64::from(state[1 + ch * 2]), Ordering::Relaxed);
            audio
                .pointer
                .store(u64::from(state[2 + ch * 2]), Ordering::Relaxed);
        }
        self.disk.store(u64::from(state[9]), Ordering::Relaxed);
        self.data.drive(state[10] as u16);
    }

    /// Back to power-on: every channel disabled, every pointer zero, nothing
    /// on the data bus.
    pub fn reset(&self) {
        self.restore([0; 11]);
        self.refused.store(0, Ordering::Relaxed);
        self.faults.store(0, Ordering::Relaxed);
    }
}

/// Replace the high or low word of a pointer register pair.
///
/// `H` is the more significant word and `L` the less, "so you write the 18-bit
/// address by moving one long word to the register whose name ends in H"
/// (chapter 2, *Location Registers*).
pub fn set_half(cell: &AtomicU64, high: bool, value: u16) {
    let old = cell.load(Ordering::Relaxed) as u32;
    cell.store(u64::from(merge_half(old, high, value)), Ordering::Relaxed);
}

/// The pure half of [`set_half`].
#[must_use]
#[inline]
pub const fn merge_half(old: u32, high: bool, value: u16) -> u32 {
    if high {
        (old & 0xffff) | ((value as u32) << 16)
    } else {
        (old & !0xffff) | value as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};

    fn with_ram(len: u64) -> ChipDma {
        let dma = ChipDma::new();
        let space = AddressSpace::new("chip", 24);
        let ram = Arc::new(Region::ram("chip", Arc::new(RamStore::new(len))));
        space.topology().map(ram, 0).unwrap();
        dma.attach_ram(Arc::new(space), len);
        dma
    }

    #[test]
    fn a_channel_needs_its_own_bit_and_the_master() {
        let dma = with_ram(0x8_0000);
        dma.set_dmacon(DmaChannel::AUD2.0);
        assert!(!dma.enabled(DmaChannel::AUD2), "no DMAEN");
        dma.set_dmacon(DMAEN | DmaChannel::AUD2.0);
        assert!(dma.enabled(DmaChannel::AUD2));
        assert!(!dma.enabled(DmaChannel::AUD1));
        assert_eq!(DmaChannel::audio(2), DmaChannel::AUD2);
    }

    #[test]
    fn audio_restarts_from_its_location_and_walks_by_words() {
        let dma = with_ram(0x8_0000);
        dma.poke(dma.space().as_deref(), 0x1000, 0x1234);
        dma.poke(dma.space().as_deref(), 0x1002, 0xabcd);
        dma.set_audio_location(1, true, 0x0000);
        dma.set_audio_location(1, false, 0x1000);
        dma.audio_restart(1);

        assert_eq!(dma.audio_fetch(1), None, "disabled: no word");
        assert_eq!(dma.audio_pointer(1), 0x1000, "and no advance");

        dma.set_dmacon(DMAEN | DmaChannel::AUD1.0);
        assert_eq!(dma.audio_fetch(1), Some(0x1234));
        assert_eq!(dma.audio_fetch(1), Some(0xabcd));
        assert_eq!(dma.audio_pointer(1), 0x1004);
        dma.audio_restart(1);
        assert_eq!(dma.audio_fetch(1), Some(0x1234));
        assert_eq!(dma.refused(), 1);
    }

    #[test]
    fn the_disk_reads_and_stores_through_one_pointer() {
        let dma = with_ram(0x8_0000);
        dma.set_disk_pointer(false, 0x2000);
        dma.set_dmacon(DMAEN | DmaChannel::DISK.0);
        assert!(dma.disk_store(0x4489));
        assert!(dma.disk_store(0x2aaa));
        assert_eq!(dma.disk_pointer(), 0x2004);
        dma.set_disk_pointer(false, 0x2000);
        assert_eq!(dma.disk_fetch(), Some(0x4489));
        assert_eq!(dma.disk_fetch(), Some(0x2aaa));
    }

    #[test]
    fn addresses_wrap_at_the_chip_ram_size_and_ignore_the_low_bit() {
        let dma = with_ram(0x8_0000);
        assert_eq!(dma.address_mask(), 0x7_fffe);
        dma.poke(dma.space().as_deref(), 0x10, 0x5555);
        assert_eq!(dma.peek(dma.space().as_deref(), 0x8_0011), 0x5555);
        assert_eq!(dma.faults(), 0);
    }

    #[test]
    fn the_beam_round_trips_through_its_packing() {
        let dma = ChipDma::new();
        let at = BeamPosition {
            tick: 99,
            field: 3,
            vpos: 312,
            hpos: 227,
            lof: true,
        };
        dma.set_beam(at);
        assert_eq!(dma.beam(), at);
    }

    #[test]
    fn a_pointer_pair_is_two_halves_of_one_long() {
        assert_eq!(merge_half(0x0001_2345, true, 0x0007), 0x0007_2345);
        assert_eq!(merge_half(0x0007_2345, false, 0xfffe), 0x0007_fffe);
    }
}
