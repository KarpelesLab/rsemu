//! The taskfile seam: an ATA command as a **struct** rather than as eight
//! register writes in the right order.
//!
//! # Why this exists
//!
//! [`super`]'s five-call interface is the ribbon cable, and it is the right
//! model of one: a host adapter that decodes eight ports drives a command by
//! writing Features, Sector Count, three address bytes, Device and finally
//! Command, and the drive starts on the last of those. That is what
//! [`crate::dev::pc::ide`] does and it will keep doing it.
//!
//! A **Serial ATA** host adapter has no such cable. It receives a
//! Register - Host to Device FIS — a twenty-byte structure carrying the whole
//! command block at once — hands it to the drive, and gets a
//! Register - Device to Host FIS back. There is no ordering, no chip select and
//! no register offset: the command is one value.
//!
//! So there are two ways in, and the thing that must **not** happen is two
//! command sets. [`Taskfile`] is therefore not a second front door onto a copy
//! of the decode; it is loaded into the very same command block registers that
//! a port write would have left, and then the very same `AtaDisk::command`
//! dispatch runs. Delete this file and the drive is unchanged; delete
//! `AtaDisk::command` and both callers stop working. That is the falsifiable
//! form of "one implementation underneath".
//!
//! ```text
//!   pc::ide   ──► write_reg(Reg, u16) x8 ──┐
//!                                          ├──► AtaDisk::command  ──► the medium
//!   ahci      ──► taskfile_start(&Taskfile)┘
//! ```
//!
//! # The data phase
//!
//! A port-driven host moves a sector through the 16-bit data register, a word
//! per `IN`. A bus master moves it in whatever pieces its scatter/gather list
//! describes. [`AtaDisk::taskfile_read`] and [`AtaDisk::taskfile_write`] are
//! that same buffer, in bulk: they copy as much as the caller asks for out of
//! (or into) the block the drive currently has under `DRQ`, advance by exactly
//! that much, and when a block is finished they run the identical
//! `block_consumed` / `block_filled` path a word-at-a-time drain would have
//! run. So the busy/DRQ handshake, the per-block interrupt timing, the media
//! access and the completion write-back are shared, not reimplemented.
//!
//! **The caller loops.** One call moves at most the rest of the current block,
//! which is how a caller with a scatter list that does not align to a sector
//! stays correct without this module knowing what a scatter list is.
//!
//! # What is *not* here
//!
//! **No byte offsets.** A FIS is a byte layout and belongs to the host adapter
//! that receives one, exactly as a port number belongs to the adapter that
//! decodes it. `src/dev/ata/` still contains no register offset and no FIS
//! field position; grep for `0x27` here and find nothing.
//!
//! # Sources
//!
//! T13, *AT Attachment with Packet Interface - 6* (ATA/ATAPI-6, T13/1410D) for
//! the command block and the two data transfer protocols. The FIS that carries
//! a taskfile is *Serial ATA: High Speed Serialized AT Attachment*, Revision 1.0
//! §8.5.2/§8.5.3 — cited where it is used, in `src/dev/ahci/`, not here.

use super::{AtaDisk, DEV_HEAD, DEV_OBSOLETE, SECTOR, ST_DRQ, Volatile};

/// One ATA command, whole.
///
/// The six things a Register - Host to Device FIS carries that the drive acts
/// on. Every field is the *host's* value; what comes back is a [`Registers`].
///
/// The two-byte fields are the 48-bit Address feature set's two-deep register
/// FIFOs flattened: the low byte is what a 28-bit command reads and the high
/// byte is the one an EXT command's second half reads. A 28-bit command simply
/// never looks at the high halves, which is why one struct serves both and why
/// there is no `ext` flag here — **the opcode decides that**, and the opcode is
/// already in the struct.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Taskfile {
    /// The Command register: which command to run.
    pub command: u8,
    /// Features, both halves.
    pub feature: u16,
    /// Sector Count, both halves. Zero means the maximum, as it does in the
    /// register.
    pub count: u16,
    /// The address: a 48-bit LBA, a 28-bit one in the low 28 bits, or a CHS
    /// triple packed the way the registers pack it — sector in bits 7:0 and
    /// cylinder in bits 23:8, with the head in [`Taskfile::device`].
    pub lba: u64,
    /// The Device register: `DEV`, `LBA` and the head or LBA bits 27:24.
    pub device: u8,
}

/// The command block as the drive left it.
///
/// What a Register - Device to Host FIS reports: the status and error the
/// command ended with, and the address write-back ATA leaves in the registers
/// so that a host which chains commands knows where the last one got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    /// The Status register.
    pub status: u8,
    /// The Error register, meaningful when `status` has `ERR` set.
    pub error: u8,
    /// Sector Count, both halves.
    pub count: u16,
    /// The address write-back, packed as [`Taskfile::lba`] is.
    pub lba: u64,
    /// The Device register.
    pub device: u8,
}

/// Where a command has got to.
///
/// Returned by [`AtaDisk::taskfile_start`] and by
/// [`AtaDisk::taskfile_phase`] afterwards. A caller moves data while it says
/// [`Phase::Data`] and reads [`AtaDisk::taskfile_registers`] when it says
/// [`Phase::Done`] — including when a command that never had a data phase
/// says it immediately.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Nothing more to move. The command is over, successfully or not.
    Done,
    /// The drive is holding `DRQ` up over a block.
    Data {
        /// Host to device. A read is `false`.
        out: bool,
        /// Whether the command uses the **DMA** data transfer protocol rather
        /// than the PIO one.
        ///
        /// The bytes are the same either way; what differs is what the host
        /// adapter puts on the link around them — a Serial ATA PIO command is
        /// announced by a PIO Setup FIS before every block and a DMA one is
        /// not. An adapter that got this wrong would work with one driver and
        /// hang another, which is why the drive says rather than the adapter
        /// guessing from the opcode.
        dma: bool,
        /// How many bytes are left in the block the drive currently has open.
        ///
        /// The size a PIO Setup FIS's Transfer Count reports, and the most a
        /// single [`AtaDisk::taskfile_read`] or [`AtaDisk::taskfile_write`]
        /// will move.
        block: u64,
    },
}

impl Phase {
    /// Whether there is still data to move.
    #[must_use]
    pub fn is_data(self) -> bool {
        matches!(self, Phase::Data { .. })
    }
}

impl AtaDisk {
    /// The phase `state` is in.
    fn phase_of(state: &Volatile) -> Phase {
        if state.status & ST_DRQ == 0 {
            return Phase::Done;
        }
        // `IDENTIFY DEVICE` and its relatives raise DRQ with no transfer behind
        // them: one block, device to host, and PIO by construction — there is
        // no DMA form of them in this command set.
        let (out, dma) = match state.xfer.as_ref() {
            Some(xfer) => (xfer.out, xfer.dma),
            None => (false, false),
        };
        Phase::Data {
            out,
            dma,
            block: (state.buf.len() - state.pos) as u64,
        }
    }

    /// Run `tf`.
    ///
    /// The command block is loaded from the struct and then the ordinary
    /// command dispatch runs — the same one a write to the Command register
    /// reaches. What comes back is where it got to: [`Phase::Done`] for a
    /// command with no data phase or one that failed, and [`Phase::Data`] for
    /// one that wants bytes moved.
    ///
    /// # Selection
    ///
    /// A taskfile addresses **this** drive, whatever the `DEV` bit says. That
    /// is not a shortcut: a Serial ATA port has exactly one device on it, there
    /// is no second drive to share the register block with, and the `DEV` bit
    /// in a Register - Host to Device FIS is therefore data the command carries
    /// rather than a selection between two listeners. A drive reached this way
    /// is left selected, which is what a port that only ever speaks to one
    /// device leaves behind.
    pub fn taskfile_start(&self, tf: &Taskfile) -> Phase {
        let mut state = self.state.lock();
        state.device = tf.device;
        state.selected = true;
        // While the host is holding SRST asserted the command block is the
        // drive's, not the host's — the same refusal `write_reg` makes.
        if state.in_reset {
            return AtaDisk::phase_of(&state);
        }
        // Low byte current, high byte previous: exactly the two writes an EXT
        // command's host performs, high half first.
        state
            .features
            .load(tf.feature as u8, (tf.feature >> 8) as u8);
        state.count.load(tf.count as u8, (tf.count >> 8) as u8);
        state.lba_low.load(tf.lba as u8, (tf.lba >> 24) as u8);
        state
            .lba_mid
            .load((tf.lba >> 8) as u8, (tf.lba >> 32) as u8);
        state
            .lba_high
            .load((tf.lba >> 16) as u8, (tf.lba >> 40) as u8);
        self.command(&mut state, tf.command);
        AtaDisk::phase_of(&state)
    }

    /// Where the command in flight has got to.
    #[must_use]
    pub fn taskfile_phase(&self) -> Phase {
        AtaDisk::phase_of(&self.state.lock())
    }

    /// The command block, for the completion a host adapter reports.
    ///
    /// No side effect: this is the *Alternate* Status register's promise, which
    /// is what makes it safe for a debugger as well as for the adapter.
    #[must_use]
    pub fn taskfile_registers(&self) -> Registers {
        let state = self.state.lock();
        Registers {
            status: state.status,
            error: state.error,
            count: u16::from(state.count.current) | (u16::from(state.count.previous) << 8),
            lba: u64::from(state.lba_low.current)
                | (u64::from(state.lba_mid.current) << 8)
                | (u64::from(state.lba_high.current) << 16)
                | (u64::from(state.lba_low.previous) << 24)
                | (u64::from(state.lba_mid.previous) << 32)
                | (u64::from(state.lba_high.previous) << 40),
            device: state.device | DEV_OBSOLETE,
        }
    }

    /// The head the Device register currently names, for a CHS write-back.
    #[must_use]
    pub fn taskfile_head(&self) -> u8 {
        self.state.lock().device & DEV_HEAD
    }

    /// Take the pending `INTRQ`, as reading the Status register does.
    ///
    /// A host adapter latches the drive's interrupt into its own status when it
    /// takes the completion; the drive's own line drops at that point, and a
    /// model that left it up would hand the *next* command a stale interrupt.
    ///
    /// Returns whether there was one — the `I` bit a Register - Device to Host
    /// FIS carries.
    pub fn taskfile_acknowledge(&self) -> bool {
        let mut state = self.state.lock();
        let had = state.irq;
        state.irq = false;
        had
    }

    /// Copy out of the block the drive is holding under `DRQ`, device to host.
    ///
    /// Moves `min(dst.len(), the rest of the block)` bytes and returns how many;
    /// zero when there is no device-to-host block open, which is how a caller
    /// with a scatter list longer than the transfer stops. Finishing a block
    /// runs the same completion path a word-at-a-time drain runs: the next block
    /// is fetched from the medium, or the command completes.
    pub fn taskfile_read(&self, dst: &mut [u8]) -> u64 {
        let mut state = self.state.lock();
        if state.status & ST_DRQ == 0 {
            return 0;
        }
        if state.xfer.as_ref().is_some_and(|x| x.out) {
            // A host-to-device block. Reading it would hand the caller the
            // buffer it is supposed to be filling.
            return 0;
        }
        let at = state.pos;
        let n = core::cmp::min(dst.len(), state.buf.len().saturating_sub(at));
        if n == 0 {
            return 0;
        }
        dst[..n].copy_from_slice(&state.buf[at..at + n]);
        state.pos = at + n;
        if state.pos >= state.buf.len() {
            self.block_consumed(&mut state);
        }
        n as u64
    }

    /// Fill the block the drive is holding under `DRQ`, host to device.
    ///
    /// Moves `min(src.len(), the rest of the block)` bytes and returns how many;
    /// zero when there is no host-to-device block open. Finishing a block writes
    /// it to the medium and opens the next one, exactly as filling it through
    /// the data register would.
    pub fn taskfile_write(&self, src: &[u8]) -> u64 {
        let mut state = self.state.lock();
        if state.status & ST_DRQ == 0 {
            return 0;
        }
        if !state.xfer.as_ref().is_some_and(|x| x.out) {
            return 0;
        }
        let at = state.pos;
        let n = core::cmp::min(src.len(), state.buf.len().saturating_sub(at));
        if n == 0 {
            return 0;
        }
        state.buf[at..at + n].copy_from_slice(&src[..n]);
        state.pos = at + n;
        if state.pos >= state.buf.len() {
            self.block_filled(&mut state);
        }
        n as u64
    }

    /// How many bytes a sector holds, so a caller can size a staging buffer
    /// without naming the constant itself.
    #[must_use]
    pub fn sector_bytes(&self) -> u64 {
        SECTOR
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RamStore;
    use crate::dev::ata::disk::{
        CTL_SRST, ERR_ABRT, ERR_IDNF, Geometry, Identity, Medium, Position, Reg, ST_BSY, ST_DRDY,
        ST_ERR, cmd, default_geometry,
    };
    use alloc::sync::Arc;
    use alloc::vec;
    use alloc::vec::Vec;

    const SECTORS: u64 = 128;
    const BYTES: u64 = SECTORS * SECTOR;

    fn stamp(lba: u64) -> Vec<u8> {
        let mut out = vec![0u8; SECTOR as usize];
        out[0] = lba as u8;
        out[1] = 0xc3;
        out[511] = !(lba as u8);
        out
    }

    fn drive(dma: bool) -> (AtaDisk, Arc<RamStore>) {
        let store = Arc::new(RamStore::new(BYTES));
        for lba in 0..SECTORS {
            RamStore::write_at(&store, lba * SECTOR, &stamp(lba)).expect("it fits");
        }
        let mut id =
            Identity::new(SECTORS, default_geometry(SECTORS), true, 16).expect("an identity");
        id.dma = dma;
        let disk =
            AtaDisk::with_medium(id, Position::Device0, Arc::clone(&store) as Arc<dyn Medium>)
                .expect("the medium fits");
        (disk, store)
    }

    /// Drain a whole data-in phase into one vector, the way a host adapter with
    /// a scatter list does — in pieces that do not divide the block.
    fn drain(disk: &AtaDisk) -> Vec<u8> {
        let mut out = Vec::new();
        let mut chunk = [0u8; 100];
        while disk.taskfile_phase().is_data() {
            let n = disk.taskfile_read(&mut chunk);
            assert!(n > 0, "a data phase that moved nothing");
            out.extend_from_slice(&chunk[..n as usize]);
        }
        out
    }

    /// The same sectors, fetched the way `dev/pc/ide` fetches them: register
    /// writes and a drain of the 16-bit data port.
    fn by_registers(disk: &AtaDisk, command: u8, lba: u32, count: u8) -> Vec<u8> {
        disk.write_reg(
            Reg::Device,
            u16::from(DEV_OBSOLETE | 0x40 | ((lba >> 24) as u8 & 0x0f)),
        );
        disk.write_reg(Reg::SectorCount, u16::from(count));
        disk.write_reg(Reg::LbaLow, u16::from(lba as u8));
        disk.write_reg(Reg::LbaMid, u16::from((lba >> 8) as u8));
        disk.write_reg(Reg::LbaHigh, u16::from((lba >> 16) as u8));
        disk.write_reg(Reg::Command, u16::from(command));
        let want = u64::from(if count == 0 { 256u16 } else { u16::from(count) }) * SECTOR;
        let mut out = Vec::new();
        while (out.len() as u64) < want {
            let word = disk.read_reg(Reg::Data, false);
            out.push(word as u8);
            out.push((word >> 8) as u8);
        }
        out
    }

    #[test]
    fn a_taskfile_read_and_a_register_read_return_the_same_bytes() {
        // **The claim this module exists to make good.** Two front doors, one
        // command set: if these ever disagree, the decode has been written
        // twice.
        let (disk, _store) = drive(false);
        let by_ports = by_registers(&disk, cmd::READ_SECTORS, 7, 3);

        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS,
            count: 3,
            lba: 7,
            device: 0x40,
            ..Taskfile::default()
        });
        assert!(matches!(
            phase,
            Phase::Data {
                out: false,
                dma: false,
                block: 512
            }
        ));
        let by_taskfile = drain(&disk);

        assert_eq!(by_taskfile, by_ports);
        assert_eq!(&by_taskfile[..512], &stamp(7)[..]);
        assert_eq!(&by_taskfile[1024..], &stamp(9)[..]);
        assert_eq!(disk.taskfile_registers().status, ST_DRDY | 0x10);
    }

    #[test]
    fn identify_device_is_the_same_512_bytes_either_way() {
        let (disk, _store) = drive(false);
        disk.write_reg(Reg::Command, u16::from(cmd::IDENTIFY));
        let mut by_ports = Vec::new();
        while by_ports.len() < 512 {
            let word = disk.read_reg(Reg::Data, false);
            by_ports.push(word as u8);
            by_ports.push((word >> 8) as u8);
        }

        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::IDENTIFY,
            ..Taskfile::default()
        });
        // No transfer behind it, so it is PIO device-to-host by construction.
        assert!(matches!(
            phase,
            Phase::Data {
                out: false,
                dma: false,
                block: 512
            }
        ));
        assert_eq!(drain(&disk), by_ports);
        assert_eq!(disk.taskfile_phase(), Phase::Done);
    }

    #[test]
    fn a_forty_eight_bit_taskfile_reaches_the_high_halves_of_the_registers() {
        // The trap the two-deep FIFOs exist to make possible and easy to get
        // wrong: an EXT command's address is six bytes and its count is two, and
        // the high halves live in the `previous` slot of the same registers.
        let (disk, _store) = drive(false);
        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS_EXT,
            count: 2,
            lba: 100,
            device: 0x40,
            ..Taskfile::default()
        });
        assert!(phase.is_data());
        let got = drain(&disk);
        assert_eq!(&got[..512], &stamp(100)[..]);
        assert_eq!(&got[512..], &stamp(101)[..]);
        // ATA leaves the last sector transferred in the registers, and an EXT
        // command leaves all six bytes of it.
        let regs = disk.taskfile_registers();
        assert_eq!(regs.lba, 101);
        assert_eq!(regs.count, 0);
        assert_eq!(regs.status & ST_ERR, 0);

        // And an address that needs all six bytes comes back as an error about
        // the address rather than as a truncated read of a different sector.
        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS_EXT,
            count: 1,
            lba: 0x0000_5566_7788_99aa,
            device: 0x40,
            ..Taskfile::default()
        });
        assert_eq!(phase, Phase::Done, "an address past the end has no data");
        let regs = disk.taskfile_registers();
        assert_eq!(regs.status & ST_ERR, ST_ERR);
        assert_eq!(regs.error, ERR_IDNF);
    }

    #[test]
    fn a_taskfile_write_reaches_the_medium() {
        let (disk, store) = drive(false);
        let payload: Vec<u8> = (0..SECTOR as usize).map(|i| (i as u8) ^ 0x77).collect();
        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::WRITE_SECTORS,
            count: 1,
            lba: 20,
            device: 0x40,
            ..Taskfile::default()
        });
        assert!(matches!(phase, Phase::Data { out: true, .. }));
        // In pieces that do not divide the sector, as a scatter list gives them.
        let mut at = 0usize;
        while disk.taskfile_phase().is_data() {
            let end = (at + 300).min(payload.len());
            let n = disk.taskfile_write(&payload[at..end]);
            assert!(n > 0);
            at += n as usize;
        }
        assert_eq!(at, payload.len());

        let mut got = vec![0u8; SECTOR as usize];
        Medium::read_at(&*store, 20 * SECTOR, &mut got).expect("the medium reads");
        assert_eq!(got, payload);
        // The sector next door is untouched, which is what catches an off-by-one.
        Medium::read_at(&*store, 21 * SECTOR, &mut got).expect("the medium reads");
        assert_eq!(got, stamp(21));
    }

    #[test]
    fn a_read_and_a_write_will_not_move_each_others_blocks() {
        // A host adapter that got the direction backwards must move nothing
        // rather than hand the caller the buffer it was supposed to fill.
        let (disk, _store) = drive(false);
        disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS,
            count: 1,
            lba: 1,
            device: 0x40,
            ..Taskfile::default()
        });
        assert_eq!(disk.taskfile_write(&[0u8; 512]), 0, "a data-in block");

        disk.taskfile_start(&Taskfile {
            command: cmd::WRITE_SECTORS,
            count: 1,
            lba: 1,
            device: 0x40,
            ..Taskfile::default()
        });
        let mut buf = [0u8; 512];
        assert_eq!(disk.taskfile_read(&mut buf), 0, "a data-out block");

        // And with no command in flight at all, neither moves anything.
        disk.taskfile_start(&Taskfile {
            command: cmd::FLUSH_CACHE,
            ..Taskfile::default()
        });
        assert_eq!(disk.taskfile_phase(), Phase::Done);
        assert_eq!(disk.taskfile_read(&mut buf), 0);
        assert_eq!(disk.taskfile_write(&buf), 0);
    }

    #[test]
    fn the_dma_family_answers_only_on_a_drive_that_has_dma() {
        // The default is off, which is what keeps an AT-class IDE cable honest:
        // nothing on that board moves bytes for the drive.
        let (plain, _store) = drive(false);
        let phase = plain.taskfile_start(&Taskfile {
            command: cmd::READ_DMA_EXT,
            count: 1,
            lba: 3,
            device: 0x40,
            ..Taskfile::default()
        });
        assert_eq!(phase, Phase::Done);
        let regs = plain.taskfile_registers();
        assert_eq!(regs.status & ST_ERR, ST_ERR);
        assert_eq!(
            regs.error, ERR_ABRT,
            "aborted, as a device without a command must"
        );

        let (fast, _store) = drive(true);
        let phase = fast.taskfile_start(&Taskfile {
            command: cmd::READ_DMA_EXT,
            count: 1,
            lba: 3,
            device: 0x40,
            ..Taskfile::default()
        });
        assert!(matches!(
            phase,
            Phase::Data {
                out: false,
                dma: true,
                block: 512
            }
        ));
        assert_eq!(drain(&fast), stamp(3));
    }

    #[test]
    fn the_two_protocols_read_the_same_sector() {
        // The bytes are the same; only the protocol flag differs, which is the
        // whole reason it is a flag and not a second path.
        let (disk, _store) = drive(true);
        disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS_EXT,
            count: 1,
            lba: 11,
            device: 0x40,
            ..Taskfile::default()
        });
        let by_pio = drain(&disk);
        disk.taskfile_start(&Taskfile {
            command: cmd::READ_DMA_EXT,
            count: 1,
            lba: 11,
            device: 0x40,
            ..Taskfile::default()
        });
        let by_dma = drain(&disk);
        assert_eq!(by_pio, by_dma);
        assert_eq!(by_pio, stamp(11));
    }

    #[test]
    fn a_drive_held_in_software_reset_starts_nothing() {
        let (disk, _store) = drive(false);
        disk.write_device_control(CTL_SRST);
        let phase = disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS,
            count: 1,
            lba: 0,
            device: 0x40,
            ..Taskfile::default()
        });
        assert_eq!(phase, Phase::Done);
        assert_eq!(disk.taskfile_registers().status, ST_BSY);

        // Released, and the drive is back with its signature and no interrupt.
        disk.write_device_control(0);
        let regs = disk.taskfile_registers();
        assert_eq!(regs.status, ST_DRDY | 0x10);
        assert_eq!(regs.count, 1);
        assert_eq!(regs.lba, 1);
        assert!(
            !disk.taskfile_acknowledge(),
            "a software reset raises no INTRQ"
        );
    }

    #[test]
    fn the_interrupt_is_taken_once() {
        let (disk, _store) = drive(false);
        disk.taskfile_start(&Taskfile {
            command: cmd::FLUSH_CACHE,
            ..Taskfile::default()
        });
        assert!(disk.taskfile_acknowledge(), "the command completed");
        assert!(!disk.taskfile_acknowledge(), "and the line dropped");
    }

    #[test]
    fn a_chs_taskfile_names_the_same_sector_an_lba_one_does() {
        // A taskfile carries CHS the way the registers do — sector in the low
        // byte, cylinder in the next two — and the drive decodes it against the
        // *current* translation. Both paths must land on the same platter.
        let (disk, _store) = drive(false);
        let geometry: Geometry = disk.current_geometry();
        let lba = 40u64;
        let head = (lba / u64::from(geometry.sectors)) % u64::from(geometry.heads);
        let sector = lba % u64::from(geometry.sectors) + 1;
        let cylinder = lba / (u64::from(geometry.sectors) * u64::from(geometry.heads));

        disk.taskfile_start(&Taskfile {
            command: cmd::READ_SECTORS,
            count: 1,
            lba: sector | (cylinder << 8),
            device: head as u8,
            ..Taskfile::default()
        });
        assert_eq!(drain(&disk), stamp(lba));
        assert_eq!(disk.sector_bytes(), SECTOR);
        assert_eq!(disk.taskfile_head(), head as u8);
    }
}
