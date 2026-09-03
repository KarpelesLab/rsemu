//! The board, as a firmware's tables have to describe it — **read out of the
//! machine description rather than written down here**.
//!
//! # Why this exists at all
//!
//! [`super::tables`] lays out an MP configuration table, a MADT and an SMBIOS
//! structure table. Every one of them is a *description of the board*: how many
//! processors it has, what their local APIC IDs are, where the I/O APIC
//! decodes, which global system interrupt an ISA IRQ arrives on. A firmware
//! that stated those as constants would publish "one processor" on a board with
//! two, which is the same as publishing nothing.
//!
//! `src/dev/q35/acpi.rs` solves this by *surveying the realized machine* — it
//! reads the local APIC's ID register through a debug read and finds each
//! device's base address by asking the address space where its region landed.
//! **That route is closed to a firmware**, and the reason is ordering rather
//! than taste: a ROM image is a *medium*, bound into `BuildOptions` before
//! [`crate::machine::build`] is called, so at the moment the image is assembled
//! there is no realized machine to survey — the machine is waiting for the
//! image. What does exist is the machine *description*, which is where the
//! addresses, the APIC IDs and the wires were written in the first place.
//!
//! So this module resolves the same `.machine` text the board is about to be
//! built from and reads the facts back out of it. The description is the
//! authority in both cases; q35 reads it after realization and a BIOS reads it
//! before, and they agree because there is only one document.
//!
//! # What is derived, and what is declared
//!
//! | Field | Where it comes from |
//! | --- | --- |
//! | the processors, and their order | every `cpu.x86` object, in declaration order |
//! | each processor's local APIC ID | the `pc.lapic` whose `intr` pin is wired to that processor, and its `id` property |
//! | which processor is the bootstrap one | that APIC's `bsp` property, defaulting as `pc.lapic` does |
//! | the local APIC address | where the bootstrap processor's APIC `regs` region is mapped |
//! | the I/O APIC's ID and address | the `pc.ioapic` object's `id`, and where its `regs` region is mapped |
//! | ISA IRQ *n* arrives on global system interrupt *m* | a pin wired to **both** `picN.irM` and `ioapic.irqK` — the two halves of the same source |
//! | PIC mode is implemented (IMCRP) | a `pc.imcr` object exists |
//! | an 8042 is present | a `pc.kbc` object exists |
//! | the 8259 chain reaches `LINTIN0` | a wire into the bootstrap APIC's `lint0` pin |
//! | each processor's CPU signature | the `cpu.x86` object's `model` property |
//! | the processor speed SMBIOS reports | the `cpu.x86` object's clock domain, resolved to hertz |
//!
//! Declared rather than derived, each for a reason that is a property of the
//! description language rather than a shortcut:
//!
//! * **The local APIC and I/O APIC version bytes.** Both are *register* values
//!   (*Intel SDM* Vol 3A §11.4.8; *82093AA* §3.2.2) and a description does not
//!   contain register contents. [`LOCAL_APIC_VERSION`] and
//!   [`IOAPIC_VERSION`] are what `dev::pc::apic` and `dev::pc::ioapic` report,
//!   and the numbers are the parts' own.
//! * **The I/O APIC's global system interrupt base.** Nothing in a `.machine`
//!   file says which global interrupt an I/O APIC's input 0 is; with one I/O
//!   APIC on the board it is 0 by definition (*ACPI* §5.2.12.3).
//! * **The ISA bus's ID**, which is 0 because it is the only bus with interrupt
//!   sources on it (*MP* §4.3.2: the BIOS assigns them sequentially from zero).
//! * **`LINTIN1` is the NMI.** A convention rather than a wire: the board takes
//!   its NMI to the *processor*, and no `.machine` statement says "this local
//!   interrupt input is the non-maskable one".
//!
//! # Sources
//!
//! *MultiProcessor Specification* version 1.4 (Intel, order 242016-006) §4.3
//! for what an entry has to say; *ACPI Specification* revision 6.5 (UEFI Forum)
//! §5.2.12 for the MADT's equivalents. No firmware source was read
//! (`ROADMAP.md` §1).

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use crate::core::{Error, Result};
use crate::machine::resolver::{ClockParent, MapTarget, Object, ObjectId, Resolved};
use crate::machine::{ResolveOptions, resolve_file};

// ---------------------------------------------------------------------------
// the class names this reads a board through
// ---------------------------------------------------------------------------

/// The processor class a PC board declares.
///
/// A string rather than `crate::cpu::x86::CLASS_NAME`, because `fw-pcbios` does
/// not imply `cpu-x86`: the ROM is a `Vec<u8>` and assembling it needs no core.
const CPU_CLASS: &str = "cpu.x86";
/// The local APIC's class.
const LAPIC_CLASS: &str = "pc.lapic";
/// The I/O APIC's class.
const IOAPIC_CLASS: &str = "pc.ioapic";
/// The 8259A's class, whose `mode` property says master or slave.
const PIC_CLASS: &str = "pc.pic";
/// The interrupt mode configuration register's class (*MP* §3.6.2.1).
const IMCR_CLASS: &str = "pc.imcr";
/// The keyboard controller's class, which is what `IAPC_BOOT_ARCH`'s 8042 bit
/// is asking about.
const KBC_CLASS: &str = "pc.kbc";

/// The region a local APIC or an I/O APIC publishes its register page as.
const REGS_REGION: &str = "regs";

// ---------------------------------------------------------------------------
// what a description cannot say
// ---------------------------------------------------------------------------

/// Bits 0-7 of the local APIC's version register, which an MP processor entry
/// carries (*MP* §4.3.1, Table 4-4).
///
/// `0x14` is an integrated APIC — *Intel SDM* Vol 3A §11.4.8 puts the external
/// 82489DX below `0x10` and every on-chip APIC in `0x10`-`0x15` — and it is
/// what `dev::pc::apic` answers a read of offset `0x30` with.
pub const LOCAL_APIC_VERSION: u8 = 0x14;

/// Bits 0-7 of the I/O APIC's version register (*MP* §4.3.3, Table 4-9).
///
/// `0x11` is the 82093AA's (*82093AA I/O APIC* §3.2.2), and what
/// `dev::pc::ioapic` reports through index `0x01`.
pub const IOAPIC_VERSION: u8 = 0x11;

/// The bus ID given to the board's ISA bus.
///
/// *MP* §4.3.2: "The BIOS assigns identifiers sequentially, starting at zero."
/// There is one bus with interrupt sources on this board, so it is zero.
pub const ISA_BUS_ID: u8 = 0;

/// Which local interrupt input the 8259A chain reaches.
///
/// PIC mode takes the 8259A's `INTR` to `LINTIN0` of the bootstrap processor's
/// local APIC (*MP* §3.6.2, Figure 3-3), which is what `wire imcr.lint0 ->
/// lapic0.lint0` is on this board. Derived where that wire exists, and this is
/// the number it resolves to.
pub const EXTINT_LINTIN: u8 = 0;

/// Which local interrupt input is the non-maskable one.
///
/// A convention, not a wire: every PC since the AT has taken the NMI to
/// `LINTIN1`, and no `.machine` statement expresses "this input is the NMI".
pub const NMI_LINTIN: u8 = 1;

/// `FEATURE FLAGS` bit 9, `APIC`: "Indicates that an integrated APIC is present
/// and hardware enabled" (*MP* §4.3.1, Table 4-6).
const CPUID_FEATURE_APIC: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// the facts
// ---------------------------------------------------------------------------

/// One processor, as an MP processor entry and a MADT local APIC entry
/// describe it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Processor {
    /// Its local APIC's ID.
    pub apic_id: u8,
    /// Bits 0-7 of that APIC's version register.
    pub apic_version: u8,
    /// Whether this is the processor that comes out of reset running.
    pub bootstrap: bool,
    /// Its CPU signature: stepping, model and family, as *MP* §4.3.1 defines
    /// them. Zero where the model has none to report.
    pub signature: u32,
    /// Its CPUID feature flags, as far as a firmware can establish them.
    pub features: u32,
    /// Its speed in megahertz, for SMBIOS' processor structure. Zero if the
    /// description gave the processor no clock domain.
    pub mhz: u16,
}

/// The board's I/O APIC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoApic {
    /// Its ID, as the description declares it.
    pub id: u8,
    /// Bits 0-7 of its version register.
    pub version: u8,
    /// Where its register page is mapped.
    pub address: u32,
    /// The global system interrupt its input 0 is (*ACPI* §5.2.12.3).
    pub gsi_base: u32,
}

/// One ISA interrupt source, and the I/O APIC input it also reaches.
///
/// The pair is what makes both an MP I/O interrupt assignment entry (*MP*
/// §4.3.4) and — where the two numbers differ — an ACPI interrupt source
/// override (*ACPI* §5.2.12.5) possible without either being guessed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct IsaInterrupt {
    /// The ISA IRQ number, as the 8259A pair numbers them: 0-7 on the master,
    /// 8-15 on the slave.
    pub irq: u8,
    /// The global system interrupt it arrives on, which is the I/O APIC input
    /// number plus that APIC's [`gsi_base`](IoApic::gsi_base).
    pub gsi: u32,
}

/// Everything the firmware's tables say about the board.
///
/// [`Platform::at`] is `machines/pc-at.machine`; [`Platform::from_machine`] is
/// any board, including that one with a second processor added.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Platform {
    /// The OEM identification the tables carry. Six characters, blank-filled.
    pub oem_id: [u8; 6],
    /// The product identification an MP configuration table carries. Twelve
    /// characters, blank-filled (*MP* §4.2).
    pub product_id: [u8; 12],
    /// The processors, in the order the description declares them.
    pub processors: Vec<Processor>,
    /// Where every processor reaches its own local APIC.
    pub lapic: u32,
    /// The I/O APIC, where there is one.
    pub ioapic: Option<IoApic>,
    /// The ISA interrupt sources that also reach the I/O APIC, by IRQ.
    pub interrupts: Vec<IsaInterrupt>,
    /// Whether the interrupt mode configuration register is present, which is
    /// what makes PIC mode rather than virtual wire mode the one implemented
    /// (*MP* §4.1, `IMCRP`).
    pub imcr: bool,
    /// Whether the 8259A chain reaches a local APIC's `LINTIN0`.
    pub extint: bool,
    /// Whether a port 60/64 keyboard controller is present, which ACPI's
    /// `IAPC_BOOT_ARCH` asks about (*ACPI* §5.2.9.3).
    pub kbc: bool,
}

impl Default for Platform {
    fn default() -> Platform {
        Platform::at()
    }
}

impl Platform {
    /// `machines/pc-at.machine`, as shipped: one 80486 with a local APIC, one
    /// I/O APIC, the AT's eight ISA interrupt sources, and an IMCR.
    ///
    /// The one place in this module where a number is written down rather than
    /// read, and it is guarded: `the_default_is_the_board_it_claims_to_be`
    /// asserts it is byte-for-byte what [`from_machine`](Platform::from_machine)
    /// derives from that file, wherever the build has `dev-pc` to supply the
    /// text. It exists because [`super::image`] takes no arguments — the
    /// firmware has to describe *some* board when nobody said which — and
    /// because an image whose bytes depended on which features were compiled in
    /// would not be reproducible.
    #[must_use]
    pub fn at() -> Platform {
        Platform {
            oem_id: *b"RSEMU ",
            product_id: *b"pc-at       ",
            processors: alloc::vec![Processor {
                apic_id: 0,
                apic_version: LOCAL_APIC_VERSION,
                bootstrap: true,
                signature: 0x0400,
                features: CPUID_FEATURE_APIC,
                mhz: 25,
            }],
            lapic: 0xfee0_0000,
            ioapic: Some(IoApic {
                id: 1,
                version: IOAPIC_VERSION,
                address: 0xfec0_0000,
                gsi_base: 0,
            }),
            interrupts: alloc::vec![
                IsaInterrupt { irq: 0, gsi: 2 },
                IsaInterrupt { irq: 1, gsi: 1 },
                IsaInterrupt { irq: 6, gsi: 6 },
                IsaInterrupt { irq: 8, gsi: 8 },
                IsaInterrupt { irq: 12, gsi: 12 },
                IsaInterrupt { irq: 14, gsi: 14 },
                IsaInterrupt { irq: 15, gsi: 15 },
            ],
            imcr: true,
            extint: true,
            kbc: true,
        }
    }

    /// Resolve `text` as a machine description and read the board out of it.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the text is not a machine description, if it
    /// declares no `cpu.x86`, or if the processors it declares have no local
    /// APIC mapped — a table describing no processor is worse than no table,
    /// because an operating system believes it.
    pub fn from_machine(file: &str, text: &str) -> Result<Platform> {
        let resolved = resolve_file(file, text, &ResolveOptions::new())?;
        Platform::from_resolved(&resolved)
    }

    /// Read the board out of an already-resolved description.
    ///
    /// # Errors
    ///
    /// As [`from_machine`](Platform::from_machine).
    pub fn from_resolved(machine: &Resolved) -> Result<Platform> {
        let survey = Survey::new(machine);
        let processors = survey.processors()?;
        let lapic = survey.lapic_address(&processors)?;
        Ok(Platform {
            oem_id: *b"RSEMU ",
            product_id: product_id(&machine.name),
            processors: processors.into_iter().map(|(_, p)| p).collect(),
            lapic,
            ioapic: survey.ioapic(),
            interrupts: survey.interrupts(),
            imcr: survey.has_class(IMCR_CLASS),
            extint: survey.extint(),
            kbc: survey.has_class(KBC_CLASS),
        })
    }

    /// How many processors the tables will describe.
    #[must_use]
    pub fn processor_count(&self) -> usize {
        self.processors.len()
    }
}

/// The machine's name, blank-filled into the twelve characters *MP* §4.2 gives
/// the product identification, and truncated if it is longer.
///
/// Strings in an MP configuration table are "coded in ASCII […] the extra
/// character locations are filled with space characters. Strings are not null
/// terminated" (*MP* §4). A name with a non-ASCII byte in it becomes a blank
/// rather than an unspecified encoding.
fn product_id(name: &str) -> [u8; 12] {
    let mut out = [b' '; 12];
    for (slot, byte) in out.iter_mut().zip(name.bytes()) {
        *slot = if byte.is_ascii_graphic() { byte } else { b' ' };
    }
    out
}

// ---------------------------------------------------------------------------
// reading a description
// ---------------------------------------------------------------------------

/// A resolved description with the queries this module asks of it.
struct Survey<'a> {
    machine: &'a Resolved,
}

impl<'a> Survey<'a> {
    fn new(machine: &'a Resolved) -> Survey<'a> {
        Survey { machine }
    }

    /// Whether any object of `class` was declared.
    fn has_class(&self, class: &str) -> bool {
        self.machine.objects.iter().any(|o| o.class == class)
    }

    /// Every object of `class`, with its id, in declaration order.
    fn objects_of(&self, class: &str) -> Vec<(ObjectId, &'a Object)> {
        self.machine
            .objects
            .iter()
            .enumerate()
            .filter(|(_, o)| o.class == class)
            .map(|(i, o)| (ObjectId(u32::try_from(i).unwrap_or(u32::MAX)), o))
            .collect()
    }

    /// The processors, each with the id of the local APIC that serves it.
    ///
    /// The pairing comes from `wire lapicN.intr -> cpuM.intr`, which is the
    /// statement that makes an APIC *that processor's* rather than merely
    /// present. A processor with no such wire is still a processor and is
    /// described — it takes the APIC declared in the same position, and where
    /// there is not one it is reported as unusable rather than omitted, because
    /// *MP* §4.3.1's `EN` bit exists for exactly that.
    fn processors(&self) -> Result<Vec<(Option<ObjectId>, Processor)>> {
        let cpus = self.objects_of(CPU_CLASS);
        if cpus.is_empty() {
            return Err(Error::Config {
                at: String::from("fw::pcbios"),
                message: format!(
                    "this machine declares no `{CPU_CLASS}`, so the firmware's tables would \
                     describe a board with no processor on it"
                ),
            });
        }
        let apics = self.objects_of(LAPIC_CLASS);

        let mut out = Vec::with_capacity(cpus.len());
        for (index, (cpu_id, cpu)) in cpus.iter().enumerate() {
            let apic = self
                .driver_of(*cpu_id, "intr", LAPIC_CLASS)
                .or_else(|| apics.get(index).map(|(id, o)| (*id, *o)));
            let (apic_id, bsp) = match apic {
                Some((_, object)) => {
                    let id = object
                        .props
                        .get("id")
                        .and_then(|v| v.as_uint())
                        .unwrap_or(0);
                    // `pc.lapic`'s own default: the APIC with ID 0 is the
                    // bootstrap processor unless the board says otherwise.
                    let bsp = object
                        .props
                        .get("bsp")
                        .and_then(crate::core::props::Value::as_bool)
                        .unwrap_or(id == 0);
                    (u8::try_from(id).unwrap_or(u8::MAX), bsp)
                }
                None => (u8::try_from(index).unwrap_or(u8::MAX), index == 0),
            };
            out.push((
                apic.map(|(id, _)| id),
                Processor {
                    apic_id,
                    apic_version: LOCAL_APIC_VERSION,
                    bootstrap: bsp,
                    signature: signature_of(cpu),
                    features: if apic.is_some() {
                        CPUID_FEATURE_APIC
                    } else {
                        0
                    },
                    mhz: self.megahertz(cpu),
                },
            ));
        }
        Ok(out)
    }

    /// Where every processor reaches its own local APIC.
    ///
    /// One address for the whole board, because that is what both tables have
    /// room for (*MP* §4.2's `ADDRESS OF LOCAL APIC`, *ACPI* §5.2.12's Local
    /// Interrupt Controller Address): on real silicon each processor's APIC
    /// answers the same physical address, and the one taken here is the
    /// bootstrap processor's.
    ///
    /// **This is where the tables and rsemu's model of a board disagree, and
    /// it is worth stating rather than discovering.** rsemu models each local
    /// APIC as its own device with its own mapping, so a two-processor board
    /// puts the second one somewhere else — `machines/pc-apic.machine` and
    /// `tests/kvm_pc_at_smp.rs` both use `0xfef00000`. The table says
    /// `0xfee00000`, which is true of the bootstrap processor and of every
    /// real machine, and is what an operating system will use on *both*
    /// processors. So an application processor that reads its own APIC ID
    /// through the architectural address reads the bootstrap processor's. The
    /// fix is a per-processor alias of one page, in the board model rather
    /// than here; until it exists, an operating system can *enumerate* the
    /// second processor and start it, and code running on it that programs
    /// its own APIC is programming the wrong one.
    fn lapic_address(&self, processors: &[(Option<ObjectId>, Processor)]) -> Result<u32> {
        let bootstrap = processors
            .iter()
            .find(|(_, p)| p.bootstrap)
            .or_else(|| processors.first());
        let apic = bootstrap.and_then(|(apic, _)| *apic);
        let address = apic.and_then(|id| self.mapping_of(id, REGS_REGION));
        address
            .and_then(|base| u32::try_from(base).ok())
            .ok_or_else(|| Error::Config {
                at: String::from("fw::pcbios"),
                message: String::from(
                    "no `pc.lapic` register page is mapped below 4 GiB in this machine, so the \
                     firmware's tables could not say where a processor reaches its own local APIC",
                ),
            })
    }

    /// The board's first I/O APIC, if it has one.
    fn ioapic(&self) -> Option<IoApic> {
        let (id, object) = *self.objects_of(IOAPIC_CLASS).first()?;
        let address = u32::try_from(self.mapping_of(id, REGS_REGION)?).ok()?;
        Some(IoApic {
            id: u8::try_from(
                object
                    .props
                    .get("id")
                    .and_then(|v| v.as_uint())
                    .unwrap_or(0),
            )
            .unwrap_or(u8::MAX),
            version: IOAPIC_VERSION,
            address,
            gsi_base: 0,
        })
    }

    /// Whether anything drives a local APIC's `LINTIN0`, which in PIC mode is
    /// the 8259A chain arriving (*MP* §3.6.2).
    fn extint(&self) -> bool {
        self.machine.wires.iter().any(|w| {
            w.to.port == "lint0"
                && self
                    .machine
                    .object(w.to.object)
                    .is_some_and(|o| o.class == LAPIC_CLASS)
        })
    }

    /// Every ISA interrupt source that reaches the I/O APIC as well as the
    /// 8259A pair.
    ///
    /// The derivation is the board's own redundancy: an AT-class machine wires
    /// each device's interrupt pin to *both* an 8259A input and an I/O APIC
    /// input, so a pin with two such wires states an (ISA IRQ, global system
    /// interrupt) pair in the machine file itself. `pit0.out0 -> pic1.ir0` plus
    /// `pit0.out0 -> ioapic.irq2` is the AT's famous timer offset, written down
    /// by the board rather than known by the firmware.
    fn interrupts(&self) -> Vec<IsaInterrupt> {
        let mut out: Vec<IsaInterrupt> = Vec::new();
        for wire in &self.machine.wires {
            let Some(irq) = self.isa_irq(wire.to.object, &wire.to.port) else {
                continue;
            };
            // The same driving pin, wherever else it goes.
            let gsi = self.machine.wires.iter().find_map(|other| {
                if other.from.object != wire.from.object || other.from.port != wire.from.port {
                    return None;
                }
                self.ioapic_input(other.to.object, &other.to.port)
            });
            let Some(gsi) = gsi else { continue };
            if !out.iter().any(|e| e.irq == irq) {
                out.push(IsaInterrupt { irq, gsi });
            }
        }
        out.sort_unstable();
        out
    }

    /// The ISA IRQ an 8259A input pin is, or `None` if the pin is not one.
    ///
    /// `ir0`-`ir7` on the master are IRQ0-7 and on the slave IRQ8-15, which is
    /// the AT's cascade and the numbering every program uses (*Intel 8259A*
    /// data sheet, and the AT's own wiring).
    fn isa_irq(&self, object: ObjectId, port: &str) -> Option<u8> {
        let pic = self.machine.object(object)?;
        if pic.class != PIC_CLASS {
            return None;
        }
        let index: u8 = port.strip_prefix("ir")?.parse().ok()?;
        let slave = pic.props.get("mode").and_then(|v| v.as_str()) == Some("slave");
        Some(if slave { index + 8 } else { index })
    }

    /// The global system interrupt an I/O APIC input pin is.
    fn ioapic_input(&self, object: ObjectId, port: &str) -> Option<u32> {
        let apic = self.machine.object(object)?;
        if apic.class != IOAPIC_CLASS {
            return None;
        }
        let index: u32 = port.strip_prefix("irq")?.parse().ok()?;
        Some(index)
    }

    /// What drives `port` of `object`, if it is an object of `class`.
    fn driver_of(
        &self,
        object: ObjectId,
        port: &str,
        class: &str,
    ) -> Option<(ObjectId, &'a Object)> {
        self.machine.wires.iter().find_map(|w| {
            if w.to.object != object || w.to.port != port {
                return None;
            }
            let from = self.machine.object(w.from.object)?;
            (from.class == class).then_some((w.from.object, from))
        })
    }

    /// Where `object`'s `region` is mapped, in the first space it is mapped in.
    fn mapping_of(&self, object: ObjectId, region: &str) -> Option<u64> {
        self.machine.maps.iter().find_map(|m| match &m.target {
            MapTarget::Region {
                object: mapped,
                region: named,
                ..
            } if *mapped == object && named.as_deref() == Some(region) => Some(m.base),
            _ => None,
        })
    }

    /// A processor's clock domain in whole megahertz, for SMBIOS.
    ///
    /// Follows the domain up to the crystal it divides, which is exact integer
    /// arithmetic all the way (`ROADMAP.md` §4.2) and is then truncated once,
    /// here, because SMBIOS' field is megahertz.
    fn megahertz(&self, cpu: &Object) -> u16 {
        let mut mul: i128 = 1;
        let mut div: i128 = 1;
        let mut clock = cpu.clock;
        // Bounded: a chain of clock domains is a tree by construction, and the
        // bound is belt and braces rather than a real limit.
        for _ in 0..16 {
            let Some(domain) = clock else { return 0 };
            mul *= i128::from(domain.mul);
            div *= i128::from(domain.div);
            match domain.parent {
                ClockParent::Osc(osc) => {
                    let Some(crystal) = self.machine.oscillator(osc) else {
                        return 0;
                    };
                    let hz = crystal.hz.numerator() * mul;
                    let per = crystal.hz.denominator() * div * 1_000_000;
                    if per <= 0 {
                        return 0;
                    }
                    return u16::try_from(hz / per).unwrap_or(u16::MAX);
                }
                ClockParent::Object(parent) => {
                    clock = self.machine.object(parent).and_then(|o| o.clock);
                }
            }
        }
        0
    }
}

/// The CPU signature an MP processor entry carries, from the core's model.
///
/// *MP* §4.3.1: "If the processor does not have a CPUID instruction, the BIOS
/// must fill these […] fields with information returned by the processor in the
/// EDX register after a processor reset" — which is the family and model, and
/// for an Intel486 DX is `0400h` (Table 4-5: family `0100`, model `0000`). An
/// 8086 has neither CPUID nor a reset signature, and Table 4-5 spells that
/// "not a valid CPU signature", which is all zeros.
fn signature_of(cpu: &Object) -> u32 {
    let model = cpu
        .props
        .get("model")
        .or_else(|| cpu.props.get("variant"))
        .and_then(|v| v.as_str())
        .unwrap_or("");
    match model {
        "80386" | "386" | "i386" => 0x0300,
        "80486" | "486" | "i486" => 0x0400,
        // Anything long-mode capable is a P6 or later, and family 6 model 0 is
        // the oldest signature that is true of all of them.
        "x86-64" | "x86_64" | "amd64" => 0x0600,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A two-processor AT, built the way `tests/kvm_pc_at_smp.rs` builds one.
    #[cfg(feature = "dev-pc")]
    fn two_processor_at() -> String {
        let text = String::from(crate::dev::pc::PC_AT);
        super::super::tests::add_second_processor(&text)
    }

    #[test]
    #[cfg(feature = "dev-pc")]
    fn the_default_is_the_board_it_claims_to_be() {
        // `Platform::at` is the one hard-coded description in this module, and
        // this is what stops it from drifting away from the file it describes.
        let derived = Platform::from_machine("pc-at.machine", crate::dev::pc::PC_AT)
            .expect("the shipped board resolves");
        assert_eq!(derived, Platform::at());
    }

    #[test]
    #[cfg(feature = "dev-pc")]
    fn a_second_processor_in_the_description_is_a_second_processor_in_the_tables() {
        let text = two_processor_at();
        let derived = Platform::from_machine("pc-at-smp.machine", &text).expect("it resolves");
        assert_eq!(derived.processor_count(), 2);
        assert!(derived.processors[0].bootstrap);
        assert!(!derived.processors[1].bootstrap);
        assert_eq!(derived.processors[0].apic_id, 0);
        assert_eq!(derived.processors[1].apic_id, 1);
        // The I/O APIC's ID moved out of the second APIC's way, and the table
        // follows it rather than restating 1.
        assert_eq!(derived.ioapic.expect("an I/O APIC").id, 2);
        // Everything else is the same board.
        assert_eq!(derived.interrupts, Platform::at().interrupts);
        assert_eq!(derived.lapic, Platform::at().lapic);
    }

    #[test]
    #[cfg(feature = "dev-pc")]
    fn the_timer_offset_is_read_off_the_wires() {
        let derived = Platform::from_machine("pc-at.machine", crate::dev::pc::PC_AT)
            .expect("the shipped board resolves");
        // `pit0.out0 -> pic1.ir0` and `pit0.out0 -> ioapic.irq2` in one file:
        // the AT's timer is ISA IRQ0 and global system interrupt 2, and neither
        // number is written in this crate.
        assert!(
            derived
                .interrupts
                .contains(&IsaInterrupt { irq: 0, gsi: 2 })
        );
        // The RTC is IRQ8 on the slave, and identity-mapped.
        assert!(
            derived
                .interrupts
                .contains(&IsaInterrupt { irq: 8, gsi: 8 })
        );
    }

    #[test]
    fn a_machine_with_no_processor_is_refused() {
        let text = "machine \"empty\" { space mem { width = 32 } }";
        let err = Platform::from_machine("empty.machine", text).expect_err("no processor");
        assert!(format!("{err}").contains("no processor"), "{err}");
    }

    #[test]
    fn the_product_identification_is_blank_filled_to_twelve() {
        assert_eq!(&product_id("pc-at"), b"pc-at       ");
        assert_eq!(&product_id("a-very-long-machine-name"), b"a-very-long-");
    }
}
