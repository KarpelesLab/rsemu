//! SD/MMC: the card, independent of whatever is talking to it.
//!
//! [`card`] holds the whole of it — the command set, the state machine, the
//! OCR/CID/CSD/SCR registers, and the difference between standard-capacity byte
//! addressing and high-capacity block addressing. That module's own
//! documentation argues where the line between a card and a controller falls
//! and why it falls there; the short version is that an SPI-mode card and an
//! SD-mode card are the same die behind different framing, so the card model
//! must not contain either framing.
//!
//! # Finding each other
//!
//! A card and its controller are separate objects in a machine description, and
//! there is no `core::bus` yet, so they meet through [`slots`] — a named card
//! slot in the build's [`HostObjects`](crate::core::hosts::HostObjects), the
//! same rendezvous pattern `bus::spi::buses` and `host::chardev::ports` use.
//! Both ends name the same slot (`slot = "sd0"`), and whichever is constructed
//! first creates it. A slot with no card in it is a slot with no card in it: a
//! controller finds nothing, every command times out, and firmware concludes
//! there is no card — which is what an empty socket does.
//!
//! # Sources
//!
//! The SD Association's *Physical Layer Simplified Specification*, which
//! `docs/buses/storage.md` names as the free and correct source for this
//! transport. Section references are on the items they justify.

pub mod card;

pub use card::{BusMode, CardDevice, Data, Identity, IdentityText, Phase, Reply, SdCard};

/// The slot name a card and a controller get when neither says.
pub const DEFAULT_SLOT: &str = "sd0";

/// Named card slots: how a card and its controller find each other.
///
/// A [`Slot`](slots::Slot) is the socket, not the card. It exists whether or not something
/// is in it, because that is the honest model of a board with an empty SD
/// socket soldered to it — and because the controller is usually constructed
/// before the card.
pub mod slots {
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;
    use core::fmt;

    use super::card::SdCard;
    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::sync::{LockRank, Mutex};

    /// The kind a card slot is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::new("sd-slot");

    /// Where a card socket's lock sits in the ranked order.
    ///
    /// A controller looks the card up *before* it touches its own registers and
    /// releases the socket immediately, so this rank sits above a controller's
    /// register lock and well below the card's own state. The whole ladder a
    /// command travels is:
    ///
    /// ```text
    ///   CPU session                (BUS 0x4000)
    ///     → the card socket        (0x4c00, here)
    ///       → the controller's registers  (a controller's own rank)
    ///         → the card's own state      (DEVICE 0x5000)
    ///           → the controller's interrupt wire (WIRE 0x6000)
    /// ```
    ///
    /// `LockRank::new` is what the ladders defined outside `core::sync` use;
    /// `bus::spi` and `bus::usb` define theirs the same way.
    pub const SLOT_RANK: LockRank = LockRank::new(0x4c00);

    /// One card socket.
    ///
    /// Holds at most one card. `Mutex` rather than an atomic because the
    /// contents are an `Arc` and this is a cold path — a card is inserted once,
    /// during construction, and looked at once per command afterwards.
    pub struct Slot {
        card: Mutex<Option<Arc<SdCard>>>,
    }

    impl fmt::Debug for Slot {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Slot")
                .field("occupied", &self.card.lock().is_some())
                .finish()
        }
    }

    impl Slot {
        /// An empty socket.
        #[must_use]
        pub fn new() -> Slot {
            Slot {
                card: Mutex::with_rank(SLOT_RANK, None),
            }
        }

        /// Put `card` in, if the socket is empty.
        ///
        /// # Errors
        ///
        /// The card back, unchanged, if something is already in the socket.
        /// The caller has the names and makes the message; handing the card
        /// back rather than dropping it means a host that loses the race still
        /// has something to put somewhere else.
        pub fn insert(&self, card: Arc<SdCard>) -> core::result::Result<(), Arc<SdCard>> {
            let mut slot = self.card.lock();
            if slot.is_some() {
                return Err(card);
            }
            *slot = Some(card);
            Ok(())
        }

        /// Take the card out, if there is one.
        pub fn eject(&self) -> Option<Arc<SdCard>> {
            self.card.lock().take()
        }

        /// The card in the socket, if any.
        #[must_use]
        pub fn card(&self) -> Option<Arc<SdCard>> {
            self.card.lock().clone()
        }

        /// Whether there is a card in it.
        #[must_use]
        pub fn is_occupied(&self) -> bool {
            self.card.lock().is_some()
        }
    }

    impl Default for Slot {
        fn default() -> Slot {
            Slot::new()
        }
    }

    /// The slot `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous: called before a build to put a card
    /// in, or after one to take it out.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object is already open
    /// under that name, which is a collision between two host modules rather
    /// than anything a machine file can cause.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Slot>> {
        hosts.open(KIND, name, Slot::new)
    }

    /// The slot `name` refers to in the build these properties are being read
    /// for, creating it on first mention.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs to
    /// no build gets a private slot, so a device a unit test constructed
    /// directly still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Slot>> {
        props.host(KIND, name, Slot::new)
    }

    /// The slot called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Slot>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open slot name, in name order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }
}

/// Add every `sd` class to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::error::Result<()> {
    card::register(registry)
}

/// Bind every `sd` class into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::error::Result<()> {
    card::bind(bindings)
}

/// What the validator should know about the `sd` classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![card::schema()]
}
