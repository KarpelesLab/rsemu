#![no_main]
//! The translation block cache's own invariants, under arbitrary sequences of
//! insert, link, invalidate and evict.
//!
//! `ROADMAP.md` §9.1 asks for block chaining. Patching an exit is easy;
//! **unpatching** one is where translation caches go wrong, because the
//! pointer that has to be cleared is held by a block that is not the one being
//! invalidated. The `jit::cache` module keeps back edges for exactly that, and
//! this target is what says the back edges stay symmetric under a sequence no
//! test author thought of.
//!
//! Three things are checked after every operation:
//!
//! * `BlockCache::check` — every exit points at a live slot with the stamp the
//!   link recorded, every such target holds the matching back edge, every back
//!   edge points at a block whose exit really points back, and every block is
//!   in its page's index and in the page filter.
//! * `CacheStats::stale_links` stays zero. A link followed to a slot that has
//!   since been reused is caught rather than executed, and a non-zero count
//!   means a back edge was missed even though nothing crashed.
//! * A block that comes back from `lookup` is the one that was inserted for
//!   that `(pc, key)`, checked by its entry PC.
//!
//! # Input encoding
//!
//! One byte of opcode, then operands, repeated. Decoded by hand rather than
//! through `arbitrary`'s derive, for the reason `state_roundtrip` gives: a
//! dependency bump must not reinterpret every committed seed.
//!
//! ```text
//!   00  insert   pc(2) key(1) page(1)
//!   01  link     from(1) to(1) pc(2)
//!   02  follow   from(1) pc(2)
//!   03  lookup   pc(2) key(1)
//!   04  write    phys(2) len(1)
//!   05  evict    — insert until the capacity forces one
//!   06  sync     topology(1)
//!   07  flush
//! ```
//!
//! Addresses are small and deliberately collide: the interesting states are
//! the ones where two blocks share a page, a slot is reused immediately, and a
//! link is made to a block that is about to go.

use libfuzzer_sys::fuzz_target;

use rsemu::ir::{BlockBuilder, Const, Type};
use rsemu::jit::{BlockCache, BlockId, Epoch};

/// Small enough that collisions are the common case rather than the exception.
const CAPACITY: usize = 24;

/// A one-instruction block that carries its own entry PC, so a lookup that
/// returned the wrong block is visible rather than merely suspected.
fn block(pc: u64) -> rsemu::ir::Block {
    let mut b = BlockBuilder::new(pc, 0);
    let _ = b.imm(Type::I64, Const::Int(u128::from(pc)));
    b.exit_tb();
    b.finish()
}

fuzz_target!(|data: &[u8]| {
    let mut cache = BlockCache::with_capacity(CAPACITY);
    // Every id ever handed out, so `link` and `follow` can be pointed at ones
    // that have since been invalidated — which is the case the stamp exists
    // for.
    let mut ids: Vec<(BlockId, u64)> = Vec::new();
    let mut topology = 0u64;

    let mut at = 0usize;
    let byte = |at: &mut usize| {
        let b = data.get(*at).copied().unwrap_or(0);
        *at += 1;
        b
    };

    // Bounded so a pathological input is a slow case rather than a timeout.
    for _ in 0..2048 {
        if at >= data.len() {
            break;
        }
        let op = byte(&mut at);
        match op % 8 {
            0 => {
                let pc =
                    u64::from(u16::from_le_bytes([byte(&mut at), byte(&mut at)])) & !1;
                let key = u64::from(byte(&mut at) % 4);
                let page = u64::from(byte(&mut at)) << 12;
                let id = cache.insert(pc, key, page, 1, block(pc));
                ids.push((id, pc));
            }
            1 => {
                let from = byte(&mut at);
                let to = byte(&mut at);
                let pc = u64::from(u16::from_le_bytes([byte(&mut at), byte(&mut at)])) & !1;
                if let (Some((f, _)), Some((t, _))) = (pick(&ids, from), pick(&ids, to)) {
                    cache.link(f, pc, t);
                }
            }
            2 => {
                let from = byte(&mut at);
                let pc = u64::from(u16::from_le_bytes([byte(&mut at), byte(&mut at)])) & !1;
                if let Some((f, _)) = pick(&ids, from)
                    && let Some(found) = cache.follow(f, pc, 0)
                {
                    let entry = cache.block(found).map(|b| b.entry_pc);
                    assert_eq!(
                        entry,
                        Some(pc),
                        "a chained exit reached a block that is not at its own PC"
                    );
                }
            }
            3 => {
                let pc = u64::from(u16::from_le_bytes([byte(&mut at), byte(&mut at)])) & !1;
                let key = u64::from(byte(&mut at) % 4);
                if let Some(found) = cache.lookup(pc, key) {
                    assert_eq!(
                        cache.block(found).map(|b| b.entry_pc),
                        Some(pc),
                        "a lookup returned a block for a different PC"
                    );
                }
            }
            4 => {
                let phys = u64::from(u16::from_le_bytes([byte(&mut at), byte(&mut at)])) << 12;
                let len = u64::from(byte(&mut at));
                cache.note_write(phys, len);
            }
            5 => {
                // Fill past the capacity, so eviction runs with links live.
                for n in 0..CAPACITY as u64 {
                    let pc = 0x10_0000 + n * 4;
                    let id = cache.insert(pc, 0, 0x10_0000, 1, block(pc));
                    ids.push((id, pc));
                }
            }
            6 => {
                topology = topology.wrapping_add(u64::from(byte(&mut at)) % 3);
                cache.sync(Epoch {
                    topology,
                    translation: 0,
                });
            }
            _ => cache.flush(),
        }

        if let Err(e) = cache.check() {
            panic!("the block cache broke an invariant after op {op}: {e}");
        }
        assert_eq!(
            cache.stats().stale_links,
            0,
            "a chain link outlived its target, so an unpatch was missed"
        );
    }
});

/// One of the ids handed out, or `None` if none have been.
fn pick(ids: &[(BlockId, u64)], n: u8) -> Option<(BlockId, u64)> {
    if ids.is_empty() {
        return None;
    }
    ids.get(n as usize % ids.len()).copied()
}
