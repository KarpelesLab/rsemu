//! `st.rtc`, register by register.
//!
//! As in `st.pwr`, the device is lazily advanced and these
//! tests drive [`Device::advance_to`] themselves rather than standing a
//! scheduler up. Every count below is in RTCCLK cycles, so with the reset
//! prescalers one calendar second is [`SECOND`] of them.

use super::*;

use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::AtomicBool;
use crate::core::wire::{Wire, WireIdAllocator};
use alloc::vec::Vec;

/// One calendar second at the reset prescalers: 128 × 256 RTCCLK cycles, which
/// is exactly one second of a 32.768 kHz LSE.
const SECOND: u64 = 32_768;

/// A clock at the manual's own reset value.
fn fresh() -> Rtc {
    Rtc::with_epoch(Calendar::default())
}

fn at(epoch: &str) -> Rtc {
    Rtc::with_epoch(parse_epoch(epoch).expect("a well-formed epoch"))
}

fn peek(rtc: &Rtc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    rtc.regs
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a word read");
    u32::from_le_bytes(buf)
}

fn peek_debug(rtc: &Rtc, offset: u64) -> u32 {
    let mut buf = [0u8; 4];
    rtc.regs
        .read(offset, &mut buf, MemAttrs::DEBUG)
        .expect("a debug word read");
    u32::from_le_bytes(buf)
}

fn poke(rtc: &Rtc, offset: u64, value: u32) {
    rtc.regs
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write");
}

fn tick(rtc: &Rtc, cycles: u64) {
    rtc.advance_to(rtc.tick() + cycles);
}

/// The key sequence, as every vendor driver writes it.
fn unlock(rtc: &Rtc) {
    poke(rtc, WPR, WPR_KEY1);
    poke(rtc, WPR, WPR_KEY2);
}

/// Ask for initialisation mode and wait for `INITF`, which is what a driver's
/// `while (!(RTC->ISR & RTC_ISR_INITF))` does.
fn enter_init(rtc: &Rtc) {
    // The HAL writes all-ones: `INIT` is set and every `rc_w0` flag is left
    // alone, because a zero is what clears one.
    poke(rtc, ISR, 0xffff_ffff);
    assert_eq!(peek(rtc, ISR) & ISR_INITF, 0, "INITF is not immediate");
    tick(rtc, SYNC_CYCLES);
    assert_ne!(peek(rtc, ISR) & ISR_INITF, 0, "INITF follows two cycles on");
}

/// Clear one of `ISR`'s `rc_w0` flags the way the vendor HAL does.
///
/// A zero is what clears a flag, so clearing one means writing ones everywhere
/// else — and `INIT` sits in the same register, so it has to be carried across
/// rather than set by accident. That is exactly what
/// `__HAL_RTC_ALARM_CLEAR_FLAG` writes, and a test that wrote a bare `!flag`
/// would drop the machine into initialisation mode and stop the clock.
fn clear_flag(rtc: &Rtc, flag: u32) {
    let init = peek(rtc, ISR) & ISR_INIT;
    poke(rtc, ISR, !(flag | ISR_INIT) | init);
}

/// Drop `INIT`, which restarts the counter from the top of the divider chain.
fn exit_init(rtc: &Rtc) {
    let isr = peek(rtc, ISR);
    poke(rtc, ISR, isr & !ISR_INIT);
    assert_eq!(peek(rtc, ISR) & (ISR_INIT | ISR_INITF), 0);
}

/// The whole initialisation sequence: unlock, stop, set, restart.
fn set_calendar(rtc: &Rtc, tr: u32, dr: u32) {
    unlock(rtc);
    enter_init(rtc);
    poke(rtc, TR, tr);
    poke(rtc, DR, dr);
    exit_init(rtc);
}

/// `DR` for a date, the way the manual packs it.
fn dr_of(year: u8, month: u8, day: u8, weekday: u8) -> u32 {
    to_bcd(day) | (to_bcd(month) << 8) | (u32::from(weekday) << 13) | (to_bcd(year) << 16)
}

/// `TR` for a twenty-four-hour time.
fn tr_of(hour: u8, minute: u8, second: u8) -> u32 {
    to_bcd(second) | (to_bcd(minute) << 8) | (to_bcd(hour) << 16)
}

/// Follows the level of one output pin.
#[derive(Debug, Default)]
struct Probe {
    high: AtomicBool,
    edges: AtomicU32,
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        if level.is_high() && !self.high.swap(true, Ordering::Relaxed) {
            self.edges.fetch_add(1, Ordering::Relaxed);
        } else if !level.is_high() {
            self.high.store(false, Ordering::Relaxed);
        }
    }
}

impl Probe {
    fn is_high(&self) -> bool {
        self.high.load(Ordering::Relaxed)
    }

    fn edges(&self) -> u32 {
        self.edges.load(Ordering::Relaxed)
    }
}

fn watch(rtc: &Rtc, port: &str) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(rtc, port, WireSource::new(wire, id)).expect("an output pin");
    probe
}

/// Drive one of the input pins, as a wire from RCC does.
fn drive(rtc: &Rtc, port: &str, level: Level) {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin = Device::sink(rtc, port, &[id]).expect("an input pin");
    pin.sink.set_level(id, pin.line, level);
}

// ---------------------------------------------------------------------------
// Write protection and the initialisation dance
// ---------------------------------------------------------------------------

#[test]
fn everything_is_locked_until_ca_then_53() {
    let rtc = fresh();

    // Out of reset the key register is closed, so a `CR` write lands nowhere.
    poke(&rtc, CR, CR_FMT);
    assert_eq!(peek(&rtc, CR), 0, "still locked");

    // Half a sequence is not a sequence, in either direction.
    poke(&rtc, WPR, WPR_KEY2);
    poke(&rtc, CR, CR_FMT);
    assert_eq!(peek(&rtc, CR), 0, "the second key alone opens nothing");
    poke(&rtc, WPR, WPR_KEY1);
    poke(&rtc, CR, CR_FMT);
    assert_eq!(peek(&rtc, CR), 0, "nor the first");

    // `0xCA` then `0x53`.
    poke(&rtc, WPR, WPR_KEY2);
    poke(&rtc, CR, CR_FMT);
    assert_eq!(peek(&rtc, CR), CR_FMT, "open");

    // "Writing a wrong key reactivates the write protection."
    poke(&rtc, WPR, 0xff);
    poke(&rtc, CR, 0);
    assert_eq!(peek(&rtc, CR), CR_FMT, "shut again");

    // A fresh `0xCA` restarts the sequence rather than continuing to hold it
    // open, so a driver that writes the first key twice still needs the second.
    unlock(&rtc);
    poke(&rtc, WPR, WPR_KEY1);
    poke(&rtc, CR, 0);
    assert_eq!(peek(&rtc, CR), CR_FMT, "the restarted sequence re-armed it");

    // `WPR` never reads back: the unlock has to be latched, not inferred.
    assert_eq!(peek(&rtc, WPR), 0);
}

#[test]
fn the_registers_the_key_does_not_protect_are_writable_throughout() {
    let rtc = fresh();
    assert_ne!(peek(&rtc, ISR) & ISR_RSF, 0, "locked, and RSF is real");

    // `BKPxR`, `TAMPCR` and `OR` are outside the protection entirely.
    poke(&rtc, BKP0R, 0xdead_beef);
    poke(&rtc, BKP0R + 4 * 31, 0x1234_5678);
    poke(&rtc, TAMPCR, 0x0000_0005);
    poke(&rtc, OR, 0x3);
    assert_eq!(peek(&rtc, BKP0R), 0xdead_beef);
    assert_eq!(peek(&rtc, BKP0R + 4 * 31), 0x1234_5678);
    assert_eq!(peek(&rtc, TAMPCR), 0x0000_0005);
    assert_eq!(peek(&rtc, OR), 0x3);

    // So is the flag half of `ISR`: clearing `ALRAF` is something a handler
    // does without touching the key register.
    rtc.regs.state.lock().isr |= ISR_ALRAF | ISR_WUTF;
    assert_ne!(peek(&rtc, ISR) & (ISR_ALRAF | ISR_WUTF), 0);
    clear_flag(&rtc, ISR_ALRAF);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAF, 0, "cleared while locked");
    assert_ne!(peek(&rtc, ISR) & ISR_WUTF, 0, "and only that one");

    // `INIT` and `RSF`, in the same register, are protected.
    poke(&rtc, ISR, 0xffff_ffff);
    tick(&rtc, SYNC_CYCLES);
    assert_eq!(peek(&rtc, ISR) & (ISR_INIT | ISR_INITF), 0, "INIT is not");
}

#[test]
fn tr_and_dr_are_writable_only_while_initf_is_set() {
    let rtc = fresh();
    unlock(&rtc);

    // Unlocked but not in initialisation mode: the calendar is untouchable.
    poke(&rtc, TR, tr_of(12, 34, 56));
    poke(&rtc, DR, dr_of(24, 6, 1, 6));
    assert_eq!(peek(&rtc, TR), 0);
    assert_eq!(peek(&rtc, DR), DR_RESET_VALUE);

    // Asked for, but not yet arrived: `INITF` takes the manual's two cycles and
    // a driver that skips the wait silently keeps the old time.
    poke(&rtc, ISR, 0xffff_ffff);
    poke(&rtc, TR, tr_of(12, 34, 56));
    assert_eq!(peek(&rtc, TR), 0, "INIT is not INITF");

    tick(&rtc, SYNC_CYCLES);
    poke(&rtc, TR, tr_of(12, 34, 56));
    poke(&rtc, DR, dr_of(24, 6, 1, 6));
    assert_eq!(peek(&rtc, TR), tr_of(12, 34, 56));
    assert_eq!(peek(&rtc, DR), dr_of(24, 6, 1, 6));
}

/// `DR`'s documented reset value, 1 January 2000 with `WDU = 1`.
const DR_RESET_VALUE: u32 = 0x0000_2101;

#[test]
fn the_calendar_is_stopped_while_init_is_asked_for() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    tick(&rtc, 10 * SECOND);
    assert_eq!(rtc.calendar().second, 10);

    unlock(&rtc);
    enter_init(&rtc);
    tick(&rtc, 100 * SECOND);
    assert_eq!(rtc.calendar().second, 10, "initialisation mode stops it");
    assert_eq!(
        Device::next_event_tick(&rtc),
        None,
        "and a stopped calendar asks the scheduler for nothing"
    );

    exit_init(&rtc);
    tick(&rtc, SECOND);
    assert_eq!(rtc.calendar().second, 11);
}

// ---------------------------------------------------------------------------
// The calendar
// ---------------------------------------------------------------------------

#[test]
fn one_second_of_lse_ticks_advances_the_seconds_field_in_bcd() {
    let rtc = fresh();
    // 23:59:59 on Sunday 31 December 2023. `DR` packs as YT=2 YU=3 WDU=7 MT=1
    // MU=2 DT=3 DU=1, which is 0x0023_F231.
    let midnight = dr_of(23, 12, 31, 7);
    assert_eq!(midnight, 0x0023_f231, "the manual's field packing");
    set_calendar(&rtc, tr_of(23, 59, 59), midnight);

    assert_eq!(peek(&rtc, TR), 0x0023_5959, "and the time is BCD too");
    assert_eq!(peek(&rtc, DR), midnight);
    let inits = peek(&rtc, ISR) & ISR_INITS;
    assert_ne!(inits, 0, "a non-zero year is what sets INITS");

    tick(&rtc, SECOND - 1);
    assert_eq!(peek(&rtc, TR), 0x0023_5959, "not one cycle early");
    assert_eq!(peek(&rtc, DR), midnight);

    tick(&rtc, 1);
    // Read `TR` before `DR`: the other order is the coherency trap.
    assert_eq!(peek(&rtc, TR), 0x0000_0000, "00:00:00");
    assert_eq!(
        peek(&rtc, DR),
        dr_of(24, 1, 1, 1),
        "Monday 1 January 2024 — 0x0024_2101"
    );
    assert_eq!(peek(&rtc, DR), 0x0024_2101);
    assert_eq!(
        peek(&rtc, ISR) & ISR_INITS,
        inits,
        "rolling the year over is not an initialisation"
    );
}

#[test]
fn february_29_exists_in_2024_and_not_in_2023() {
    let leap = fresh();
    set_calendar(&leap, tr_of(23, 59, 59), dr_of(24, 2, 28, 3));
    tick(&leap, SECOND);
    assert_eq!(peek(&leap, TR), 0);
    assert_eq!(
        peek(&leap, DR),
        dr_of(24, 2, 29, 4),
        "29 February 2024, a Thursday"
    );
    tick(&leap, SECOND * 86_400);
    assert_eq!(peek(&leap, DR), dr_of(24, 3, 1, 5), "then March");

    let common = fresh();
    set_calendar(&common, tr_of(23, 59, 59), dr_of(23, 2, 28, 2));
    tick(&common, SECOND);
    assert_eq!(
        peek(&common, DR),
        dr_of(23, 3, 1, 3),
        "2023 has no 29 February"
    );

    // 2000 is the case the short leap rule gets wrong: divisible by 100 and
    // still a leap year, because it is divisible by 400.
    let y2k = fresh();
    set_calendar(&y2k, tr_of(23, 59, 59), dr_of(0, 2, 28, 1));
    tick(&y2k, SECOND);
    assert_eq!(peek(&y2k, DR), dr_of(0, 2, 29, 2), "2000 is a leap year");
    assert!(is_leap_year(2000) && !is_leap_year(2100));
}

#[test]
fn the_weekday_of_an_epoch_is_the_real_one() {
    // 1 January 2000 was a Saturday and 1 January 2024 a Monday; ST's own reset
    // value for `DR` says Monday for the first of those, which is why the two
    // ways in disagree on exactly that date.
    assert_eq!(Calendar::weekday_of(2000, 1, 1), 6);
    assert_eq!(Calendar::weekday_of(2024, 1, 1), 1);
    assert_eq!(Calendar::weekday_of(2023, 12, 31), 7);
    assert_eq!(peek(&at("2000-01-01"), DR), dr_of(0, 1, 1, 6));
    assert_eq!(
        peek(&fresh(), DR),
        DR_RESET_VALUE,
        "the reset value, verbatim"
    );

    let rtc = at("2024-06-01T12:34:56");
    assert_eq!(peek(&rtc, TR), tr_of(12, 34, 56));
    assert_eq!(peek(&rtc, DR), dr_of(24, 6, 1, 6), "a Saturday");
}

#[test]
fn fmt_selects_the_hour_format_without_moving_the_counter() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(13, 5, 0), dr_of(24, 1, 1, 1));
    assert_eq!(peek(&rtc, TR), 0x0013_0500, "24-hour: 13:05:00");

    unlock(&rtc);
    poke(&rtc, CR, CR_FMT);
    // Same instant, read as 1:05 PM.
    assert_eq!(peek(&rtc, TR), (1 << 22) | 0x0001_0500);
    assert_eq!(rtc.calendar().hour, 13, "the counter did not move");

    // Midnight and noon are the two a naive conversion gets wrong.
    for (hour, want) in [(0u8, 0x0012_0000), (12, (1 << 22) | 0x0012_0000)] {
        let rtc = fresh();
        set_calendar(&rtc, tr_of(hour, 0, 0), dr_of(24, 1, 1, 1));
        unlock(&rtc);
        poke(&rtc, CR, CR_FMT);
        assert_eq!(peek(&rtc, TR), want);
        // And a write in that format comes back as the same instant.
        enter_init(&rtc);
        poke(&rtc, TR, want);
        exit_init(&rtc);
        assert_eq!(rtc.calendar().hour, hour);
    }
}

#[test]
fn add1h_and_sub1h_move_the_hour_and_never_read_back() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 30, 0), dr_of(24, 3, 31, 7));
    unlock(&rtc);

    poke(&rtc, CR, CR_SUB1H);
    assert_eq!(peek(&rtc, CR), 0, "an action, not storage");
    assert_eq!(peek(&rtc, TR), tr_of(23, 30, 0));
    assert_eq!(
        peek(&rtc, DR),
        dr_of(24, 3, 30, 6),
        "and it carried the day"
    );

    poke(&rtc, CR, CR_ADD1H);
    assert_eq!(peek(&rtc, TR), tr_of(0, 30, 0));
    assert_eq!(peek(&rtc, DR), dr_of(24, 3, 31, 7));

    // Both at once cancel: two opposite corrections are no correction.
    poke(&rtc, CR, CR_ADD1H | CR_SUB1H);
    assert_eq!(peek(&rtc, TR), tr_of(0, 30, 0));
}

#[test]
fn the_prescalers_decide_how_long_a_second_is() {
    let rtc = fresh();
    assert_eq!(peek(&rtc, PRER), PRER_RESET);
    unlock(&rtc);
    enter_init(&rtc);
    // A 32 kHz LSI wants 125 × 256: PREDIV_A = 124, PREDIV_S = 255.
    poke(&rtc, PRER, (124 << 16) | 255);
    exit_init(&rtc);
    assert_eq!(peek(&rtc, PRER), (124 << 16) | 255);

    tick(&rtc, 125 * 256 - 1);
    assert_eq!(rtc.calendar().second, 0);
    tick(&rtc, 1);
    assert_eq!(rtc.calendar().second, 1, "exactly 32000 cycles");
}

// ---------------------------------------------------------------------------
// Shadow registers
// ---------------------------------------------------------------------------

#[test]
fn reading_tr_freezes_dr_until_dr_is_read() {
    let rtc = fresh();
    let eve = dr_of(23, 12, 31, 7);
    set_calendar(&rtc, tr_of(23, 59, 59), eve);

    // The read that takes the lock.
    assert_eq!(peek(&rtc, TR), tr_of(23, 59, 59));

    // A whole second goes by underneath. The live calendar has rolled over;
    // the shadow the guest reads has not, because it is held.
    tick(&rtc, SECOND);
    assert_eq!(rtc.calendar().day, 1, "the counter moved");
    assert_eq!(peek(&rtc, TR), tr_of(23, 59, 59), "the shadow did not");
    assert_eq!(
        peek(&rtc, DR),
        eve,
        "and `DR` still matches the `TR` that was read with it"
    );

    // That read of `DR` released the lock, and the copy lands two cycles on.
    tick(&rtc, SYNC_CYCLES);
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0));
    assert_eq!(peek(&rtc, DR), dr_of(24, 1, 1, 1));

    // Reading `SSR` takes the same lock (RM0351 §37.3.7).
    let before = peek(&rtc, SSR);
    tick(&rtc, SECOND);
    assert_eq!(peek(&rtc, SSR), before, "held by the `SSR` read");
    assert_eq!(peek(&rtc, DR), dr_of(24, 1, 1, 1));
    tick(&rtc, SYNC_CYCLES);
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 1), "released");
}

#[test]
fn a_debug_read_neither_takes_the_lock_nor_releases_it() {
    let rtc = fresh();
    let eve = dr_of(23, 12, 31, 7);
    set_calendar(&rtc, tr_of(23, 59, 59), eve);

    // A debugger reading `TR` must not freeze the guest's next calendar read.
    assert_eq!(peek_debug(&rtc, TR), tr_of(23, 59, 59));
    tick(&rtc, SECOND + SYNC_CYCLES);
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0), "the shadow kept refreshing");
    assert_eq!(peek(&rtc, DR), dr_of(24, 1, 1, 1));

    // And reading `DR` must not release a lock the guest is relying on.
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0));
    assert_eq!(peek_debug(&rtc, DR), dr_of(24, 1, 1, 1));
    tick(&rtc, SECOND + SYNC_CYCLES);
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0), "still held");
}

#[test]
fn a_debug_read_advances_nothing_and_a_debug_write_is_refused() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    let before = rtc.tick();
    let _ = peek_debug(&rtc, ISR);
    assert_eq!(rtc.tick(), before, "a debug read advances nothing");

    assert!(
        rtc.regs
            .write(WPR, &WPR_KEY1.to_le_bytes(), MemAttrs::DEBUG)
            .is_err(),
        "a debug write to `WPR` would open the protection"
    );
}

#[test]
fn rsf_reports_when_the_shadow_registers_are_valid() {
    let rtc = fresh();
    assert_ne!(peek(&rtc, ISR) & ISR_RSF, 0, "valid out of reset");

    unlock(&rtc);
    // `HAL_RTC_WaitForSynchro`: clear it, then spin until hardware sets it.
    clear_flag(&rtc, ISR_RSF);
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, 0);
    tick(&rtc, SYNC_CYCLES - 1);
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, 0, "the wait is a real wait");
    tick(&rtc, 1);
    assert_ne!(peek(&rtc, ISR) & ISR_RSF, 0, "two RTCCLK cycles");

    // "RSF is cleared by hardware in initialization mode."
    enter_init(&rtc);
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, 0);
    exit_init(&rtc);
    tick(&rtc, SYNC_CYCLES);
    assert_ne!(peek(&rtc, ISR) & ISR_RSF, 0);
}

#[test]
fn bypshad_reads_the_live_registers_and_keeps_rsf_clear() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    unlock(&rtc);
    poke(&rtc, CR, CR_BYPSHAD);
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, 0, "there is no copy to be valid");

    // A `TR` read no longer holds anything, because there is nothing held.
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0));
    tick(&rtc, SECOND);
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 1), "straight off the counter");
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, 0);

    // And the sub-second is the live down-counter.
    tick(&rtc, 128);
    assert_eq!(peek(&rtc, SSR), 254);

    poke(&rtc, CR, 0);
    tick(&rtc, SYNC_CYCLES);
    assert_ne!(peek(&rtc, ISR) & ISR_RSF, 0, "the copy came back");
}

#[test]
fn ssr_counts_down_from_prediv_s() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    assert_eq!(peek(&rtc, SSR), 255, "reloaded by leaving init mode");
    // An `SSR` read holds the shadow exactly as a `TR` read does, so a driver
    // that wants to see the *next* value has to read `DR` to let go of it.
    assert_eq!(peek(&rtc, DR), dr_of(24, 1, 1, 1), "releases the lock");

    // One step of the synchronous prescaler is `PREDIV_A + 1` = 128 cycles.
    // `SSR` is read here through the shadow, so give the copy its two cycles.
    tick(&rtc, 128 + SYNC_CYCLES);
    assert_eq!(peek(&rtc, SSR), 254);
    assert_eq!(peek(&rtc, DR), dr_of(24, 1, 1, 1), "releases the lock");

    tick(&rtc, 128 * 255 + SYNC_CYCLES);
    assert_eq!(peek(&rtc, SSR), 255, "it reloads as the second ticks");
    assert_eq!(rtc.calendar().second, 1);
}

// ---------------------------------------------------------------------------
// Alarms
// ---------------------------------------------------------------------------

#[test]
fn alarm_a_fires_on_the_masked_match_and_raises_exti_18() {
    // The pin: which EXTI line it lands on — 18 on an L4, 17 on an F4 — and
    // which NVIC position that line raises is the board's business and lives in
    // the machine file, so what this device owes is one level per event.
    let rtc = fresh();
    let alarm = watch(&rtc, ALARM_PIN);
    set_calendar(&rtc, tr_of(12, 0, 0), dr_of(24, 1, 1, 1));
    assert!(!alarm.is_high());

    unlock(&rtc);
    // Every field masked except the seconds: fire at :30 of every minute.
    let mask = (1 << 31) | (1 << 23) | (1 << 15);
    poke(&rtc, ALRMAR, mask | to_bcd(30));
    assert_ne!(peek(&rtc, ISR) & ISR_ALRAWF, 0, "writable while disabled");
    poke(&rtc, CR, CR_ALRAE | CR_ALRAIE);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAWF, 0, "and not while enabled");

    tick(&rtc, 29 * SECOND);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAF, 0, "not yet");
    assert!(!alarm.is_high());

    tick(&rtc, SECOND);
    assert_ne!(peek(&rtc, ISR) & ISR_ALRAF, 0, "at :30");
    assert!(alarm.is_high(), "and the EXTI line went up");
    assert_eq!(alarm.edges(), 1);

    // The flag holds the line up until software clears it, which is what makes
    // an edge-triggered EXTI see exactly one edge per alarm.
    tick(&rtc, SECOND);
    assert!(alarm.is_high());
    clear_flag(&rtc, ISR_ALRAF);
    assert!(!alarm.is_high());

    // A minute later, the next match.
    tick(&rtc, 59 * SECOND);
    assert_eq!(alarm.edges(), 2);
    assert_eq!(rtc.calendar().second, 30);
    assert_eq!(rtc.calendar().minute, 1);
}

#[test]
fn an_unmasked_alarm_compares_the_whole_calendar() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(23, 59, 55), dr_of(24, 2, 28, 3));
    unlock(&rtc);
    // Nothing masked: 00:00:00 on the 29th.
    poke(&rtc, ALRMAR, to_bcd(29) << 24);
    poke(&rtc, CR, CR_ALRAE);

    tick(&rtc, 4 * SECOND);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAF, 0);
    tick(&rtc, SECOND);
    assert_ne!(peek(&rtc, ISR) & ISR_ALRAF, 0, "midnight on 29 February");

    // `WDSEL` swaps the date field for the day of the week.
    let rtc = fresh();
    set_calendar(&rtc, tr_of(23, 59, 59), dr_of(24, 1, 1, 1));
    unlock(&rtc);
    poke(&rtc, ALRMAR, (1 << 30) | (2 << 24));
    poke(&rtc, CR, CR_ALRAE);
    tick(&rtc, SECOND);
    assert_ne!(peek(&rtc, ISR) & ISR_ALRAF, 0, "00:00:00 on a Tuesday");
}

#[test]
fn alarm_b_is_its_own_alarm_on_the_same_pin() {
    let rtc = fresh();
    let alarm = watch(&rtc, ALARM_PIN);
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    unlock(&rtc);
    let mask = (1 << 31) | (1 << 23) | (1 << 15);
    poke(&rtc, ALRMBR, mask | to_bcd(5));
    poke(&rtc, CR, CR_ALRBE | CR_ALRBIE);

    tick(&rtc, 5 * SECOND);
    assert_ne!(peek(&rtc, ISR) & ISR_ALRBF, 0);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAF, 0, "and A is untouched");
    assert!(alarm.is_high(), "both alarms share the one output");
}

#[test]
fn a_sub_second_alarm_matches_inside_the_second() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    unlock(&rtc);
    // Seconds, minutes, hours and the date all masked, so the whole comparison
    // is `SSR`: fire when the down-counter reads 250.
    let mask = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
    poke(&rtc, ALRMAR, mask);
    // `MASKSS = 15` compares `SS[14:0]` — the whole field.
    poke(&rtc, ALRMASSR, (15 << 24) | 250);
    poke(&rtc, CR, CR_ALRAE);

    // `SSR` reaches 250 five steps of the asynchronous prescaler in.
    tick(&rtc, 128 * 5 - 1);
    assert_eq!(peek(&rtc, ISR) & ISR_ALRAF, 0);
    tick(&rtc, 1);
    assert_ne!(peek(&rtc, ISR) & ISR_ALRAF, 0, "1280 cycles, not 32768");
}

// ---------------------------------------------------------------------------
// The wakeup timer
// ---------------------------------------------------------------------------

#[test]
fn the_wakeup_timer_counts_down_at_rtcclk_over_16() {
    let rtc = fresh();
    let wakeup = watch(&rtc, WAKEUP_PIN);
    unlock(&rtc);

    assert_ne!(peek(&rtc, ISR) & ISR_WUTWF, 0, "writable while disabled");
    poke(&rtc, WUTR, 9);
    // `WUCKSEL = 000` is RTCCLK/16, so the period is (9 + 1) × 16 = 160.
    poke(&rtc, CR, CR_WUTE | CR_WUTIE);
    assert_eq!(peek(&rtc, ISR) & ISR_WUTWF, 0, "and not while running");

    tick(&rtc, 159);
    assert_eq!(peek(&rtc, ISR) & ISR_WUTF, 0);
    assert!(!wakeup.is_high());
    tick(&rtc, 1);
    assert_ne!(peek(&rtc, ISR) & ISR_WUTF, 0, "160 RTCCLK cycles");
    assert!(wakeup.is_high());

    // It reloads and goes again rather than stopping.
    clear_flag(&rtc, ISR_WUTF);
    assert!(!wakeup.is_high());
    tick(&rtc, 159);
    assert_eq!(peek(&rtc, ISR) & ISR_WUTF, 0);
    tick(&rtc, 1);
    assert_eq!(wakeup.edges(), 2);

    // `WUTR` is refused while the timer runs, which is what `WUTWF` says.
    poke(&rtc, WUTR, 42);
    assert_eq!(peek(&rtc, WUTR), 9);
}

#[test]
fn wucksel_picks_the_divider_and_ck_spre() {
    for (sel, period) in [(0u32, 16u64), (1, 8), (2, 4), (3, 2)] {
        let rtc = fresh();
        unlock(&rtc);
        poke(&rtc, WUTR, 0);
        poke(&rtc, CR, sel | CR_WUTE);
        tick(&rtc, period - 1);
        assert_eq!(peek(&rtc, ISR) & ISR_WUTF, 0, "sel {sel}");
        tick(&rtc, 1);
        assert_ne!(peek(&rtc, ISR) & ISR_WUTF, 0, "sel {sel}");
    }

    // `WUCKSEL = 10x` clocks it from `ck_spre` — one calendar second a step.
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    unlock(&rtc);
    poke(&rtc, WUTR, 2);
    poke(&rtc, CR, 0b100 | CR_WUTE);
    tick(&rtc, 3 * SECOND - 1);
    assert_eq!(peek(&rtc, ISR) & ISR_WUTF, 0);
    tick(&rtc, 1);
    assert_ne!(peek(&rtc, ISR) & ISR_WUTF, 0, "three seconds");
}

// ---------------------------------------------------------------------------
// The timestamp
// ---------------------------------------------------------------------------

#[test]
fn the_timestamp_input_latches_the_calendar_on_the_selected_edge() {
    let rtc = fresh();
    let stamp = watch(&rtc, STAMP_PIN);
    set_calendar(&rtc, tr_of(1, 2, 3), dr_of(24, 5, 6, 1));
    unlock(&rtc);
    poke(&rtc, CR, CR_TSE | CR_TSIE);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin = Device::sink(&rtc, TS_PIN, &[id]).expect("the `ts` pin");
    pin.sink.set_level(id, pin.line, Level::High);

    assert_ne!(peek(&rtc, ISR) & ISR_TSF, 0);
    assert_eq!(peek(&rtc, TSTR), tr_of(1, 2, 3));
    assert_eq!(peek(&rtc, TSDR), dr_of(24, 5, 6, 1));
    assert!(stamp.is_high());

    // A second event on top of an unread one is an overflow, and it is the
    // *new* capture that is dropped.
    pin.sink.set_level(id, pin.line, Level::Low);
    tick(&rtc, SECOND);
    pin.sink.set_level(id, pin.line, Level::High);
    assert_ne!(peek(&rtc, ISR) & ISR_TSOVF, 0);
    assert_eq!(peek(&rtc, TSTR), tr_of(1, 2, 3), "the first one stands");

    // Clearing `TSF` clears the overflow with it, and drops the line.
    clear_flag(&rtc, ISR_TSF);
    assert_eq!(peek(&rtc, ISR) & (ISR_TSF | ISR_TSOVF), 0);
    assert!(!stamp.is_high());
}

// ---------------------------------------------------------------------------
// The backup domain
// ---------------------------------------------------------------------------

#[test]
fn backup_registers_survive_a_system_reset_but_not_bdrst() {
    let rtc = at("2024-01-01T00:00:00");
    for n in 0..BKP_COUNT {
        poke(&rtc, BKP0R + 4 * n as u64, 0xb0_0000 | n as u32);
    }
    set_calendar(&rtc, tr_of(9, 30, 0), dr_of(24, 7, 4, 4));

    // A system reset does not reach the backup domain. That is the whole
    // reason firmware parks a reset reason in `BKP0R`.
    Device::reset(&rtc, ResetKind::Warm);
    for n in 0..BKP_COUNT {
        assert_eq!(rtc.backup(n), 0xb0_0000 | n as u32, "BKP{n}R");
    }
    assert_eq!(peek(&rtc, TR), tr_of(9, 30, 0), "and neither the calendar");
    assert_eq!(peek(&rtc, DR), dr_of(24, 7, 4, 4));

    // `BDCR.BDRST` does, and takes everything with it.
    drive(&rtc, BDRST_PIN, Level::High);
    for n in 0..BKP_COUNT {
        assert_eq!(rtc.backup(n), 0, "BKP{n}R");
    }
    assert_eq!(peek(&rtc, TR), tr_of(0, 0, 0));
    assert_eq!(
        peek(&rtc, DR),
        dr_of(24, 1, 1, 1),
        "back to `epoch`, which is this machine's backup-reset value"
    );
    assert_eq!(peek(&rtc, PRER), PRER_RESET);
    assert_eq!(peek(&rtc, ISR) & ISR_RSF, ISR_RSF);

    // A power-on reset is the same thing: no battery, no backup domain.
    poke(&rtc, BKP0R, 0x1234);
    Device::reset(&rtc, ResetKind::Cold);
    assert_eq!(rtc.backup(0), 0);
}

#[test]
fn rtcen_gates_the_counter_and_an_unwired_board_still_runs() {
    let rtc = fresh();
    set_calendar(&rtc, tr_of(0, 0, 0), dr_of(24, 1, 1, 1));
    tick(&rtc, SECOND);
    assert_eq!(rtc.calendar().second, 1, "nothing wired: it counts");

    // RCC takes the clock away.
    drive(&rtc, RTCEN_PIN, Level::Low);
    tick(&rtc, 100 * SECOND);
    assert_eq!(rtc.calendar().second, 1, "a stopped clock does not count");
    assert_eq!(Device::next_event_tick(&rtc), None);

    rtc.set_rtcen(true);
    tick(&rtc, SECOND);
    assert_eq!(rtc.calendar().second, 2);
}

// ---------------------------------------------------------------------------
// Snapshots and determinism
// ---------------------------------------------------------------------------

/// Save one device's chunk.
fn save(rtc: &Rtc) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("rtc", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("rtc", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(rtc, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// Load a chunk into a device.
fn load(rtc: &Rtc, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("rtc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(rtc, &mut chunk.reader()).unwrap();
}

/// Every register, read the way a debugger reads them — which is the way that
/// does not disturb the coherency lock.
fn dump(rtc: &Rtc) -> Vec<u32> {
    (0..REGISTER_BYTES / 4)
        .map(|i| peek_debug(rtc, i * 4))
        .collect()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = at("2024-02-28T23:59:50");
    for n in 0..BKP_COUNT {
        poke(&saved, BKP0R + 4 * n as u64, 0xc0ffee00 | n as u32);
    }
    unlock(&saved);
    poke(
        &saved,
        ALRMAR,
        (1 << 31) | (1 << 23) | (1 << 15) | to_bcd(55),
    );
    poke(&saved, WUTR, 300);
    poke(&saved, CR, CR_ALRAE | CR_ALRAIE | CR_WUTE | CR_WUTIE);
    // Saved mid-second, mid-wakeup-period, and with a synchronisation still
    // owed — the three things a naive encoding drops.
    tick(&saved, 3 * SECOND + 777);
    clear_flag(&saved, ISR_RSF);
    assert_eq!(peek(&saved, ISR) & ISR_RSF, 0);

    let bytes = save(&saved);
    let restored = at("2024-02-28T23:59:50");
    load(&restored, &bytes);

    assert_eq!(dump(&saved), dump(&restored), "every register");
    assert_eq!(
        Device::next_event_tick(&saved),
        Device::next_event_tick(&restored),
        "and what was still pending came across"
    );
    assert_eq!(save(&restored), bytes, "an identical state hash");

    // The two go on being the same machine, not merely the same bytes.
    tick(&saved, 10 * SECOND);
    tick(&restored, 10 * SECOND);
    assert_eq!(dump(&saved), dump(&restored));
    assert_eq!(save(&restored), save(&saved));
    assert_eq!(restored.calendar().day, 29, "and through the leap day");
}

#[test]
fn a_snapshot_that_is_not_a_date_is_refused() {
    let rtc = fresh();
    let mut bytes = save(&rtc);
    // The month field of the live calendar: two calendars of seven bytes sit
    // at the front of the chunk's payload, and the payload starts where the
    // saved bytes and a fresh one first differ.
    let month = bytes
        .windows(7)
        .position(|w| w == [0, 0, 0, 1, 1, 0, 1])
        .expect("the live calendar")
        + 4;
    bytes[month] = 13;
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("rtc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    assert!(Device::load(&fresh(), &mut chunk.reader()).is_err());
}

#[test]
fn two_runs_with_the_same_epoch_hash_identically() {
    // The point of `epoch`: the calendar is machine configuration and never the
    // host clock, so two runs of the same machine are the same run.
    let run = |rtc: &Rtc| {
        unlock(rtc);
        poke(rtc, WUTR, 1000);
        poke(rtc, CR, CR_WUTE | CR_WUTIE);
        poke(rtc, ALRMAR, (1 << 31) | (1 << 23) | (1 << 15) | to_bcd(7));
        poke(rtc, CR, CR_WUTE | CR_WUTIE | CR_ALRAE | CR_ALRAIE);
        for _ in 0..5 {
            tick(rtc, SECOND + 61);
            let _ = peek(rtc, TR);
            let _ = peek(rtc, DR);
        }
        poke(rtc, BKP7R_OFFSET, peek(rtc, SSR));
    };

    let a = at("2023-12-31T23:59:50");
    let b = at("2023-12-31T23:59:50");
    run(&a);
    run(&b);
    assert_eq!(dump(&a), dump(&b));
    assert_eq!(save(&a), save(&b), "an identical state hash");

    // And a different epoch is a different machine, so the hash moves.
    let c = at("2023-12-31T23:59:51");
    run(&c);
    assert_ne!(save(&c), save(&a));
}

/// `BKP7R`, used above as somewhere to park a value the run computed.
const BKP7R_OFFSET: u64 = BKP0R + 4 * 7;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn an_epoch_is_parsed_or_refused() {
    assert_eq!(
        parse_epoch("2024-06-01T12:34:56").unwrap(),
        Calendar {
            second: 56,
            minute: 34,
            hour: 12,
            day: 1,
            month: 6,
            year: 24,
            weekday: 6,
        }
    );
    // A space for the `T`, and a bare date.
    assert_eq!(
        parse_epoch("2024-06-01 12:34:56").unwrap(),
        parse_epoch("2024-06-01T12:34:56").unwrap()
    );
    assert_eq!(parse_epoch("2024-06-01").unwrap().hour, 0);

    for bad in [
        "",
        "2024",
        "2024-06",
        "2024-06-01-02",
        "1999-12-31",       // before `YT`/`YU` can reach
        "2100-01-01",       // and after
        "2023-02-29",       // not a leap year
        "2024-13-01",       // no such month
        "2024-00-01",       // nor that one
        "2024-06-00",       // nor that day
        "2024-06-01T24:00", // no such hour
        "2024-06-01T12:60",
        "2024-06-01T12:00:60", // a leap second is not representable
        "2024-06-01T12:00:00:00",
        "20xx-06-01",
    ] {
        assert!(parse_epoch(bad).is_err(), "`{bad}` should be refused");
    }
}

#[test]
fn a_property_this_class_does_not_know_is_a_typo() {
    assert_eq!(
        Rtc::new(&Props::new()).unwrap().epoch(),
        Calendar::default()
    );
    let props = Props::new().with("epoch", Value::from("2024-06-01"));
    assert_eq!(Rtc::new(&props).unwrap().epoch().year, 24);
    assert!(Rtc::new(&Props::new().with("epoch", Value::from("nope"))).is_err());
    assert!(Rtc::new(&Props::new().with("epock", Value::from("2024-06-01"))).is_err());
}

#[test]
fn the_class_is_registrable_and_agrees_with_its_schema() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);

    let schema = schema();
    assert_eq!(schema.class, CLASS_NAME);
    for prop in CLASS.properties {
        assert!(
            schema.props.iter().any(|p| p.name == prop.name),
            "`{}` is in the class and not the schema",
            prop.name
        );
    }
    for port in [
        ALARM_PIN, WAKEUP_PIN, STAMP_PIN, RTCEN_PIN, BDRST_PIN, TS_PIN,
    ] {
        assert!(schema.port_named(port).is_some(), "`{port}`");
    }
}

#[test]
fn the_pins_are_the_pins_and_nothing_else() {
    let rtc = fresh();
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    assert!(Device::connect(&rtc, "irq", WireSource::new(wire, id)).is_err());
    assert!(Device::sink(&rtc, "tamper1", &[id]).is_none());
    assert!(Device::region(&rtc, "regs").is_some());
    assert!(Device::region(&rtc, "bkp").is_none());
    assert_eq!(
        rtc.regs.constraints(),
        AccessConstraints::word(Width::U32, Endian::Little)
    );
    assert_eq!(REGISTER_BYTES, 0xd0);
}
