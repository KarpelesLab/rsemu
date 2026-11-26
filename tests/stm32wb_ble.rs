//! ST's own STM32WB HAL and a BLE mailbox client, on the `stm32wb55` and
//! `stm32wb35` boards.
//!
//! `tests/stm32wb_board.rs` proves the boards from the host. This proves them
//! with **firmware**: ST's HAL and CMSIS startup (BSD-3-Clause and Apache-2.0)
//! linked with our app (`tests/fixtures/stm32wb-ble/main.c`), which brings the
//! part to 64 MHz on the HSE32 PLL, programs flash, takes a semaphore, and then
//! boots CPU2 and drives its BLE stack through the IPCC mailbox — GATT and GAP
//! init, a service and a characteristic, discoverable mode — with both IPCC
//! interrupts taken through the NVIC. A model that answers a HAL wait loop
//! wrong shows up as `FAIL <step> status 3`, a mailbox step CPU2 never answers
//! as `status 65535`.
//!
//! The firmware is not in the repository. `scripts/fetch-testdata.sh
//! stm32wb-ble` fetches the vendor sources at pinned commits and builds it with
//! `arm-none-eabi-gcc`; point `RSEMU_STM32WB_BLE_DIR` at the result. Without
//! the variable every test here passes vacuously, as the other corpus-backed
//! tests do (CLAUDE.md, Testing).

#![cfg(all(feature = "machine-stm32wb55", feature = "machine-stm32wb35"))]

use std::path::PathBuf;

use rsemu::core::clock::GlobalTime;
use rsemu::host::chardev::ports;
use rsemu::machine::catalog;

fn corpus() -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("RSEMU_STM32WB_BLE_DIR")?);
    assert!(
        dir.is_dir(),
        "RSEMU_STM32WB_BLE_DIR={} is not a directory",
        dir.display()
    );
    Some(dir)
}

/// Boot `board` with `image` in flash and run it until the app says `PASS`,
/// reports a failure, or the emulated-time budget runs out.
fn run(board: &str, image: Vec<u8>) -> String {
    let entry = catalog::machine(board).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", image);
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{board} does not realize: {e}"));
    let port = ports::open(&options.realize.hosts, "console").expect("USART1 opened it");

    let mut out = Vec::new();
    for _ in 0..100 {
        m.run_for(GlobalTime::from_nanos(20_000_000))
            .expect("it runs");
        port.drain_into(&mut out);
        let text = String::from_utf8_lossy(&out);
        if text.contains("PASS") || text.contains("FAIL") || text.contains("FAULT") {
            break;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn value(transcript: &str, name: &str) -> u32 {
    let prefix = format!("{name}: 0x");
    let line = transcript
        .lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .unwrap_or_else(|| panic!("no `{name}` line in:\n{transcript}"));
    u32::from_str_radix(line.trim(), 16).unwrap_or_else(|_| panic!("`{name}` is not hex: {line}"))
}

fn check_common(t: &str) {
    assert!(t.contains("PASS"), "the app did not finish:\n{t}");
    // A Cortex-M4: implementer Arm, architecture 0xF, part number 0xC24.
    assert_eq!(value(t, "cpuid") & 0xff0f_fff0, 0x410f_c240);
    // HSE32 / 2 × 8 / 2 = 64 MHz; HCLK2 at SYSCLK / 2.
    assert_eq!(value(t, "sysclk"), 64_000_000);
    assert_eq!(value(t, "hclk"), 64_000_000);
    assert_eq!(value(t, "hclk2"), 32_000_000);
    assert_eq!(value(t, "lserdy"), 1);
    assert_eq!(value(t, "idcode") & 0xfff, 0x495, "DBGMCU DEV_ID");
    assert_eq!(value(t, "delay-ok"), 1, "HAL_Delay on SysTick");
    assert_eq!(value(t, "hsem-taken"), 1);
    assert_eq!(value(t, "hsem-free"), 1);
    // CPU2: READY with the wireless stack running, then BLE_Init accepted.
    assert_eq!(value(t, "ready"), 0);
    assert_ne!(value(t, "ws-version"), 0);
    assert_eq!(value(t, "ble-init"), 0);
    // The GATT service owns handles 1–4 (with Service Changed), GAP from 5.
    assert_eq!(value(t, "gap-service"), 5);
    let name = value(t, "gap-name");
    let appearance = value(t, "gap-appearance");
    assert!(name > 5 && appearance > name, "{name} {appearance}");
    let battery = value(t, "battery-service");
    assert!(battery > appearance);
    assert!(value(t, "battery-level") > battery);
    // aci_hal_write_config_data's address, read back through HCI_Read_BD_ADDR.
    assert_eq!(value(t, "bdaddr-lo"), 0x4433_2211);
    assert_eq!(value(t, "bdaddr-hi"), 0x6655);
    assert_eq!(value(t, "unknown"), 0x01, "Unknown HCI Command");
    assert_eq!(value(t, "erase-activity"), 0);
    assert_eq!(value(t, "flash-erased"), 0xffff_ffff);
    assert_eq!(value(t, "flash-word0"), 0x0123_4567);
    assert_eq!(value(t, "flash-word1"), 0x0bad_f00d);
    assert_eq!(value(t, "rx-irqs"), 1, "IPCC_C1_RX through the NVIC");
    assert_eq!(value(t, "tx-irqs"), 1, "IPCC_C1_TX through the NVIC");
    assert_eq!(value(t, "faults"), 0);
}

#[test]
fn the_wb55_runs_sts_hal_and_a_ble_stack() {
    let Some(dir) = corpus() else { return };
    let image = std::fs::read(dir.join("wb55.bin")).expect("wb55.bin");
    let t = run("stm32wb55", image);
    println!("{t}");
    check_common(&t);
    assert_eq!(value(&t, "flash-kb"), 1024);
    assert_eq!(value(&t, "sfsa"), 0xD0);
}

#[test]
fn the_wb35_runs_the_same_firmware() {
    let Some(dir) = corpus() else { return };
    let image = std::fs::read(dir.join("wb35.bin")).expect("wb35.bin");
    let t = run("stm32wb35", image);
    println!("{t}");
    check_common(&t);
    assert_eq!(value(&t, "flash-kb"), 512);
    assert_eq!(value(&t, "sfsa"), 0x5C);
}
