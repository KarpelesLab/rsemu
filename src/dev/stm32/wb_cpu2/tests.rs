//! Tests for the STM32WB CPU2 model, against the real `st.ipcc` and a page
//! of SRAM2 laid out the way CPU1's transport layer lays it out.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::core::space::{RamStore, Region as MemRegion};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireIdAllocator, WireSource};
use crate::dev::stm32::ipcc::Ipcc;

const SRAM2: u64 = 0x2003_0000;
const SRAM2_LEN: u64 = 0x2800;
const IPCC: u64 = DEFAULT_IPCC;

// Where the test's "CPU1" puts things.
const REF: u32 = 0x2003_0000;
const DIT: u32 = 0x2003_0030;
const BLE_TABLE: u32 = 0x2003_0060;
const SYS_TABLE: u32 = 0x2003_0070;
const MM_TABLE: u32 = 0x2003_0080;
const FREE_Q: u32 = 0x2003_00A0;
const BLE_Q: u32 = 0x2003_00A8;
const SYS_Q: u32 = 0x2003_00B0;
const CS_BUF: u32 = 0x2003_00C0;
const SYS_BUF: u32 = 0x2003_0100;
const BLE_BUF: u32 = 0x2003_0220;
const ACL_BUF: u32 = 0x2003_0340;
const SPARE_SYS: u32 = 0x2003_0460;
const SPARE_BLE: u32 = 0x2003_0580;
const POOL: u32 = 0x2003_06A0;

// IPCC, processor-1 view.
const C1SCR: u64 = 0x08;
const C1TOC2SR_1: u64 = 0x0C;
const C2CR_1: u64 = 0x10;
const C2MR_1: u64 = 0x14;
const C2TOC1SR_1: u64 = 0x1C;

const LATENCY: u64 = 3200;
const BOOT: u64 = 32_000;

/// One packet CPU1 took off a queue: where it was, and its bytes after the node.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Got {
    addr: u32,
    body: Vec<u8>,
}

/// A CPU2, an IPCC and SRAM2, with the test playing CPU1.
struct Rig {
    cpu2: WbCpu2,
    ipcc: Arc<Ipcc>,
    space: Arc<AddressSpace>,
}

impl Rig {
    fn new() -> Rig {
        Rig::with(WbCpu2::with_defaults(), 4)
    }

    fn with(cpu2: WbCpu2, pool_slots: u32) -> Rig {
        let space = Arc::new(AddressSpace::new("mem", 32));
        let ipcc = Arc::new(Ipcc::build());
        {
            let mut topo = space.topology();
            topo.map(
                Arc::new(MemRegion::ram("sram2", Arc::new(RamStore::new(SRAM2_LEN)))),
                SRAM2,
            )
            .expect("maps");
            topo.map(Device::region(&*ipcc, "regs").expect("a region"), IPCC)
                .expect("maps");
        }
        cpu2.attach_bus(&space, RequesterId(2));
        // The two processor-2 interrupt outputs into CPU2's inputs.
        let ids = WireIdAllocator::new();
        for (out, input) in [("c2-rx", "ipcc-rx"), ("c2-tx", "ipcc-tx")] {
            let id = ids.alloc();
            let pin = Device::sink(&cpu2, input, &[id]).expect("an input");
            let wire = Wire::builder()
                .source(id)
                .sink(pin.sink, pin.line)
                .build_shared();
            Device::connect(&*ipcc, out, WireSource::new(wire, id)).expect("an output");
        }
        let rig = Rig { cpu2, ipcc, space };
        rig.lay_out(pool_slots);
        rig
    }

    fn w32(&self, addr: u32, v: u32) {
        self.space
            .write(u64::from(addr), Width::U32, u64::from(v), MemAttrs::DEFAULT)
            .expect("mapped");
    }

    fn r32(&self, addr: u32) -> u32 {
        self.space
            .read(u64::from(addr), Width::U32, MemAttrs::DEFAULT)
            .expect("mapped") as u32
    }

    fn put(&self, addr: u32, bytes: &[u8]) {
        self.space
            .write_bytes(u64::from(addr), bytes, MemAttrs::DEFAULT)
            .expect("mapped");
    }

    fn get(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut b = vec![0; len];
        self.space
            .read_bytes(u64::from(addr), &mut b, MemAttrs::DEFAULT)
            .expect("mapped");
        b
    }

    fn ipcc_w(&self, reg: u64, v: u32) {
        self.space
            .write(IPCC + reg, Width::U32, u64::from(v), MemAttrs::DEFAULT)
            .expect("mapped");
    }

    fn ipcc_r(&self, reg: u64) -> u32 {
        self.space
            .read(IPCC + reg, Width::U32, MemAttrs::DEFAULT)
            .expect("mapped") as u32
    }

    /// What `TL_Init`, `TL_SYS_Init`, `TL_MM_Init` and `TL_BLE_Init` set up.
    fn lay_out(&self, pool_slots: u32) {
        for (i, w) in [DIT, BLE_TABLE, 0, SYS_TABLE, MM_TABLE, 0, 0, 0, 0, 0]
            .iter()
            .enumerate()
        {
            self.w32(REF + 4 * i as u32, *w);
        }
        for (i, w) in [BLE_BUF, CS_BUF, BLE_Q, ACL_BUF].iter().enumerate() {
            self.w32(BLE_TABLE + 4 * i as u32, *w);
        }
        self.w32(SYS_TABLE, SYS_BUF);
        self.w32(SYS_TABLE + 4, SYS_Q);
        for (i, w) in [SPARE_BLE, SPARE_SYS, POOL, pool_slots * 268, FREE_Q, 0, 0]
            .iter()
            .enumerate()
        {
            self.w32(MM_TABLE + 4 * i as u32, *w);
        }
        for head in [FREE_Q, BLE_Q, SYS_Q] {
            self.w32(head, head);
            self.w32(head + 4, head);
        }
    }

    fn run(&self, ticks: u64) {
        self.cpu2.advance_to(self.cpu2.tick() + ticks);
    }

    fn boot(&self) {
        self.cpu2.set_boot(true);
        self.run(BOOT);
    }

    /// Remove everything from a queue, CPU1-style, and clear the channel.
    fn drain(&self, head: u32, ch: u32) -> Vec<Got> {
        let mut out = Vec::new();
        if self.ipcc_r(C2TOC1SR_1) & low(ch) == 0 {
            return out;
        }
        let mut node = self.r32(head);
        while node != head {
            let len = self.get(node + 10, 1)[0];
            out.push(Got {
                addr: node,
                body: self.get(node + 8, 3 + usize::from(len)),
            });
            node = self.r32(node);
            assert!(out.len() < 64, "the queue closes");
        }
        self.w32(head, head);
        self.w32(head + 4, head);
        self.ipcc_w(C1SCR, low(ch));
        out
    }

    /// Hand buffers back on channel 4, as `TL_MM_EvtDone` does.
    fn release(&self, addrs: &[u32]) {
        for &a in addrs {
            let tail = self.r32(FREE_Q + 4);
            self.w32(a, FREE_Q);
            self.w32(a + 4, tail);
            self.w32(tail, a);
            self.w32(FREE_Q + 4, a);
        }
        self.ipcc_w(C1SCR, high(CH_MM));
        self.run(LATENCY + 1);
    }

    fn sys_events(&self) -> Vec<Got> {
        self.drain(SYS_Q, CH_SYS)
    }

    /// Send a system command and return the response bytes after the node.
    fn sys(&self, opcode: u16, params: &[u8]) -> Vec<u8> {
        let mut pkt = vec![PKT_SYS_CMD];
        pkt.extend_from_slice(&opcode.to_le_bytes());
        pkt.push(params.len() as u8);
        pkt.extend_from_slice(params);
        self.put(SYS_BUF + 8, &pkt);
        self.ipcc_w(C1SCR, high(CH_SYS));
        assert_ne!(self.ipcc_r(C1TOC2SR_1) & low(CH_SYS), 0);
        self.run(LATENCY + 1);
        assert_eq!(
            self.ipcc_r(C1TOC2SR_1) & low(CH_SYS),
            0,
            "CPU2 clears channel 2 to say the response is there"
        );
        let len = self.get(SYS_BUF + 10, 1)[0];
        self.get(SYS_BUF + 8, 3 + usize::from(len))
    }

    /// Boot, take READY, start the BLE stack.
    fn ble_up(&self) {
        self.boot();
        let ready = self.sys_events();
        self.release(&[ready[0].addr]);
        assert_eq!(self.sys(SHCI_BLE_INIT, &[0; 37])[6], 0);
    }

    /// Write a BLE command and ring channel 1, without running.
    fn ring_ble(&self, opcode: u16, params: &[u8]) {
        let mut pkt = vec![PKT_BLE_CMD];
        pkt.extend_from_slice(&opcode.to_le_bytes());
        pkt.push(params.len() as u8);
        pkt.extend_from_slice(params);
        self.put(BLE_BUF + 8, &pkt);
        self.ipcc_w(C1SCR, high(CH_BLE));
    }

    /// Send a BLE command; return every event, handing the non-response ones
    /// back to the memory manager as ST's transport does.
    fn ble(&self, opcode: u16, params: &[u8]) -> Vec<Got> {
        self.ring_ble(opcode, params);
        self.run(LATENCY + 1);
        let got = self.drain(BLE_Q, CH_BLE);
        let pooled: Vec<u32> = got
            .iter()
            .filter(|g| g.addr != BLE_BUF && g.addr != CS_BUF)
            .map(|g| g.addr)
            .collect();
        if !pooled.is_empty() {
            self.release(&pooled);
        }
        got
    }

    /// The return parameters of the Command Complete answering `opcode`.
    fn cc(&self, opcode: u16, params: &[u8]) -> Vec<u8> {
        let got = self.ble(opcode, params);
        let first = &got[0];
        assert_eq!(
            first.addr, BLE_BUF,
            "a Command Complete rides in the command buffer"
        );
        assert_eq!(first.body[0], PKT_BLE_EVT);
        assert_eq!(first.body[1], 0x0E, "Command Complete");
        assert_eq!(first.body[3], 1, "Num_HCI_Command_Packets");
        assert_eq!(u16::from_le_bytes([first.body[4], first.body[5]]), opcode);
        first.body[6..].to_vec()
    }
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

#[test]
fn nothing_happens_without_c2boot() {
    let rig = Rig::new();
    rig.run(10 * BOOT);
    assert_eq!(rig.cpu2.run_state(), Run::Halted);
    assert_eq!(rig.ipcc_r(C2CR_1), 0);
    assert_eq!(rig.r32(DIT), 0);
}

#[test]
fn boot_posts_ready_on_the_system_queue_after_the_boot_latency() {
    let rig = Rig::new();
    rig.cpu2.set_boot(true);
    rig.run(BOOT - 1);
    assert_eq!(rig.ipcc.c2_to_c1(), 0, "not before the latency");
    rig.run(1);
    assert_eq!(rig.cpu2.run_state(), Run::Running);
    assert_eq!(rig.ipcc.c2_to_c1() & low(CH_SYS), low(CH_SYS));

    let ready = rig.sys_events();
    assert_eq!(ready.len(), 1);
    assert_eq!(ready[0].addr, POOL, "carved from the pool");
    // System event, vendor 0xFF, length 3, sub-event 0x9200, "wireless stack
    // running" (AN5185 Table 12).
    assert_eq!(ready[0].body, [0x12, 0xFF, 3, 0x00, 0x92, 0x00]);

    // CPU2 enabled both interrupts and listens on 2, 4 and 6 only: BLE waits
    // for BLE_Init.
    assert_eq!(rig.ipcc_r(C2CR_1), CR_RXOIE | CR_TXFIE);
    let mr = rig.ipcc_r(C2MR_1);
    assert_eq!(mr & 0x3F, !(low(2) | low(4) | low(6)) & 0x3F);
}

#[test]
fn the_wireless_device_information_table_is_an5289s_layout() {
    let rig = Rig::new();
    rig.boot();
    // Safe boot, FUS, FUS memory, FUS info, stack version, stack memory,
    // InfoStack (AN5289 §4.10).
    assert_eq!(rig.r32(DIT + 4), 0x0102_0001, "FUS version");
    assert_eq!(rig.r32(DIT + 12), 0, "FusInfo is reserved");
    assert_eq!(rig.r32(DIT + 16), 0x0111_0301, "wireless stack version");
    assert_eq!(
        rig.r32(DIT + 24) & 0xFF,
        0x01,
        "INFO_STACK_TYPE_BLE_STANDARD"
    );
}

#[test]
fn clearing_c2boot_does_not_stop_cpu2_but_a_reset_does() {
    let rig = Rig::new();
    rig.boot();
    rig.cpu2.set_boot(false);
    rig.run(BOOT);
    assert_eq!(
        rig.cpu2.run_state(),
        Run::Running,
        "RM0434 §6: C2BOOT gates boot only"
    );
    Device::reset(&rig.cpu2, ResetKind::Cold);
    assert_eq!(rig.cpu2.run_state(), Run::Halted);
    rig.run(10 * BOOT);
    assert_eq!(rig.cpu2.run_state(), Run::Halted);
    rig.cpu2.set_boot(true);
    rig.run(BOOT);
    assert_eq!(rig.cpu2.run_state(), Run::Running);
}

// ---------------------------------------------------------------------------
// The system channel
// ---------------------------------------------------------------------------

#[test]
fn ble_init_is_answered_in_the_system_buffer_and_the_flag_cleared() {
    let rig = Rig::new();
    rig.boot();
    assert!(!rig.cpu2.ble_ready());
    let rsp = rig.sys(SHCI_BLE_INIT, &[0; 37]);
    // Response packet 0x11, Command Complete, length 4, NumHCI 0xFF, opcode
    // 0xFC66, status 0 (AN5185 §6.3).
    assert_eq!(rsp, [0x11, 0x0E, 4, 0xFF, 0x66, 0xFC, 0x00]);
    assert!(rig.cpu2.ble_ready());
    // Now channel 1 is listened to.
    assert_eq!(rig.ipcc_r(C2MR_1) & low(CH_BLE), 0);
}

#[test]
fn an_unknown_system_command_is_acknowledged_and_noted() {
    let rig = Rig::new();
    rig.boot();
    let rsp = rig.sys(0xFCEE, &[]);
    assert_eq!(rsp[6], 0);
    assert!(
        rig.cpu2
            .notes()
            .contains(&(Note::UnknownSysCommand(0xFCEE), 1))
    );
}

#[test]
fn two_fus_get_states_reboot_into_fus_and_start_ws_reboots_back() {
    let rig = Rig::new();
    rig.boot();
    let ready = rig.sys_events();
    rig.release(&[ready[0].addr]);

    let first = rig.sys(SHCI_FUS_GET_STATE, &[]);
    assert_eq!(first, [0x11, 0x0E, 5, 0xFF, 0x52, 0xFC, 0xFE, 0xFE]);
    assert_eq!(rig.cpu2.firmware(), Firmware::Wireless);
    rig.sys(SHCI_FUS_GET_STATE, &[]);
    assert_eq!(rig.cpu2.firmware(), Firmware::Fus);
    rig.run(BOOT);
    let ready = rig.sys_events();
    assert_eq!(
        ready[0].body,
        [0x12, 0xFF, 3, 0x00, 0x92, 0x01],
        "FUS running"
    );
    rig.release(&[ready[0].addr]);
    assert_eq!(rig.r32(DIT), 0xA946_56B9, "AN5185 Table 9 magic");
    assert_eq!(rig.get(DIT + 7, 1)[0], 0x01, "stack type BLE");

    assert_eq!(rig.sys(SHCI_FUS_GET_STATE, &[])[6..], [0x00, 0x00], "idle");
    // BLE is not acknowledged while FUS runs.
    rig.ring_ble(0x0C03, &[]);
    rig.run(LATENCY + 1);
    assert_ne!(rig.ipcc_r(C1TOC2SR_1) & low(CH_BLE), 0);

    rig.ipcc_w(C1SCR, 0); // nothing
    assert_eq!(rig.sys(SHCI_FUS_START_WS, &[])[6], 0);
    rig.run(BOOT);
    let ready = rig.sys_events();
    assert_eq!(ready[0].body[5], 0x00, "wireless stack running again");
    assert_eq!(rig.cpu2.firmware(), Firmware::Wireless);
}

// ---------------------------------------------------------------------------
// The BLE channel
// ---------------------------------------------------------------------------

#[test]
fn a_ble_command_before_ble_init_waits_for_it() {
    let rig = Rig::new();
    rig.boot();
    rig.ring_ble(0x0C03, &[]);
    // Channel 1 is masked until BLE_Init, so only a pass that happens for
    // another reason sees it.
    rig.cpu2.kick();
    rig.run(4 * LATENCY);
    assert_ne!(rig.ipcc_r(C1TOC2SR_1) & low(CH_BLE), 0, "still CPU2's");
    assert!(
        rig.cpu2
            .notes()
            .iter()
            .any(|(n, _)| *n == Note::BleCommandBeforeInit(0x0C03))
    );
    rig.sys(SHCI_BLE_INIT, &[0; 37]);
    rig.run(LATENCY + 1);
    let got = rig.drain(BLE_Q, CH_BLE);
    assert_eq!(got[0].body, [0x04, 0x0E, 4, 1, 0x03, 0x0C, 0x00]);
}

#[test]
fn hci_reset_round_trips_through_the_event_queue() {
    let rig = Rig::new();
    rig.ble_up();
    rig.ring_ble(0x0C03, &[]);
    rig.run(LATENCY - 1);
    assert_ne!(
        rig.ipcc_r(C1TOC2SR_1) & low(CH_BLE),
        0,
        "not before the latency"
    );
    rig.run(2);
    assert_eq!(
        rig.ipcc_r(C1TOC2SR_1) & low(CH_BLE),
        0,
        "the command buffer is CPU1's again"
    );
    assert_ne!(rig.ipcc.c2_to_c1() & low(CH_BLE), 0, "and an event waits");
    let got = rig.drain(BLE_Q, CH_BLE);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].addr, BLE_BUF);
    assert_eq!(got[0].body, [0x04, 0x0E, 4, 1, 0x03, 0x0C, 0x00]);
}

#[test]
fn command_status_commands_get_a_command_status_in_the_cs_buffer() {
    let rig = Rig::new();
    rig.ble_up();
    // HCI_Disconnect: there is no connection.
    let got = rig.ble(0x0406, &[0x01, 0x00, 0x13]);
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].addr, CS_BUF);
    assert_eq!(got[0].body, [0x04, 0x0F, 4, 0x02, 1, 0x06, 0x04]);

    // A GAP procedure starts (status 0) and, terminated, says so.
    let got = rig.ble(0xFC97, &[0x10, 0, 0x10, 0, 0, 0]);
    assert_eq!(got[0].body, [0x04, 0x0F, 4, 0x00, 1, 0x97, 0xFC]);
    let got = rig.ble(0xFC9D, &[0x02]);
    assert_eq!(got.len(), 2);
    assert_eq!(got[0].body, [0x04, 0x0E, 4, 1, 0x9D, 0xFC, 0x00]);
    assert_ne!(got[1].addr, BLE_BUF, "the procedure event is a pool buffer");
    assert_eq!(got[1].body, [0x04, 0xFF, 5, 0x07, 0x04, 0x02, 0x00, 0x00]);
}

#[test]
fn start_scan_runs_until_terminated_by_its_own_procedure_code() {
    // AN5270 rev 36 §2.5.52 / Table 273: Scan_Mode, Procedure, then the scan
    // parameters, which this model does not read.
    let rig = Rig::new();
    rig.ble_up();
    let start = |proc_code: u8| {
        let mut p = vec![0x00, proc_code];
        p.resize(20, 0);
        rig.ble(0xFCD0, &p)[0].body.clone()
    };
    assert_eq!(start(0xB0), [0x04, 0x0F, 4, 0x00, 1, 0xD0, 0xFC]);
    assert_eq!(start(0x02)[3], 0x0C, "one scan at a time");
    // Neither legacy observation (one of 0xB0's bits) nor a lone bit of it
    // ends nor overlaps the generic scan.
    assert_eq!(rig.ble(0xFCA2, &[0; 7])[0].body[3], 0x0C);
    assert_eq!(rig.cc(0xFC9D, &[0x80]), [0x0C]);
    let got = rig.ble(0xFC9D, &[0xB0]);
    assert_eq!(got[0].body, [0x04, 0x0E, 4, 1, 0x9D, 0xFC, 0x00]);
    assert_eq!(got[1].body, [0x04, 0xFF, 5, 0x07, 0x04, 0xB0, 0x00, 0x00]);
    // A procedure Table 273 does not list.
    assert_eq!(start(0x10)[3], 0x92);
    assert_eq!(start(0x01)[3], 0x00, "limited discovery");
}

#[test]
fn cancelling_a_create_connection_reports_unknown_connection() {
    let rig = Rig::new();
    rig.ble_up();
    let got = rig.ble(0x200D, &[0; 25]);
    assert_eq!(got[0].body[3], 0x00, "Command Status, pending");
    let got = rig.ble(0x200E, &[]);
    assert_eq!(got[0].body, [0x04, 0x0E, 4, 1, 0x0E, 0x20, 0x00]);
    // LE Meta, LE Connection Complete, status 0x02.
    assert_eq!(&got[1].body[..5], &[0x04, 0x3E, 19, 0x01, 0x02]);
}

#[test]
fn an_unknown_opcode_is_answered_and_noted() {
    let rig = Rig::new();
    rig.ble_up();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&seen);
    rig.cpu2
        .set_hook(Some(Arc::new(move |n| sink.lock().push(n))));
    assert_eq!(rig.cc(0xFC00, &[]), [0x01], "Unknown HCI Command");
    assert_eq!(*seen.lock(), [Note::UnknownBleCommand(0xFC00)]);
    assert_eq!(command_name(0xFC8A), Some("ACI_GAP_INIT"));
    assert_eq!(command_name(0xFC00), None);
}

#[test]
fn local_information_is_well_formed() {
    let rig = Rig::new();
    rig.ble_up();
    let v = rig.cc(0x1001, &[]);
    assert_eq!(v.len(), 9);
    assert_eq!(v[0], 0);
    assert_eq!(
        u16::from_le_bytes([v[5], v[6]]),
        0x0030,
        "STMicroelectronics"
    );
    let bd = rig.cc(0x1009, &[]);
    assert_eq!(
        bd,
        [0, 0x00, 0x00, 0x00, 0xE1, 0x80, 0x02],
        "AN5270 Table 470"
    );
    let cmds = rig.cc(0x1002, &[]);
    assert_eq!(cmds.len(), 65);
    assert_eq!(cmds[1 + 5] & 0xC0, 0xC0, "Set Event Mask and Reset");
    assert_eq!(cmds[1 + 27] & 0x80, 0x80, "LE Rand");
    let feat = rig.cc(0x2003, &[]);
    assert_eq!(feat[1] & 1, 1, "LE encryption");
}

#[test]
fn config_data_writes_reach_read_bd_addr_and_read_back() {
    let rig = Rig::new();
    rig.ble_up();
    let addr = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    let mut p = vec![0x00, 6];
    p.extend_from_slice(&addr);
    assert_eq!(rig.cc(0xFC0C, &p), [0]);
    assert_eq!(rig.cc(0x1009, &[])[1..], addr);
    let back = rig.cc(0xFC0D, &[0x00]);
    assert_eq!(back[..2], [0, 6]);
    assert_eq!(back[2..], addr);
    let random = rig.cc(0xFC0D, &[0x80]);
    assert_eq!(random[1], 6);
    assert_eq!(random[7] & 0xC0, 0xC0, "a static random address");
    assert_eq!(rig.cc(0xFC0D, &[0x55])[0], 0x92, "not a readable offset");
    // HCI_Reset puts the configuration data back (AN5270 §2.3.1).
    rig.cc(0x0C03, &[]);
    assert_eq!(rig.cc(0x1009, &[])[1..], [0, 0, 0, 0xE1, 0x80, 0x02]);
}

#[test]
fn gap_and_gatt_build_a_database_with_st_handles() {
    let rig = Rig::new();
    rig.ble_up();
    assert_eq!(rig.cc(0xFD01, &[]), [0], "ACI_GATT_INIT");
    // Peripheral, no privacy, 8-byte name.
    let gap = rig.cc(0xFC8A, &[0x01, 0x00, 0x08]);
    // GATT holds 1..4 (service, Service Changed decl, value, CCCD); GAP
    // starts at 5 with the name at 6 and appearance at 8 (AN5289 §8.6.5's
    // nine records; AN5270 §2.5.10).
    assert_eq!(gap, [0, 0x05, 0x00, 0x06, 0x00, 0x08, 0x00]);

    // A 16-bit primary service of seven records.
    let svc = rig.cc(0xFD02, &[0x01, 0x0D, 0x18, 0x01, 7]);
    assert_eq!(svc, [0, 0x0C, 0x00], "after GAP's 5..=11");
    // A notify characteristic: decl 0x0D, value 0x0E, CCCD 0x0F.
    let ch = rig.cc(
        0xFD04,
        &[0x0C, 0x00, 0x01, 0x37, 0x2A, 4, 0, 0x10, 0, 0, 7, 0],
    );
    assert_eq!(ch, [0, 0x0D, 0x00]);
    // A read characteristic: decl 0x10, value 0x11.
    let ch2 = rig.cc(
        0xFD04,
        &[0x0C, 0x00, 0x01, 0x38, 0x2A, 1, 0, 0x02, 0, 0, 7, 0],
    );
    assert_eq!(ch2, [0, 0x10, 0x00]);
    // A user descriptor: 0x12.
    let d = rig.cc(
        0xFD05,
        &[
            0x0C, 0x00, 0x10, 0x00, 0x01, 0x01, 0x29, 4, 2, b'h', b'i', 0, 1, 0, 7, 1,
        ],
    );
    assert_eq!(d, [0, 0x12, 0x00]);
    // The range holds seven: 0x0C..=0x12 is full.
    let full = rig.cc(
        0xFD04,
        &[0x0C, 0x00, 0x01, 0x39, 0x2A, 1, 0, 0x02, 0, 0, 7, 0],
    );
    assert_eq!(full[0], 0x61, "out of handles");

    // Update the heart rate value and read it back both ways.
    assert_eq!(
        rig.cc(0xFD06, &[0x0C, 0x00, 0x0D, 0x00, 0, 2, 0x06, 0x48]),
        [0]
    );
    assert_eq!(rig.cpu2.attribute_value(0x0E), Some(vec![0x06, 0x48, 0, 0]));
    let read = rig.cc(0xFD2A, &[0x0E, 0x00, 0, 0, 0x10, 0]);
    assert_eq!(read, [0, 4, 0, 4, 0, 0x06, 0x48, 0, 0]);
    // The declaration reads as properties, value handle, UUID.
    let decl = rig.cc(0xFD2A, &[0x0D, 0x00, 0, 0, 0x10, 0]);
    assert_eq!(decl[5..], [0x10, 0x0E, 0x00, 0x37, 0x2A]);
    // A handle that is not a characteristic declaration.
    assert_eq!(rig.cc(0xFD06, &[0x0C, 0x00, 0x0E, 0x00, 0, 1, 0])[0], 0x60);

    let db = rig.cpu2.gatt();
    assert!(
        db.iter()
            .any(|(h, k, _, _)| *h == 0x0F && *k == AttrKind::Descriptor)
    );
}

#[test]
fn gatt_commands_ignore_the_service_handle() {
    // AN5270 rev 36 Tables 285, 287, 289, 347, 353: "Deprecated; this
    // parameter is ignored by the BLE stack". The characteristic handle alone
    // decides.
    let rig = Rig::new();
    rig.ble_up();
    rig.cc(0xFD01, &[]);
    rig.cc(0xFC8A, &[0x01, 0x00, 0x08]);
    assert_eq!(
        rig.cc(0xFD02, &[0x01, 0x0D, 0x18, 0x01, 7]),
        [0, 0x0C, 0x00]
    );
    let ch = rig.cc(
        0xFD04,
        &[0x0C, 0x00, 0x01, 0x37, 0x2A, 4, 0, 0x10, 0, 0, 7, 0],
    );
    assert_eq!(ch, [0, 0x0D, 0x00], "decl 0x0D, value 0x0E, CCCD 0x0F");

    // A descriptor added under a nonsense service lands in the
    // characteristic's own range.
    let d = rig.cc(
        0xFD05,
        &[
            0x34, 0x12, 0x0D, 0x00, 0x01, 0x01, 0x29, 4, 2, b'h', b'i', 0, 1, 0, 7, 1,
        ],
    );
    assert_eq!(d, [0, 0x10, 0x00]);
    assert_eq!(
        rig.cc(0xFD29, &[0, 0, 0x0D, 0, 0x10, 0, 0, 0, 2, b'o', b'k']),
        [0],
        "ACI_GATT_SET_DESC_VALUE"
    );
    assert_eq!(rig.cpu2.attribute_value(0x10), Some(b"ok".to_vec()));
    assert_eq!(
        rig.cc(0xFD06, &[0xFF, 0xFF, 0x0D, 0x00, 0, 2, 0x06, 0x48]),
        [0],
        "ACI_GATT_UPDATE_CHAR_VALUE"
    );
    assert_eq!(rig.cpu2.attribute_value(0x0E), Some(vec![0x06, 0x48, 0, 0]));
    assert_eq!(
        rig.cc(0xFD2C, &[0, 0, 0, 0, 0x0D, 0, 0, 4, 0, 0, 0, 2, 1, 2]),
        [0],
        "ACI_GATT_UPDATE_CHAR_VALUE_EXT"
    );
    assert_eq!(rig.cpu2.attribute_value(0x0E), Some(vec![1, 2, 0, 0]));
    assert_eq!(rig.cc(0xFD07, &[0, 0, 0x0D, 0]), [0], "ACI_GATT_DEL_CHAR");
    assert_eq!(rig.cc(0xFD06, &[0x0C, 0x00, 0x0D, 0x00, 0, 1, 0])[0], 0x60);
    assert_eq!(rig.cpu2.attribute_value(0x0E), None);
}

#[test]
fn ble_init_limits_are_enforced() {
    let rig = Rig::new();
    rig.boot();
    let ready = rig.sys_events();
    rig.release(&[ready[0].addr]);
    // NumAttrRecord 11 (the nine defaults and two), NumAttrServ 3.
    let mut init = [0u8; 37];
    init[8] = 11;
    init[10] = 3;
    rig.sys(SHCI_BLE_INIT, &init);
    rig.cc(0xFD01, &[]);
    rig.cc(0xFC8A, &[0x01, 0x00, 0x08]);
    assert_eq!(rig.cc(0xFD02, &[0x01, 0x0D, 0x18, 0x01, 9])[0], 0);
    assert_eq!(
        rig.cc(0xFD02, &[0x01, 0x0E, 0x18, 0x01, 9])[0],
        0x64,
        "service pool"
    );
    let c = [0x0C, 0x00, 0x01, 0x37, 0x2A, 1, 0, 0x02, 0, 0, 7, 0];
    assert_eq!(rig.cc(0xFD04, &c)[0], 0);
    assert_eq!(rig.cc(0xFD04, &c)[0], 0x64, "attribute pool");
}

#[test]
fn discoverable_builds_advertising_data_and_updates_merge() {
    let rig = Rig::new();
    rig.ble_up();
    let mut p = vec![0x00, 0x80, 0x00, 0xA0, 0x00, 0x00, 0x00];
    let name = [0x09, b'W', b'B'];
    p.push(name.len() as u8);
    p.extend_from_slice(&name);
    p.push(0);
    p.extend_from_slice(&[0, 0, 0, 0]);
    assert_eq!(rig.cc(0xFC83, &p), [0]);
    assert_eq!(rig.cpu2.advertising(), Advertising::Discoverable);
    assert_eq!(
        rig.cpu2.adv_data(),
        [0x02, 0x01, 0x06, 0x03, 0x09, b'W', b'B']
    );
    // Manufacturer data appended; a new name replaces the old.
    assert_eq!(rig.cc(0xFC8E, &[4, 3, 0xFF, 0x30, 0x00]), [0]);
    assert_eq!(rig.cc(0xFC8E, &[2, 1, 0x09]), [0]);
    assert_eq!(
        rig.cpu2.adv_data(),
        [0x02, 0x01, 0x06, 0x03, 0xFF, 0x30, 0x00, 0x01, 0x09]
    );
    assert_eq!(rig.cc(0xFC81, &[]), [0]);
    assert_eq!(rig.cpu2.advertising(), Advertising::Off);
}

#[test]
fn le_rand_is_deterministic_from_the_seed() {
    let draw = |seed: u64| {
        let rig = Rig::with(
            WbCpu2::new(&Props::new().with("seed", seed)).expect("valid"),
            4,
        );
        rig.ble_up();
        (rig.cc(0x2018, &[]), rig.cc(0x2018, &[]))
    };
    let (a1, a2) = draw(7);
    let (b1, b2) = draw(7);
    assert_eq!((a1.clone(), a2.clone()), (b1, b2));
    assert_eq!(a1.len(), 9);
    assert_ne!(a1, a2, "the stream moves");
    assert_ne!(draw(8).0, a1, "and depends on the seed");
}

#[test]
fn le_encrypt_is_aes_128_in_hci_byte_order() {
    let rig = Rig::new();
    rig.ble_up();
    // FIPS-197 Appendix C.1, each value reversed as HCI carries it.
    let mut key: Vec<u8> = (0u8..16).collect();
    let mut text: Vec<u8> = (0u8..16).map(|i| i * 0x11).collect();
    key.reverse();
    text.reverse();
    let mut p = key;
    p.extend_from_slice(&text);
    let out = rig.cc(0x2017, &p);
    let mut want = vec![
        0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5,
        0x5a,
    ];
    want.reverse();
    assert_eq!(out[0], 0);
    assert_eq!(out[1..], want);
}

#[test]
fn a_command_naming_a_connection_fails_with_unknown_connection() {
    let rig = Rig::new();
    rig.ble_up();
    // HCI_Read_RSSI echoes the handle.
    assert_eq!(rig.cc(0x1405, &[0x40, 0x00]), [0x02, 0x40, 0x00, 0x00]);
    // A GATT client procedure is a Command Status.
    let got = rig.ble(0xFD12, &[0x00, 0x00]);
    assert_eq!(got[0].body, [0x04, 0x0F, 4, 0x02, 1, 0x12, 0xFD]);
    // Except the two AN5270 (§2.6.19, §2.6.43) answers by Command Complete.
    assert_eq!(rig.cc(0xFD13, &[0x00, 0x00, 0x01, 0x0D, 0x18]), [0x02]);
    assert_eq!(rig.cc(0xFD32, &[0x00, 0x00, 2, 3, 0, 5, 0]), [0x02]);
}

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

#[test]
fn events_wait_for_buffers_when_the_pool_and_spare_are_gone() {
    // One pool slot. READY takes it; the BLE spare is the only other buffer.
    let rig = Rig::with(WbCpu2::with_defaults(), 1);
    rig.boot();
    let ready = rig.sys_events();
    assert_eq!(ready[0].addr, POOL);
    rig.sys(SHCI_BLE_INIT, &[0; 37]); // READY not handed back yet

    let cycle = |rig: &Rig| {
        rig.ring_ble(0x200D, &[0; 25]);
        rig.run(LATENCY + 1);
        rig.drain(BLE_Q, CH_BLE);
        rig.ring_ble(0x200E, &[]);
        rig.run(LATENCY + 1);
        rig.drain(BLE_Q, CH_BLE)
    };
    let first = cycle(&rig);
    assert_eq!(first.len(), 2);
    assert_eq!(first[1].addr, SPARE_BLE, "the pool is empty, so the spare");
    let second = cycle(&rig);
    assert_eq!(second.len(), 1, "the reply goes, the event waits");
    assert_eq!(rig.cpu2.events_waiting(), (1, 0));
    assert_eq!(rig.cpu2.buffers_lent(), 1);
    // Nothing was written over: READY is intact.
    assert_eq!(rig.get(POOL + 8, 6), [0x12, 0xFF, 3, 0x00, 0x92, 0x00]);

    // Hand back READY's slot: the waiting event goes out in it.
    rig.release(&[POOL]);
    let late = rig.drain(BLE_Q, CH_BLE);
    assert_eq!(late.len(), 1);
    assert_eq!(late[0].addr, POOL);
    assert_eq!(late[0].body[1], 0x3E);
    assert_eq!(rig.cpu2.events_waiting(), (0, 0));
}

#[test]
fn a_buffer_never_lent_is_noted_and_ignored() {
    let rig = Rig::new();
    rig.boot();
    rig.release(&[0x2003_1000]);
    assert!(
        rig.cpu2
            .notes()
            .contains(&(Note::ForeignBuffer(0x2003_1000), 1))
    );
    // A command response handed back (as NuttX does) is not foreign.
    rig.release(&[BLE_BUF, CS_BUF]);
    assert_eq!(rig.cpu2.notes().len(), 1);
    assert_eq!(rig.r32(FREE_Q), FREE_Q, "the free queue was emptied");
}

#[test]
fn acl_data_is_consumed_and_dropped() {
    let rig = Rig::new();
    rig.ble_up();
    rig.put(ACL_BUF + 8, &[0x02, 0x01, 0x20, 0x02, 0x00, 0xAA, 0xBB]);
    rig.ipcc_w(C1SCR, high(CH_ACL));
    rig.run(LATENCY + 1);
    assert_eq!(rig.ipcc_r(C1TOC2SR_1) & low(CH_ACL), 0, "acknowledged");
    assert_eq!(rig.ipcc.c2_to_c1() & low(CH_BLE), 0, "and nothing said");
    assert!(rig.cpu2.notes().contains(&(Note::AclDropped(0x0001), 1)));
}

#[test]
fn polling_works_without_the_interrupt_lines() {
    let cpu2 = WbCpu2::new(&Props::new().with("poll", 1000u64)).expect("valid");
    // No wires: build the space by hand.
    let space = Arc::new(AddressSpace::new("mem", 32));
    let ipcc = Arc::new(Ipcc::build());
    {
        let mut topo = space.topology();
        topo.map(
            Arc::new(MemRegion::ram("sram2", Arc::new(RamStore::new(SRAM2_LEN)))),
            SRAM2,
        )
        .expect("maps");
        topo.map(Device::region(&*ipcc, "regs").expect("a region"), IPCC)
            .expect("maps");
    }
    cpu2.attach_bus(&space, RequesterId(2));
    let rig = Rig { cpu2, ipcc, space };
    rig.lay_out(4);
    rig.boot();
    assert_eq!(rig.sys_events().len(), 1);
    let mut pkt = vec![PKT_SYS_CMD, 0x66, 0xFC, 0];
    pkt.extend_from_slice(&[0; 4]);
    rig.put(SYS_BUF + 8, &pkt);
    rig.ipcc_w(C1SCR, high(CH_SYS));
    rig.run(1001);
    assert_eq!(rig.ipcc_r(C1TOC2SR_1) & low(CH_SYS), 0);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn save(d: &dyn Device, name: &str, class: &str, version: u32) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device(name, class).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk(name, class, version).unwrap();
        d.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

fn load(d: &dyn Device, bytes: &[u8], name: &str, class: &str, version: u32) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load(name, class, version, &Migrations::new())
        .unwrap();
    d.load(&mut chunk.reader()).unwrap();
}

#[test]
fn a_snapshot_mid_command_resumes_to_the_same_answer() {
    let a = Rig::new();
    a.ble_up();
    a.cc(0xFD01, &[]);
    a.cc(0xFC8A, &[0x01, 0x00, 0x08]);
    a.cc(0x2018, &[]);
    // Ring a command and stop half-way through the latency.
    a.ring_ble(0xFD02, &[0x01, 0x0D, 0x18, 0x01, 4]);
    a.run(LATENCY / 2);

    let cpu2_bytes = save(&a.cpu2, "cpu2", CLASS_NAME, STATE_VERSION);
    let ipcc_bytes = save(
        &*a.ipcc,
        "ipcc",
        crate::dev::stm32::ipcc::CLASS.name,
        crate::dev::stm32::ipcc::CLASS.version,
    );
    let ram = a.get(SRAM2 as u32, SRAM2_LEN as usize);

    let b = Rig::new();
    b.put(SRAM2 as u32, &ram);
    load(
        &*b.ipcc,
        &ipcc_bytes,
        "ipcc",
        crate::dev::stm32::ipcc::CLASS.name,
        crate::dev::stm32::ipcc::CLASS.version,
    );
    load(&b.cpu2, &cpu2_bytes, "cpu2", CLASS_NAME, STATE_VERSION);
    assert_eq!(b.cpu2.tick(), a.cpu2.tick());
    assert_eq!(
        save(&b.cpu2, "cpu2", CLASS_NAME, STATE_VERSION),
        cpu2_bytes,
        "the state hash survives the round trip"
    );

    for rig in [&a, &b] {
        rig.run(LATENCY);
    }
    let ga = a.drain(BLE_Q, CH_BLE);
    let gb = b.drain(BLE_Q, CH_BLE);
    assert_eq!(ga, gb);
    assert_eq!(ga[0].body[6..], [0, 0x0C, 0x00]);
    // And the next random number is the same on both.
    assert_eq!(a.cc(0x2018, &[]), b.cc(0x2018, &[]));
    assert_eq!(
        save(&a.cpu2, "cpu2", CLASS_NAME, STATE_VERSION),
        save(&b.cpu2, "cpu2", CLASS_NAME, STATE_VERSION)
    );
}

#[test]
fn a_halted_cpu2_round_trips() {
    let d = WbCpu2::with_defaults();
    let bytes = save(&d, "cpu2", CLASS_NAME, STATE_VERSION);
    let e = WbCpu2::with_defaults();
    load(&e, &bytes, "cpu2", CLASS_NAME, STATE_VERSION);
    assert_eq!(save(&e, "cpu2", CLASS_NAME, STATE_VERSION), bytes);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn properties_are_validated() {
    assert!(WbCpu2::new(&Props::new().with("nonsense", 1u64)).is_err());
    assert!(WbCpu2::new(&Props::new().with("bd-addr", 1u64 << 48)).is_err());
    assert!(WbCpu2::new(&Props::new().with("latency", 0u64)).is_err());
    assert!(WbCpu2::new(&Props::new().with("ipcc", 0x1_0000_0000u64)).is_err());
    let d = WbCpu2::with_defaults();
    assert!(Device::is_lazy(&d));
    assert_eq!(Device::next_event_tick(&d), None, "idle until booted");
    assert!(Device::sink(&d, "nope", &[]).is_none());
}

#[test]
fn every_table_row_is_unique_and_answers() {
    let mut seen = alloc::collections::BTreeSet::new();
    for c in ble::COMMANDS {
        assert!(seen.insert(c.opcode), "{:#06x} twice", c.opcode);
        assert!(c.ret >= 1 || c.reply == ble::Reply::Silent, "{}", c.name);
    }
    // Every row produces a well-formed reply from zero-filled parameters.
    let mut b = ble::Ble::new([0; 6], [0; 6], 0, Stream::new(1));
    for c in ble::COMMANDS {
        let out = b.command(c.opcode, &[0; 64]);
        if let Some(e) = &out.reply {
            assert_eq!(usize::from(e[1]) + 2, e.len(), "{}", c.name);
            let want = match c.reply {
                ble::Reply::Complete => 0x0E,
                ble::Reply::Status => 0x0F,
                ble::Reply::Silent => unreachable!(),
            };
            assert_eq!(e[0], want, "{}", c.name);
        }
    }
}
