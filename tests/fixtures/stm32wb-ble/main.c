/*
 * STM32WB BLE smoke firmware for rsemu's `stm32wb55` / `stm32wb35` boards.
 *
 * Written for rsemu; MIT, like the rest of the repository. It is built by
 * `scripts/fetch-testdata.sh stm32wb-ble` against ST's own CMSIS device files
 * (Apache-2.0) and HAL driver (BSD-3-Clause), which that script downloads and
 * which are never committed here. tests/stm32wb_ble.rs runs the result.
 *
 * Two halves. The first is ST's HAL bringing the part up the way every Cube
 * BLE example does: HSE32 -> PLL -> 64 MHz with HCLK2 at 32 MHz, the LSE for
 * the radio's wake-up clock, the SMPS clock, HSEM, a flash page erase and
 * double-word program. Each wait loop in it is a question the board's RCC,
 * PWR and FLASH models must answer as the silicon does.
 *
 * The second is a **mailbox client of our own**, written from ST's AN5289
 * (§4.10, §14.2) and AN5185 (§6) and the AN5270 command reference, because
 * ST's STM32_WPAN transport is under ST's SLA0044 and is not used here. It
 * lays out the reference table at the start of SRAM2a, boots CPU2 with
 * PWR_CR4.C2BOOT, waits for the system READY event on IPCC channel 2, sends
 * SHCI_C2_BLE_Init, and then drives the BLE stack through channel 1: reset,
 * version, the public address, GATT and GAP init, a service and a
 * characteristic, discoverable mode, LE_Rand. Events come back on the queue
 * from CPU2's pool and are returned on channel 4, as AN5289 §14.2.2 says:
 * every event except the command responses. Both IPCC interrupts are taken
 * through the NVIC, so the board's wiring is under test as much as CPU2.
 *
 * Each step prints one line on USART1 (the Nucleo's ST-LINK virtual COM
 * port), and the test compares the transcript.
 */
#include <stdint.h>
#include <string.h>

#include "stm32wbxx_hal.h"

static UART_HandleTypeDef huart1;
static volatile uint32_t faults;

/* -------------------------------------------------------------------------
 * The three C library routines the compiler and this file need. The firmware
 * is freestanding: the toolchain it is built with may ship no libc at all.
 * ------------------------------------------------------------------------- */

void *memset(void *dst, int c, size_t n)
{
    unsigned char *d = dst;
    while (n--) {
        *d++ = (unsigned char)c;
    }
    return dst;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    unsigned char *d = dst;
    const unsigned char *s = src;
    while (n--) {
        *d++ = *s++;
    }
    return dst;
}

size_t strlen(const char *s)
{
    size_t n = 0;
    while (s[n] != 0) {
        n++;
    }
    return n;
}

/* ST's startup file calls this to run C++ constructors; there are none. */
void __libc_init_array(void) {}

/* -------------------------------------------------------------------------
 * Output
 * ------------------------------------------------------------------------- */

static void put(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 1000);
}

static void put_hex(uint32_t v)
{
    char buf[11] = "0x";
    for (int i = 0; i < 8; i++) {
        uint32_t nib = (v >> (28 - 4 * i)) & 0xf;
        buf[2 + i] = (char)(nib < 10 ? '0' + nib : 'a' + nib - 10);
    }
    buf[10] = 0;
    put(buf);
}

static void put_dec(uint32_t v)
{
    char buf[11];
    int i = 10;
    buf[i] = 0;
    do {
        buf[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v != 0);
    put(&buf[i]);
}

static void line(const char *name, uint32_t v)
{
    put(name);
    put(": ");
    put_hex(v);
    put("\r\n");
}

static void stop(const char *step, uint32_t code)
{
    put("FAIL ");
    put(step);
    put(" status ");
    put_dec(code);
    put("\r\n");
    for (;;) {
    }
}

static void check(const char *step, HAL_StatusTypeDef st)
{
    if (st != HAL_OK) {
        stop(step, (uint32_t)st);
    }
}

static void fault(const char *what)
{
    faults++;
    put("FAULT ");
    put(what);
    put(" CFSR ");
    put_hex(SCB->CFSR);
    put("\r\n");
    for (;;) {
    }
}

void HardFault_Handler(void) { fault("hard"); }
void MemManage_Handler(void) { fault("memmanage"); }
void BusFault_Handler(void) { fault("bus"); }
void UsageFault_Handler(void) { fault("usage"); }
void SysTick_Handler(void) { HAL_IncTick(); }

/* -------------------------------------------------------------------------
 * Clocks: HSE32 -> PLL (M=2, N=8, R=2) -> 64 MHz; HCLK2 (CPU2) at 32 MHz.
 * The P-NUCLEO-WB55 examples' configuration, plus the LSE for RFWKP.
 * ------------------------------------------------------------------------- */

static void clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef per = {0};

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    osc.HSEState = RCC_HSE_ON;
    osc.LSEState = RCC_LSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = RCC_PLLM_DIV2;
    osc.PLL.PLLN = 8;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    check("clock-osc", HAL_RCC_OscConfig(&osc));

    clk.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.AHBCLK2Divider = RCC_SYSCLK_DIV2;
    clk.AHBCLK4Divider = RCC_SYSCLK_DIV1;
    check("clock-bus", HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3));

    per.PeriphClockSelection = RCC_PERIPHCLK_RFWAKEUP | RCC_PERIPHCLK_SMPS;
    per.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
    per.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
    per.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
    check("clock-periph", HAL_RCCEx_PeriphCLKConfig(&per));
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    (void)h;
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &g);
}

static HAL_StatusTypeDef console_init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    return HAL_UART_Init(&huart1);
}

/* -------------------------------------------------------------------------
 * The mailbox (AN5289 §14.2, AN5185 §6.1). Every pointer is a 32-bit
 * address CPU2 dereferences, so the structures are exactly the words it
 * reads; the reference table must be the first thing in SRAM2a.
 * ------------------------------------------------------------------------- */

typedef struct node {
    struct node *next;
    struct node *prev;
} node_t;

/* A packet as it sits in a queue: the list node, then the serial bytes. */
typedef struct {
    node_t node;
    uint8_t type;
    uint8_t body[263];
} __attribute__((aligned(4))) packet_t;

#define PKT_BLE_CMD 0x01
#define PKT_BLE_EVT 0x04
#define PKT_SYS_CMD 0x10
#define EVT_CC 0x0E
#define EVT_CS 0x0F
#define EVT_VENDOR 0xFF

/* IPCC channels (AN5289 §14.2, AN5185 §6.2): n is bit n-1. */
#define CH_BLE 1
#define CH_SYS 2
#define CH_MM 4
#define CHBIT(n) (1UL << ((n) - 1))

#define POOL_SLOTS 8
#define SLOT_SIZE 268

static uint32_t ref_table[10] __attribute__((section(".mapping_table"), used));

static struct {
    uint32_t dit[16];
    uint32_t ble_table[4];
    uint32_t sys_table[2];
    uint32_t mm_table[7];
    uint32_t traces_table[1];
    node_t evt_queue;
    node_t sys_queue;
    node_t free_queue;
    node_t traces_queue;
    packet_t ble_cmd;
    uint32_t ble_cs[4];
    packet_t acl;
    packet_t sys_cmd;
    packet_t spare_ble;
    packet_t spare_sys;
    uint8_t pool[POOL_SLOTS * SLOT_SIZE] __attribute__((aligned(4)));
} mb __attribute__((section(".mailbox")));

static void list_init(node_t *head)
{
    head->next = head;
    head->prev = head;
}

static node_t *list_pop(node_t *head)
{
    node_t *n = head->next;
    if (n == head) {
        return 0;
    }
    head->next = n->next;
    n->next->prev = head;
    return n;
}

static void list_push(node_t *head, node_t *n)
{
    n->next = head;
    n->prev = head->prev;
    head->prev->next = n;
    head->prev = n;
}

/* What the IPCC interrupts saw, channel bits. The handlers mask a channel
 * when they flag it; the main loop unmasks it once it has served it. */
static volatile uint32_t rx_flags;
static volatile uint32_t tx_flags;
static volatile uint32_t rx_irqs;
static volatile uint32_t tx_irqs;

void IPCC_C1_RX_IRQHandler(void)
{
    uint32_t occupied = IPCC->C2TOC1SR & ~IPCC_C1->MR & 0x3FU;
    IPCC_C1->MR |= occupied;
    rx_flags |= occupied;
    rx_irqs++;
}

void IPCC_C1_TX_IRQHandler(void)
{
    uint32_t freed = ~IPCC->C1TOC2SR & ~(IPCC_C1->MR >> 16) & 0x3FU;
    IPCC_C1->MR |= freed << 16;
    tx_flags |= freed;
    tx_irqs++;
}

static uint32_t wait_bit(volatile uint32_t *flags, uint32_t bit, const char *what)
{
    uint32_t start = HAL_GetTick();
    while ((*flags & bit) == 0) {
        if (HAL_GetTick() - start > 500) {
            stop(what, 0xffff);
        }
    }
    __disable_irq();
    *flags &= ~bit;
    __enable_irq();
    return bit;
}

/* Channel 4: hand an event buffer back (AN5289 §14.2.2). */
static uint32_t released;

static void release(packet_t *p)
{
    while (IPCC->C1TOC2SR & CHBIT(CH_MM)) {
        /* CPU2 has not drained the last batch yet. */
    }
    list_push(&mb.free_queue, &p->node);
    IPCC_C1->SCR = CHBIT(CH_MM) << 16;
    released++;
}

static uint32_t ready_state = 0xff;
static uint32_t sys_events;

/* Drain the system queue: READY is the only event this app expects. */
static void sys_drain(void)
{
    node_t *n;
    while ((n = list_pop(&mb.sys_queue)) != 0) {
        packet_t *p = (packet_t *)n;
        sys_events++;
        if (p->body[0] == EVT_VENDOR && p->body[2] == 0x00 && p->body[3] == 0x92) {
            ready_state = p->body[4];
        }
        release(p);
    }
    IPCC_C1->SCR = CHBIT(CH_SYS);
    IPCC_C1->MR &= ~CHBIT(CH_SYS);
}

/* A system command, answered in place in the command buffer once CPU2
 * clears the channel (AN5185 §6.2). Returns the status byte. */
static uint8_t sys_cmd(uint16_t opcode, const void *params, uint8_t len)
{
    mb.sys_cmd.type = PKT_SYS_CMD;
    mb.sys_cmd.body[0] = (uint8_t)opcode;
    mb.sys_cmd.body[1] = (uint8_t)(opcode >> 8);
    mb.sys_cmd.body[2] = len;
    memcpy(&mb.sys_cmd.body[3], params, len);
    IPCC_C1->SCR = CHBIT(CH_SYS) << 16;
    IPCC_C1->MR &= ~(CHBIT(CH_SYS) << 16);
    wait_bit(&tx_flags, CHBIT(CH_SYS), "sys-cmd");
    /* type, evtcode, plen, numhci, opcode lo, opcode hi, status */
    if (mb.sys_cmd.body[0] != EVT_CC || mb.sys_cmd.body[3] != (uint8_t)opcode ||
        mb.sys_cmd.body[4] != (uint8_t)(opcode >> 8)) {
        stop("sys-rsp", mb.sys_cmd.body[0]);
    }
    return mb.sys_cmd.body[5];
}

/* The reply to the last BLE command: its return parameters. */
static uint8_t rsp[64];
static uint32_t rsp_len;
static uint32_t ble_events;

/* A BLE command; waits for the Command Complete or Command Status with the
 * same opcode on the event queue. Returns the status byte. */
static uint8_t ble_cmd(uint16_t opcode, const void *params, uint8_t len)
{
    int done = 0;
    uint8_t status = 0xff;
    mb.ble_cmd.type = PKT_BLE_CMD;
    mb.ble_cmd.body[0] = (uint8_t)opcode;
    mb.ble_cmd.body[1] = (uint8_t)(opcode >> 8);
    mb.ble_cmd.body[2] = len;
    memcpy(&mb.ble_cmd.body[3], params, len);
    IPCC_C1->SCR = CHBIT(CH_BLE) << 16;
    while (!done) {
        node_t *n;
        wait_bit(&rx_flags, CHBIT(CH_BLE), "ble-evt");
        while ((n = list_pop(&mb.evt_queue)) != 0) {
            packet_t *p = (packet_t *)n;
            const uint8_t *e = p->body;
            ble_events++;
            if (e[0] == EVT_CC && e[3] == (uint8_t)opcode && e[4] == (uint8_t)(opcode >> 8)) {
                status = e[5];
                rsp_len = e[1] > 3 ? e[1] - 3U : 0;
                memcpy(rsp, &e[5], rsp_len < sizeof rsp ? rsp_len : sizeof rsp);
                done = 1;
            } else if (e[0] == EVT_CS && e[4] == (uint8_t)opcode &&
                       e[5] == (uint8_t)(opcode >> 8)) {
                status = e[2];
                rsp_len = 1;
                rsp[0] = status;
                done = 1;
            }
            /* Command responses sit in the command buffers and are not
             * returned; everything else came from CPU2's pool. */
            if (p != &mb.ble_cmd && (void *)p != (void *)mb.ble_cs) {
                release(p);
            }
        }
        IPCC_C1->SCR = CHBIT(CH_BLE);
        IPCC_C1->MR &= ~CHBIT(CH_BLE);
    }
    return status;
}

static uint16_t u16_at(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void ble_ok(const char *step, uint16_t opcode, const void *params, uint8_t len)
{
    uint8_t st = ble_cmd(opcode, params, len);
    if (st != 0) {
        stop(step, st);
    }
}

static void mailbox_init(void)
{
    memset(&mb, 0, sizeof mb);
    list_init(&mb.evt_queue);
    list_init(&mb.sys_queue);
    list_init(&mb.free_queue);
    list_init(&mb.traces_queue);

    mb.ble_table[0] = (uint32_t)&mb.ble_cmd;
    mb.ble_table[1] = (uint32_t)mb.ble_cs;
    mb.ble_table[2] = (uint32_t)&mb.evt_queue;
    mb.ble_table[3] = (uint32_t)&mb.acl;
    mb.sys_table[0] = (uint32_t)&mb.sys_cmd;
    mb.sys_table[1] = (uint32_t)&mb.sys_queue;
    mb.mm_table[0] = (uint32_t)&mb.spare_ble;
    mb.mm_table[1] = (uint32_t)&mb.spare_sys;
    mb.mm_table[2] = (uint32_t)mb.pool;
    mb.mm_table[3] = sizeof mb.pool;
    mb.mm_table[4] = (uint32_t)&mb.free_queue;
    mb.traces_table[0] = (uint32_t)&mb.traces_queue;

    for (int i = 0; i < 10; i++) {
        ref_table[i] = 0;
    }
    ref_table[0] = (uint32_t)mb.dit;
    ref_table[1] = (uint32_t)mb.ble_table;
    ref_table[3] = (uint32_t)mb.sys_table;
    ref_table[4] = (uint32_t)mb.mm_table;
    ref_table[5] = (uint32_t)mb.traces_table;

    /* IPCC: both interrupts on, the two receive channels unmasked, every
     * transmit-free mask set until a command needs it. */
    __HAL_RCC_IPCC_CLK_ENABLE();
    __HAL_RCC_C2IPCC_CLK_ENABLE();
    IPCC_C1->CR = IPCC_CR_RXOIE | IPCC_CR_TXFIE;
    IPCC_C1->MR = 0x003F003FU & ~(CHBIT(CH_BLE) | CHBIT(CH_SYS));
    HAL_NVIC_SetPriority(IPCC_C1_RX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
    HAL_NVIC_SetPriority(IPCC_C1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);
}

/* -------------------------------------------------------------------------
 * The application
 * ------------------------------------------------------------------------- */

int main(void)
{
    HAL_Init();
    clock_config();
    check("console", console_init());
    put("stm32wb-ble\r\n");
    line("cpuid", SCB->CPUID);
    line("sysclk", HAL_RCC_GetSysClockFreq());
    line("hclk", HAL_RCC_GetHCLKFreq());
    line("hclk2", HAL_RCC_GetHCLK2Freq());
    line("lserdy", (RCC->BDCR & RCC_BDCR_LSERDY) != 0);
    line("idcode", DBGMCU->IDCODE);
    line("flash-kb", *(volatile uint16_t *)FLASHSIZE_BASE);
    line("sfsa", (FLASH->SFR & FLASH_SFR_SFSA) >> FLASH_SFR_SFSA_Pos);

    uint32_t t0 = HAL_GetTick();
    HAL_Delay(5);
    line("delay-ok", HAL_GetTick() - t0 >= 5);

    /* HSEM: the 1-step lock, then a release by the same core. */
    __HAL_RCC_HSEM_CLK_ENABLE();
    check("hsem-take", HAL_HSEM_FastTake(5));
    line("hsem-taken", HAL_HSEM_IsSemTaken(5));
    HAL_HSEM_Release(5, 0);
    line("hsem-free", !HAL_HSEM_IsSemTaken(5));

    /* CPU2 up: the mailbox, then C2BOOT (AN5289 §5.2). */
    mailbox_init();
    HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);
    wait_bit(&rx_flags, CHBIT(CH_SYS), "ready");
    sys_drain();
    line("ready", ready_state);
    line("ws-version", mb.dit[4]);

    /* SHCI_C2_BLE_Init (AN5289 §8.6.5): 68 attribute records, 8 services,
     * 1344 bytes of attribute values, 8 links, MTU 156. */
    static const uint8_t ble_init[32] = {
        0, 0, 0, 0,       /* pBleBufferAddress */
        0, 0, 0, 0,       /* BleBufferSize */
        68, 0,            /* NumAttrRecord */
        8, 0,             /* NumAttrServ */
        0x40, 0x05,       /* AttrValueArrSize */
        8,                /* NumOfLinks */
        1,                /* ExtendedPacketLengthEnable */
        0x3a,             /* PrWriteListSize */
        0x79,             /* MblockCount */
        156, 0,           /* AttMtu */
        0xf4, 0x01,       /* SlaveSca */
        0,                /* MasterSca */
        1,                /* LsSource */
        0xff, 0xff, 0xff, 0xff, /* MaxConnEventLength */
        0x48, 0x01,       /* HsStartupTime */
        1,                /* ViterbiEnable */
        0,                /* Options */
    };
    line("ble-init", sys_cmd(0xFC66, ble_init, sizeof ble_init));

    ble_ok("hci-reset", 0x0C03, 0, 0);
    ble_ok("hci-version", 0x1001, 0, 0);
    line("hci-version", rsp[1]);

    /* aci_hal_write_config_data: the public address, offset 0, 6 bytes. */
    static const uint8_t bdaddr[8] = {0x00, 6, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    ble_ok("write-bdaddr", 0xFC0C, bdaddr, sizeof bdaddr);
    ble_ok("read-bdaddr", 0x1009, 0, 0);
    line("bdaddr-lo", rsp[1] | (rsp[2] << 8) | (rsp[3] << 16) | ((uint32_t)rsp[4] << 24));
    line("bdaddr-hi", rsp[5] | (rsp[6] << 8));

    static const uint8_t tx_power[2] = {1, 0x18};
    ble_ok("tx-power", 0xFC0F, tx_power, sizeof tx_power);
    ble_ok("gatt-init", 0xFD01, 0, 0);

    /* aci_gap_init: peripheral, no privacy, an 8-byte device name. */
    static const uint8_t gap_init[3] = {0x01, 0x00, 8};
    ble_ok("gap-init", 0xFC8A, gap_init, sizeof gap_init);
    uint16_t gap_service = u16_at(&rsp[1]);
    uint16_t name_char = u16_at(&rsp[3]);
    line("gap-service", gap_service);
    line("gap-name", name_char);
    line("gap-appearance", u16_at(&rsp[5]));

    /* aci_gatt_update_char_value: the device name. */
    uint8_t name[14] = {0};
    name[0] = (uint8_t)gap_service;
    name[1] = (uint8_t)(gap_service >> 8);
    name[2] = (uint8_t)name_char;
    name[3] = (uint8_t)(name_char >> 8);
    name[4] = 0;
    name[5] = 8;
    memcpy(&name[6], "rsemu-wb", 8);
    ble_ok("set-name", 0xFD06, name, sizeof name);

    /* aci_gatt_add_service: Battery Service (0x180F), primary, 4 records. */
    static const uint8_t add_service[5] = {0x01, 0x0F, 0x18, 0x01, 4};
    ble_ok("add-service", 0xFD02, add_service, sizeof add_service);
    uint16_t battery = u16_at(&rsp[1]);
    line("battery-service", battery);

    /* aci_gatt_add_char: Battery Level (0x2A19), 1 byte, read + notify. */
    uint8_t add_char[12] = {0};
    add_char[0] = (uint8_t)battery;
    add_char[1] = (uint8_t)(battery >> 8);
    add_char[2] = 0x01;
    add_char[3] = 0x19;
    add_char[4] = 0x2A;
    add_char[5] = 1;
    add_char[6] = 0;
    add_char[7] = 0x12;
    add_char[8] = 0x00;
    add_char[9] = 0x00;
    add_char[10] = 0x10;
    add_char[11] = 0x00;
    ble_ok("add-char", 0xFD04, add_char, sizeof add_char);
    line("battery-level", u16_at(&rsp[1]));

    /* aci_gap_set_discoverable: undirected, 80-100 ms, the name in the
     * advertising data. It goes nowhere: there is no radio. */
    uint8_t disc[22] = {0};
    disc[0] = 0x00;
    disc[1] = 0x80;
    disc[3] = 0xA0;
    disc[5] = 0x00;
    disc[6] = 0x00;
    disc[7] = 9;
    disc[8] = 0x09;
    memcpy(&disc[9], "rsemu-wb", 8);
    disc[17] = 0;
    ble_ok("discoverable", 0xFC83, disc, sizeof disc);

    ble_ok("le-rand", 0x2018, 0, 0);
    line("le-rand-lo", rsp[1] | (rsp[2] << 8) | (rsp[3] << 16) | ((uint32_t)rsp[4] << 24));

    /* An opcode nothing defines: Unknown HCI Command (Core Vol 4 Part E §4.5). */
    line("unknown", ble_cmd(0xFC3F, 0, 0));

    /* Flash, with CPU2 told first (SHCI_C2_FLASH_EraseActivity, 0xFC69). */
    static const uint8_t erase_on[1] = {1};
    static const uint8_t erase_off[1] = {0};
    line("erase-activity", sys_cmd(0xFC69, erase_on, 1));
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t bad = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = 0x30;
    erase.NbPages = 1;
    check("flash-unlock", HAL_FLASH_Unlock());
    check("flash-erase", HAL_FLASHEx_Erase(&erase, &bad));
    line("flash-erased", *(volatile uint32_t *)0x08030000);
    check("flash-program",
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x08030000, 0x0badf00d01234567ULL));
    HAL_FLASH_Lock();
    line("flash-word0", *(volatile uint32_t *)0x08030000);
    line("flash-word1", *(volatile uint32_t *)0x08030004);
    sys_cmd(0xFC69, erase_off, 1);

    line("sys-events", sys_events);
    line("ble-events", ble_events);
    line("released", released);
    line("rx-irqs", rx_irqs != 0);
    line("tx-irqs", tx_irqs != 0);
    line("faults", faults);
    put("PASS\r\n");
    for (;;) {
        __WFI();
    }
}
