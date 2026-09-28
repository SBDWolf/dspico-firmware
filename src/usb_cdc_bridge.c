// DSpico firmware side of the PMDSky uplink: local TinyUSB CDC-ACM stack.
// See usb_cdc_bridge.h for the mode/protocol overview.
//
// The descriptor set is the same CDC-ACM device the NDS-side uplink used
// to present (VID 0x2020, EP1 OUT / EP2 IN bulk 64, EP3 IN notification),
// so the PC-side driver/protocol does not change when the stack moves
// from the NDS to the DSpico.
#include "common.h"
#include "usb_cdc_bridge.h"
#include "pico.h"
#include "tinyusb/tusb.h"

// Bridge state (defined here, reset in usb_cdc_local_stack_start()).
// The TX/RX ring heads/tails are single 32-bit volatile words so the
// producer/consumer contexts (card PIO IRQ vs main loop) share them
// without locks.
static volatile bool s_local_active;
static volatile uint32_t s_tx_head; // TX ring consumer (main loop)
static volatile uint32_t s_tx_tail; // TX ring producer (card PIO context)
static volatile uint32_t s_rx_head; // RX ring consumer (card PIO context)
static volatile uint32_t s_rx_tail; // RX ring producer (main loop)
static bool s_dtr;
static bool s_rts;

/*----------------------------------------------------------------------*
 * USB descriptors
 *----------------------------------------------------------------------*/

#define UPLINK_VID          0x2020 // locally administered (DevKitPro-style)
#define UPLINK_PID          0xD801
#define UPLINK_BCD_USB      0x0110
#define UPLINK_EPNUM_NOTIF  0x83 // notification IN
#define UPLINK_EPNUM_DATA_OUT 0x01
#define UPLINK_EPNUM_DATA_IN  0x82
#define UPLINK_EPSIZE_NOTIF   16
#define UPLINK_EPSIZE_DATA    64 // full-speed max packet
#define UPLINK_CFG_DESC_LEN   (9 + (9 + 5 + 5 + 4 + 5) + 7 + (9 + 7 + 7))

static const uint8_t s_dev_desc[] = {
    18, /* bLength */
    0x01, /* bDescriptorType = DEVICE */
    UPLINK_BCD_USB & 0xFF, (UPLINK_BCD_USB >> 8) & 0xFF, /* bcdUSB 1.10 */
    0x02, /* bDeviceClass = CDC */
    0x00, /* bDeviceSubClass */
    0x00, /* bDeviceProtocol */
    UPLINK_EPSIZE_DATA, /* bMaxPacketSize0 */
    UPLINK_VID & 0xFF, (UPLINK_VID >> 8) & 0xFF, /* idVendor */
    UPLINK_PID & 0xFF, (UPLINK_PID >> 8) & 0xFF, /* idProduct */
    0x00, 0x01, /* bcdDevice 1.00 */
    0x01, /* iManufacturer */
    0x02, /* iProduct */
    0x03, /* iSerialNumber */
    0x01, /* bNumConfigurations */
};

static const uint8_t s_cfg_desc[] = {
    // Configuration descriptor
    9,
    0x02, /* bDescriptorType = CONFIGURATION */
    UPLINK_CFG_DESC_LEN & 0xFF, (UPLINK_CFG_DESC_LEN >> 8) & 0xFF, /* wTotalLength */
    0x02, /* bNumInterfaces */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0x80, /* bmAttributes: bus powered, no remote wakeup */
    0x32, /* bMaxPower: 100 mA */

    // CDC control interface
    9,
    0x04, /* bDescriptorType = INTERFACE */
    0x00, /* bInterfaceNumber */
    0x00, /* bAlternateSetting */
    0x01, /* bNumEndpoints (notification) */
    0x02, /* bInterfaceClass = CDC */
    0x02, /* bInterfaceSubClass = CDC Communications */
    0x01, /* bInterfaceProtocol = AT command subset */
    0x00, /* iInterface */

    // Class-specific: CDC header (1.10)
    5, 0x24, 0x01, 0x10, 0x01,
    // Class-specific: call management (no call support, data interface 1)
    5, 0x24, 0x02, 0x00, 0x01,
    // Class-specific: abstract control management (no capabilities)
    4, 0x24, 0x03, 0x00,
    // Class-specific: union (control -> data)
    5, 0x24, 0x06, 0x00, 0x01,

    // Notification endpoint
    7,
    0x05, /* bDescriptorType = ENDPOINT */
    UPLINK_EPNUM_NOTIF, /* bEndpointAddress */
    0x03, /* bmAttributes = INTERRUPT */
    UPLINK_EPSIZE_NOTIF & 0xFF, (UPLINK_EPSIZE_NOTIF >> 8) & 0xFF, /* wMaxPacketSize */
    0x0A, /* bInterval */

    // CDC data interface
    9,
    0x04, /* bDescriptorType = INTERFACE */
    0x01, /* bInterfaceNumber */
    0x00, /* bAlternateSetting */
    0x02, /* bNumEndpoints */
    0x0A, /* bInterfaceClass = CDC Data */
    0x00, /* bInterfaceSubClass */
    0x00, /* bInterfaceProtocol */
    0x01, /* iInterface = "Uplink CDC" */

    // Data endpoints
    7,
    0x05,
    UPLINK_EPNUM_DATA_OUT,
    0x02, /* bmAttributes = BULK */
    UPLINK_EPSIZE_DATA & 0xFF, (UPLINK_EPSIZE_DATA >> 8) & 0xFF,
    0x00,
    7,
    0x05,
    UPLINK_EPNUM_DATA_IN,
    0x02,
    UPLINK_EPSIZE_DATA & 0xFF, (UPLINK_EPSIZE_DATA >> 8) & 0xFF,
    0x00,
};

// String descriptors (UTF-16LE).
//
// IMPORTANT (TinyUSB 0.17 convention): usbd.c computes the string response
// length with tu_desc_len(), i.e. the FIRST BYTE ON THE WIRE is bLength.
// On this little-endian target a header word of (0x03 << 8) | bLength
// serializes as [bLength, 0x03], a valid string descriptor header.
// (The old (len << 8) | 0x03 form serializes as [0x03, len] and made the
// host see bLength=3 / a wrong bDescriptorType on every string.)
#define UPLINK_STR_HDR(blen) ((uint16_t)((0x03 << 8) | (blen)))
#define UPLINK_STR(blen, ...) (UPLINK_STR_HDR(blen)), ##__VA_ARGS__
// Language descriptor header: wire bytes 04 03 09 04 (bLength=4,
// bDescriptorType=3/string, wLANGID=0x0409). A language descriptor IS a
// string descriptor, so the type byte is 0x03 like every other entry here.
#define UPLINK_LANG_HDR ((uint16_t)(0x03 << 8) | 0x04)

#define UPLINK_STR_OFF_LANG 0  // language descriptor (string 0)
#define UPLINK_STR_OFF_MFG  2  // 0 + 2 language words
#define UPLINK_STR_OFF_PROD  (UPLINK_STR_OFF_MFG + 1 + 6)      // + 1 header + 6 chars
#define UPLINK_STR_OFF_SERIAL (UPLINK_STR_OFF_PROD + 1 + 10)   // + 1 header + 10 chars

static const uint16_t s_str_desc[] = {
    // String 0: language descriptor, wire bytes 04 03 09 04
    // (bLength=4, bDescriptorType=3/string, wLANGID=0x0409 en-US little-endian).
    UPLINK_LANG_HDR,
    0x0409,
    // String 1: "PMDSky" (6 chars -> bLength 14)
    UPLINK_STR(14, 'P', 'M', 'D', 'S', 'k', 'y'),
    // String 2: "Uplink CDC" (10 chars -> bLength 22)
    UPLINK_STR(22, 'U', 'p', 'l', 'i', 'n', 'k', ' ', 'C', 'D', 'C'),
    // String 3: "0001" (4 chars -> bLength 10)
    UPLINK_STR(10, '0', '0', '0', '1'),
};

// Compile-time guards: the descriptor arrays must be exactly as long as
// they advertise (a silent truncation would put malformed descriptors on
// the wire).
typedef char usb_cdc_desc_device_size_check[(sizeof(s_dev_desc) == 18) ? 1 : -1];
typedef char usb_cdc_desc_cfg_len_check[(UPLINK_CFG_DESC_LEN == 67) ? 1 : -1];
typedef char usb_cdc_desc_cfg_size_check[(sizeof(s_cfg_desc) == (size_t)UPLINK_CFG_DESC_LEN) ? 1 : -1];
// String table: 4 (language) + 14 + 22 + 10 = 50 bytes.
typedef char usb_cdc_str_size_check[(sizeof(s_str_desc) == 50) ? 1 : -1];
// Header words must serialize (little-endian) as [bLength, bDescriptorType]
// so tu_desc_len() (first wire byte) sees the real bLength.
typedef char usb_cdc_str_hdr_check[(((int)UPLINK_STR_HDR(14) & 0xFF) == 14 &&
                                    (((int)UPLINK_STR_HDR(14) >> 8) & 0xFF) == 3 &&
                                    ((int)UPLINK_STR_HDR(22) & 0xFF) == 22 &&
                                    ((int)UPLINK_STR_HDR(10) & 0xFF) == 10 &&
                                    ((int)UPLINK_LANG_HDR & 0xFF) == 4 &&
                                    (((int)UPLINK_LANG_HDR >> 8) & 0xFF) == 3) ? 1 : -1];
typedef char usb_cdc_str_off_check[(((UPLINK_STR_OFF_LANG + 2) <= (int)(sizeof(s_str_desc) / 2)) &&
                                    ((UPLINK_STR_OFF_MFG + 1 + 6) <= (int)(sizeof(s_str_desc) / 2)) &&
                                    ((UPLINK_STR_OFF_PROD + 1 + 10) <= (int)(sizeof(s_str_desc) / 2)) &&
                                    ((UPLINK_STR_OFF_SERIAL + 1 + 4) <= (int)(sizeof(s_str_desc) / 2))) ? 1 : -1];

uint8_t const* tud_descriptor_device_cb(void) {
    return s_dev_desc;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return s_cfg_desc;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    switch (index) {
        case 0: return &s_str_desc[UPLINK_STR_OFF_LANG];
        case 1: return &s_str_desc[UPLINK_STR_OFF_MFG];
        case 2: return &s_str_desc[UPLINK_STR_OFF_PROD];
        case 3: return &s_str_desc[UPLINK_STR_OFF_SERIAL];
        default: return NULL;
    }
}

/*----------------------------------------------------------------------*
 * Mode control
 *----------------------------------------------------------------------*/

bool usb_cdc_local_stack_active(void) {
    return s_local_active;
}

// Coarse millisecond clock for TinyUSB's delay handling (the OPT_OS_NONE
// weak tusb_time_delay_ms_api() busy-waits on it).
uint32_t tusb_time_millis_api(void) {
    return millis();
}

void usb_cdc_local_stack_start(void) {
    if (s_local_active) {
        return;
    }
    // Gate the DCD IRQ handler to the local path BEFORE dcd_init() arms
    // the USB interrupts (tusb_init -> usbd_init -> dcd_init).
    s_local_active = true;
    s_tx_head = 0;
    s_tx_tail = 0;
    s_rx_head = 0;
    s_rx_tail = 0;
    s_dtr = false;
    s_rts = false;

    tusb_rhport_init_t init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    tusb_init(0, &init);
    tud_connect();
}

void usb_cdc_local_stack_stop(void) {
    if (!s_local_active) {
        return;
    }
    tud_disconnect();
    s_local_active = false;
}
/*----------------------------------------------------------------------*
 * TX ring (card PIO context -> main loop)
 *----------------------------------------------------------------------*/

#define TX_RING_SIZE  (16u * 1024u)
#define TX_RING_MASK  (TX_RING_SIZE - 1)

static uint8_t s_tx_ring[TX_RING_SIZE];
static uint32_t s_tx_sent;
static uint32_t s_tx_dropped;

uint32_t usb_cdc_bridge_tx_append(const uint8_t* data, uint32_t len) {
    uint32_t accepted = 0;
    uint32_t tail = s_tx_tail;
    while (accepted < len) {
        uint32_t head = s_tx_head;
        if (tail - head >= TX_RING_SIZE - 1) {
            s_tx_dropped += len - accepted;
            break;
        }
        s_tx_ring[tail & TX_RING_MASK] = data[accepted];
        tail++;
        accepted++;
    }
    s_tx_tail = tail;
    return accepted;
}

/*----------------------------------------------------------------------*
 * RX ring (main loop -> card PIO context)
 *----------------------------------------------------------------------*/

#define RX_RING_SIZE  512u
#define RX_RING_MASK  (RX_RING_SIZE - 1)

static uint8_t s_rx_ring[RX_RING_SIZE];
static uint32_t s_rx_overflow;

uint32_t usb_cdc_bridge_read_rx(uint8_t* out512) {
    uint32_t n = 0;
    uint32_t head = s_rx_head;
    while (n < 512) {
        if (head == s_rx_tail) {
            break;
        }
        out512[n++] = s_rx_ring[head & RX_RING_MASK];
        head++;
    }
    s_rx_head = head;
    if (n < 512) {
        memset(out512 + n, 0, 512 - n);
    }
    return n;
}

/*----------------------------------------------------------------------*
 * TinyUSB CDC callbacks
 *----------------------------------------------------------------------*/

// CDC receive: the class driver copies OUT packets into its FIFO and
// invokes this callback; bytes are forwarded to the RX ring where the
// NDS picks them up over READ_DATA (0xEA, endpoint 1).
void tud_cdc_rx_cb(uint8_t itf) {
    uint8_t buf[64];
    for (;;) {
        uint32_t n = tud_cdc_n_read(itf, buf, sizeof(buf));
        if (n == 0) {
            return;
        }
        uint32_t tail = s_rx_tail;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t head = s_rx_head;
            if (tail - head >= RX_RING_SIZE - 1) {
                s_rx_overflow++;
                break; // ring full; drop the rest of this packet
            }
            s_rx_ring[tail & RX_RING_MASK] = buf[i];
            tail++;
        }
        s_rx_tail = tail;
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void) itf;
    s_dtr = dtr;
    s_rts = rts;
}

/*----------------------------------------------------------------------*
 * Status
 *----------------------------------------------------------------------*/

void usb_cdc_bridge_read_status(uint8_t* out512) {
    memset(out512, 0, 512);
    uint32_t w0 = (s_local_active ? 1u : 0u)
               | (tud_mounted() ? 2u : 0u)
               | (tud_cdc_connected() ? 4u : 0u)
               | (s_dtr ? 8u : 0u)
               | (s_rts ? 16u : 0u);
    uint32_t words[4] = { w0, s_tx_sent, s_tx_dropped, s_rx_tail - s_rx_head };
    for (int i = 0; i < 4; i++) {
        out512[i * 4 + 0] = (uint8_t)(words[i] & 0xFF);
        out512[i * 4 + 1] = (uint8_t)((words[i] >> 8) & 0xFF);
        out512[i * 4 + 2] = (uint8_t)((words[i] >> 16) & 0xFF);
        out512[i * 4 + 3] = (uint8_t)((words[i] >> 24) & 0xFF);
    }
}

/*----------------------------------------------------------------------*
 * Task (firmware main loop)
 *----------------------------------------------------------------------*/

void usb_cdc_task(void) {
    if (!s_local_active) {
        return;
    }
    tud_task();

    // Drain the TX ring into the CDC class FIFO. tud_cdc_write() accepts
    // whatever fits and the class driver streams it to EP2 IN as USB
    // bandwidth allows; leftover bytes wait in the ring for the next
    // iteration (independent of the NDS frame timing).
    for (;;) {
        uint32_t head = s_tx_head;
        uint32_t tail = s_tx_tail;
        if (head == tail) {
            break;
        }
        uint32_t pos = head & TX_RING_MASK;
        uint32_t contig = TX_RING_SIZE - pos;
        uint32_t avail = tail - head;
        if (contig > avail) {
            contig = avail;
        }
        uint32_t n = tud_cdc_write(&s_tx_ring[pos], contig);
        if (n == 0) {
            break; // CDC TX FIFO full; retry on the next iteration
        }
        s_tx_head = head + n;
        s_tx_sent += n;
    }

    // tud_cdc_write() only starts the EP2 IN transfer once the class TX FIFO
    // holds a full bulk packet (64 bytes); anything smaller (a command reply,
    // a short sample burst) would sit in the FIFO until enough more data
    // arrives, so short responses would never reach the host. Force-send
    // whatever is pending. No-op when the FIFO is empty or a transfer is
    // already in flight (tud_cdc_n_write_flush() claims the endpoint).
    tud_cdc_write_flush();
}

