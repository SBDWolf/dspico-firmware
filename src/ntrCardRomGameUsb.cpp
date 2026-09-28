#include "common.h"
#include "hardware/structs/usb.h"
#include "tinyusb/dcd.h"
#include "ntrCardRom.h"
#include "ntrCardRomGameNoScramble.h"
#include "usbEventQueue.h"
#include "powerSaving.h"
#include "ntrCardRomGameUsb.h"
#include "usb_cdc_bridge.h"

// dcd_rp2040.c defines dcd_edpt_close() unconditionally, but the vendor
// dcd.h only declares it when TUP_DCD_EDPT_ISO_ALLOC is unset (it IS set
// for RP2040, which declares the ISO alloc/activate pair instead). The
// legacy USB_SUB_COMMAND_EP_CLOSE path below still needs this call.
extern "C" void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr);

static u8 sUsbDataBuffers[32][1024] alignas(4);
static bool sUsbActive;

extern "C" void __scratch_y("cpu0") ntrc_gameReqUsbCommandCmd1(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_noPayload(pio);
    ntrc_finishGameNoScrambleCmd1(romEmu);
    u32 subCommand = (romEmu->cmd0 >> 16) & 0xFF;
    switch (subCommand)
    {
        case USB_SUB_COMMAND_INIT:
        {
            pwr_disableUsbPowerSaving();
            dcd_init(0, nullptr);
            sUsbActive = true;
            break;
        }
        case USB_SUB_COMMAND_BEGIN_SET_ADDRESS:
        {
            dcd_set_address(0, 0);
            break;
        }
        case USB_SUB_COMMAND_REMOTE_WAKEUP:
        {
            dcd_remote_wakeup(0);
            break;
        }
        case USB_SUB_COMMAND_CONNECT:
        {
            dcd_connect(0);
            break;
        }
        case USB_SUB_COMMAND_DISCONNECT:
        {
            dcd_disconnect(0);
            break;
        }
        case USB_SUB_COMMAND_SOF_ENABLE:
        {
            dcd_sof_enable(0, true);
            break;
        }
        case USB_SUB_COMMAND_SOF_DISABLE:
        {
            dcd_sof_enable(0, false);
            break;
        }
        case USB_SUB_COMMAND_EP_CLOSE_ALL:
        {
            dcd_edpt_close_all(0);
            break;
        }
        case USB_SUB_COMMAND_EP_STALL:
        {
            dcd_edpt_stall(0, (romEmu->cmd0 >> 8) & 0xFF);
            break;
        }
        case USB_SUB_COMMAND_EP_CLEAR_STALL:
        {
            dcd_edpt_clear_stall(0, (romEmu->cmd0 >> 8) & 0xFF);
            break;
        }
        case USB_SUB_COMMAND_EP_CLOSE:
        {
            dcd_edpt_close(0, (romEmu->cmd0 >> 8) & 0xFF);
            break;
        }
        case USB_SUB_COMMAND_FINISH_SET_ADDRESS:
        {
            usb_hw->dev_addr_ctrl = (romEmu->cmd0 >> 8) & 0xFF;
            break;
        }
        case USB_SUB_COMMAND_EP_OPEN:
        {
            tusb_desc_endpoint_t endpoint;
            endpoint.bEndpointAddress = (romEmu->cmd0 >> 8) & 0xFF;
            endpoint.wMaxPacketSize = romEmu->cmd1 & 0x7FF;
            endpoint.bmAttributes.xfer = romEmu->cmd0 & 3;
            dcd_edpt_open(0, &endpoint);
            break;
        }
        case USB_SUB_COMMAND_CLEAR_EVENT_QUEUE:
        {
            usb_clearEventQueue();
            break;
        }
        case USB_SUB_COMMAND_DEINIT:
        {
            dcd_deinit(0);
            usb_clearEventQueue();
            pwr_enableUsbPowerSaving();
            break;
        }
        case USB_SUB_COMMAND_BEGIN_TRANSFER:
        {
            u32 byteCount = romEmu->cmd1;
            if (byteCount > 512)
            {
                byteCount = 512;
            }
            u32 endpoint = (romEmu->cmd0 >> 8) & 0xFF;
            u32 offset = romEmu->cmd0 & 1;
            u8* buffer = &sUsbDataBuffers[(endpoint & ~0x80) * 2 + (endpoint >> 7)][offset * 512];
            dcd_edpt_xfer(0, endpoint, buffer, byteCount);
            break;
        }
        case USB_SUB_COMMAND_INTERRUPT_ENABLE:
        {
            dcd_int_enable(0);
            break;
        }
        case USB_SUB_COMMAND_INTERRUPT_DISABLE:
        {
            dcd_int_disable(0);
            break;
        }
        case USB_SUB_COMMAND_LOCAL_STACK:
        {
            // PMDSky uplink: run the TinyUSB device stack locally on the
            // RP2040 (usb_cdc_bridge.c) instead of forwarding USB events
            // to the NDS over the card bus.
            pwr_disableUsbPowerSaving();
            usb_cdc_local_stack_start();
            sUsbActive = true;
            break;
        }
        case USB_SUB_COMMAND_LOCAL_STACK_OFF:
        {
            usb_cdc_local_stack_stop();
            break;
        }
    }
}

extern "C" void __scratch_y("cpu0") ntrc_gameReadUsbDataCmd0(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 512);
    u32 bufferIndex = romEmu->cmd0 & 1;
    u32 endpoint = (romEmu->cmd0 >> 8) & 0xFF;
    u8* buffer = &sUsbDataBuffers[(endpoint & ~0x80) * 2 + (endpoint >> 7)][bufferIndex * 512];
    if (usb_cdc_local_stack_active())
    {
        // PMDSky uplink: endpoint selects the block content
        // (0 = status block, anything else = CDC RX data)
        if (endpoint == 0)
        {
            usb_cdc_bridge_read_status(buffer);
        }
        else
        {
            usb_cdc_bridge_read_rx(buffer);
        }
    }
    ntrc_dmaToBus(buffer, 512);
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern "C" void __scratch_y("cpu0") ntrc_gameReadUsbDataCmd1(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void __scratch_y("cpu0") ntrc_gameWriteUsbDataCmd0(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

static void __scratch_y("cpu0") usbWritePayloadComplete(ntr_rom_emu_t* romEmu)
{
    if (romEmu->cmd0 & 1)
    {
        u32 bufferIndex = (romEmu->cmd0 >> 16) & 1;
        u32 endpoint = (romEmu->cmd0 >> 8) & 0xFF;
        u8* buffer = &sUsbDataBuffers[(endpoint & ~0x80) * 2 + (endpoint >> 7)][bufferIndex * 512];
        u32 byteCount = romEmu->cmd1;
        if (byteCount > 512)
        {
            byteCount = 512;
        }
        if (usb_cdc_local_stack_active())
        {
            // PMDSky uplink: feed the CDC TX ring; the firmware pumps it
            // to the host independently of the NDS frame timing.
            usb_cdc_bridge_tx_append(buffer, byteCount);
        }
        else
        {
            dcd_edpt_xfer(0, endpoint, buffer, byteCount);
        }
    }
}

extern "C" void __scratch_y("cpu0") ntrc_gameWriteUsbDataCmd1(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    u32 bufferIndex = (romEmu->cmd0 >> 16) & 1;
    u32 endpoint = (romEmu->cmd0 >> 8) & 0xFF;
    u8* buffer = &sUsbDataBuffers[(endpoint & ~0x80) * 2 + (endpoint >> 7)][bufferIndex * 512];
    if ((romEmu->cmd0 & 1) && word == 0)
    {
        ntrc_noPayload(pio);
        ntrc_finishGameNoScrambleCmd1(romEmu);
        if (!usb_cdc_local_stack_active())
        {
            dcd_edpt_xfer(0, endpoint, buffer, 0);
        }
    }
    else
    {
        ntrc_beginRead(pio, 512);
        ntrc_finishGameNoScrambleCmd1WithReadPayload(romEmu, (u32*)buffer, 512, usbWritePayloadComplete);
    }
}

extern "C" void __scratch_y("cpu0") ntrc_gameUsbGetEventCmd0(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    usb_prepareDequeueEvent();
    ntrc_finishGameNoScrambleCmd0(romEmu);
}

extern "C" void __scratch_y("cpu0") ntrc_gameUsbGetEventCmd1(ntr_rom_emu_t* romEmu, u32 word, pio_hw_t* pio)
{
    ntrc_beginWrite(pio, 4);
    ntrc_writeWord(pio, usb_dequeueEvent());
    ntrc_finishGameNoScrambleCmd1(romEmu);
}

extern "C" void ntrc_resetUsb(void)
{
    if (sUsbActive)
    {
        usb_cdc_local_stack_stop();
        dcd_disconnect(0);
        dcd_int_disable(0);
        usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_RESET;
        usb_hw->sie_ctrl = 0;
        usb_hw->inte = 0;
        usb_clearEventQueue();
        pwr_enableUsbPowerSaving();
        sUsbActive = false;
    }
}
