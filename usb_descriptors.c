/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"
#include "pico/unique_id.h"
#include "custom_gamepad.h"

// =====================================================================
// FIX: Windows shows both gamepads as "GamePad 1" in SDL/RawGameController
//
// ROOT CAUSE:
// The device uses IAD class codes (bDeviceClass=0xEF) because of the CDC
// interface, but the HID interfaces themselves have no IADs. The Windows
// USB generic parent driver (usbccgp.sys) falls back to a behaviour that
// assigns the same product string to both HID children.
//
// FIX:
// Add explicit IAD (Interface Association Descriptor) before each HID
// interface in the configuration descriptor. Even though each HID
// function is a single interface, the IAD's iFunction string index
// tells Windows exactly which string to use as the product name for
// each child device.
//
// Per Microsoft docs on HidD_GetProductString:
//   "If the interface is grouped by a USB interface association descriptor
//    and the iFunction member of the interface association descriptor for
//    the interface is nonzero, the iProduct member of the USB_DEVICE_
//    DESCRIPTOR structure for the interface is set to the iFunction
//    member of the interface association descriptor."
//
// CHANGES (search "CHANGED" for all differences):
// 1) bcdDevice bumped from 0x0100 to 0x0110 (forces Windows re-enum)
// 2) Added IAD descriptor before each TUD_HID_DESCRIPTOR
// 3) Updated CONFIG_TOTAL_LEN to account for 3 extra IAD descriptors
// 4) Interface strings changed to "Picade Player 1"/"Picade Player 2"
// =====================================================================

#ifndef USB_DEVICE_VERSION
// CHANGED: bumped so Windows re-reads descriptors after reflash
#define USB_DEVICE_VERSION (0x0110)
#endif

enum
{
    ITF_GAMEPAD_1,
    ITF_GAMEPAD_2,
    ITF_KEYBOARD,
    ITF_CDC_0,
    ITF_CDC_0_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_HID1          0x83
#define EPNUM_HID2          0x84
#define EPNUM_HID3          0x85
#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

// Storage for 8-byte unique ID, needs 16 + 1 bytes for hex representation + '\0'.
char usb_serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x2e8a,
    .idProduct          = 0x1098,
    .bcdDevice          = USB_DEVICE_VERSION,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

void usb_serial_init(void) {
    pico_get_unique_board_id_string(usb_serial, sizeof(usb_serial));
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+

uint8_t const desc_hid_report_gamepad1[] =
{
    PICADE_HID_GAMEPAD()
};

uint8_t const desc_hid_report_gamepad2[] =
{
    PICADE_HID_GAMEPAD()
};

uint8_t const desc_hid_report_keyboard[] =
{
    TUD_HID_REPORT_DESC_KEYBOARD()
};

// Invoked when received GET HID REPORT DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf)
{
    if (itf == ITF_GAMEPAD_1)
    {
        return desc_hid_report_gamepad1;
    }
    else if (itf == ITF_GAMEPAD_2)
    {
        return desc_hid_report_gamepad2;
    }
    else if (itf == ITF_KEYBOARD)
    {
        return desc_hid_report_keyboard;
    }
    return NULL;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

// CHANGED: Helper macro for an IAD (Interface Association Descriptor).
// 8 bytes that tell the Windows generic parent driver to group the
// following interface(s) as a single function, using iFunction as the
// product string index.
//
// _first_itf : first interface number in this function
// _count     : number of interfaces in this function (1 for each HID)
// _class     : function class
// _subclass  : function subclass
// _protocol  : function protocol
// _stridx    : iFunction string index (THIS is what fixes the name!)
#define TUD_IAD_DESC(_first_itf, _count, _class, _subclass, _protocol, _stridx) \
    8, TUSB_DESC_INTERFACE_ASSOCIATION, _first_itf, _count, _class, _subclass, _protocol, _stridx

// CHANGED: CONFIG_TOTAL_LEN now includes 3 extra IAD descriptors (8 bytes each)
// Original: TUD_CONFIG_DESC_LEN + 3*TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN
// New:      TUD_CONFIG_DESC_LEN + 3*(8 + TUD_HID_DESC_LEN) + TUD_CDC_DESC_LEN
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + \
    (8 + TUD_HID_DESC_LEN) + \
    (8 + TUD_HID_DESC_LEN) + \
    (8 + TUD_HID_DESC_LEN) + \
    TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] =
{
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // CHANGED: Each HID interface is now preceded by an IAD.
    // The iFunction field (last parameter) points to the string that
    // Windows will use as the product name for this child device.

    // Gamepad 1: IAD with iFunction=4 -> "Picade Player 1"
    TUD_IAD_DESC(ITF_GAMEPAD_1, 1, TUSB_CLASS_HID, 0, HID_ITF_PROTOCOL_NONE, 4),
    TUD_HID_DESCRIPTOR(ITF_GAMEPAD_1, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report_gamepad1), EPNUM_HID1, CFG_TUD_HID_EP_BUFSIZE, 1),

    // Gamepad 2: IAD with iFunction=5 -> "Picade Player 2"
    TUD_IAD_DESC(ITF_GAMEPAD_2, 1, TUSB_CLASS_HID, 0, HID_ITF_PROTOCOL_NONE, 5),
    TUD_HID_DESCRIPTOR(ITF_GAMEPAD_2, 5, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report_gamepad2), EPNUM_HID2, CFG_TUD_HID_EP_BUFSIZE, 1),

    // Keyboard: IAD with iFunction=6 -> "Keyboard"
    TUD_IAD_DESC(ITF_KEYBOARD, 1, TUSB_CLASS_HID, HID_SUBCLASS_BOOT, HID_ITF_PROTOCOL_KEYBOARD, 6),
    TUD_HID_DESCRIPTOR(ITF_KEYBOARD, 6, HID_ITF_PROTOCOL_KEYBOARD, sizeof(desc_hid_report_keyboard), EPNUM_HID3, CFG_TUD_HID_EP_BUFSIZE, 1),

    // CDC: TUD_CDC_DESCRIPTOR already includes its own IAD internally
    TUD_CDC_DESCRIPTOR(ITF_CDC_0, 7, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index; // for multiple configurations
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
    (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
    "Pimoroni",                    // 1: Manufacturer
    "Picade Max",                  // 2: Product
    usb_serial,                    // 3: Serials, should use chip ID
    // CHANGED: descriptive names that will appear in SDL/Windows
    "Picade Player 1",             // 4: Gamepad 1 (referenced by IAD iFunction AND iInterface)
    "Picade Player 2",             // 5: Gamepad 2 (referenced by IAD iFunction AND iInterface)
    "Keyboard",                    // 6: Keyboard
    "Plasma",                      // 7: CDC/Plasma
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;

    uint8_t chr_count;

    if ( index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }else
    {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str = string_desc_arr[index];

        // Cap at max char
        chr_count = (uint8_t) strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

        // Convert ASCII string into UTF-16
        for(uint8_t i=0; i<chr_count; i++)
        {
            _desc_str[1+i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

    return _desc_str;
}
