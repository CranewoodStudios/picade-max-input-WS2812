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
// CHANGES vs. original firmware (search "CHANGED" for all differences):
//
// 1) bcdUSB bumped from 0x0200 to 0x0201 (required for BOS descriptor)
// 2) bcdDevice bumped from 0x0100 to 0x0110 so Windows re-reads descriptors
// 3) Interface strings 4/5 changed to "Picade Player 1"/"Picade Player 2"
// 4) Added BOS descriptor with MS OS 2.0 Platform Capability
// 5) Added MS OS 2.0 Descriptor Set with:
//    - CCGP device flag (forces Windows composite device driver)
//    - Per-function DeviceInterfaceGUIDs (unique per gamepad)
// 6) Added tud_vendor_control_xfer_cb() to serve the MS OS 2.0 set
// =====================================================================

#ifndef USB_DEVICE_VERSION
// CHANGED: bumped from 0x0100 to 0x0110 so Windows re-reads descriptors
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
// Microsoft OS 2.0 Descriptor Support
//--------------------------------------------------------------------+
//
// WHY: Windows uses the device-level product name for all HID children
// of a composite device. Both gamepads show as "Picade Max" so Windows
// appends "(1)" and "(2)". RetroBat sees two instances of the same
// controller type instead of two distinct controllers.
//
// HOW: MS OS 2.0 descriptors tell Windows to:
//  a) Use the Generic Composite Parent driver (CCGP) which creates
//     separate device nodes per interface, each with its own name
//     taken from the interface string descriptor.
//  b) Assign unique DeviceInterfaceGUIDs per gamepad so applications
//     can distinguish the two controllers at the API level.
//
// The mechanism:
//  1. BOS descriptor advertises MS OS 2.0 capability + a vendor code
//  2. Windows sends a vendor request with that code
//  3. We return the MS OS 2.0 descriptor set
//  4. Windows stores the per-interface registry properties
//--------------------------------------------------------------------+

// Vendor request code for MS OS 2.0 descriptor set retrieval
#define MS_OS_20_VENDOR_CODE  0x01

// MS OS 2.0 descriptor types
#define MS_OS_20_SET_HEADER_DESCRIPTOR       0x00
#define MS_OS_20_SUBSET_HEADER_CONFIGURATION 0x01
#define MS_OS_20_SUBSET_HEADER_FUNCTION      0x02
#define MS_OS_20_FEATURE_COMPATIBLE_ID       0x03
#define MS_OS_20_FEATURE_REG_PROPERTY        0x04
#define MS_OS_20_FEATURE_MIN_RESUME_TIME     0x05
#define MS_OS_20_FEATURE_MODEL_ID            0x06
#define MS_OS_20_FEATURE_CCGP_DEVICE         0x07

// Property data type for registry strings
#define MS_OS_20_PROPERTY_DATA_TYPE_REG_MULTI_SZ  0x07

// UUID for MS OS 2.0 Platform Capability (from Microsoft spec):
// {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
#define MS_OS_20_PLATFORM_UUID \
    0xDF, 0x60, 0xDD, 0xD8, \
    0x89, 0x45, \
    0xC7, 0x4C, \
    0x9C, 0xD2, \
    0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F

// ---- Size calculations (verified with Python script) ----

// Property name: "DeviceInterfaceGUIDs" + null = 21 UTF-16LE chars = 42 bytes
#define PROP_NAME_LEN  42

// Property value: "{GUID}\0\0" in REG_MULTI_SZ format
// GUID string = 38 chars + null = 39 * 2 = 78 bytes
// Extra null for MULTI_SZ termination = 2 bytes
// Total = 80 bytes
#define PROP_VALUE_LEN 80

// Registry property descriptor:
// wLength(2) + wDescriptorType(2) + wPropertyDataType(2) +
// wPropertyNameLength(2) + PropertyName(42) +
// wPropertyDataLength(2) + PropertyData(80) = 132
#define REG_PROPERTY_DESC_LEN  132

// Function subset: header(8) + registry property(132) = 140
#define FUNCTION_SUBSET_LEN  140

// Configuration subset: header(8) + CCGP(4) + 2 function subsets(280) = 292
#define CONFIG_SUBSET_LEN  292

// Total descriptor set: header(10) + config subset(292) = 302
#define MS_OS_20_DESC_SET_LEN  302

// ---- Property name in UTF-16LE ----
// "DeviceInterfaceGUIDs\0"
#define DEVINTERFACE_GUIDS_NAME_UTF16LE \
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, \
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, \
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0

// ---- Unique GUIDs per gamepad ----
// Gamepad 1: {B1E44B8A-1001-4A01-B8E6-1F3A7B6C8D01}
#define GAMEPAD1_GUID_UTF16LE \
    '{', 0, 'B', 0, '1', 0, 'E', 0, '4', 0, '4', 0, 'B', 0, '8', 0, \
    'A', 0, '-', 0, '1', 0, '0', 0, '0', 0, '1', 0, '-', 0, '4', 0, \
    'A', 0, '0', 0, '1', 0, '-', 0, 'B', 0, '8', 0, 'E', 0, '6', 0, \
    '-', 0, '1', 0, 'F', 0, '3', 0, 'A', 0, '7', 0, 'B', 0, '6', 0, \
    'C', 0, '8', 0, 'D', 0, '0', 0, '1', 0, '}', 0, 0, 0

// Gamepad 2: {B1E44B8A-1002-4A01-B8E6-1F3A7B6C8D02}
#define GAMEPAD2_GUID_UTF16LE \
    '{', 0, 'B', 0, '1', 0, 'E', 0, '4', 0, '4', 0, 'B', 0, '8', 0, \
    'A', 0, '-', 0, '1', 0, '0', 0, '0', 0, '2', 0, '-', 0, '4', 0, \
    'A', 0, '0', 0, '1', 0, '-', 0, 'B', 0, '8', 0, 'E', 0, '6', 0, \
    '-', 0, '1', 0, 'F', 0, '3', 0, 'A', 0, '7', 0, 'B', 0, '6', 0, \
    'C', 0, '8', 0, 'D', 0, '0', 0, '2', 0, '}', 0, 0, 0

// ---- The MS OS 2.0 Descriptor Set ----
static const uint8_t ms_os_20_descriptor_set[MS_OS_20_DESC_SET_LEN] = {

    // ======== Descriptor Set Header (10 bytes) ========
    0x0A, 0x00,                                      // wLength
    MS_OS_20_SET_HEADER_DESCRIPTOR, 0x00,            // wDescriptorType
    0x00, 0x00, 0x03, 0x06,                          // dwWindowsVersion: Win 8.1+
    (MS_OS_20_DESC_SET_LEN & 0xFF),                  // wTotalLength (low)
    ((MS_OS_20_DESC_SET_LEN >> 8) & 0xFF),           // wTotalLength (high)

    // ======== Configuration Subset Header (8 bytes) ========
    0x08, 0x00,                                      // wLength
    MS_OS_20_SUBSET_HEADER_CONFIGURATION, 0x00,      // wDescriptorType
    0x00,                                            // bConfigurationValue
    0x00,                                            // bReserved
    (CONFIG_SUBSET_LEN & 0xFF),                      // wTotalLength (low)
    ((CONFIG_SUBSET_LEN >> 8) & 0xFF),               // wTotalLength (high)

    // ======== CCGP Device Descriptor (4 bytes) ========
    // Tells Windows to use Generic Composite Parent driver.
    // This makes Windows create separate child device nodes for each
    // interface, and each child gets its name from the interface
    // string descriptor (indices 4 and 5).
    0x04, 0x00,                                      // wLength
    MS_OS_20_FEATURE_CCGP_DEVICE, 0x00,              // wDescriptorType

    // ======== Function Subset: Gamepad 1 — Interface 0 (140 bytes) ========

    // ---- Function Subset Header (8 bytes) ----
    0x08, 0x00,                                      // wLength
    MS_OS_20_SUBSET_HEADER_FUNCTION, 0x00,           // wDescriptorType
    ITF_GAMEPAD_1,                                   // bFirstInterface
    0x00,                                            // bReserved
    (FUNCTION_SUBSET_LEN & 0xFF),                    // wSubsetLength (low)
    ((FUNCTION_SUBSET_LEN >> 8) & 0xFF),             // wSubsetLength (high)

    // ---- Registry Property: DeviceInterfaceGUIDs (132 bytes) ----
    (REG_PROPERTY_DESC_LEN & 0xFF),                  // wLength (low)
    ((REG_PROPERTY_DESC_LEN >> 8) & 0xFF),           // wLength (high)
    MS_OS_20_FEATURE_REG_PROPERTY, 0x00,             // wDescriptorType
    MS_OS_20_PROPERTY_DATA_TYPE_REG_MULTI_SZ, 0x00,  // wPropertyDataType
    (PROP_NAME_LEN & 0xFF), ((PROP_NAME_LEN >> 8) & 0xFF),  // wPropertyNameLength
    DEVINTERFACE_GUIDS_NAME_UTF16LE,                 // PropertyName (42 bytes)
    (PROP_VALUE_LEN & 0xFF), ((PROP_VALUE_LEN >> 8) & 0xFF), // wPropertyDataLength
    GAMEPAD1_GUID_UTF16LE,                           // GUID string (78 bytes)
    0x00, 0x00,                                      // Extra null for MULTI_SZ

    // ======== Function Subset: Gamepad 2 — Interface 1 (140 bytes) ========

    // ---- Function Subset Header (8 bytes) ----
    0x08, 0x00,                                      // wLength
    MS_OS_20_SUBSET_HEADER_FUNCTION, 0x00,           // wDescriptorType
    ITF_GAMEPAD_2,                                   // bFirstInterface
    0x00,                                            // bReserved
    (FUNCTION_SUBSET_LEN & 0xFF),                    // wSubsetLength (low)
    ((FUNCTION_SUBSET_LEN >> 8) & 0xFF),             // wSubsetLength (high)

    // ---- Registry Property: DeviceInterfaceGUIDs (132 bytes) ----
    (REG_PROPERTY_DESC_LEN & 0xFF),                  // wLength (low)
    ((REG_PROPERTY_DESC_LEN >> 8) & 0xFF),           // wLength (high)
    MS_OS_20_FEATURE_REG_PROPERTY, 0x00,             // wDescriptorType
    MS_OS_20_PROPERTY_DATA_TYPE_REG_MULTI_SZ, 0x00,  // wPropertyDataType
    (PROP_NAME_LEN & 0xFF), ((PROP_NAME_LEN >> 8) & 0xFF),  // wPropertyNameLength
    DEVINTERFACE_GUIDS_NAME_UTF16LE,                 // PropertyName (42 bytes)
    (PROP_VALUE_LEN & 0xFF), ((PROP_VALUE_LEN >> 8) & 0xFF), // wPropertyDataLength
    GAMEPAD2_GUID_UTF16LE,                           // GUID string (78 bytes)
    0x00, 0x00,                                      // Extra null for MULTI_SZ
};

// Compile-time sanity check
_Static_assert(sizeof(ms_os_20_descriptor_set) == MS_OS_20_DESC_SET_LEN,
               "MS OS 2.0 descriptor set size mismatch — check calculations!");

//--------------------------------------------------------------------+
// BOS (Binary Object Store) Descriptor
//--------------------------------------------------------------------+
// Windows 8.1+ reads the BOS descriptor on first enumeration.
// The MS OS 2.0 Platform Capability tells Windows which vendor
// request code to use when fetching the descriptor set.

#define BOS_TOTAL_LEN  (5 + 28)  // BOS header (5) + Platform capability (28)

static const uint8_t desc_bos[BOS_TOTAL_LEN] = {
    // ---- BOS Descriptor Header (5 bytes) ----
    0x05,                       // bLength
    TUSB_DESC_BOS,              // bDescriptorType (0x0F)
    (BOS_TOTAL_LEN & 0xFF),    // wTotalLength (low)
    (BOS_TOTAL_LEN >> 8),      // wTotalLength (high)
    0x01,                       // bNumDeviceCaps

    // ---- MS OS 2.0 Platform Capability Descriptor (28 bytes) ----
    0x1C,                       // bLength (28)
    TUSB_DESC_DEVICE_CAPABILITY,// bDescriptorType (0x10)
    0x05,                       // bDevCapabilityType: PLATFORM
    0x00,                       // bReserved
    MS_OS_20_PLATFORM_UUID,     // PlatformCapabilityUUID (16 bytes)
    0x00, 0x00, 0x03, 0x06,     // dwWindowsVersion: Win 8.1 (0x06030000)
    (MS_OS_20_DESC_SET_LEN & 0xFF),       // wMSOSDescriptorSetTotalLength (low)
    ((MS_OS_20_DESC_SET_LEN >> 8) & 0xFF),// wMSOSDescriptorSetTotalLength (high)
    MS_OS_20_VENDOR_CODE,       // bMS_VendorCode
    0x00,                       // bAltEnumCode: no alternate enumeration
};

_Static_assert(sizeof(desc_bos) == BOS_TOTAL_LEN,
               "BOS descriptor size mismatch!");

// CHANGED: new callback — Invoked when received GET BOS DESCRIPTOR
uint8_t const * tud_descriptor_bos_cb(void)
{
    return desc_bos;
}


//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+

// Storage for 8-byte unique ID
char usb_serial[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,

    // CHANGED: 0x0200 -> 0x0201. USB 2.01 is required for BOS descriptor support.
    // Without this, Windows won't request the BOS descriptor and the MS OS 2.0
    // mechanism won't activate.
    .bcdUSB             = 0x0201,

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

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const desc_configuration[] =
{
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(ITF_GAMEPAD_1, 4, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report_gamepad1), EPNUM_HID1, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(ITF_GAMEPAD_2, 5, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report_gamepad2), EPNUM_HID2, CFG_TUD_HID_EP_BUFSIZE, 1),
    TUD_HID_DESCRIPTOR(ITF_KEYBOARD, 6, HID_ITF_PROTOCOL_KEYBOARD, sizeof(desc_hid_report_keyboard), EPNUM_HID3, CFG_TUD_HID_EP_BUFSIZE, 1),

    TUD_CDC_DESCRIPTOR(ITF_CDC_0, 7, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

char const* string_desc_arr [] =
{
    (const char[]) { 0x09, 0x04 }, // 0: supported language is English (0x0409)
    "Pimoroni",                    // 1: Manufacturer
    "Picade Max",                  // 2: Product
    usb_serial,                    // 3: Serial number from chip ID
    // CHANGED: More descriptive interface names.
    // With the CCGP driver (from MS OS 2.0), Windows uses these as the
    // child device names instead of the generic product string.
    "Picade Player 1",             // 4: Gamepad 1 interface
    "Picade Player 2",             // 5: Gamepad 2 interface
    "Keyboard",                    // 6: Keyboard interface
    "Plasma",                      // 7: CDC/Plasma interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
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
        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str = string_desc_arr[index];

        chr_count = (uint8_t) strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

        for(uint8_t i=0; i<chr_count; i++)
        {
            _desc_str[1+i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));

    return _desc_str;
}

//--------------------------------------------------------------------+
// MS OS 2.0 Vendor Request Handler
//--------------------------------------------------------------------+
// CHANGED: new function — handles the vendor control request from Windows
// to retrieve the MS OS 2.0 descriptor set.
//
// Requires CFG_TUD_VENDOR >= 1 in tusb_config.h to enable this callback.

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    // Only handle our MS OS 2.0 vendor request
    if (request->bRequest != MS_OS_20_VENDOR_CODE) return false;

    // Only process at SETUP stage
    if (stage != CONTROL_STAGE_SETUP) return true;

    // wIndex == 7 means "Get MS OS 2.0 Descriptor Set"
    if (request->wIndex == 7)
    {
        uint16_t total_len = sizeof(ms_os_20_descriptor_set);
        if (request->wLength < total_len) total_len = request->wLength;

        return tud_control_xfer(rhport, request,
                                (void*)(uintptr_t)ms_os_20_descriptor_set,
                                total_len);
    }

    return false;
}
