/*
 * SPDX-FileCopyrightText: Copyright 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB host HID class driver.
 *
 * Two paths share one driver:
 *  - Report protocol (preferred): the interface's report descriptor is fetched
 *    and parsed into a field model; incoming reports are sliced per field and
 *    translated to input events. Used for every interface that exposes a usable
 *    descriptor -- mice, gamepads / joysticks, keyboards (with N-key rollover)
 *    and consumer (multimedia) controls -- including boot-capable keyboards and
 *    mice, which are switched to the report protocol on probe.
 *  - Boot protocol (fallback): if a boot keyboard (3/1/1) or mouse (3/1/2)
 *    exposes no usable report descriptor, its fixed-format report is decoded
 *    directly.
 *
 * Each class instance owns one input device (see USBH_HID_DEVICE_DEFINE at the
 * tail); consumers bind with INPUT_CALLBACK_DEFINE() on that device.
 *
 * Modeled on subsys/usb/host/class/usbh_uvc.c.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/hid.h>

#include "usbh_ch9.h"
#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_device.h"

LOG_MODULE_REGISTER(usbh_hid, CONFIG_USBH_HID_LOG_LEVEL);

/* HID class-specific requests (HID 1.11 section 7.2) on the interface recipient. */
#define HID_REQTYPE_SET                                                                            \
	(USB_REQTYPE_DIR_TO_DEVICE << 7 | USB_REQTYPE_TYPE_CLASS << 5 |                            \
	 USB_REQTYPE_RECIPIENT_INTERFACE)

/*
 * Standard GET_DESCRIPTOR on the interface recipient. The HID report descriptor
 * (0x22) is fetched per HID 1.11 section 7.1.1: bmRequestType 0x81 (dir-IN,
 * type standard, recipient interface), wIndex = interface number.
 * usbh_req_desc() cannot be used here: it hardcodes the device recipient.
 */
#define HID_REQTYPE_GET_DESC (USB_REQTYPE_DIR_TO_HOST << 7 | USB_REQTYPE_RECIPIENT_INTERFACE)

/* bInterfaceSubClass values (HID 1.11 section 4.2). */
#define HID_SUBCLASS_NONE 0U
#define HID_SUBCLASS_BOOT 1U

/* Offset of wDescriptorLength in the HID descriptor (HID 1.11 section 6.2.1). */
#define HID_DESC_REPORT_LEN_OFFSET 7U

/* Short item prefix fields and the long item prefix (HID 1.11 sections 6.2.2.2-6.2.2.3). */
#define HID_ITEM_BSIZE(b)    ((b) & 0x03U)
#define HID_ITEM_BTYPE(b)    (((b) >> 2) & 0x03U)
#define HID_ITEM_BTAG(b)     (((b) >> 4) & 0x0FU)
#define HID_ITEM_BSIZE_4     3U
#define HID_ITEM_LONG_PREFIX 0xFEU

/* Item tags not defined in hid.h (HID 1.11 sections 6.2.2.7 and 6.2.2.8). */
#define HID_ITEM_TAG_PUSH      0xAU
#define HID_ITEM_TAG_POP       0xBU
#define HID_ITEM_TAG_DELIMITER 0xAU
#define HID_DELIMITER_OPEN     1U

/* Generic Desktop usages not defined in hid.h (HID Usage Tables). */
#define HID_USAGE_GEN_DESKTOP_Z      0x32U
#define HID_USAGE_GEN_DESKTOP_RX     0x33U
#define HID_USAGE_GEN_DESKTOP_RY     0x34U
#define HID_USAGE_GEN_DESKTOP_RZ     0x35U
#define HID_USAGE_GEN_DESKTOP_SLIDER 0x36U
#define HID_USAGE_GEN_DESKTOP_DIAL   0x37U

/* Keyboard/Keypad usages (HID Usage Tables). */
#define HID_KBD_USAGE_ERR_ROLLOVER 0x01U
#define HID_KBD_USAGE_LEFTCTRL     0xE0U
#define HID_KBD_USAGE_RIGHTGUI     0xE7U

/* Consumer Controls usage page (HID Usage Tables); not in hid.h. */
#define HID_USAGE_CONSUMER        0x0C
/* Vendor-defined usage page range (HID Usage Tables): 0xFF00-0xFFFF. */
#define HID_USAGE_PAGE_VENDOR_MIN 0xFF00

/* Boot keyboard input report: [modifiers][reserved][6 keycodes]. */
#define HID_KBD_REPORT_SIZE  8U
#define HID_KBD_NUM_KEYS     6U
#define HID_KBD_KEYS_OFFSET  2U
/* Boot mouse input report: [buttons][dx][dy](+[wheel]). */
#define HID_MOUSE_REPORT_MIN 3U

/* Interrupt-IN transfers kept queued on the endpoint. */
#define HID_HOST_QUEUE_DEPTH 2U

/* Generic-parser limits. */
#define HID_FIELD_USAGES_MAX   6U  /* explicit usages captured per field */
#define HID_GLOBAL_STACK_DEPTH 4U  /* HID Push/Pop nesting */
#define HID_FIELD_BITS_MAX     32U /* widest field item (HID 1.11 section 8.4) */
/* Report length in bits beyond which a report is truncated anyway. */
#define HID_REPORT_BITS_CAP    ((CONFIG_USBH_HID_MAX_REPORT_SIZE + 1U) * 8U)
/* No data->reports[] slot for a report ID. */
#define HID_REPORT_NONE        UINT8_MAX

/* Number of codes in the INPUT_BTN_LEFT and INPUT_BTN_SOUTH button ranges. */
#define HID_MOUSE_BTN_CODES   (INPUT_BTN_TASK - INPUT_BTN_LEFT + 1U)
#define HID_GAMEPAD_BTN_CODES (INPUT_BTN_THUMBR - INPUT_BTN_SOUTH + 1U)

/*
 * HID Keyboard/Keypad usage (page 0x07) -> Zephyr input key code. Covers the
 * boot-protocol range; unmapped usages are 0 and dropped. Mirrors the standard
 * HID-usage to evdev keycode mapping used by Linux hid-input.
 */
static const uint16_t hid_kbd_keymap[] = {
	[0x04] = INPUT_KEY_A,          [0x05] = INPUT_KEY_B,          [0x06] = INPUT_KEY_C,
	[0x07] = INPUT_KEY_D,          [0x08] = INPUT_KEY_E,          [0x09] = INPUT_KEY_F,
	[0x0a] = INPUT_KEY_G,          [0x0b] = INPUT_KEY_H,          [0x0c] = INPUT_KEY_I,
	[0x0d] = INPUT_KEY_J,          [0x0e] = INPUT_KEY_K,          [0x0f] = INPUT_KEY_L,
	[0x10] = INPUT_KEY_M,          [0x11] = INPUT_KEY_N,          [0x12] = INPUT_KEY_O,
	[0x13] = INPUT_KEY_P,          [0x14] = INPUT_KEY_Q,          [0x15] = INPUT_KEY_R,
	[0x16] = INPUT_KEY_S,          [0x17] = INPUT_KEY_T,          [0x18] = INPUT_KEY_U,
	[0x19] = INPUT_KEY_V,          [0x1a] = INPUT_KEY_W,          [0x1b] = INPUT_KEY_X,
	[0x1c] = INPUT_KEY_Y,          [0x1d] = INPUT_KEY_Z,          [0x1e] = INPUT_KEY_1,
	[0x1f] = INPUT_KEY_2,          [0x20] = INPUT_KEY_3,          [0x21] = INPUT_KEY_4,
	[0x22] = INPUT_KEY_5,          [0x23] = INPUT_KEY_6,          [0x24] = INPUT_KEY_7,
	[0x25] = INPUT_KEY_8,          [0x26] = INPUT_KEY_9,          [0x27] = INPUT_KEY_0,
	[0x28] = INPUT_KEY_ENTER,      [0x29] = INPUT_KEY_ESC,        [0x2a] = INPUT_KEY_BACKSPACE,
	[0x2b] = INPUT_KEY_TAB,        [0x2c] = INPUT_KEY_SPACE,      [0x2d] = INPUT_KEY_MINUS,
	[0x2e] = INPUT_KEY_EQUAL,      [0x2f] = INPUT_KEY_LEFTBRACE,  [0x30] = INPUT_KEY_RIGHTBRACE,
	[0x31] = INPUT_KEY_BACKSLASH,  [0x32] = INPUT_KEY_BACKSLASH,  [0x33] = INPUT_KEY_SEMICOLON,
	[0x34] = INPUT_KEY_APOSTROPHE, [0x35] = INPUT_KEY_GRAVE,      [0x36] = INPUT_KEY_COMMA,
	[0x37] = INPUT_KEY_DOT,        [0x38] = INPUT_KEY_SLASH,      [0x39] = INPUT_KEY_CAPSLOCK,
	[0x3a] = INPUT_KEY_F1,         [0x3b] = INPUT_KEY_F2,         [0x3c] = INPUT_KEY_F3,
	[0x3d] = INPUT_KEY_F4,         [0x3e] = INPUT_KEY_F5,         [0x3f] = INPUT_KEY_F6,
	[0x40] = INPUT_KEY_F7,         [0x41] = INPUT_KEY_F8,         [0x42] = INPUT_KEY_F9,
	[0x43] = INPUT_KEY_F10,        [0x44] = INPUT_KEY_F11,        [0x45] = INPUT_KEY_F12,
	[0x46] = INPUT_KEY_SYSRQ,      [0x47] = INPUT_KEY_SCROLLLOCK, [0x48] = INPUT_KEY_PAUSE,
	[0x49] = INPUT_KEY_INSERT,     [0x4a] = INPUT_KEY_HOME,       [0x4b] = INPUT_KEY_PAGEUP,
	[0x4c] = INPUT_KEY_DELETE,     [0x4d] = INPUT_KEY_END,        [0x4e] = INPUT_KEY_PAGEDOWN,
	[0x4f] = INPUT_KEY_RIGHT,      [0x50] = INPUT_KEY_LEFT,       [0x51] = INPUT_KEY_DOWN,
	[0x52] = INPUT_KEY_UP,         [0x53] = INPUT_KEY_NUMLOCK,    [0x54] = INPUT_KEY_KPSLASH,
	[0x55] = INPUT_KEY_KPASTERISK, [0x56] = INPUT_KEY_KPMINUS,    [0x57] = INPUT_KEY_KPPLUS,
	[0x58] = INPUT_KEY_KPENTER,    [0x59] = INPUT_KEY_KP1,        [0x5a] = INPUT_KEY_KP2,
	[0x5b] = INPUT_KEY_KP3,        [0x5c] = INPUT_KEY_KP4,        [0x5d] = INPUT_KEY_KP5,
	[0x5e] = INPUT_KEY_KP6,        [0x5f] = INPUT_KEY_KP7,        [0x60] = INPUT_KEY_KP8,
	[0x61] = INPUT_KEY_KP9,        [0x62] = INPUT_KEY_KP0,        [0x63] = INPUT_KEY_KPDOT,
	[0x65] = INPUT_KEY_COMPOSE,
};

/* Boot keyboard modifier byte (report[0]) bit -> input key code. */
static const uint16_t hid_kbd_modmap[8] = {
	INPUT_KEY_LEFTCTRL,  INPUT_KEY_LEFTSHIFT,  INPUT_KEY_LEFTALT,  INPUT_KEY_LEFTMETA,
	INPUT_KEY_RIGHTCTRL, INPUT_KEY_RIGHTSHIFT, INPUT_KEY_RIGHTALT, INPUT_KEY_RIGHTMETA,
};

/* Boot mouse button byte bit -> input button code. */
static const uint16_t hid_mouse_btnmap[3] = {
	INPUT_BTN_LEFT,
	INPUT_BTN_RIGHT,
	INPUT_BTN_MIDDLE,
};

/*
 * A HID Button usage is just an ordinal (Button 1, 2, 3...); its meaning depends
 * on the device. We pick which INPUT_BTN_* family to map it to from the enclosing
 * application collection: e.g. Button 1 -> BTN_LEFT for a mouse, BTN_SOUTH (A) for
 * a gamepad. See hid_map_usage().
 */
enum hid_btn_ns {
	HID_BTN_NS_GENERIC = 0,
	HID_BTN_NS_MOUSE,
	HID_BTN_NS_JOYSTICK,
	HID_BTN_NS_GAMEPAD,
};

/* Input/Output/Feature main-item data bits (HID 1.11 section 6.2.2.5). */
#define HID_MAIN_ITEM_CONSTANT BIT(0) /* constant (1) vs data (0) */
#define HID_MAIN_ITEM_VARIABLE BIT(1) /* variable (1) vs array (0) */
#define HID_MAIN_ITEM_RELATIVE BIT(2) /* relative (1) vs absolute (0) */
#define HID_MAIN_ITEM_NULL     BIT(6) /* null state */

/* One parsed, mappable Input field from the report descriptor. */
struct hid_field {
	uint16_t bit_offset; /* bit position within the report payload */
	uint16_t count;      /* number of items */
	uint16_t usage_page;
	uint16_t usage_min;                    /* usage range start (when has_range) */
	uint16_t usage_max;                    /* usage range end (when has_range) */
	uint16_t usages[HID_FIELD_USAGES_MAX]; /* explicit usage list */
	int32_t logical_min;                   /* <0 means the field is signed */
	int32_t logical_max;                   /* upper bound of the valid range */
	uint8_t report_id;
	uint8_t size; /* bits per item, 1 to HID_FIELD_BITS_MAX */
	uint8_t n_usages;
	uint8_t btn_ns; /* enum hid_btn_ns */
	bool has_range;
	bool variable;   /* variable (true) vs array (false) */
	bool relative;   /* relative (true) vs absolute (false) */
	bool null_state; /* ignore out-of-range values (HID 1.11 section 5.10) */
};

/* Previous payload of a report id, for change detection on the next report. */
struct hid_report_cache {
	uint8_t id;
	uint8_t len; /* cached payload bytes */
	bool have;
	uint8_t buf[CONFIG_USBH_HID_MAX_REPORT_SIZE];
};

struct hid_host_data {
	/* Back-pointer to the input device this instance reports through. */
	const struct device *dev;
	/*
	 * Guards connected, udev and in_xfer. Probe writes the other members
	 * while connected is false, when the completion cb does not read them.
	 */
	struct k_mutex lock;
	/* Bound USB device, or NULL when idle. */
	struct usb_device *udev;
	/* In-flight interrupt-IN transfers (owned by the completion cb). */
	struct uhc_transfer *in_xfer[HID_HOST_QUEUE_DEPTH];
	bool connected;
	uint8_t iface;
	uint8_t subclass; /* bInterfaceSubClass */
	uint8_t proto;    /* HID_BOOT_IFACE_CODE_KEYBOARD/_MOUSE */
	uint8_t ep_addr;  /* interrupt-IN endpoint */
	uint16_t ep_mps;
	bool boot_fallback; /* decode the fixed boot layout (fallback) */
	bool uses_report_ids;
	/* Boot keyboard diff state. */
	uint8_t prev_keys[HID_KBD_NUM_KEYS];
	uint8_t prev_mods;
	/* Boot mouse diff state. */
	uint8_t prev_buttons;
	/* Parsed report model (report-protocol path). */
	uint8_t n_fields;
	uint8_t n_reports;
	uint8_t max_report_len; /* wire length of the largest input report */
	uint16_t app_page;      /* application-collection usage page (for vendor detection) */
	struct hid_field fields[CONFIG_USBH_HID_MAX_FIELDS];
	struct hid_report_cache reports[CONFIG_USBH_HID_MAX_REPORTS];
};

struct hid_input_evt {
	uint8_t type;
	uint16_t code;
	int32_t value;
};

/* Global item state (HID 1.11 section 6.2.2.7). */
struct hid_global {
	int32_t logical_min;
	int32_t logical_max;       /* item data sign-extended */
	uint16_t usage_page;
	uint16_t report_size;      /* saturated at UINT16_MAX */
	uint16_t report_count;     /* saturated at UINT16_MAX */
	uint8_t report_id;
	uint8_t logical_max_bytes; /* item data size of logical_max */
};

/* Local item state (HID 1.11 section 6.2.2.8); applies to the next Main item. */
struct hid_local {
	uint16_t usages[HID_FIELD_USAGES_MAX];
	uint8_t n_usages;
	uint16_t usage_min;
	uint16_t usage_max;
	bool has_range;
	uint16_t ext_page; /* extended-usage page override (item bSize=4) */
	bool in_delim;     /* inside a Delimiter alias set */
	bool delim_took;   /* already captured this set's first usage */
};

static int hid_host_report_cb(struct usb_device *const udev, struct uhc_transfer *const xfer);
static void hid_emit(const struct device *dev, struct hid_input_evt *const pending,
		     bool *const have, uint8_t type, uint16_t code, int32_t value);
static void hid_emit_flush(const struct device *dev, const struct hid_input_evt *const pending,
			   const bool have);

/* Extract n_bits (LSB-first, little-endian) starting at bit_off, unsigned. */
static uint32_t hid_extract_bits(const uint8_t *const data, uint16_t bit_off, uint8_t n_bits)
{
	uint32_t val = 0U;

	for (uint8_t i = 0U; (i < n_bits) && (i < HID_FIELD_BITS_MAX); i++) {
		uint16_t b = bit_off + i;

		if ((data[b >> 3] & BIT(b & 0x07U)) != 0U) {
			val |= BIT(i);
		}
	}

	return val;
}

/* Extract n_bits as a sign-extended value. */
static int32_t hid_extract_signed(const uint8_t *const data, uint16_t bit_off, uint8_t n_bits)
{
	uint32_t v = hid_extract_bits(data, bit_off, n_bits);

	if ((n_bits > 0U) && (n_bits < HID_FIELD_BITS_MAX)) {
		return sign_extend(v, n_bits - 1U);
	}

	return (int32_t)v;
}

/* Extract one item of a field, signed when the field's logical range is. */
static int32_t hid_field_value(const struct hid_field *const f, const uint8_t *const data,
			       uint16_t bit_off)
{
	if (f->logical_min < 0) {
		return hid_extract_signed(data, bit_off, f->size);
	}

	return (int32_t)hid_extract_bits(data, bit_off, f->size);
}

/* Copy n_bits at bit_off from src to dst, or clear them when src is NULL. */
static void hid_copy_bits(uint8_t *const dst, const uint8_t *const src, uint16_t bit_off,
			  uint16_t n_bits)
{
	for (uint16_t i = 0U; i < n_bits; i++) {
		uint16_t b = bit_off + i;
		uint8_t mask = BIT(b & 0x07U);

		if ((src != NULL) && ((src[b >> 3] & mask) != 0U)) {
			dst[b >> 3] |= mask;
		} else {
			dst[b >> 3] &= (uint8_t)~mask;
		}
	}
}

/* Keyboard/Keypad usage -> input key code, 0 if unmapped. */
static uint16_t hid_kbd_code(uint16_t usage)
{
	return (usage < ARRAY_SIZE(hid_kbd_keymap)) ? hid_kbd_keymap[usage] : 0U;
}

/* Consumer (multimedia) usage -> input key code, 0 if unmapped. */
static uint16_t hid_consumer_keycode(uint16_t usage)
{
	switch (usage) {
	case 0x0b0:
		return INPUT_KEY_PLAY;
	case 0x0b1:
		return INPUT_KEY_PAUSE;
	case 0x0b3:
		return INPUT_KEY_FASTFORWARD;
	case 0x0b4:
		return INPUT_KEY_REWIND;
	case 0x0b5:
		return INPUT_KEY_NEXTSONG;
	case 0x0b6:
		return INPUT_KEY_PREVIOUSSONG;
	case 0x0b7:
		return INPUT_KEY_STOPCD;
	case 0x0b8:
		return INPUT_KEY_EJECTCD;
	case 0x0cd:
		return INPUT_KEY_PLAYPAUSE;
	case 0x0e2:
		return INPUT_KEY_MUTE;
	case 0x0e9:
		return INPUT_KEY_VOLUMEUP;
	case 0x0ea:
		return INPUT_KEY_VOLUMEDOWN;
	case 0x224:
		return INPUT_KEY_BACK;
	case 0x225:
		return INPUT_KEY_FORWARD;
	default:
		return 0;
	}
}

/*
 * Map a (usage page, usage) to an input event type + code. `relative` picks
 * REL vs ABS for axes; `btn_ns` picks the button namespace. Returns false for
 * usages this driver does not translate.
 */
static bool hid_map_usage(uint16_t page, uint16_t usage, bool relative, uint8_t btn_ns,
			  uint8_t *const type, uint16_t *const code)
{
	switch (page) {
	case HID_USAGE_GEN_DESKTOP: {
		uint16_t rel_code, abs_code;

		switch (usage) {
		case HID_USAGE_GEN_DESKTOP_X:
			rel_code = INPUT_REL_X;
			abs_code = INPUT_ABS_X;
			break;
		case HID_USAGE_GEN_DESKTOP_Y:
			rel_code = INPUT_REL_Y;
			abs_code = INPUT_ABS_Y;
			break;
		case HID_USAGE_GEN_DESKTOP_Z:
			rel_code = INPUT_REL_Z;
			abs_code = INPUT_ABS_Z;
			break;
		case HID_USAGE_GEN_DESKTOP_RX:
			rel_code = INPUT_REL_RX;
			abs_code = INPUT_ABS_RX;
			break;
		case HID_USAGE_GEN_DESKTOP_RY:
			rel_code = INPUT_REL_RY;
			abs_code = INPUT_ABS_RY;
			break;
		case HID_USAGE_GEN_DESKTOP_RZ:
			rel_code = INPUT_REL_RZ;
			abs_code = INPUT_ABS_RZ;
			break;
		case HID_USAGE_GEN_DESKTOP_SLIDER:
			rel_code = INPUT_REL_MISC;
			abs_code = INPUT_ABS_THROTTLE;
			break;
		case HID_USAGE_GEN_DESKTOP_DIAL:
			rel_code = INPUT_REL_DIAL;
			abs_code = INPUT_ABS_WHEEL;
			break;
		case HID_USAGE_GEN_DESKTOP_WHEEL:
			rel_code = INPUT_REL_WHEEL;
			abs_code = INPUT_ABS_WHEEL;
			break;
		default:
			return false;
		}

		*type = relative ? INPUT_EV_REL : INPUT_EV_ABS;
		*code = relative ? rel_code : abs_code;
		return true;
	}
	case HID_USAGE_GEN_KEYBOARD:
		*type = INPUT_EV_KEY;
		if ((usage >= HID_KBD_USAGE_LEFTCTRL) && (usage <= HID_KBD_USAGE_RIGHTGUI)) {
			*code = hid_kbd_modmap[usage - HID_KBD_USAGE_LEFTCTRL];
			return true;
		}
		*code = hid_kbd_code(usage);
		return *code != 0U;
	case HID_USAGE_GEN_BUTTON: {
		uint16_t n;

		if (usage == 0U) {
			return false;
		}
		n = usage - 1U;
		*type = INPUT_EV_KEY;
		switch (btn_ns) {
		case HID_BTN_NS_MOUSE:
			*code = (n < HID_MOUSE_BTN_CODES) ? (uint16_t)(INPUT_BTN_LEFT + n)
							  : (uint16_t)(INPUT_BTN_0 + n);
			break;
		case HID_BTN_NS_GAMEPAD:
			*code = (n < HID_GAMEPAD_BTN_CODES) ? (uint16_t)(INPUT_BTN_SOUTH + n)
							    : (uint16_t)(INPUT_BTN_0 + n);
			break;
		case HID_BTN_NS_JOYSTICK:
		case HID_BTN_NS_GENERIC:
		default:
			*code = (uint16_t)(INPUT_BTN_0 + n);
			break;
		}
		return true;
	}
	case HID_USAGE_CONSUMER: {
		uint16_t kc = hid_consumer_keycode(usage);

		if (kc == 0U) {
			return false;
		}
		*type = INPUT_EV_KEY;
		*code = kc;
		return true;
	}
	default:
		return false;
	}
}

/*
 * Usage for item `i` of a variable field. When a field has more items than
 * declared usages, the surplus repeat the last usage (explicit list) or clamp
 * to usage_max (range) rather than mapping to nothing.
 */
static uint16_t hid_field_usage(const struct hid_field *const f, uint16_t i)
{
	if (f->n_usages > 0U) {
		return f->usages[MIN(i, (uint16_t)(f->n_usages - 1U))];
	}
	if (f->has_range) {
		uint32_t u = (uint32_t)f->usage_min + i;

		if ((f->usage_max != 0U) && (u > f->usage_max)) {
			u = f->usage_max;
		}
		return (uint16_t)MIN(u, (uint32_t)UINT16_MAX);
	}
	return 0U;
}

/*
 * Usage selected by an array field value. The value is an index into the
 * field's usages (HID 1.11 section 8.3): Usage Minimum + (value - Logical
 * Minimum) for a usage range, or the matching entry of the explicit Usage list.
 * Returns false for a value outside the logical range or one that selects no
 * usage.
 */
static bool hid_array_usage(const struct hid_field *const f, int32_t value, uint16_t *const usage)
{
	uint32_t idx;
	uint32_t u;

	if ((value < f->logical_min) || (value > f->logical_max)) {
		return false;
	}
	idx = (uint32_t)((int64_t)value - (int64_t)f->logical_min);

	if (f->n_usages > 0U) {
		if (idx >= f->n_usages) {
			return false;
		}
		u = f->usages[idx];
	} else if (f->has_range) {
		if (idx > (uint32_t)(UINT16_MAX - f->usage_min)) {
			return false;
		}
		u = f->usage_min + idx;
		if ((f->usage_max != 0U) && (u > f->usage_max)) {
			return false;
		}
	} else {
		return false;
	}

	*usage = (uint16_t)u;

	return u != 0U;
}

/* Usage selected by item `k` of an array field in `buf`. */
static bool hid_array_slot_usage(const struct hid_field *const f, const uint8_t *const buf,
				 uint16_t k, uint16_t *const usage)
{
	uint16_t bit = f->bit_offset + (uint16_t)(k * f->size);

	return hid_array_usage(f, hid_field_value(f, buf, bit), usage);
}

/* Whether any item of an array field in `buf` selects `usage`. */
static bool hid_array_has_usage(const struct hid_field *const f, const uint8_t *const buf,
				uint16_t usage)
{
	uint16_t u;

	for (uint16_t k = 0U; k < f->count; k++) {
		if (hid_array_slot_usage(f, buf, k, &u) && (u == usage)) {
			return true;
		}
	}

	return false;
}

/*
 * Keyboard phantom state (HID 1.11 Appendix C): every item of a keyboard array
 * field selects ErrorRollOver when more keys are pressed than the field holds.
 */
static bool hid_array_rollover(const struct hid_field *const f, const uint8_t *const buf)
{
	uint16_t u;

	if ((f->usage_page != HID_USAGE_GEN_KEYBOARD) || (f->count == 0U)) {
		return false;
	}

	for (uint16_t k = 0U; k < f->count; k++) {
		if (!hid_array_slot_usage(f, buf, k, &u) || (u != HID_KBD_USAGE_ERR_ROLLOVER)) {
			return false;
		}
	}

	return true;
}

/*
 * Allocate and enqueue one interrupt-IN transfer, registering it so removed()
 * can cancel it. The completion cb owns the transfer and frees it.
 */
static int hid_host_arm(struct hid_host_data *const data)
{
	struct usb_device *udev;
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	uint8_t slot = HID_HOST_QUEUE_DEPTH;
	int ret;

	/*
	 * Hold the lock across the whole arm and take udev from it. removed()
	 * grabs the same lock to clear `connected` and null data->udev, so a
	 * re-arm racing teardown is either fully ordered before removal or
	 * rejected here -- it never allocates against a stale/NULL device. The
	 * allocators are non-blocking (K_NO_WAIT), so holding the lock is safe.
	 */
	k_mutex_lock(&data->lock, K_FOREVER);

	if (!data->connected) {
		ret = -ENODEV;
		goto unlock;
	}
	udev = data->udev;

	xfer = usbh_xfer_alloc(udev, data->ep_addr, hid_host_report_cb, data);
	if (xfer == NULL) {
		ret = -ENOMEM;
		goto unlock;
	}

	/*
	 * Size the buffer to exactly one (max) report, not ep_mps: this both
	 * admits a report larger than MPS arriving across several packets
	 * (HID 1.11 section 8.4) and, because there is never room for a second
	 * whole report, stops the controller stacking reports into an over-sized
	 * transfer. Fall back to ep_mps only when no report length was parsed.
	 */
	buf = usbh_xfer_buf_alloc(udev, (data->max_report_len != 0U) ? data->max_report_len
								     : data->ep_mps);
	if (buf == NULL) {
		ret = -ENOMEM;
		goto free_xfer;
	}

	xfer->buf = buf;

	for (uint8_t i = 0U; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (data->in_xfer[i] == NULL) {
			data->in_xfer[i] = xfer;
			slot = i;
			break;
		}
	}

	if (slot == HID_HOST_QUEUE_DEPTH) {
		ret = -ENOBUFS;
		goto free_xfer;
	}

	ret = usbh_xfer_enqueue(udev, xfer);
	if (ret != 0) {
		data->in_xfer[slot] = NULL;
		goto free_xfer;
	}

	k_mutex_unlock(&data->lock);

	return 0;

free_xfer:
	/*
	 * xfer->buf is NULL until the buffer is attached, so this also covers
	 * the failed-buffer-alloc path.
	 */
	if (xfer->buf != NULL) {
		usbh_xfer_buf_free(udev, xfer->buf);
	}
	(void)uhc_xfer_unref(xfer);
unlock:
	k_mutex_unlock(&data->lock);

	return ret;
}

/*
 * Keyboard phantom state (HID 1.11 Appendix C): all six boot report keycodes
 * are ErrorRollOver when more keys are pressed than the report holds.
 */
static bool hid_kbd_phantom(const uint8_t *const keys)
{
	for (uint8_t i = 0U; i < HID_KBD_NUM_KEYS; i++) {
		if (keys[i] != HID_KBD_USAGE_ERR_ROLLOVER) {
			return false;
		}
	}

	return true;
}

/* Translate one boot keyboard report into input key events (diff vs previous). */
static void hid_kbd_decode(struct hid_host_data *const data, const uint8_t *report, size_t len)
{
	const uint8_t *keys = &report[HID_KBD_KEYS_OFFSET];
	uint8_t mods = report[0];
	struct hid_input_evt pending;
	bool have_pending = false;

	if (len < HID_KBD_REPORT_SIZE) {
		return;
	}

	/* Modifier transitions. */
	for (uint8_t b = 0U; b < ARRAY_SIZE(hid_kbd_modmap); b++) {
		bool now = (mods & BIT(b)) != 0U;
		bool was = (data->prev_mods & BIT(b)) != 0U;

		if (now != was) {
			hid_emit(data->dev, &pending, &have_pending, INPUT_EV_KEY,
				 hid_kbd_modmap[b], now ? 1 : 0);
		}
	}
	data->prev_mods = mods;

	/* A phantom report emits no key events and keeps the previous key state. */
	if (hid_kbd_phantom(keys)) {
		LOG_DBG("boot keyboard rollover (ErrorRollOver) report");
		hid_emit_flush(data->dev, &pending, have_pending);
		return;
	}

	/* Releases: previously-pressed keys absent from the new report. */
	for (uint8_t i = 0U; i < HID_KBD_NUM_KEYS; i++) {
		uint8_t usage = data->prev_keys[i];
		bool still = false;

		if (hid_kbd_code(usage) == 0U) {
			continue;
		}

		for (uint8_t j = 0U; j < HID_KBD_NUM_KEYS; j++) {
			if (keys[j] == usage) {
				still = true;
				break;
			}
		}

		if (!still) {
			hid_emit(data->dev, &pending, &have_pending, INPUT_EV_KEY,
				 hid_kbd_code(usage), 0);
		}
	}

	/* Presses: keys in the new report not present before. */
	for (uint8_t i = 0U; i < HID_KBD_NUM_KEYS; i++) {
		uint8_t usage = keys[i];
		bool had = false;

		if (hid_kbd_code(usage) == 0U) {
			continue;
		}

		for (uint8_t j = 0U; j < HID_KBD_NUM_KEYS; j++) {
			if (data->prev_keys[j] == usage) {
				had = true;
				break;
			}
		}

		if (!had) {
			hid_emit(data->dev, &pending, &have_pending, INPUT_EV_KEY,
				 hid_kbd_code(usage), 1);
		}
	}

	hid_emit_flush(data->dev, &pending, have_pending);

	memcpy(data->prev_keys, keys, HID_KBD_NUM_KEYS);
}

/* Translate one boot mouse report into input button/relative events. */
static void hid_mouse_decode(struct hid_host_data *const data, const uint8_t *report, size_t len)
{
	struct hid_input_evt pending;
	bool have_pending = false;
	uint8_t buttons = report[0];

	if (len < HID_MOUSE_REPORT_MIN) {
		return;
	}

	for (uint8_t b = 0U; b < ARRAY_SIZE(hid_mouse_btnmap); b++) {
		bool now = (buttons & BIT(b)) != 0U;
		bool was = (data->prev_buttons & BIT(b)) != 0U;

		if (now != was) {
			hid_emit(data->dev, &pending, &have_pending, INPUT_EV_KEY,
				 hid_mouse_btnmap[b], now ? 1 : 0);
		}
	}
	data->prev_buttons = buttons;

	if ((int8_t)report[1] != 0) {
		hid_emit(data->dev, &pending, &have_pending, INPUT_EV_REL, INPUT_REL_X,
			 (int8_t)report[1]);
	}
	if ((int8_t)report[2] != 0) {
		hid_emit(data->dev, &pending, &have_pending, INPUT_EV_REL, INPUT_REL_Y,
			 (int8_t)report[2]);
	}
	if ((len > HID_MOUSE_REPORT_MIN) && (report[3] != 0U)) {
		hid_emit(data->dev, &pending, &have_pending, INPUT_EV_REL, INPUT_REL_WHEEL,
			 (int8_t)report[3]);
	}

	hid_emit_flush(data->dev, &pending, have_pending);
}

/* Whether the driver maps any usage on this page. */
static bool hid_page_is_mapped(uint16_t page)
{
	return (page == HID_USAGE_GEN_DESKTOP) || (page == HID_USAGE_GEN_KEYBOARD) ||
	       (page == HID_USAGE_GEN_BUTTON) || (page == HID_USAGE_CONSUMER);
}

/* Button namespace for fields in the given application collection. */
static uint8_t hid_btn_ns(uint16_t app_page, uint16_t app_usage)
{
	if (app_page != HID_USAGE_GEN_DESKTOP) {
		return HID_BTN_NS_GENERIC;
	}

	switch (app_usage) {
	case HID_USAGE_GEN_DESKTOP_MOUSE:
	case HID_USAGE_GEN_DESKTOP_POINTER:
		return HID_BTN_NS_MOUSE;
	case HID_USAGE_GEN_DESKTOP_JOYSTICK:
		return HID_BTN_NS_JOYSTICK;
	case HID_USAGE_GEN_DESKTOP_GAMEPAD:
		return HID_BTN_NS_GAMEPAD;
	default:
		return HID_BTN_NS_GENERIC;
	}
}

/* Report length in bits after an Input item, saturated at HID_REPORT_BITS_CAP. */
static uint16_t hid_add_bits(uint16_t bits, uint32_t report_size, uint32_t report_count)
{
	uint64_t total = (uint64_t)bits + ((uint64_t)report_size * report_count);

	return (uint16_t)MIN(total, (uint64_t)HID_REPORT_BITS_CAP);
}

/*
 * Store an Input item as a field when it is data on a mapped page and every
 * item fits in the decoded payload. Returns false when the field is not kept.
 */
static bool hid_store_field(struct hid_host_data *const data, const struct hid_global *const g,
			    const struct hid_local *const l, uint32_t item_flags, uint16_t off,
			    uint8_t btn_ns)
{
	uint16_t page = (l->ext_page != 0U) ? l->ext_page : g->usage_page;
	uint64_t end = (uint64_t)off + ((uint64_t)g->report_size * g->report_count);
	struct hid_field *f;

	if (((item_flags & HID_MAIN_ITEM_CONSTANT) != 0U) || !hid_page_is_mapped(page) ||
	    (g->report_size == 0U) || (g->report_size > HID_FIELD_BITS_MAX) ||
	    (g->report_count == 0U) || (end > ((uint64_t)CONFIG_USBH_HID_MAX_REPORT_SIZE * 8U)) ||
	    (data->n_fields >= CONFIG_USBH_HID_MAX_FIELDS)) {
		return false;
	}

	f = &data->fields[data->n_fields++];
	memset(f, 0, sizeof(*f));
	f->bit_offset = off;
	f->report_id = g->report_id;
	f->size = (uint8_t)g->report_size;
	f->count = (uint16_t)g->report_count;
	f->usage_page = page;
	f->logical_min = g->logical_min;
	/*
	 * Devices often encode an unsigned Logical Maximum in too few bytes
	 * (0x25 0xFF for 255). With a non-negative Logical Minimum, Logical
	 * Maximum is read zero-extended, as Linux hid-core does.
	 */
	if ((g->logical_min >= 0) && (g->logical_max < 0)) {
		uint32_t umax = (uint32_t)g->logical_max;

		if (g->logical_max_bytes < 4U) {
			umax &= BIT(8U * g->logical_max_bytes) - 1U;
		}
		f->logical_max = (int32_t)MIN(umax, (uint32_t)INT32_MAX);
	} else {
		f->logical_max = g->logical_max;
	}
	f->variable = (item_flags & HID_MAIN_ITEM_VARIABLE) != 0U;
	f->relative = (item_flags & HID_MAIN_ITEM_RELATIVE) != 0U;
	f->null_state = (item_flags & HID_MAIN_ITEM_NULL) != 0U;
	f->n_usages = l->n_usages;
	memcpy(f->usages, l->usages, l->n_usages * sizeof(l->usages[0]));
	f->has_range = l->has_range;
	f->usage_min = l->usage_min;
	f->usage_max = l->usage_max;
	f->btn_ns = btn_ns;

	return true;
}

/* Apply one Global item to the global state and its Push/Pop stack. */
static void hid_parse_global(struct hid_host_data *const data, struct hid_global *const g,
			     struct hid_global *const stack, uint8_t *const sp, uint8_t tag,
			     uint8_t isize, uint32_t uval, int32_t sval)
{
	switch (tag) {
	case HID_ITEM_TAG_USAGE_PAGE:
		g->usage_page = (uint16_t)uval;
		break;
	case HID_ITEM_TAG_LOGICAL_MIN:
		g->logical_min = sval;
		break;
	case HID_ITEM_TAG_LOGICAL_MAX:
		g->logical_max = sval;
		g->logical_max_bytes = isize;
		break;
	case HID_ITEM_TAG_REPORT_SIZE:
		g->report_size = (uint16_t)MIN(uval, UINT16_MAX);
		break;
	case HID_ITEM_TAG_REPORT_ID:
		g->report_id = (uint8_t)uval;
		data->uses_report_ids = true;
		break;
	case HID_ITEM_TAG_REPORT_COUNT:
		g->report_count = (uint16_t)MIN(uval, UINT16_MAX);
		break;
	case HID_ITEM_TAG_PUSH:
		if (*sp < HID_GLOBAL_STACK_DEPTH) {
			stack[*sp] = *g;
			(*sp)++;
		}
		break;
	case HID_ITEM_TAG_POP:
		if (*sp > 0U) {
			(*sp)--;
			*g = stack[*sp];
		}
		break;
	default:
		break;
	}
}

/* Apply one Local item to the local state. */
static void hid_parse_local(struct hid_local *const l, uint8_t tag, uint8_t isize, uint32_t uval)
{
	switch (tag) {
	case HID_ITEM_TAG_USAGE:
	case HID_ITEM_TAG_USAGE_MIN:
	case HID_ITEM_TAG_USAGE_MAX:
		/*
		 * Delimiter set (HID 1.11 section 6.2.2.8): aliases for one
		 * control. Take only the first usage, ignore the rest until the
		 * set closes.
		 */
		if (l->in_delim) {
			if (l->delim_took) {
				break;
			}
			l->delim_took = true;
		}
		/*
		 * Extended usage (4-byte data, HID 1.11 section 6.2.2.8): the
		 * high 16 bits override the usage page, the low 16 are the
		 * usage. One page per field: the override applies to the whole
		 * field at store time (a per-usage page is not modelled).
		 */
		if (isize == 4U) {
			l->ext_page = (uint16_t)(uval >> 16);
		}
		if (tag == HID_ITEM_TAG_USAGE) {
			if (l->n_usages < HID_FIELD_USAGES_MAX) {
				l->usages[l->n_usages++] = (uint16_t)uval;
			}
		} else if (tag == HID_ITEM_TAG_USAGE_MIN) {
			l->usage_min = (uint16_t)uval;
			l->has_range = true;
		} else {
			l->usage_max = (uint16_t)uval;
			l->has_range = true;
		}
		break;
	case HID_ITEM_TAG_DELIMITER:
		l->in_delim = (uval == HID_DELIMITER_OPEN);
		l->delim_took = false;
		break;
	default:
		break;
	}
}

/*
 * Parse a HID report descriptor into data->fields[] (mappable Input items only)
 * and register the report ids in data->reports[]. Best-effort: unrecognized or
 * unsupported items are skipped; the bit cursor is still advanced so following
 * fields stay correctly aligned. Input items of a report ID without a
 * data->reports[] slot are dropped. Returns true when that happened.
 *
 * __noinline keeps this large frame off the small usbh_bus stack: it runs after
 * the descriptor fetch returns, not nested beneath it.
 */
static __noinline bool hid_parse_report_desc(struct hid_host_data *const data,
					     const uint8_t *const rd, const size_t rd_len)
{
	struct hid_global g = {0};
	struct hid_global stack[HID_GLOBAL_STACK_DEPTH];
	uint8_t sp = 0U;
	struct hid_local l = {0};
	uint16_t app_page = 0U;
	uint16_t app_usage = 0U;
	struct {
		uint8_t id;
		uint16_t bits;
	} rbits[CONFIG_USBH_HID_MAX_REPORTS];
	uint8_t n_rbits = 0U;
	bool ids_dropped = false;
	uint16_t maxlen = 0U;
	size_t i = 0U;

	data->n_fields = 0U;
	data->n_reports = 0U;
	data->uses_report_ids = false;

	while (i < rd_len) {
		uint8_t b0 = rd[i];
		uint8_t isize;
		uint8_t btype;
		uint8_t btag;
		uint32_t uval = 0U;
		int32_t sval;

		if (b0 == HID_ITEM_LONG_PREFIX) { /* 0xFE, bDataSize, bLongItemTag, data */
			if ((i + 1U) >= rd_len) {
				break;
			}
			i += 3U + rd[i + 1U];
			continue;
		}

		isize = (HID_ITEM_BSIZE(b0) == HID_ITEM_BSIZE_4) ? 4U : HID_ITEM_BSIZE(b0);
		btype = HID_ITEM_BTYPE(b0);
		btag = HID_ITEM_BTAG(b0);

		if ((i + 1U + isize) > rd_len) {
			break;
		}

		for (uint8_t k = 0U; k < isize; k++) {
			uval |= (uint32_t)rd[i + 1U + k] << (8U * k);
		}
		if ((isize > 0U) && (isize < 4U)) {
			sval = sign_extend(uval, (isize * 8U) - 1U);
		} else {
			sval = (int32_t)uval;
		}

		i += 1U + isize;

		switch (btype) {
		case HID_ITEM_TYPE_GLOBAL:
			hid_parse_global(data, &g, stack, &sp, btag, isize, uval, sval);
			break;

		case HID_ITEM_TYPE_LOCAL:
			hid_parse_local(&l, btag, isize, uval);
			break;

		case HID_ITEM_TYPE_MAIN:
			if ((btag == HID_ITEM_TAG_COLLECTION) &&
			    ((uint8_t)uval == HID_COLLECTION_APPLICATION)) {
				app_page = (l.ext_page != 0U) ? l.ext_page : g.usage_page;
				if (l.n_usages > 0U) {
					app_usage = l.usages[0];
				} else {
					app_usage = l.has_range ? l.usage_min : 0U;
				}
			} else if (btag == HID_ITEM_TAG_INPUT) {
				uint8_t ri = HID_REPORT_NONE;

				for (uint8_t r = 0U; r < n_rbits; r++) {
					if (rbits[r].id == g.report_id) {
						ri = r;
						break;
					}
				}
				if ((ri == HID_REPORT_NONE) &&
				    (n_rbits < CONFIG_USBH_HID_MAX_REPORTS)) {
					ri = n_rbits++;
					rbits[ri].id = g.report_id;
					rbits[ri].bits = 0U;
				}

				if (ri != HID_REPORT_NONE) {
					(void)hid_store_field(data, &g, &l, uval, rbits[ri].bits,
							      hid_btn_ns(app_page, app_usage));
					rbits[ri].bits = hid_add_bits(rbits[ri].bits, g.report_size,
								      g.report_count);
				} else {
					ids_dropped = true;
				}
			}

			/* Local item state applies only to the next Main item. */
			l = (struct hid_local){0};
			break;

		default:
			break;
		}
	}

	for (uint8_t r = 0U; r < n_rbits; r++) {
		data->reports[r].id = rbits[r].id;
		data->reports[r].len = (uint8_t)MIN(DIV_ROUND_UP(rbits[r].bits, 8U),
						    CONFIG_USBH_HID_MAX_REPORT_SIZE);
		data->reports[r].have = false;
		maxlen = MAX(maxlen, (uint16_t)data->reports[r].len);
	}
	data->n_reports = n_rbits;

	/*
	 * Wire length of the largest input report: payload plus the leading
	 * report-id byte when report ids are in use. Sizing the interrupt-IN
	 * buffer to exactly this (hid_host_arm) lets a report span multiple
	 * packets (HID 1.11 section 8.4) without the controller stacking several
	 * reports into one over-sized transfer.
	 */
	if (data->uses_report_ids) {
		maxlen += 1U;
	}
	data->max_report_len = (uint8_t)MIN(maxlen, CONFIG_USBH_HID_MAX_REPORT_SIZE + 1U);
	data->app_page = app_page;

	return ids_dropped;
}

/*
 * Emit input events with a one-event lookahead: hold the most recent event and
 * flush it (sync=false) when the next one arrives, so the report's final event
 * can carry sync=true. This avoids buffering the whole batch on the stack.
 */
static void hid_emit(const struct device *dev, struct hid_input_evt *const pending,
		     bool *const have, uint8_t type, uint16_t code, int32_t value)
{
	if (*have) {
		input_report(dev, pending->type, pending->code, pending->value, false, K_NO_WAIT);
	}
	pending->type = type;
	pending->code = code;
	pending->value = value;
	*have = true;
}

/* Flush the held event as the report's sync point (sync=true); no-op if none. */
static void hid_emit_flush(const struct device *dev, const struct hid_input_evt *const pending,
			   const bool have)
{
	if (have) {
		input_report(dev, pending->type, pending->code, pending->value, true, K_NO_WAIT);
	}
}

/* Emit the changes of one variable field against the previous payload (or NULL). */
static void hid_decode_variable(struct hid_host_data *const data, const struct hid_field *const f,
				const uint8_t *const payload, const uint8_t *const prev,
				struct hid_input_evt *const pending, bool *const have)
{
	for (uint16_t k = 0U; k < f->count; k++) {
		uint16_t bit = f->bit_offset + (uint16_t)(k * f->size);
		uint16_t usage = hid_field_usage(f, k);
		uint8_t type;
		uint16_t code;
		int32_t v;

		if ((usage == 0U) ||
		    !hid_map_usage(f->usage_page, usage, f->relative, f->btn_ns, &type, &code)) {
			continue;
		}

		if (type == INPUT_EV_KEY) {
			bool now = hid_extract_bits(payload, bit, f->size) != 0U;
			bool was = (prev != NULL) && (hid_extract_bits(prev, bit, f->size) != 0U);

			if (now != was) {
				hid_emit(data->dev, pending, have, INPUT_EV_KEY, code, now ? 1 : 0);
			}
		} else if (type == INPUT_EV_REL) {
			v = hid_extract_signed(payload, bit, f->size);
			if (v != 0) {
				hid_emit(data->dev, pending, have, INPUT_EV_REL, code, v);
			}
		} else { /* INPUT_EV_ABS */
			v = hid_field_value(f, payload, bit);

			/*
			 * Null-state field (HID 1.11 section 5.10): a value
			 * outside [logical_min, logical_max] means "not engaged"
			 * (e.g. a centered hat switch) and must be ignored, not
			 * emitted as a position.
			 */
			if (f->null_state && ((v < f->logical_min) || (v > f->logical_max))) {
				continue;
			}

			if ((prev == NULL) || (v != hid_field_value(f, prev, bit))) {
				hid_emit(data->dev, pending, have, INPUT_EV_ABS, code, v);
			}
		}
	}
}

/*
 * Emit key releases and presses of one array field against the previous payload
 * (or NULL). A keyboard rollover report emits nothing and its array bits in
 * `payload` are replaced by the previous state, which the report cache keeps.
 */
static void hid_decode_array(struct hid_host_data *const data, const struct hid_field *const f,
			     uint8_t *const payload, const uint8_t *const prev,
			     struct hid_input_evt *const pending, bool *const have)
{
	uint16_t usage;
	uint8_t type;
	uint16_t code;

	if (hid_array_rollover(f, payload)) {
		LOG_DBG("keyboard rollover (ErrorRollOver) report");
		hid_copy_bits(payload, prev, f->bit_offset, (uint16_t)(f->size * f->count));
		return;
	}

	if (prev != NULL) {
		for (uint16_t k = 0U; k < f->count; k++) {
			if (!hid_array_slot_usage(f, prev, k, &usage) ||
			    hid_array_has_usage(f, payload, usage)) {
				continue;
			}
			if (hid_map_usage(f->usage_page, usage, false, f->btn_ns, &type, &code) &&
			    (type == INPUT_EV_KEY)) {
				hid_emit(data->dev, pending, have, INPUT_EV_KEY, code, 0);
			}
		}
	}

	for (uint16_t k = 0U; k < f->count; k++) {
		if (!hid_array_slot_usage(f, payload, k, &usage) ||
		    ((prev != NULL) && hid_array_has_usage(f, prev, usage))) {
			continue;
		}
		if (hid_map_usage(f->usage_page, usage, false, f->btn_ns, &type, &code) &&
		    (type == INPUT_EV_KEY)) {
			hid_emit(data->dev, pending, have, INPUT_EV_KEY, code, 1);
		}
	}
}

/* Translate one report-protocol report into input events using the parsed model. */
static void hid_generic_decode(struct hid_host_data *const data, uint8_t *const report,
			       const size_t len)
{
	struct hid_input_evt pending;
	bool have_pending = false;
	struct hid_report_cache *cache = NULL;
	const uint8_t *prev;
	uint8_t *payload;
	size_t paylen;
	uint8_t id;

	if (data->uses_report_ids) {
		if (len < 1U) {
			return;
		}
		id = report[0];
		payload = &report[1];
		paylen = len - 1U;
	} else {
		id = 0U;
		payload = report;
		paylen = len;
	}

	for (uint8_t r = 0U; r < data->n_reports; r++) {
		if (data->reports[r].id == id) {
			cache = &data->reports[r];
			break;
		}
	}

	/* Only report IDs with a cache slot have stored fields. */
	if (cache == NULL) {
		return;
	}
	prev = cache->have ? cache->buf : NULL;

	for (uint8_t fi = 0U; fi < data->n_fields; fi++) {
		const struct hid_field *f = &data->fields[fi];

		if (f->report_id != id) {
			continue;
		}
		if (((size_t)f->bit_offset + ((size_t)f->size * f->count)) > (paylen * 8U)) {
			continue; /* report shorter than this field */
		}

		if (f->variable) {
			hid_decode_variable(data, f, payload, prev, &pending, &have_pending);
		} else {
			hid_decode_array(data, f, payload, prev, &pending, &have_pending);
		}
	}

	/* The report's final event carries the sync flag. */
	hid_emit_flush(data->dev, &pending, have_pending);

	cache->len = (uint8_t)MIN(paylen, (size_t)CONFIG_USBH_HID_MAX_REPORT_SIZE);
	memcpy(cache->buf, payload, cache->len);
	if (cache->len < CONFIG_USBH_HID_MAX_REPORT_SIZE) {
		memset(cache->buf + cache->len, 0, CONFIG_USBH_HID_MAX_REPORT_SIZE - cache->len);
	}
	cache->have = true;
}

/*
 * Interrupt-IN completion. Runs on the usbh request thread, concurrently with
 * probe/removed on the usbh bus thread. Re-arm is driven by the connected flag,
 * not xfer->err: a cancelled transfer's error code is controller-specific and
 * cannot reliably distinguish teardown from a genuine fault.
 *
 * A transfer cancelled by removed() can complete after its device is freed,
 * so this callback does not dereference udev. A transfer that is no longer in
 * data->in_xfer belongs to an earlier binding and is only freed.
 */
static int hid_host_report_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	struct hid_host_data *const data = xfer->priv;
	uint8_t report[CONFIG_USBH_HID_MAX_REPORT_SIZE + 1];
	size_t len = 0U;
	bool active = false;
	int err = xfer->err;

	ARG_UNUSED(udev);

	k_mutex_lock(&data->lock, K_FOREVER);

	for (uint8_t i = 0U; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (data->in_xfer[i] == xfer) {
			data->in_xfer[i] = NULL;
			active = data->connected;
			break;
		}
	}

	if ((err == 0) && active && (xfer->buf != NULL)) {
		len = MIN(xfer->buf->len, sizeof(report));
		memcpy(report, xfer->buf->data, len);
	}

	k_mutex_unlock(&data->lock);

	/*
	 * A non-zero error on an active transfer is a genuine fault. A transfer
	 * cancelled during removal completes after `connected` is cleared
	 * (usbh_hid_removed), so it is filtered out here. The error value itself
	 * is not interpreted: its meaning on cancellation is controller-specific.
	 */
	if ((err != 0) && active) {
		LOG_WRN("interrupt-IN transfer error: %d", err);
	}

	if (xfer->buf != NULL) {
		net_buf_unref(xfer->buf);
	}
	(void)uhc_xfer_unref(xfer);

	if (len > 0U) {
		if (data->boot_fallback) {
			if (data->proto == HID_BOOT_IFACE_CODE_KEYBOARD) {
				hid_kbd_decode(data, report, len);
			} else if (data->proto == HID_BOOT_IFACE_CODE_MOUSE) {
				hid_mouse_decode(data, report, len);
			}
		} else {
			hid_generic_decode(data, report, len);
		}
	}

	/* Re-arm a replacement transfer while the device is attached. */
	if (active) {
		(void)hid_host_arm(data);
	}

	return 0;
}

/*
 * Walk the matched interface's descriptors to find the interrupt-IN endpoint,
 * the boot protocol/subclass, and the report descriptor length (from the HID
 * class descriptor 0x21). Per-interface, so multi-interface devices bind each
 * interface independently.
 */
static int hid_host_setup_iface(struct hid_host_data *const data, struct usb_device *const udev,
				const uint8_t iface, uint16_t *const rdlen)
{
	const struct usb_if_descriptor *if_desc;
	const struct usb_desc_header *desc;
	bool have_ep = false;

	*rdlen = 0U;

	if_desc = usbh_desc_get_iface(udev, iface);
	if (if_desc == NULL) {
		return -ENODEV;
	}

	data->subclass = if_desc->bInterfaceSubClass;
	data->proto = if_desc->bInterfaceProtocol;

	for (desc = usbh_desc_get_next(if_desc); desc != NULL; desc = usbh_desc_get_next(desc)) {
		/* Stop at the next interface: end of this interface's descriptors. */
		if (usbh_desc_is_valid_interface(desc)) {
			break;
		}

		if (usbh_desc_is_valid(desc, HID_DESC_REPORT_LEN_OFFSET + sizeof(uint16_t),
				       USB_DESC_HID)) {
			*rdlen = sys_get_le16((const uint8_t *)desc + HID_DESC_REPORT_LEN_OFFSET);
		}

		if (usbh_desc_is_valid_endpoint(desc) && !have_ep) {
			const struct usb_ep_descriptor *ep = (const struct usb_ep_descriptor *)desc;

			if (USB_EP_DIR_IS_IN(ep->bEndpointAddress) &&
			    ((ep->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
			     USB_EP_TYPE_INTERRUPT)) {
				data->ep_addr = ep->bEndpointAddress;
				data->ep_mps = sys_le16_to_cpu(ep->wMaxPacketSize);
				have_ep = true;
			}
		}
	}

	return have_ep ? 0 : -ENODEV;
}

/* Fetch and parse the report descriptor for a report-protocol interface. */
static int hid_setup_generic(struct hid_host_data *const data, struct usb_device *const udev,
			     const uint8_t iface, uint16_t rdlen)
{
	struct net_buf *buf;
	int ret;

	if (rdlen == 0U) {
		LOG_WRN("interface %u has no HID report descriptor", iface);
		return -ENOTSUP;
	}
	rdlen = MIN(rdlen, (uint16_t)CONFIG_USBH_HID_REPORT_DESC_MAX);

	buf = usbh_xfer_buf_alloc(udev, rdlen);
	if (buf == NULL) {
		return -ENOMEM;
	}

	ret = usbh_req_setup(udev, HID_REQTYPE_GET_DESC, USB_SREQ_GET_DESCRIPTOR,
			     (USB_DESC_HID_REPORT << 8), iface, rdlen, buf);
	if (ret == 0) {
		if (hid_parse_report_desc(data, buf->data, buf->len)) {
			LOG_WRN("interface %u has more report IDs than "
				"CONFIG_USBH_HID_MAX_REPORTS (%u), their input fields are ignored",
				iface, CONFIG_USBH_HID_MAX_REPORTS);
		}
		LOG_INF("interface %u report desc %u bytes: %u field(s), %u report(s)%s", iface,
			buf->len, data->n_fields, data->n_reports,
			data->uses_report_ids ? ", report IDs" : "");
	} else {
		LOG_ERR("GET report descriptor (interface %u) failed: %d", iface, ret);
	}

	usbh_xfer_buf_free(udev, buf);

	if (ret != 0) {
		return ret;
	}
	if (data->n_fields == 0U) {
		if (data->app_page >= HID_USAGE_PAGE_VENDOR_MIN) {
			LOG_INF("interface %u: vendor interface (usage page 0x%04x), "
				"no input mapping",
				iface, data->app_page);
		} else {
			LOG_WRN("interface %u: no mappable input fields", iface);
		}
	}

	return 0;
}

/*
 * Reason a boot-capable interface uses the boot protocol, or NULL to keep the
 * report protocol. setup_ret is the result of hid_setup_generic().
 */
static const char *hid_boot_reason(const struct hid_host_data *const data, int setup_ret)
{
	if (setup_ret != 0) {
		return "no report descriptor";
	}
	if (data->n_fields == 0U) {
		return "no mappable fields";
	}
	if (IS_ENABLED(CONFIG_USBH_HID_FORCE_BOOT_PROTOCOL)) {
		return "CONFIG_USBH_HID_FORCE_BOOT_PROTOCOL";
	}

	return NULL;
}

static int usbh_hid_probe(struct usbh_class_data *const c_data, struct usb_device *const udev,
			  const uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct hid_host_data *const data = dev->data;
	uint8_t target_iface = (iface == USBH_CLASS_IFNUM_DEVICE) ? 0U : iface;
	uint16_t rdlen = 0U;
	const char *boot_reason;
	bool boot_capable;
	int ret;

	if ((udev == NULL) || (udev->state != USB_STATE_CONFIGURED)) {
		LOG_ERR("device not configured");
		return -ENODEV;
	}

	/*
	 * Drop transfers left from an earlier binding: their completion only
	 * frees them. The control requests below block until the usbh request
	 * thread completes them, and that thread also runs hid_host_report_cb(),
	 * so the lock is not held across them.
	 */
	k_mutex_lock(&data->lock, K_FOREVER);
	data->connected = false;
	data->udev = NULL;
	memset(data->in_xfer, 0, sizeof(data->in_xfer));
	k_mutex_unlock(&data->lock);

	data->iface = target_iface;
	data->prev_mods = 0U;
	data->prev_buttons = 0U;
	data->n_fields = 0U;
	data->n_reports = 0U;
	data->uses_report_ids = false;
	data->max_report_len = 0U;
	memset(data->prev_keys, 0, sizeof(data->prev_keys));
	memset(data->reports, 0, sizeof(data->reports));

	ret = hid_host_setup_iface(data, udev, target_iface, &rdlen);
	if (ret != 0) {
		LOG_ERR("no interrupt-IN endpoint on interface %u", target_iface);
		return ret;
	}

	if (data->ep_mps > CONFIG_USBH_HID_MAX_REPORT_SIZE) {
		LOG_WRN("interface %u: endpoint mps %u exceeds "
			"CONFIG_USBH_HID_MAX_REPORT_SIZE %u; reports will be truncated",
			target_iface, data->ep_mps, CONFIG_USBH_HID_MAX_REPORT_SIZE);
	}

	/*
	 * Prefer the report protocol for every interface, including boot-capable
	 * keyboards and mice: the parsed report descriptor exposes the device's
	 * full report (N-key rollover, extra and media keys) that the fixed boot
	 * layout cannot. The boot decoders are a fallback, used only when a
	 * boot-capable device offers no usable report descriptor.
	 */
	boot_capable = (data->subclass == HID_SUBCLASS_BOOT) &&
		       ((data->proto == HID_BOOT_IFACE_CODE_KEYBOARD) ||
			(data->proto == HID_BOOT_IFACE_CODE_MOUSE));
	data->boot_fallback = false;

	if (boot_capable) {
		/* Don't assume device state (HID 1.11 section 7.2.6): select report. */
		ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_PROTOCOL,
				     HID_PROTOCOL_REPORT, target_iface, 0, NULL);
		if (ret != 0) {
			LOG_WRN("SET_PROTOCOL(report) failed: %d", ret);
		}
	}

	ret = hid_setup_generic(data, udev, target_iface, rdlen);
	if ((ret != 0) && !boot_capable) {
		/* Generic-only interface with no report descriptor. */
		return ret;
	}

	/* A generic interface with no mappable fields still binds; nothing maps. */
	boot_reason = boot_capable ? hid_boot_reason(data, ret) : NULL;
	if (boot_reason != NULL) {
		LOG_INF("interface %u: boot protocol (%s)", target_iface, boot_reason);
		ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_PROTOCOL, HID_PROTOCOL_BOOT,
				     target_iface, 0, NULL);
		if (ret != 0) {
			LOG_WRN("SET_PROTOCOL(boot) failed: %d", ret);
		}
		data->boot_fallback = true;
		/*
		 * The boot keyboard report is 8 bytes. A boot mouse report is at
		 * least 3 bytes and may append device-specific bytes (HID 1.11
		 * Appendix B.2), so it gets the same 8-byte buffer.
		 */
		data->max_report_len = HID_KBD_REPORT_SIZE;
	}

	/* Idle 0: report only on change (best-effort; some devices STALL it). */
	ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_IDLE, 0, target_iface, 0, NULL);
	if (ret != 0) {
		LOG_DBG("SET_IDLE not accepted (interface %u): %d", target_iface, ret);
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->udev = udev;
	data->connected = true;
	k_mutex_unlock(&data->lock);

	for (uint8_t i = 0U; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (hid_host_arm(data) != 0) {
			break;
		}
	}

	LOG_INF("HID %s ready (addr %u, ep 0x%02x mps %u)",
		data->boot_fallback
			? ((data->proto == HID_BOOT_IFACE_CODE_KEYBOARD) ? "boot keyboard"
									 : "boot mouse")
			: "report device",
		udev->addr, data->ep_addr, data->ep_mps);

	return 0;
}

static int usbh_hid_removed(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct hid_host_data *const data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->connected = false;

	/* Cancel in-flight transfers; their completion cb frees them. */
	for (uint8_t i = 0U; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (data->in_xfer[i] != NULL) {
			(void)usbh_xfer_dequeue(data->udev, data->in_xfer[i]);
		}
	}

	data->udev = NULL;
	k_mutex_unlock(&data->lock);

	LOG_INF("HID device removed");

	return 0;
}

static int usbh_hid_init(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct hid_host_data *const data = dev->data;

	memset(data, 0, sizeof(*data));
	data->dev = dev;
	k_mutex_init(&data->lock);
	data->connected = false;

	return 0;
}

static struct usbh_class_api usbh_hid_class_api = {
	.init = usbh_hid_init,
	.probe = usbh_hid_probe,
	.removed = usbh_hid_removed,
};

/*
 * Match HID boot keyboard (3/1/1), boot mouse (3/1/2), and generic report-mode
 * (3/0/0) interfaces. probe() picks the boot or report-parser path by subclass.
 */
static const struct usbh_class_filter usbh_hid_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = HID_SUBCLASS_BOOT,
		.proto = HID_BOOT_IFACE_CODE_KEYBOARD,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = HID_SUBCLASS_BOOT,
		.proto = HID_BOOT_IFACE_CODE_MOUSE,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = HID_SUBCLASS_NONE,
		.proto = HID_BOOT_IFACE_CODE_NONE,
	},
	{0},
};

#define USBH_HID_DEVICE_DEFINE(n, _)                                                               \
	static struct hid_host_data hid_host_data##n;                                              \
                                                                                                   \
	DEVICE_DEFINE(usbh_hid_##n, "usbh_hid_" #n, NULL, NULL, &hid_host_data##n, NULL,           \
		      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);                              \
                                                                                                   \
	USBH_DEFINE_CLASS(usbh_hid_c_data_##n, &usbh_hid_class_api,                                \
			  (void *)DEVICE_GET(usbh_hid_##n), usbh_hid_filters);

LISTIFY(CONFIG_USBH_HID_INSTANCES_COUNT, USBH_HID_DEVICE_DEFINE, (;), _)
