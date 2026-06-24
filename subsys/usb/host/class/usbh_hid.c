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
#include <zephyr/sys/atomic.h>
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

/* HID class-specific requests (HID 1.11 §7.2) on the interface recipient. */
#define HID_REQTYPE_SET		(USB_REQTYPE_DIR_TO_DEVICE << 7 |	\
				 USB_REQTYPE_TYPE_CLASS << 5 |		\
				 USB_REQTYPE_RECIPIENT_INTERFACE)

/*
 * Standard GET_DESCRIPTOR on the interface recipient. The HID report descriptor
 * (0x22) is fetched per HID 1.11 §7.1.1: bmRequestType 0x81 (dir-IN, type
 * standard, recipient interface), wIndex = interface number. (usbh_req_desc()
 * cannot be used here: it hardcodes the device recipient.)
 */
#define HID_REQTYPE_GET_DESC	(USB_REQTYPE_DIR_TO_HOST << 7 |		\
				 USB_REQTYPE_RECIPIENT_INTERFACE)

/* Consumer Controls usage page (HID Usage Tables); not in hid.h. */
#define HID_USAGE_CONSUMER	0x0C

/* Boot keyboard input report: [modifiers][reserved][6 keycodes]. */
#define HID_KBD_REPORT_SIZE	8
#define HID_KBD_NUM_KEYS	6
/* Boot mouse input report: [buttons][dx][dy](+[wheel]). */
#define HID_MOUSE_REPORT_MIN	3

/* Interrupt-IN transfers kept queued on the endpoint. */
#define HID_HOST_QUEUE_DEPTH	2

/* Generic-parser limits. */
#define HID_FIELD_USAGES_MAX	6	/* explicit usages captured per field */
#define HID_GLOBAL_STACK_DEPTH	4	/* HID Push/Pop nesting */
/*
 * Keep the report-descriptor parser and its setup helpers out of probe()'s own
 * stack frame. The usbh bus thread has a small stack (CONFIG_USBH_STACK_SIZE,
 * 1 KB by default), and the enumeration call chain already consumes a good part
 * of it before probe() runs. As separate (non-inlined) frames the parser's
 * scratch is transient and probe() stays small enough to log safely. (UVC, the
 * other in-tree host class, fits the same budget.)
 */
#define HID_NOINLINE	__attribute__((noinline))

/*
 * HID Keyboard/Keypad usage (page 0x07) -> Zephyr input key code. Covers the
 * boot-protocol range; unmapped usages are 0 and dropped. Mirrors the standard
 * HID-usage to evdev keycode mapping used by Linux hid-input.
 */
static const uint16_t hid_kbd_keymap[] = {
	[0x04] = INPUT_KEY_A,         [0x05] = INPUT_KEY_B,
	[0x06] = INPUT_KEY_C,         [0x07] = INPUT_KEY_D,
	[0x08] = INPUT_KEY_E,         [0x09] = INPUT_KEY_F,
	[0x0a] = INPUT_KEY_G,         [0x0b] = INPUT_KEY_H,
	[0x0c] = INPUT_KEY_I,         [0x0d] = INPUT_KEY_J,
	[0x0e] = INPUT_KEY_K,         [0x0f] = INPUT_KEY_L,
	[0x10] = INPUT_KEY_M,         [0x11] = INPUT_KEY_N,
	[0x12] = INPUT_KEY_O,         [0x13] = INPUT_KEY_P,
	[0x14] = INPUT_KEY_Q,         [0x15] = INPUT_KEY_R,
	[0x16] = INPUT_KEY_S,         [0x17] = INPUT_KEY_T,
	[0x18] = INPUT_KEY_U,         [0x19] = INPUT_KEY_V,
	[0x1a] = INPUT_KEY_W,         [0x1b] = INPUT_KEY_X,
	[0x1c] = INPUT_KEY_Y,         [0x1d] = INPUT_KEY_Z,
	[0x1e] = INPUT_KEY_1,         [0x1f] = INPUT_KEY_2,
	[0x20] = INPUT_KEY_3,         [0x21] = INPUT_KEY_4,
	[0x22] = INPUT_KEY_5,         [0x23] = INPUT_KEY_6,
	[0x24] = INPUT_KEY_7,         [0x25] = INPUT_KEY_8,
	[0x26] = INPUT_KEY_9,         [0x27] = INPUT_KEY_0,
	[0x28] = INPUT_KEY_ENTER,     [0x29] = INPUT_KEY_ESC,
	[0x2a] = INPUT_KEY_BACKSPACE, [0x2b] = INPUT_KEY_TAB,
	[0x2c] = INPUT_KEY_SPACE,     [0x2d] = INPUT_KEY_MINUS,
	[0x2e] = INPUT_KEY_EQUAL,     [0x2f] = INPUT_KEY_LEFTBRACE,
	[0x30] = INPUT_KEY_RIGHTBRACE,[0x31] = INPUT_KEY_BACKSLASH,
	[0x32] = INPUT_KEY_BACKSLASH, [0x33] = INPUT_KEY_SEMICOLON,
	[0x34] = INPUT_KEY_APOSTROPHE,[0x35] = INPUT_KEY_GRAVE,
	[0x36] = INPUT_KEY_COMMA,     [0x37] = INPUT_KEY_DOT,
	[0x38] = INPUT_KEY_SLASH,     [0x39] = INPUT_KEY_CAPSLOCK,
	[0x3a] = INPUT_KEY_F1,        [0x3b] = INPUT_KEY_F2,
	[0x3c] = INPUT_KEY_F3,        [0x3d] = INPUT_KEY_F4,
	[0x3e] = INPUT_KEY_F5,        [0x3f] = INPUT_KEY_F6,
	[0x40] = INPUT_KEY_F7,        [0x41] = INPUT_KEY_F8,
	[0x42] = INPUT_KEY_F9,        [0x43] = INPUT_KEY_F10,
	[0x44] = INPUT_KEY_F11,       [0x45] = INPUT_KEY_F12,
	[0x46] = INPUT_KEY_SYSRQ,     [0x47] = INPUT_KEY_SCROLLLOCK,
	[0x48] = INPUT_KEY_PAUSE,     [0x49] = INPUT_KEY_INSERT,
	[0x4a] = INPUT_KEY_HOME,      [0x4b] = INPUT_KEY_PAGEUP,
	[0x4c] = INPUT_KEY_DELETE,    [0x4d] = INPUT_KEY_END,
	[0x4e] = INPUT_KEY_PAGEDOWN,  [0x4f] = INPUT_KEY_RIGHT,
	[0x50] = INPUT_KEY_LEFT,      [0x51] = INPUT_KEY_DOWN,
	[0x52] = INPUT_KEY_UP,        [0x53] = INPUT_KEY_NUMLOCK,
	[0x54] = INPUT_KEY_KPSLASH,   [0x55] = INPUT_KEY_KPASTERISK,
	[0x56] = INPUT_KEY_KPMINUS,   [0x57] = INPUT_KEY_KPPLUS,
	[0x58] = INPUT_KEY_KPENTER,   [0x59] = INPUT_KEY_KP1,
	[0x5a] = INPUT_KEY_KP2,       [0x5b] = INPUT_KEY_KP3,
	[0x5c] = INPUT_KEY_KP4,       [0x5d] = INPUT_KEY_KP5,
	[0x5e] = INPUT_KEY_KP6,       [0x5f] = INPUT_KEY_KP7,
	[0x60] = INPUT_KEY_KP8,       [0x61] = INPUT_KEY_KP9,
	[0x62] = INPUT_KEY_KP0,       [0x63] = INPUT_KEY_KPDOT,
	[0x65] = INPUT_KEY_COMPOSE,
};

/* Boot keyboard modifier byte (report[0]) bit -> input key code. */
static const uint16_t hid_kbd_modmap[8] = {
	INPUT_KEY_LEFTCTRL,  INPUT_KEY_LEFTSHIFT,
	INPUT_KEY_LEFTALT,   INPUT_KEY_LEFTMETA,
	INPUT_KEY_RIGHTCTRL, INPUT_KEY_RIGHTSHIFT,
	INPUT_KEY_RIGHTALT,  INPUT_KEY_RIGHTMETA,
};

/* Boot mouse button byte bit -> input button code. */
static const uint16_t hid_mouse_btnmap[3] = {
	INPUT_BTN_LEFT, INPUT_BTN_RIGHT, INPUT_BTN_MIDDLE,
};

/* Button namespace, selected from the enclosing application collection. */
enum hid_btn_ns {
	HID_BTN_NS_GENERIC = 0,
	HID_BTN_NS_MOUSE,
	HID_BTN_NS_JOYSTICK,
	HID_BTN_NS_GAMEPAD,
};

/* Parsed Input field flags. */
#define HID_FIELD_VAR	BIT(0)	/* variable (1) vs array (0) */
#define HID_FIELD_REL	BIT(1)	/* relative (1) vs absolute (0) */
#define HID_FIELD_NULL	BIT(2)	/* null state: ignore out-of-range values (§5.10) */

/* One parsed, mappable Input field from the report descriptor. */
struct hid_field {
	uint16_t bit_offset;	/* bit position within the report payload */
	uint16_t usage_page;
	uint16_t usage_min;	/* usage range start (when has_range) */
	uint16_t usage_max;	/* usage range end (when has_range) */
	uint16_t usages[HID_FIELD_USAGES_MAX];	/* explicit usage list */
	int32_t  logical_min;	/* <0 means the field is signed */
	int32_t  logical_max;	/* upper bound (null-state range check) */
	uint8_t  report_id;
	uint8_t  size;		/* bits per item */
	uint8_t  count;		/* number of items */
	uint8_t  flags;		/* HID_FIELD_* */
	uint8_t  n_usages;
	uint8_t  has_range;
	uint8_t  btn_ns;	/* enum hid_btn_ns */
};

/* Previous payload of a report id, for change detection on the next report. */
struct hid_report_cache {
	uint8_t id;
	uint8_t len;		/* cached payload bytes */
	uint8_t have;
	uint8_t buf[CONFIG_USBH_HID_MAX_REPORT_SIZE];
};

struct hid_host_data {
	/* Back-pointer to the input device this instance reports through. */
	const struct device *dev;
	/* Bound USB device, or NULL when idle. */
	struct usb_device *udev;
	/* In-flight interrupt-IN transfers (owned by the completion cb). */
	struct uhc_transfer *in_xfer[HID_HOST_QUEUE_DEPTH];
	/* Serializes probe/removed against the completion cb. */
	struct k_mutex lock;
	uint8_t iface;
	uint8_t subclass;		/* bInterfaceSubClass */
	uint8_t proto;			/* HID_BOOT_IFACE_CODE_KEYBOARD/_MOUSE */
	uint8_t ep_addr;		/* interrupt-IN endpoint */
	uint16_t ep_mps;
	atomic_t connected;
	bool boot_fallback;		/* decode the fixed boot layout (fallback) */
	bool uses_report_ids;
	/* Boot keyboard diff state. */
	uint8_t prev_keys[HID_KBD_NUM_KEYS];
	uint8_t prev_mods;
	/* Boot mouse diff state. */
	uint8_t prev_buttons;
	/* Parsed report model (report-protocol path). */
	uint8_t n_fields;
	uint8_t n_reports;
	uint8_t max_report_len;		/* wire length of the largest input report */
	struct hid_field fields[CONFIG_USBH_HID_MAX_FIELDS];
	struct hid_report_cache reports[CONFIG_USBH_HID_MAX_REPORTS];
};

struct hid_input_evt {
	uint8_t type;
	uint16_t code;
	int32_t value;
};

static int hid_host_report_cb(struct usb_device *const udev, struct uhc_transfer *const xfer);

/* Extract n_bits (LSB-first, little-endian) starting at bit_off, unsigned. */
static uint32_t hid_extract_bits(const uint8_t *const data, uint16_t bit_off, uint8_t n_bits)
{
	uint32_t val = 0;

	for (uint8_t i = 0; i < n_bits && i < 32; i++) {
		uint16_t b = bit_off + i;

		if (data[b >> 3] & BIT(b & 0x07)) {
			val |= (1UL << i);
		}
	}

	return val;
}

/* Extract n_bits as a sign-extended value. */
static int32_t hid_extract_signed(const uint8_t *const data, uint16_t bit_off, uint8_t n_bits)
{
	uint32_t v = hid_extract_bits(data, bit_off, n_bits);

	if (n_bits > 0 && n_bits < 32 && (v & (1UL << (n_bits - 1)))) {
		v |= ~((1UL << n_bits) - 1);
	}

	return (int32_t)v;
}

/* Consumer (multimedia) usage -> input key code, 0 if unmapped. */
static uint16_t hid_consumer_keycode(uint16_t usage)
{
	switch (usage) {
	case 0x0b0: return INPUT_KEY_PLAY;
	case 0x0b1: return INPUT_KEY_PAUSE;
	case 0x0b3: return INPUT_KEY_FASTFORWARD;
	case 0x0b4: return INPUT_KEY_REWIND;
	case 0x0b5: return INPUT_KEY_NEXTSONG;
	case 0x0b6: return INPUT_KEY_PREVIOUSSONG;
	case 0x0b7: return INPUT_KEY_STOPCD;
	case 0x0b8: return INPUT_KEY_EJECTCD;
	case 0x0cd: return INPUT_KEY_PLAYPAUSE;
	case 0x0e2: return INPUT_KEY_MUTE;
	case 0x0e9: return INPUT_KEY_VOLUMEUP;
	case 0x0ea: return INPUT_KEY_VOLUMEDOWN;
	case 0x224: return INPUT_KEY_BACK;
	case 0x225: return INPUT_KEY_FORWARD;
	default:    return 0;
	}
}

/*
 * Map a (usage page, usage) to an input event type + code. `relative` picks
 * REL vs ABS for axes; `btn_ns` picks the button namespace. Returns false for
 * usages this driver does not translate.
 */
static bool hid_map_usage(uint16_t page, uint16_t usage, bool relative,
			  uint8_t btn_ns, uint8_t *const type, uint16_t *const code)
{
	switch (page) {
	case HID_USAGE_GEN_DESKTOP: {
		uint16_t rel_code, abs_code;

		switch (usage) {
		case HID_USAGE_GEN_DESKTOP_X:	  rel_code = INPUT_REL_X;  abs_code = INPUT_ABS_X;  break;
		case HID_USAGE_GEN_DESKTOP_Y:	  rel_code = INPUT_REL_Y;  abs_code = INPUT_ABS_Y;  break;
		case 0x32: /* Z */		  rel_code = INPUT_REL_Z;  abs_code = INPUT_ABS_Z;  break;
		case 0x33: /* Rx */		  rel_code = INPUT_REL_RX; abs_code = INPUT_ABS_RX; break;
		case 0x34: /* Ry */		  rel_code = INPUT_REL_RY; abs_code = INPUT_ABS_RY; break;
		case 0x35: /* Rz */		  rel_code = INPUT_REL_RZ; abs_code = INPUT_ABS_RZ; break;
		case 0x36: /* Slider */		  rel_code = INPUT_REL_MISC; abs_code = INPUT_ABS_THROTTLE; break;
		case 0x37: /* Dial */		  rel_code = INPUT_REL_DIAL; abs_code = INPUT_ABS_WHEEL; break;
		case HID_USAGE_GEN_DESKTOP_WHEEL: rel_code = INPUT_REL_WHEEL; abs_code = INPUT_ABS_WHEEL; break;
		default: return false;
		}

		*type = relative ? INPUT_EV_REL : INPUT_EV_ABS;
		*code = relative ? rel_code : abs_code;
		return true;
	}
	case HID_USAGE_GEN_KEYBOARD:
		*type = INPUT_EV_KEY;
		if (usage >= 0xe0 && usage <= 0xe7) {
			*code = hid_kbd_modmap[usage - 0xe0];
			return true;
		}
		if (usage < ARRAY_SIZE(hid_kbd_keymap) && hid_kbd_keymap[usage] != 0) {
			*code = hid_kbd_keymap[usage];
			return true;
		}
		return false;
	case HID_USAGE_GEN_BUTTON: {
		uint16_t n;

		if (usage == 0) {
			return false;
		}
		n = usage - 1;
		*type = INPUT_EV_KEY;
		switch (btn_ns) {
		case HID_BTN_NS_MOUSE:
			*code = (n < 8) ? (uint16_t)(INPUT_BTN_LEFT + n)
					: (uint16_t)(INPUT_BTN_0 + n);
			break;
		case HID_BTN_NS_GAMEPAD:
			*code = (n < 15) ? (uint16_t)(INPUT_BTN_SOUTH + n)
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

		if (kc == 0) {
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

/* Resolve the usage of item `i` within a variable field. */
static uint16_t hid_field_usage(const struct hid_field *const f, uint8_t i)
{
	if (f->n_usages > 0) {
		return f->usages[MIN(i, (uint8_t)(f->n_usages - 1))];
	}
	if (f->has_range) {
		uint16_t u = f->usage_min + i;

		if (f->usage_max != 0 && u > f->usage_max) {
			u = f->usage_max;
		}
		return u;
	}
	return 0;
}

/*
 * Allocate and enqueue one interrupt-IN transfer, registering it so removed()
 * can cancel it. The completion cb owns the transfer and frees it.
 */
static int hid_host_arm(struct hid_host_data *const data)
{
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	int slot;
	int ret;

	xfer = usbh_xfer_alloc(data->udev, data->ep_addr, hid_host_report_cb, data);
	if (xfer == NULL) {
		return -ENOMEM;
	}

	/*
	 * Size the buffer to exactly one (max) report, not ep_mps: this both
	 * admits a report larger than MPS arriving across several packets (§8.4)
	 * and, because there is never room for a second whole report, stops the
	 * controller stacking reports into an over-sized transfer. Fall back to
	 * ep_mps only when no report length was parsed.
	 */
	buf = usbh_xfer_buf_alloc(data->udev,
				  data->max_report_len ? data->max_report_len : data->ep_mps);
	if (buf == NULL) {
		usbh_xfer_free(data->udev, xfer);
		return -ENOMEM;
	}

	xfer->buf = buf;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (!atomic_get(&data->connected)) {
		k_mutex_unlock(&data->lock);
		usbh_xfer_buf_free(data->udev, buf);
		usbh_xfer_free(data->udev, xfer);
		return -ENODEV;
	}

	slot = -1;
	for (int i = 0; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (data->in_xfer[i] == NULL) {
			data->in_xfer[i] = xfer;
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		k_mutex_unlock(&data->lock);
		usbh_xfer_buf_free(data->udev, buf);
		usbh_xfer_free(data->udev, xfer);
		return -ENOBUFS;
	}

	ret = usbh_xfer_enqueue(data->udev, xfer);
	if (ret != 0) {
		data->in_xfer[slot] = NULL;
		k_mutex_unlock(&data->lock);
		usbh_xfer_buf_free(data->udev, buf);
		usbh_xfer_free(data->udev, xfer);
		return ret;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

/* Translate one boot keyboard report into input key events (diff vs previous). */
static void hid_kbd_decode(struct hid_host_data *const data, const uint8_t *report, size_t len)
{
	const uint8_t *keys = &report[2];
	uint8_t mods = report[0];

	if (len < HID_KBD_REPORT_SIZE) {
		return;
	}

	/* Modifier transitions. */
	for (int b = 0; b < 8; b++) {
		bool now = (mods & BIT(b)) != 0;
		bool was = (data->prev_mods & BIT(b)) != 0;

		if (now != was) {
			input_report_key(data->dev, hid_kbd_modmap[b], now, true, K_NO_WAIT);
		}
	}

	/* Releases: previously-pressed keys absent from the new report. */
	for (int i = 0; i < HID_KBD_NUM_KEYS; i++) {
		uint8_t usage = data->prev_keys[i];
		bool still = false;

		if (usage <= 0x03 || usage >= ARRAY_SIZE(hid_kbd_keymap) ||
		    hid_kbd_keymap[usage] == 0) {
			continue;
		}

		for (int j = 0; j < HID_KBD_NUM_KEYS; j++) {
			if (keys[j] == usage) {
				still = true;
				break;
			}
		}

		if (!still) {
			input_report_key(data->dev, hid_kbd_keymap[usage], 0, true, K_NO_WAIT);
		}
	}

	/* Presses: keys in the new report not present before. */
	for (int i = 0; i < HID_KBD_NUM_KEYS; i++) {
		uint8_t usage = keys[i];
		bool had = false;

		if (usage <= 0x03 || usage >= ARRAY_SIZE(hid_kbd_keymap) ||
		    hid_kbd_keymap[usage] == 0) {
			continue;
		}

		for (int j = 0; j < HID_KBD_NUM_KEYS; j++) {
			if (data->prev_keys[j] == usage) {
				had = true;
				break;
			}
		}

		if (!had) {
			input_report_key(data->dev, hid_kbd_keymap[usage], 1, true, K_NO_WAIT);
		}
	}

	memcpy(data->prev_keys, keys, HID_KBD_NUM_KEYS);
	data->prev_mods = mods;
}

/* Translate one boot mouse report into input button/relative events. */
static void hid_mouse_decode(struct hid_host_data *const data, const uint8_t *report, size_t len)
{
	struct hid_input_evt evt[ARRAY_SIZE(hid_mouse_btnmap) + 3];
	uint8_t buttons = report[0];
	int n = 0;

	if (len < HID_MOUSE_REPORT_MIN) {
		return;
	}

	for (int b = 0; b < (int)ARRAY_SIZE(hid_mouse_btnmap); b++) {
		bool now = (buttons & BIT(b)) != 0;
		bool was = (data->prev_buttons & BIT(b)) != 0;

		if (now != was) {
			evt[n].type = INPUT_EV_KEY;
			evt[n].code = hid_mouse_btnmap[b];
			evt[n].value = now;
			n++;
		}
	}
	data->prev_buttons = buttons;

	if ((int8_t)report[1] != 0) {
		evt[n].type = INPUT_EV_REL;
		evt[n].code = INPUT_REL_X;
		evt[n].value = (int8_t)report[1];
		n++;
	}
	if ((int8_t)report[2] != 0) {
		evt[n].type = INPUT_EV_REL;
		evt[n].code = INPUT_REL_Y;
		evt[n].value = (int8_t)report[2];
		n++;
	}
	if (len > HID_MOUSE_REPORT_MIN && report[3] != 0) {
		evt[n].type = INPUT_EV_REL;
		evt[n].code = INPUT_REL_WHEEL;
		evt[n].value = (int8_t)report[3];
		n++;
	}

	/* Emit the batch, marking only the last event as the report sync point. */
	for (int i = 0; i < n; i++) {
		input_report(data->dev, evt[i].type, evt[i].code, evt[i].value,
			     i == n - 1, K_NO_WAIT);
	}
}

/*
 * Parse a HID report descriptor into data->fields[] (mappable Input items only)
 * and register the report ids in data->reports[]. Best-effort: unrecognized or
 * unsupported items are skipped; the bit cursor is still advanced so following
 * fields stay correctly aligned.
 */
static HID_NOINLINE void hid_parse_report_desc(struct hid_host_data *const data,
					       const uint8_t *const rd, const size_t rd_len)
{
	struct hid_global {
		uint16_t usage_page;
		int32_t logical_min;
		int32_t logical_max;
		uint8_t report_size;
		uint8_t report_count;
		uint8_t report_id;
	} g = {0};
	struct hid_global gstack[HID_GLOBAL_STACK_DEPTH];
	int gsp = 0;

	uint16_t usages[HID_FIELD_USAGES_MAX];
	uint8_t n_usages = 0;
	uint16_t usage_min = 0, usage_max = 0;
	bool has_range = false;
	uint16_t ext_page = 0;		/* extended-usage page override (item bSize=4) */
	bool in_delim = false;		/* inside a Delimiter alias set */
	bool delim_took = false;	/* already captured this set's first usage */

	uint16_t app_page = 0, app_usage = 0;

	struct {
		uint8_t id;
		uint16_t bits;
	} rbits[CONFIG_USBH_HID_MAX_REPORTS];
	int n_rbits = 0;

	size_t i = 0;

	data->n_fields = 0;
	data->n_reports = 0;
	data->uses_report_ids = false;

	while (i < rd_len) {
		uint8_t b0 = rd[i];
		uint8_t bsize, isize, btype, btag;
		uint32_t uval = 0;
		int32_t sval;

		if (b0 == 0xfe) {	/* long item: 0xFE, bDataSize, bTag, data... */
			if (i + 1 >= rd_len) {
				break;
			}
			i += 3 + rd[i + 1];
			continue;
		}

		bsize = b0 & 0x03;
		isize = (bsize == 3) ? 4 : bsize;
		btype = (b0 >> 2) & 0x03;
		btag = (b0 >> 4) & 0x0f;

		if (i + 1 + isize > rd_len) {
			break;
		}

		for (uint8_t k = 0; k < isize; k++) {
			uval |= (uint32_t)rd[i + 1 + k] << (8 * k);
		}
		sval = (int32_t)uval;
		if (isize > 0 && isize < 4 && (uval & (1UL << (isize * 8 - 1)))) {
			sval |= ~((1UL << (isize * 8)) - 1);
		}

		i += 1 + isize;

		switch (btype) {
		case HID_ITEM_TYPE_GLOBAL:
			switch (btag) {
			case HID_ITEM_TAG_USAGE_PAGE:
				g.usage_page = (uint16_t)uval;
				break;
			case HID_ITEM_TAG_LOGICAL_MIN:
				g.logical_min = sval;
				break;
			case HID_ITEM_TAG_LOGICAL_MAX:
				g.logical_max = sval;
				break;
			case HID_ITEM_TAG_REPORT_SIZE:
				g.report_size = (uint8_t)uval;
				break;
			case HID_ITEM_TAG_REPORT_ID:
				g.report_id = (uint8_t)uval;
				data->uses_report_ids = true;
				break;
			case HID_ITEM_TAG_REPORT_COUNT:
				g.report_count = (uint8_t)uval;
				break;
			case 0x0a:	/* Push */
				if (gsp < HID_GLOBAL_STACK_DEPTH) {
					gstack[gsp++] = g;
				}
				break;
			case 0x0b:	/* Pop */
				if (gsp > 0) {
					g = gstack[--gsp];
				}
				break;
			default:
				break;
			}
			break;

		case HID_ITEM_TYPE_LOCAL:
			switch (btag) {
			case HID_ITEM_TAG_USAGE:
			case HID_ITEM_TAG_USAGE_MIN:
			case HID_ITEM_TAG_USAGE_MAX:
				/*
				 * Delimiter set (§6.2.2.8): aliases for one
				 * control. Take only the first usage, ignore the
				 * rest until the set closes.
				 */
				if (in_delim) {
					if (delim_took) {
						break;
					}
					delim_took = true;
				}
				/*
				 * Extended usage (4-byte data, §6.2.2.8): the
				 * high 16 bits override the usage page, the low
				 * 16 are the usage. One page per field: the
				 * override applies to the whole field at store
				 * time (a per-usage page is not modelled).
				 */
				if (isize == 4) {
					ext_page = (uint16_t)(uval >> 16);
				}
				if (btag == HID_ITEM_TAG_USAGE) {
					if (n_usages < HID_FIELD_USAGES_MAX) {
						usages[n_usages++] = (uint16_t)uval;
					}
				} else if (btag == HID_ITEM_TAG_USAGE_MIN) {
					usage_min = (uint16_t)uval;
					has_range = true;
				} else {
					usage_max = (uint16_t)uval;
					has_range = true;
				}
				break;
			case 0x0a:	/* Delimiter (§6.2.2.8). */
				if (uval == 1) {
					in_delim = true;
					delim_took = false;
				} else {
					in_delim = false;
				}
				break;
			default:
				break;
			}
			break;

		case HID_ITEM_TYPE_MAIN:
			switch (btag) {
			case HID_ITEM_TAG_COLLECTION:
				if ((uint8_t)uval == HID_COLLECTION_APPLICATION) {
					app_page = ext_page ? ext_page : g.usage_page;
					app_usage = (n_usages > 0) ? usages[0]
						  : (has_range ? usage_min : 0);
				}
				break;
			case HID_ITEM_TAG_INPUT: {
				int ri = -1;
				uint16_t off;
				uint16_t eff_page = ext_page ? ext_page : g.usage_page;

				for (int r = 0; r < n_rbits; r++) {
					if (rbits[r].id == g.report_id) {
						ri = r;
						break;
					}
				}
				if (ri < 0 && n_rbits < CONFIG_USBH_HID_MAX_REPORTS) {
					ri = n_rbits++;
					rbits[ri].id = g.report_id;
					rbits[ri].bits = 0;
				}
				off = (ri >= 0) ? rbits[ri].bits : 0;

				/* Store mappable, non-constant Input fields. */
				if (!(uval & 0x01) &&
				    data->n_fields < CONFIG_USBH_HID_MAX_FIELDS &&
				    (eff_page == HID_USAGE_GEN_DESKTOP ||
				     eff_page == HID_USAGE_GEN_KEYBOARD ||
				     eff_page == HID_USAGE_GEN_BUTTON ||
				     eff_page == HID_USAGE_CONSUMER)) {
					struct hid_field *f = &data->fields[data->n_fields++];

					memset(f, 0, sizeof(*f));
					f->bit_offset = off;
					f->report_id = g.report_id;
					f->size = g.report_size;
					f->count = g.report_count;
					f->usage_page = eff_page;
					f->logical_min = g.logical_min;
					f->logical_max = g.logical_max;
					f->flags = ((uval & 0x02) ? HID_FIELD_VAR : 0) |
						   ((uval & 0x04) ? HID_FIELD_REL : 0) |
						   ((uval & 0x40) ? HID_FIELD_NULL : 0);
					f->n_usages = n_usages;
					for (uint8_t k = 0; k < n_usages; k++) {
						f->usages[k] = usages[k];
					}
					f->has_range = has_range;
					f->usage_min = usage_min;
					f->usage_max = usage_max;

					if (app_page == HID_USAGE_GEN_DESKTOP) {
						switch (app_usage) {
						case HID_USAGE_GEN_DESKTOP_MOUSE:
						case HID_USAGE_GEN_DESKTOP_POINTER:
							f->btn_ns = HID_BTN_NS_MOUSE;
							break;
						case HID_USAGE_GEN_DESKTOP_JOYSTICK:
							f->btn_ns = HID_BTN_NS_JOYSTICK;
							break;
						case HID_USAGE_GEN_DESKTOP_GAMEPAD:
							f->btn_ns = HID_BTN_NS_GAMEPAD;
							break;
						default:
							f->btn_ns = HID_BTN_NS_GENERIC;
							break;
						}
					} else {
						f->btn_ns = HID_BTN_NS_GENERIC;
					}
				}

				if (ri >= 0) {
					rbits[ri].bits += (uint16_t)g.report_size * g.report_count;
				}
				break;
			}
			default:	/* OUTPUT, FEATURE, END_COLLECTION, ... */
				break;
			}

			/* Local item state applies only to the next Main item. */
			n_usages = 0;
			usage_min = 0;
			usage_max = 0;
			has_range = false;
			ext_page = 0;
			in_delim = false;
			delim_took = false;
			break;

		default:
			break;
		}
	}

	uint16_t maxlen = 0;

	for (int r = 0; r < n_rbits; r++) {
		data->reports[r].id = rbits[r].id;
		data->reports[r].len = (uint8_t)MIN(DIV_ROUND_UP(rbits[r].bits, 8),
						    (uint16_t)CONFIG_USBH_HID_MAX_REPORT_SIZE);
		data->reports[r].have = 0;
		maxlen = MAX(maxlen, (uint16_t)data->reports[r].len);
	}
	data->n_reports = (uint8_t)n_rbits;

	/*
	 * Wire length of the largest input report: payload plus the leading
	 * report-id byte when report ids are in use. Sizing the interrupt-IN
	 * buffer to exactly this (hid_host_arm) lets a report span multiple
	 * packets (§8.4) without the controller stacking several reports into
	 * one over-sized transfer.
	 */
	if (data->uses_report_ids) {
		maxlen += 1;
	}
	data->max_report_len = (uint8_t)MIN(maxlen,
				(uint16_t)(CONFIG_USBH_HID_MAX_REPORT_SIZE + 1));
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
		input_report(dev, pending->type, pending->code, pending->value,
			     false, K_NO_WAIT);
	}
	pending->type = type;
	pending->code = code;
	pending->value = value;
	*have = true;
}

/* Translate one report-protocol report into input events using the parsed model. */
static void hid_generic_decode(struct hid_host_data *const data,
			       const uint8_t *const report, const size_t len)
{
	struct hid_input_evt pending;
	bool have_pending = false;
	struct hid_report_cache *cache = NULL;
	const uint8_t *payload;
	size_t paylen;
	uint8_t id;

	if (data->uses_report_ids) {
		if (len < 1) {
			return;
		}
		id = report[0];
		payload = &report[1];
		paylen = len - 1;
	} else {
		id = 0;
		payload = report;
		paylen = len;
	}

	for (int r = 0; r < data->n_reports; r++) {
		if (data->reports[r].id == id) {
			cache = &data->reports[r];
			break;
		}
	}

	for (uint8_t fi = 0; fi < data->n_fields; fi++) {
		const struct hid_field *f = &data->fields[fi];

		if (f->report_id != id) {
			continue;
		}
		if ((size_t)f->bit_offset + (size_t)f->size * f->count > paylen * 8) {
			continue;	/* report shorter than this field */
		}

		if (f->flags & HID_FIELD_VAR) {
			for (uint8_t k = 0; k < f->count; k++) {
				uint16_t bit = f->bit_offset + (uint16_t)k * f->size;
				uint16_t usage = hid_field_usage(f, k);
				uint8_t type;
				uint16_t code;

				if (usage == 0 ||
				    !hid_map_usage(f->usage_page, usage,
						   f->flags & HID_FIELD_REL, f->btn_ns,
						   &type, &code)) {
					continue;
				}

				if (type == INPUT_EV_KEY) {
					int v = hid_extract_bits(payload, bit, f->size) ? 1 : 0;
					int pv = (cache && cache->have) ?
						 (hid_extract_bits(cache->buf, bit, f->size) ? 1 : 0) : 0;

					if (v != pv) {
						hid_emit(data->dev, &pending, &have_pending,
							 INPUT_EV_KEY, code, v);
					}
				} else if (type == INPUT_EV_REL) {
					int32_t v = hid_extract_signed(payload, bit, f->size);

					if (v != 0) {
						hid_emit(data->dev, &pending, &have_pending,
							 INPUT_EV_REL, code, v);
					}
				} else {	/* INPUT_EV_ABS */
					bool have_pv = cache && cache->have;
					int32_t v = (f->logical_min < 0) ?
						hid_extract_signed(payload, bit, f->size) :
						(int32_t)hid_extract_bits(payload, bit, f->size);
					int32_t pv = have_pv ?
						((f->logical_min < 0) ?
						 hid_extract_signed(cache->buf, bit, f->size) :
						 (int32_t)hid_extract_bits(cache->buf, bit, f->size)) : 0;

					/*
					 * Null-state field (§5.10): a value outside
					 * [logical_min, logical_max] means "not
					 * engaged" (e.g. a centered hat switch) and
					 * must be ignored, not emitted as a position.
					 */
					if ((f->flags & HID_FIELD_NULL) &&
					    (v < f->logical_min || v > f->logical_max)) {
						continue;
					}

					if (!have_pv || v != pv) {
						hid_emit(data->dev, &pending, &have_pending,
							 INPUT_EV_ABS, code, v);
					}
				}
			}
		} else {
			/* Array field: each item is a usage code; diff sets. */
			if (cache && cache->have) {
				for (uint8_t k = 0; k < f->count; k++) {
					uint32_t old = hid_extract_bits(cache->buf,
						f->bit_offset + (uint16_t)k * f->size, f->size);
					bool still = false;
					uint8_t type;
					uint16_t code;

					if (old == 0) {
						continue;
					}
					for (uint8_t j = 0; j < f->count; j++) {
						if (hid_extract_bits(payload,
							f->bit_offset + (uint16_t)j * f->size,
							f->size) == old) {
							still = true;
							break;
						}
					}
					if (!still &&
					    hid_map_usage(f->usage_page, (uint16_t)old, false,
							  f->btn_ns, &type, &code) &&
					    type == INPUT_EV_KEY) {
						hid_emit(data->dev, &pending, &have_pending,
							 INPUT_EV_KEY, code, 0);
					}
				}
			}
			for (uint8_t k = 0; k < f->count; k++) {
				uint32_t nv = hid_extract_bits(payload,
					f->bit_offset + (uint16_t)k * f->size, f->size);
				bool had = false;
				uint8_t type;
				uint16_t code;

				if (nv == 0) {
					continue;
				}
				if (cache && cache->have) {
					for (uint8_t j = 0; j < f->count; j++) {
						if (hid_extract_bits(cache->buf,
							f->bit_offset + (uint16_t)j * f->size,
							f->size) == nv) {
							had = true;
							break;
						}
					}
				}
				if (!had &&
				    hid_map_usage(f->usage_page, (uint16_t)nv, false,
						  f->btn_ns, &type, &code) &&
				    type == INPUT_EV_KEY) {
					hid_emit(data->dev, &pending, &have_pending,
						 INPUT_EV_KEY, code, 1);
				}
			}
		}
	}

	/* The report's final event carries the sync flag. */
	if (have_pending) {
		input_report(data->dev, pending.type, pending.code, pending.value,
			     true, K_NO_WAIT);
	}

	if (cache != NULL) {
		cache->len = (uint8_t)MIN(paylen, (size_t)CONFIG_USBH_HID_MAX_REPORT_SIZE);
		memcpy(cache->buf, payload, cache->len);
		if (cache->len < CONFIG_USBH_HID_MAX_REPORT_SIZE) {
			memset(cache->buf + cache->len, 0,
			       CONFIG_USBH_HID_MAX_REPORT_SIZE - cache->len);
		}
		cache->have = 1;
	}
}

/*
 * Interrupt-IN completion. Runs on the usbh request thread, concurrently with
 * probe/removed on the usbh bus thread. The connected flag (not xfer->err,
 * which is -EIO for HAL-cancelled transfers) decides whether to re-arm.
 */
static int hid_host_report_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	struct hid_host_data *const data = xfer->priv;
	uint8_t report[CONFIG_USBH_HID_MAX_REPORT_SIZE + 1];
	size_t len = 0;
	bool connected;
	int err = xfer->err;

	k_mutex_lock(&data->lock, K_FOREVER);

	for (int i = 0; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (data->in_xfer[i] == xfer) {
			data->in_xfer[i] = NULL;
			break;
		}
	}

	connected = atomic_get(&data->connected);

	if (err == 0 && connected && xfer->buf != NULL) {
		len = MIN(xfer->buf->len, sizeof(report));
		memcpy(report, xfer->buf->data, len);
	}

	k_mutex_unlock(&data->lock);

	if (xfer->buf != NULL) {
		usbh_xfer_buf_free(udev, xfer->buf);
	}
	usbh_xfer_free(udev, xfer);

	if (len > 0) {
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
	if (connected) {
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
static HID_NOINLINE int hid_host_setup_iface(struct hid_host_data *const data,
					     struct usb_device *const udev,
					     const uint8_t iface, uint16_t *const rdlen)
{
	const struct usb_if_descriptor *if_desc;
	const struct usb_desc_header *desc;
	bool have_ep = false;

	*rdlen = 0;

	if_desc = usbh_desc_get_iface(udev, iface);
	if (if_desc == NULL) {
		return -ENODEV;
	}

	data->subclass = if_desc->bInterfaceSubClass;
	data->proto = if_desc->bInterfaceProtocol;

	for (desc = usbh_desc_get_next(if_desc); desc != NULL; desc = usbh_desc_get_next(desc)) {
		const uint8_t *raw = (const uint8_t *)desc;

		/* Stop at the next interface: end of this interface's descriptors. */
		if (usbh_desc_is_valid_interface(desc)) {
			break;
		}

		/* HID class descriptor: report descriptor length at offset 7..8. */
		if (raw[1] == USB_DESC_HID && raw[0] >= 9) {
			*rdlen = sys_get_le16(&raw[7]);
		}

		if (usbh_desc_is_valid_endpoint(desc) && !have_ep) {
			const struct usb_ep_descriptor *ep =
				(const struct usb_ep_descriptor *)desc;

			if (USB_EP_DIR_IS_IN(ep->bEndpointAddress) &&
			    (ep->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_INTERRUPT) {
				data->ep_addr = ep->bEndpointAddress;
				data->ep_mps = sys_le16_to_cpu(ep->wMaxPacketSize);
				have_ep = true;
			}
		}
	}

	return have_ep ? 0 : -ENODEV;
}

/* Fetch and parse the report descriptor for a report-protocol interface. */
static HID_NOINLINE int hid_setup_generic(struct hid_host_data *const data,
					  struct usb_device *const udev,
					  const uint8_t iface, uint16_t rdlen)
{
	struct net_buf *buf;
	int ret;

	if (rdlen == 0) {
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
		hid_parse_report_desc(data, buf->data, buf->len);
		LOG_INF("interface %u report desc %u bytes: %u field(s), %u report(s)%s",
			iface, buf->len, data->n_fields, data->n_reports,
			data->uses_report_ids ? ", report IDs" : "");
	} else {
		LOG_ERR("GET report descriptor (interface %u) failed: %d", iface, ret);
	}

	usbh_xfer_buf_free(udev, buf);

	if (ret != 0) {
		return ret;
	}
	if (data->n_fields == 0) {
		LOG_WRN("interface %u: no mappable input fields", iface);
	}

	return 0;
}

static int usbh_hid_probe(struct usbh_class_data *const c_data, struct usb_device *const udev,
			  const uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct hid_host_data *const data = dev->data;
	uint8_t target_iface = (iface == USBH_CLASS_IFNUM_DEVICE) ? 0 : iface;
	uint16_t rdlen = 0;
	bool boot_capable;
	int ret;

	if (udev == NULL || udev->state != USB_STATE_CONFIGURED) {
		LOG_ERR("device not configured");
		return -ENODEV;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->udev = udev;
	data->iface = target_iface;
	data->prev_mods = 0;
	data->prev_buttons = 0;
	data->n_fields = 0;
	data->n_reports = 0;
	data->uses_report_ids = false;
	data->max_report_len = 0;
	memset(data->prev_keys, 0, sizeof(data->prev_keys));
	memset(data->in_xfer, 0, sizeof(data->in_xfer));
	memset(data->reports, 0, sizeof(data->reports));

	ret = hid_host_setup_iface(data, udev, target_iface, &rdlen);
	if (ret != 0) {
		LOG_ERR("no interrupt-IN endpoint on interface %u", target_iface);
		data->udev = NULL;
		k_mutex_unlock(&data->lock);
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
	boot_capable = (data->subclass == 1 &&
			(data->proto == HID_BOOT_IFACE_CODE_KEYBOARD ||
			 data->proto == HID_BOOT_IFACE_CODE_MOUSE));
	data->boot_fallback = false;

	if (boot_capable) {
		/* Don't assume device state (HID 1.11 §7.2.6): select report. */
		ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_PROTOCOL,
				     HID_PROTOCOL_REPORT, target_iface, 0, NULL);
		if (ret != 0) {
			LOG_WRN("SET_PROTOCOL(report) failed: %d", ret);
		}
	}

	ret = hid_setup_generic(data, udev, target_iface, rdlen);
	if (ret != 0 || data->n_fields == 0) {
		if (boot_capable) {
			/* No usable report descriptor: fall back to boot layout. */
			LOG_INF("interface %u: boot-protocol fallback (%s)", target_iface,
				ret != 0 ? "no report descriptor" : "no mappable fields");
			ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_PROTOCOL,
					     HID_PROTOCOL_BOOT, target_iface, 0, NULL);
			if (ret != 0) {
				LOG_WRN("SET_PROTOCOL(boot) failed: %d", ret);
			}
			data->boot_fallback = true;
			/* Boot reports are fixed: kbd 8 bytes, mouse 3. */
			data->max_report_len = HID_KBD_REPORT_SIZE;
		} else if (ret != 0) {
			/* Generic-only interface with no report descriptor. */
			data->udev = NULL;
			k_mutex_unlock(&data->lock);
			return ret;
		}
		/* else: generic interface parsed 0 fields -- bind, nothing maps. */
	}

	/* Idle 0: report only on change (best-effort; some devices STALL it). */
	ret = usbh_req_setup(udev, HID_REQTYPE_SET, USB_HID_SET_IDLE,
			     0, target_iface, 0, NULL);
	if (ret != 0) {
		LOG_WRN("SET_IDLE failed: %d", ret);
	}

	atomic_set(&data->connected, 1);
	k_mutex_unlock(&data->lock);

	for (int i = 0; i < HID_HOST_QUEUE_DEPTH; i++) {
		if (hid_host_arm(data) != 0) {
			break;
		}
	}

	LOG_INF("HID %s ready (addr %u, ep 0x%02x mps %u)",
		data->boot_fallback ? (data->proto == HID_BOOT_IFACE_CODE_KEYBOARD ?
			      "boot keyboard" : "boot mouse") : "report device",
		udev->addr, data->ep_addr, data->ep_mps);

	return 0;
}

static int usbh_hid_removed(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct hid_host_data *const data = dev->data;

	atomic_set(&data->connected, 0);

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Cancel in-flight transfers; their completion cb frees them. */
	for (int i = 0; i < HID_HOST_QUEUE_DEPTH; i++) {
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
	atomic_clear(&data->connected);

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
static struct usbh_class_filter usbh_hid_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = 1,
		.proto = HID_BOOT_IFACE_CODE_KEYBOARD,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = 1,
		.proto = HID_BOOT_IFACE_CODE_MOUSE,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = 0,
		.proto = 0,
	},
	{0},
};

#define USBH_HID_DEVICE_DEFINE(n, _)						\
	static struct hid_host_data hid_host_data##n;				\
										\
	DEVICE_DEFINE(usbh_hid_##n, "usbh_hid_" #n, NULL, NULL,			\
		      &hid_host_data##n, NULL, POST_KERNEL,			\
		      CONFIG_INPUT_INIT_PRIORITY, NULL);			\
										\
	USBH_DEFINE_CLASS(usbh_hid_c_data_##n, &usbh_hid_class_api,		\
			  (void *)DEVICE_GET(usbh_hid_##n), usbh_hid_filters);

LISTIFY(CONFIG_USBH_HID_INSTANCES_COUNT, USBH_HID_DEVICE_DEFINE, (;), _)
