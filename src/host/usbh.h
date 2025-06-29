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
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_USBH_H_
#define _TUSB_USBH_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "common/tusb_common.h"

#if CFG_TUH_MAX3421
#include "portable/analog/max3421/hcd_max3421.h"
#endif

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Endpoint Bulk size depending on host mx speed
#define TUH_EPSIZE_BULK_MPS   (TUH_OPT_HIGH_SPEED ? TUSB_EPSIZE_BULK_HS : TUSB_EPSIZE_BULK_FS)

// forward declaration
struct tuh_xfer_s;
typedef struct tuh_xfer_s tuh_xfer_t;
typedef void (*tuh_xfer_cb_t)(tuh_xfer_t* xfer);

// Note1: layout and order of this will be changed in near future
// it is advised to initialize it using member name
// Note2: not all field is available/meaningful in callback,
// some info is not saved by usbh to save SRAM
struct tuh_xfer_s {
  uint8_t daddr;
  uint8_t ep_addr;
  uint8_t TU_RESERVED;      // reserved
  xfer_result_t result;

  uint32_t actual_len;      // excluding setup packet

  union {
    tusb_control_request_t const* setup; // setup packet pointer if control transfer
    uint32_t buflen;                     // expected length if not control transfer (not available in callback)
  };

  uint8_t* buffer;           // not available in callback if not control transfer
  tuh_xfer_cb_t complete_cb;
  uintptr_t user_data;

  // uint32_t timeout_ms;    // place holder, not supported yet
};

// Subject to change
typedef struct {
  uint8_t daddr;
  tusb_desc_interface_t desc;
} tuh_itf_info_t;

typedef struct {
  uint8_t rhport;
  uint8_t hub_addr;
  uint8_t hub_port;
  uint8_t speed;
} tuh_bus_info_t;

// backward compatibility for hcd_devtree_info_t, maybe removed in the future
#define hcd_devtree_info_t tuh_bus_info_t
#define hcd_devtree_get_info(_daddr, _bus_info) tuh_bus_info_get(_daddr, _bus_info)

// ConfigID for tuh_configure()
enum {
  TUH_CFGID_INVALID = 0,
  TUH_CFGID_RPI_PIO_USB_CONFIGURATION = 100, // cfg_param: pio_usb_configuration_t
  TUH_CFGID_MAX3421 = 200,
};

typedef struct {
  uint8_t max_nak; // max NAK per endpoint per frame to save CPU/SPI bus usage
  uint8_t cpuctl; // R16: CPU Control Register
  uint8_t pinctl; // R17: Pin Control Register. FDUPSPI bit is ignored
} tuh_configure_max3421_t;

typedef union {
  // For TUH_CFGID_RPI_PIO_USB_CONFIGURATION use pio_usb_configuration_t

  tuh_configure_max3421_t max3421;
} tuh_configure_param_t;

//--------------------------------------------------------------------+
// APPLICATION CALLBACK
//--------------------------------------------------------------------+

// Invoked when enumeration get device descriptor
// Device is not ready to communicate yet, application can copy the descriptor if needed
void tuh_enum_descriptor_device_cb(uint8_t daddr, const tusb_desc_device_t *desc_device);

// Invoked when enumeration get configuration descriptor
// For multi-configuration device return false to skip, true to proceed with this configuration (may not be implemented yet)
// Device is not ready to communicate yet, application can copy the descriptor if needed
bool tuh_enum_descriptor_configuration_cb(uint8_t daddr, uint8_t cfg_index, const tusb_desc_configuration_t *desc_config);

// Invoked when a device is mounted (configured)
TU_ATTR_WEAK void tuh_mount_cb (uint8_t daddr);

// Invoked when a device failed to mount during enumeration process
// TU_ATTR_WEAK void tuh_mount_failed_cb (uint8_t daddr);

// Invoked when a device is unmounted (detached)
TU_ATTR_WEAK void tuh_umount_cb(uint8_t daddr);

// Invoked when there is a new usb event, which need to be processed by tuh_task()/tuh_task_ext()
void tuh_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr);

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+

// Configure host stack behavior with dynamic or port-specific parameters.
// Should be called before tuh_init()
// - cfg_id   : configure ID (TBD)
// - cfg_param: configure data, structure depends on the ID
bool tuh_configure(uint8_t rhport, uint32_t cfg_id, const void* cfg_param);

// New API to replace tuh_init() to init host stack on specific roothub port
bool tuh_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init);

// Init host stack
#if TUSB_VERSION_NUMBER > 2000  // 0.20.0
TU_ATTR_DEPRECATED("Please use tusb_init(rhport, rh_init) instead")
#endif
TU_ATTR_ALWAYS_INLINE static inline bool tuh_init(uint8_t rhport) {
  const tusb_rhport_init_t rh_init = {
    .role = TUSB_ROLE_HOST,
    .speed = TUH_OPT_HIGH_SPEED ? TUSB_SPEED_HIGH : TUSB_SPEED_FULL,
  };
  return tuh_rhport_init(rhport, &rh_init);
}

// Deinit host stack on rhport
bool tuh_deinit(uint8_t rhport);

// Check if host stack is already initialized with any roothub ports
// To check if an rhport is initialized, use tuh_rhport_is_active()
bool tuh_inited(void);

// Task function should be called in main/rtos loop, extended version of tuh_task()
// - timeout_ms: millisecond to wait, zero = no wait, 0xFFFFFFFF = wait forever
// - in_isr: if function is called in ISR
void tuh_task_ext(uint32_t timeout_ms, bool in_isr);

// Task function should be called in main/rtos loop
TU_ATTR_ALWAYS_INLINE static inline void tuh_task(void) {
  tuh_task_ext(UINT32_MAX, false);
}

// Check if there is pending events need processing by tuh_task()
bool tuh_task_event_ready(void);

#ifndef _TUSB_HCD_H_
extern void hcd_int_handler(uint8_t rhport, bool in_isr);
#endif

// Interrupt handler alias to HCD with in_isr as optional parameter
#define _tuh_int_handler_arg0()                   TU_VERIFY_STATIC(false, "tuh_int_handler() must have 1 or 2 arguments")
#define _tuh_int_handler_arg1(_rhport)            hcd_int_handler(_rhport, true)
#define _tuh_int_handler_arg2(_rhport, _in_isr)   hcd_int_handler(_rhport, _in_isr)

// 1st argument is rhport (mandatory), 2nd argument in_isr (optional)
#define tuh_int_handler(...)   TU_FUNC_OPTIONAL_ARG(_tuh_int_handler, __VA_ARGS__)

// Check if roothub port is initialized and active as a host
bool tuh_rhport_is_active(uint8_t rhport);

// Assert/de-assert Bus Reset signal to roothub port. USB specs: it should last 10-50ms
bool tuh_rhport_reset_bus(uint8_t rhport, bool active);

//--------------------------------------------------------------------+
// Device API
//--------------------------------------------------------------------+

// Get VID/PID of device
bool tuh_vid_pid_get(uint8_t daddr, uint16_t* vid, uint16_t* pid);

// Get local (cached) device descriptor once device is enumerated
bool tuh_descriptor_get_device_local(uint8_t daddr, tusb_desc_device_t* desc_device);

// Get speed of device
tusb_speed_t tuh_speed_get(uint8_t daddr);

// Check if device is connected and configured
bool tuh_mounted(uint8_t daddr);

// Check if device is connected which mean device has at least 1 successful transfer
// Note: It may not be addressed/configured/mounted yet
bool tuh_connected(uint8_t daddr);

// Check if device is suspended
TU_ATTR_ALWAYS_INLINE static inline bool tuh_suspended(uint8_t daddr) {
  // TODO implement suspend & resume on host
  (void) daddr;
  return false;
}

// Check if device is ready to communicate with
TU_ATTR_ALWAYS_INLINE static inline bool tuh_ready(uint8_t daddr) {
  return tuh_mounted(daddr) && !tuh_suspended(daddr);
}

// Get bus information of device
bool tuh_bus_info_get(uint8_t daddr, tuh_bus_info_t* bus_info);

//--------------------------------------------------------------------+
// Transfer API
// Each Function will make a USB transfer request to device. If
// - complete_cb != NULL, the function will return immediately and invoke the callback when request is complete.
// - complete_cb == NULL, the function will block until request is complete.
// In this case, user_data should be tusb_xfer_result_t* to hold the transfer result.
//--------------------------------------------------------------------+

// Helper to make Sync API from async one
#define TU_API_SYNC(_async_api, ...) \
   xfer_result_t result = XFER_RESULT_INVALID;\
   TU_VERIFY(_async_api(__VA_ARGS__, NULL, (uintptr_t) &result), XFER_RESULT_TIMEOUT); \
   return result

// Submit a control transfer
//  - async: complete callback invoked when finished.
//  - sync : blocking if complete callback is NULL.
bool tuh_control_xfer(tuh_xfer_t* xfer);

// Submit a bulk/interrupt transfer
//  - async: complete callback invoked when finished.
//  - sync : blocking if complete callback is NULL.
bool tuh_edpt_xfer(tuh_xfer_t* xfer);

// Open a non-control endpoint
bool tuh_edpt_open(uint8_t daddr, tusb_desc_endpoint_t const * desc_ep);

// Close a non-control endpoint, it will abort any pending transfer
bool tuh_edpt_close(uint8_t daddr, uint8_t ep_addr);

// Abort a queued transfer. Note: it can only abort transfer that has not been started
// Return true if a queued transfer is aborted, false if there is no transfer to abort
bool tuh_edpt_abort_xfer(uint8_t daddr, uint8_t ep_addr);

// Set Address (control transfer)
bool tuh_address_set(uint8_t daddr, uint8_t new_addr,
                     tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Set Configuration (control transfer)
// config_num = 0 will un-configure device. Note: config_num = config_descriptor_index + 1
// true on success, false if there is on-going control transfer or incorrect parameters
// if complete_cb == NULL i.e blocking, user_data should be pointed to xfer_reuslt_t*
bool tuh_configuration_set(uint8_t daddr, uint8_t config_num,
                           tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Set Interface (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
// if complete_cb == NULL i.e blocking, user_data should be pointed to xfer_reuslt_t*
bool tuh_interface_set(uint8_t daddr, uint8_t itf_num, uint8_t itf_alt,
                       tuh_xfer_cb_t complete_cb, uintptr_t user_data);

//--------------------------------------------------------------------+
// Descriptors Asynchronous (non-blocking)
//--------------------------------------------------------------------+

// Get an descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get(uint8_t daddr, uint8_t type, uint8_t index, void* buffer, uint16_t len,
                        tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get device descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_device(uint8_t daddr, void* buffer, uint16_t len,
                               tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get configuration descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_configuration(uint8_t daddr, uint8_t index, void* buffer, uint16_t len,
                                      tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get HID report descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_hid_report(uint8_t daddr, uint8_t itf_num, uint8_t desc_type, uint8_t index, void* buffer, uint16_t len,
                                   tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get string descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
// Blocking if complete callback is NULL, in this case 'user_data' must contain xfer_result_t variable
bool tuh_descriptor_get_string(uint8_t daddr, uint8_t index, uint16_t language_id, void* buffer, uint16_t len,
                               tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get language id string descriptor (control transfer)
TU_ATTR_ALWAYS_INLINE static inline
bool tuh_descriptor_get_string_langid(uint8_t daddr, void* buffer, uint16_t len,
                               tuh_xfer_cb_t complete_cb, uintptr_t user_data) {
  return tuh_descriptor_get_string(daddr, 0, 0, buffer, len, complete_cb, user_data);
}

// Get manufacturer string descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_manufacturer_string(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len,
                                            tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get product string descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_product_string(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len,
                                       tuh_xfer_cb_t complete_cb, uintptr_t user_data);

// Get serial string descriptor (control transfer)
// true on success, false if there is on-going control transfer or incorrect parameters
bool tuh_descriptor_get_serial_string(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len,
                                      tuh_xfer_cb_t complete_cb, uintptr_t user_data);

//--------------------------------------------------------------------+
// Descriptors Synchronous (blocking)
// Sync API which is blocking until transfer is complete.
// return transfer result
//--------------------------------------------------------------------+

// Sync version of tuh_descriptor_get()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_sync(uint8_t daddr, uint8_t type, uint8_t index, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get, daddr, type, index, buffer, len);
}

// Sync version of tuh_descriptor_get_device()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_device_sync(uint8_t daddr, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_device, daddr, buffer, len);
}

// Sync version of tuh_descriptor_get_configuration()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_configuration_sync(uint8_t daddr, uint8_t index, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_configuration, daddr, index, buffer, len);
}

// Sync version of tuh_descriptor_get_hid_report()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_hid_report_sync(uint8_t daddr, uint8_t itf_num, uint8_t desc_type, uint8_t index, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_hid_report, daddr, itf_num, desc_type, index, buffer, len);
}

// Sync version of tuh_descriptor_get_string()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_string_sync(uint8_t daddr, uint8_t index, uint16_t language_id, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_string, daddr, index, language_id, buffer, len);
}

// Sync version of tuh_descriptor_get_string_langid()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_string_langid_sync(uint8_t daddr, void* buffer, uint16_t len) {
  return tuh_descriptor_get_string_sync(daddr, 0, 0, buffer, len);
}

// Sync version of tuh_descriptor_get_manufacturer_string()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_manufacturer_string_sync(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_manufacturer_string, daddr, language_id, buffer, len);
}

// Sync version of tuh_descriptor_get_product_string()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_product_string_sync(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_product_string, daddr, language_id, buffer, len);
}

// Sync version of tuh_descriptor_get_serial_string()
TU_ATTR_ALWAYS_INLINE static inline tusb_xfer_result_t tuh_descriptor_get_serial_string_sync(uint8_t daddr, uint16_t language_id, void* buffer, uint16_t len) {
  TU_API_SYNC(tuh_descriptor_get_serial_string, daddr, language_id, buffer, len);
}

#ifdef __cplusplus
 }
#endif

#endif
