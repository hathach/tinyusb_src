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

#include "tusb_option.h"

#if CFG_TUD_ENABLED

#include "dcd.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

//--------------------------------------------------------------------+
// Callback weak stubs (called if application does not provide)
//--------------------------------------------------------------------+
TU_ATTR_WEAK void dcd_edpt0_status_complete(uint8_t rhport, const tusb_control_request_t* request) {
  (void) rhport;
  (void) request;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+


typedef struct {
  tusb_control_request_t request;
  uint8_t* buffer;
  uint16_t data_len;
  uint16_t total_xferred;
  usbd_control_xfer_cb_t complete_cb;
} usbd_control_xfer_t;

static usbd_control_xfer_t _ctrl_xfer;

CFG_TUD_MEM_SECTION static struct {
  TUD_EPBUF_DEF(buf, CFG_TUD_ENDPOINT0_BUFSIZE);
} _ctrl_epbuf;

uint8_t* usbd_get_ctrl_buf(void) {
  return _ctrl_epbuf.buf;
}

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Endpoint used for the Status stage of a control transfer.
// Per USB 2.0 §9.3.1, when wLength == 0 the Direction bit is ignored and the Status stage
// is always IN. Otherwise the Status stage is opposite to the Data stage direction.
TU_ATTR_ALWAYS_INLINE static inline uint8_t status_stage_ep(const tusb_control_request_t* request) {
  return (request->wLength != 0 && request->bmRequestType_bit.direction) ? TU_EP0_OUT : TU_EP0_IN;
}

// Queue ZLP status transaction
TU_ATTR_ALWAYS_INLINE static inline  bool status_stage_xact(uint8_t rhport, uint8_t ep_status) {
  return usbd_edpt_xfer(rhport, ep_status, NULL, 0, false);
}

// Status phase
bool tud_control_status(uint8_t rhport, const tusb_control_request_t* request) {
  _ctrl_xfer.request = (*request);
  _ctrl_xfer.buffer = NULL;
  _ctrl_xfer.total_xferred = 0;
  _ctrl_xfer.data_len = 0;

  return status_stage_xact(rhport, status_stage_ep(request));
}

// Queue a transaction in Data Stage
// Each transaction has up to Endpoint0's max packet size.
// This function can also transfer an zero-length packet
static bool data_stage_xact(uint8_t rhport) {
  const uint16_t xact_len = tu_min16(_ctrl_xfer.data_len - _ctrl_xfer.total_xferred, CFG_TUD_ENDPOINT0_BUFSIZE);
  uint8_t ep_addr = TU_EP0_OUT;

  if (_ctrl_xfer.request.bmRequestType_bit.direction == TUSB_DIR_IN) {
    ep_addr = TU_EP0_IN;
    if (0u != xact_len && _ctrl_xfer.buffer != _ctrl_epbuf.buf) {
      TU_VERIFY(0 == tu_memcpy_s(_ctrl_epbuf.buf, CFG_TUD_ENDPOINT0_BUFSIZE, _ctrl_xfer.buffer, xact_len));
    }
  }

  return usbd_edpt_xfer(rhport, ep_addr, xact_len ? _ctrl_epbuf.buf : NULL, xact_len, false);
}

// Transmit data to/from the control endpoint.
// If the request's wLength is zero, a status packet is sent instead.
bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t* request, void* buffer, uint16_t len) {
  _ctrl_xfer.request = (*request);
  _ctrl_xfer.buffer = (uint8_t*) buffer;
  _ctrl_xfer.total_xferred = 0U;
  _ctrl_xfer.data_len = tu_min16(len, request->wLength);

  if (request->wLength > 0U) {
    if (_ctrl_xfer.data_len > 0U) {
      TU_ASSERT(buffer);
    }
    TU_ASSERT(data_stage_xact(rhport));
  } else {
    TU_ASSERT(status_stage_xact(rhport, TU_EP0_IN));
  }

  return true;
}

//--------------------------------------------------------------------+
// USBD API
//--------------------------------------------------------------------+
void usbd_control_reset(void);
void usbd_control_set_request(const tusb_control_request_t* request);
void usbd_control_set_complete_callback(usbd_control_xfer_cb_t fp);
bool usbd_control_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

void usbd_control_reset(void) {
  tu_varclr(&_ctrl_xfer);
}

// Set complete callback
void usbd_control_set_complete_callback(usbd_control_xfer_cb_t fp) {
  _ctrl_xfer.complete_cb = fp;
}

// for dcd_set_address where DCD is responsible for status response
void usbd_control_set_request(const tusb_control_request_t* request) {
  _ctrl_xfer.request = (*request);
  _ctrl_xfer.buffer = NULL;
  _ctrl_xfer.total_xferred = 0;
  _ctrl_xfer.data_len = 0;
}

// callback when a transaction complete on
// - DATA stage of control endpoint or
// - Status stage
bool usbd_control_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  // Status Stage complete: endpoint matches the Status stage endpoint
  uint8_t const ep_status = status_stage_ep(&_ctrl_xfer.request);
  if (ep_addr == ep_status) {
    TU_ASSERT(0 == xferred_bytes);

    // invoke optional dcd hook if available
    dcd_edpt0_status_complete(rhport, &_ctrl_xfer.request);

    if (NULL != _ctrl_xfer.complete_cb) {
      // TODO refactor with usbd_driver_print_control_complete_name
      _ctrl_xfer.complete_cb(rhport, CONTROL_STAGE_ACK, &_ctrl_xfer.request);
    }

    return true;
  }

  // Data stage complete
  if (_ctrl_xfer.request.bmRequestType_bit.direction == TUSB_DIR_OUT) {
    TU_VERIFY(_ctrl_xfer.buffer);
    if (_ctrl_xfer.buffer != _ctrl_epbuf.buf) {
      memcpy(_ctrl_xfer.buffer, _ctrl_epbuf.buf, xferred_bytes);
    }
    TU_LOG_MEM(CFG_TUD_LOG_LEVEL, _ctrl_xfer.buffer, xferred_bytes, 2);
  }

  _ctrl_xfer.total_xferred += (uint16_t) xferred_bytes;
  _ctrl_xfer.buffer += xferred_bytes;

  // Data Stage is complete when all request's length are transferred or
  // a short packet is sent including zero-length packet.
  if ((_ctrl_xfer.request.wLength == _ctrl_xfer.total_xferred) ||
      (xferred_bytes < CFG_TUD_ENDPOINT0_BUFSIZE)) {
    // DATA stage is complete
    bool is_ok = true;

    // invoke complete callback if set
    // callback can still stall control in status phase e.g out data does not make sense
    if (NULL != _ctrl_xfer.complete_cb) {
      #if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
      usbd_driver_print_control_complete_name(_ctrl_xfer.complete_cb);
      #endif

      is_ok = _ctrl_xfer.complete_cb(rhport, CONTROL_STAGE_DATA, &_ctrl_xfer.request);
    }

    if (is_ok) {
      TU_ASSERT(status_stage_xact(rhport, ep_status));
    } else {
      // Stall both IN and OUT control endpoint
      dcd_edpt_stall(rhport, TU_EP0_OUT);
      dcd_edpt_stall(rhport, TU_EP0_IN);
    }
  } else {
    // More data to transfer
    TU_ASSERT(data_stage_xact(rhport));
  }

  return true;
}

#endif
