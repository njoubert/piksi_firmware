/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "peripherals/usart.h"
#include "sbp.h"
#include "settings.h"

/** \addtogroup io
 * \{ */

settings_t settings /*__attribute__ ((section(".settings_area"))) */=
/* Default settings: */
{
  .settings_valid = VALID,

  .ftdi_usart = {
    .mode         = SBP,
    .baud_rate    = USART_DEFAULT_BAUD_FTDI,
    .message_mask = 0xFFFF,
  },
  .uarta_usart = {
    .mode         = SBP,
    .baud_rate    = 57600,
    .message_mask = 0x40,
  },
  .uartb_usart = {
    .mode         = SBP,
    .baud_rate    = 115200,
    .message_mask = 0xFF00,
  },
  .simulation_start_ecef = {-2700303.10144031,-4292474.39651309,3855434.34087421},
  .simulation_current_ecef = {-2700303.10144031,-4292474.39651309,3855434.34087421},
  .simulation_mode = false,
  .simulation_last_update_ticks = 0,
  .simulation_wn   = 1786,
  .simulation_tow  = 0,
};

/** \} */

