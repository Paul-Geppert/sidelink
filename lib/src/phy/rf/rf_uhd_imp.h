/**
* Copyright 2013-2019 
* Fraunhofer Institute for Telecommunications, Heinrich-Hertz-Institut (HHI)
*
* This file is part of the HHI Sidelink.
*
* HHI Sidelink is under the terms of the GNU Affero General Public License
* as published by the Free Software Foundation version 3.
*
* HHI Sidelink is distributed WITHOUT ANY WARRANTY,
* without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* A copy of the GNU Affero General Public License can be found in
* the LICENSE file in the top-level directory of this distribution
* and at http://www.gnu.org/licenses/.
*
* The HHI Sidelink is based on srsLTE.
* All necessary files and sources from srsLTE are part of HHI Sidelink.
* srsLTE is under Copyright 2013-2017 by Software Radio Systems Limited.
* srsLTE can be found under:
* https://github.com/srsLTE/srsLTE
*/

/*
 * Copyright 2013-2019 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdbool.h>
#include <stdint.h>
  
#include "srslte/config.h"
#include "srslte/phy/rf/rf.h"

#define DEVNAME_B200 "uhd_b200"
#define DEVNAME_X300 "uhd_x300"
#define DEVNAME_N300 "uhd_n300"
#define DEVNAME_E3X0 "uhd_e3x0"


SRSLTE_API int rf_uhd_open(char *args, 
                        void **handler);

SRSLTE_API int rf_uhd_open_multi(char *args, 
                                 void **handler, 
                                 uint32_t nof_channels);

SRSLTE_API char* rf_uhd_devname(void *h);

SRSLTE_API int rf_uhd_close(void *h);

SRSLTE_API int rf_uhd_start_rx_stream(void *h,
                                      bool now);

SRSLTE_API int rf_uhd_start_rx_stream_nsamples(void *h, 
                                            uint32_t nsamples);

SRSLTE_API int rf_uhd_stop_rx_stream(void *h);

SRSLTE_API void rf_uhd_flush_buffer(void *h);

SRSLTE_API bool rf_uhd_has_rssi(void *h);

SRSLTE_API float rf_uhd_get_rssi(void *h); 

SRSLTE_API bool rf_uhd_rx_wait_lo_locked(void *h);

SRSLTE_API void rf_uhd_set_master_clock_rate(void *h, 
                                          double rate); 

SRSLTE_API bool rf_uhd_is_master_clock_dynamic(void *h); 

SRSLTE_API double rf_uhd_set_rx_srate(void *h, 
                                   double freq);

SRSLTE_API double rf_uhd_set_rx_gain(void *h, 
                                  double gain);

SRSLTE_API double rf_uhd_get_rx_gain(void *h);

SRSLTE_API double rf_uhd_get_tx_gain(void *h);

SRSLTE_API srslte_rf_info_t *rf_uhd_get_info(void *h);

SRSLTE_API void rf_uhd_suppress_stdout(void *h);

SRSLTE_API void rf_uhd_register_error_handler(void *h, srslte_rf_error_handler_t error_handler);

SRSLTE_API double rf_uhd_set_rx_freq(void* h, uint32_t ch, double freq);

SRSLTE_API int rf_uhd_recv_with_time(void *h,
                                  void *data,
                                  uint32_t nsamples,
                                  bool blocking,
                                  time_t *secs,
                                  double *frac_secs);

SRSLTE_API int rf_uhd_recv_with_time_multi(void *h,
                                void **data,
                                uint32_t nsamples,
                                bool blocking,
                                time_t *secs,
                                double *frac_secs); 

SRSLTE_API double rf_uhd_set_tx_srate(void *h, 
                                    double freq);

SRSLTE_API double rf_uhd_set_tx_gain(void *h, 
                                   double gain);

SRSLTE_API double rf_uhd_set_tx_freq(void* h, uint32_t ch, double freq);

SRSLTE_API void rf_uhd_get_time(void* h, time_t* secs, double* frac_secs);

SRSLTE_API void rf_uhd_sync_pps(void* h);

SRSLTE_API int  rf_uhd_send_timed(void *h, 
                                  void *data, 
                                  int nsamples,
                                  time_t secs, 
                                  double frac_secs, 
                                  bool has_time_spec,
                                  bool blocking, 
                                  bool is_start_of_burst, 
                                  bool is_end_of_burst);

SRSLTE_API int rf_uhd_send_timed_multi(void *h,
                                       void *data[SRSLTE_MAX_PORTS],
                                       int nsamples,
                                       time_t secs,
                                       double frac_secs,
                                       bool has_time_spec,
                                       bool blocking,
                                       bool is_start_of_burst,
                                       bool is_end_of_burst);

SRSLTE_API void rf_uhd_timed_gpio(void *h,
                                  time_t secs,
                                  double frac_secs,
                                  bool enable);
