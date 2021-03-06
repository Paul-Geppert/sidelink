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

/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
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

/******************************************************************************
 *  File:         pssch.h
 *
 *  Description:  Physical sidelink shared channel
 *
 *  Reference:    3GPP TS 36.211 version 14.2.0 Release 14 Sec. 9.3
 *****************************************************************************/

#ifndef SRSLTE_PSSCH_H
#define SRSLTE_PSSCH_H

#include "srslte/config.h"
#include "srslte/phy/common/phy_common.h"
#include "srslte/phy/mimo/precoding.h"
#include "srslte/phy/mimo/layermap.h"
#include "srslte/phy/modem/mod.h"
#include "srslte/phy/modem/demod_soft.h"
#include "srslte/phy/scrambling/scrambling.h"
#include "srslte/phy/phch/dci.h"
#include "srslte/phy/phch/regs.h"
#include "srslte/phy/phch/sch.h"
#include "srslte/phy/phch/pdsch_cfg.h"
#include "srslte/phy/dft/dft_precoding.h"

typedef struct {
  srslte_sequence_t seq[SRSLTE_MAX_CODEWORDS][SRSLTE_NOF_SF_X_FRAME];
  uint32_t cell_id;
  bool sequence_generated;
} srslte_pssch_user_t;

/* PSSCH object */
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  
  uint32_t nof_rx_antennas;
  uint32_t last_nof_iterations[SRSLTE_MAX_CODEWORDS];

  uint32_t max_re;

  uint16_t ue_rnti;
  bool is_ue;

  /* Power allocation parameter 3GPP 36.213 Clause 5.2 Rho_b */
  float rho_a;

  /* buffers */
  // void buffers are shared for tx and rx
  cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS]; /* Channel estimation (Rx only) */
  cf_t *symbols[SRSLTE_MAX_PORTS];              /* PSSCH Encoded/Decoded Symbols */
  cf_t *SymSPSRsrp[SRSLTE_MAX_PORTS];  /* Proscessed PSSCH Received Symbols for PSSCH-RSRP calculation. Only included Symb 2, 5, 9 and 12, see PSSCH-RSRP, 36.214 V.14.2.0. See 36.211 V.140700, Ch.9.8.*/
  cf_t *SymSPSRssi[SRSLTE_MAX_PORTS];  /* Proscessed PSSCH Received Symbols for S-RSSI calculation. Excluded Symb 0 and 13, see S-RSSI, 36.214 V.14.2.0*/
  cf_t *x[SRSLTE_MAX_LAYERS];                   /* Layer mapped */
  cf_t *d[SRSLTE_MAX_CODEWORDS];                /* Modulated/Demodulated codewords */
  void *e[SRSLTE_MAX_CODEWORDS];
  void *h[SRSLTE_MAX_CODEWORDS];

  bool csi_enabled;
  float *csi[SRSLTE_MAX_CODEWORDS];             /* Channel Strengh Indicator */
  float sps_rsrp[10 * Pstep];             /* store the calculated PSSCH-RSRP, size generally 1000. */
  float sps_rssi[10 * Pstep];             /* store the calculated s-RSSI, size generally 1000. */

  /* tx & rx objects */
  srslte_modem_table_t mod[4];
  srslte_dft_precoding_t dft_precoding;
  srslte_dft_precoding_t dft_deprecoding;
  
  // This is to generate the scrambling seq for multiple CRNTIs
  srslte_pssch_user_t **users;

  // parameters to generate scrambling sequence
  uint32_t n_X_ID;
  uint32_t n_PSSCH_ssf;
  srslte_sequence_t tmp_seq;

  uint16_t *interleaver_lut;

  srslte_sch_t dl_sch;

  void *coworker_ptr;

} srslte_pssch_t;

typedef srslte_pdsch_cfg_t srslte_pssch_cfg_t;

SRSLTE_API int srslte_pssch_init_ue(srslte_pssch_t *q,
                                    uint32_t max_prb,
                                    uint32_t nof_rx_antennas);

SRSLTE_API int srslte_pssch_init_enb(srslte_pssch_t *q,
                                     uint32_t max_prb);

SRSLTE_API void srslte_pssch_free(srslte_pssch_t *q);

SRSLTE_API int srslte_pssch_set_cell(srslte_pssch_t *q,
                                     srslte_cell_t cell);

SRSLTE_API int srslte_pssch_set_rnti(srslte_pssch_t *q,
                                     uint16_t rnti);

SRSLTE_API void srslte_pssch_set_power_allocation(srslte_pssch_t *q,
                                                  float rho_a);

SRSLTE_API int srslte_pssch_enable_csi(srslte_pssch_t *q,
                                       bool enable);

SRSLTE_API void srslte_pssch_free_rnti(srslte_pssch_t *q, 
                                      uint16_t rnti);


# if 0
SRSLTE_API int srslte_pssch_cfg(srslte_pssch_cfg_t *cfg,
                                srslte_cell_t cell, 
                                srslte_ra_dl_grant_t *grant, 
                                uint32_t cfi, 
                                uint32_t sf_idx, 
                                int rvidx);

SRSLTE_API int srslte_pssch_cfg_mimo(srslte_pssch_cfg_t *cfg,
                                     srslte_cell_t cell,
                                     srslte_ra_dl_grant_t *grant,
                                     uint32_t cfi,
                                     uint32_t sf_idx,
                                     int rvidx[SRSLTE_MAX_CODEWORDS],
                                     srslte_mimo_type_t mimo_type,
                                     uint32_t pmi);
#endif

SRSLTE_API int srslte_pssch_encode(srslte_pssch_t *q,
                                         srslte_pssch_cfg_t *cfg,
                                         srslte_softbuffer_tx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                         uint8_t *data[SRSLTE_MAX_CODEWORDS],
                                         uint16_t rnti,
                                         cf_t *sf_symbols[SRSLTE_MAX_PORTS]);

SRSLTE_API int srslte_pssch_encode_simple(srslte_pssch_t *q,
                                          srslte_ra_sl_sci_t *sci,
                                          srslte_softbuffer_tx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                          cf_t *sf_symbols[SRSLTE_MAX_PORTS],
                                          uint32_t prb_offset,
                                          uint32_t n_prb,
                                          uint8_t *data[SRSLTE_MAX_CODEWORDS]);

SRSLTE_API int srslte_pssch_decode_simple(srslte_pssch_t *q,
                                          srslte_ra_sl_sci_t *sci,
                                          srslte_pssch_cfg_t *cfg,
                                          srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                          cf_t *sf_symbols[SRSLTE_MAX_PORTS],
                                          cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS],
                                          float noise_estimate,
                                          uint32_t prn_offset,
                                          uint32_t n_prb,
                                          uint8_t *data[SRSLTE_MAX_CODEWORDS]);

SRSLTE_API int srslte_pssch_decode(srslte_pssch_t *q, 
                                   srslte_pssch_cfg_t *cfg,
                                   srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                   cf_t *sf_symbols[SRSLTE_MAX_PORTS],
                                   cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS],
                                   float noise_estimate,
                                   uint16_t rnti,
                                   uint8_t *data[SRSLTE_MAX_CODEWORDS],
                                   bool acks[SRSLTE_MAX_CODEWORDS]);

SRSLTE_API int srslte_pssch_pmi_select(srslte_pssch_t *q,
                                       srslte_pssch_cfg_t *cfg,
                                       cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS],
                                       float noise_estimate,
                                       uint32_t nof_ce,
                                       uint32_t pmi[SRSLTE_MAX_LAYERS],
                                       float sinr[SRSLTE_MAX_LAYERS][SRSLTE_MAX_CODEBOOKS]);

SRSLTE_API int srslte_pssch_cn_compute(srslte_pssch_t *q,
                                       cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS],
                                       uint32_t nof_ce,
                                       float *cn);

SRSLTE_API void srslte_pssch_set_max_noi(srslte_pssch_t *q,
                                         uint32_t max_iter);

SRSLTE_API float srslte_pssch_last_noi(srslte_pssch_t *q);

SRSLTE_API int srslte_pssch_enable_coworker(srslte_pssch_t *q);

SRSLTE_API uint32_t srslte_pssch_last_noi_cw(srslte_pssch_t *q,
                                             uint32_t cw_idx);

SRSLTE_API int srslte_pssch_get_for_sps_rsrp(cf_t* subframe, 
                                             cf_t* pssch, 
                                             srslte_cell_t cell, 
                                             uint32_t prb_offset,
                                             uint32_t n_prb);

SRSLTE_API int srslte_pssch_cp_for_sps_rsrp(cf_t* input, 
                                            cf_t* output, 
                                            srslte_cell_t cell, 
                                            uint32_t prb_offset,
                                            uint32_t n_prb);

SRSLTE_API int srslte_pssch_get_for_sps_rssi(cf_t* subframe, 
                                             cf_t* pssch, 
                                             srslte_cell_t cell, 
                                             uint32_t prb_offset,
                                             uint32_t n_prb);

SRSLTE_API int srslte_pssch_cp_for_sps_rssi(cf_t* input, 
                                            cf_t* output, 
                                            srslte_cell_t cell, 
                                            uint32_t prb_offset,
                                            uint32_t n_prb); 

#endif // SRSLTE_PSSCH_H
