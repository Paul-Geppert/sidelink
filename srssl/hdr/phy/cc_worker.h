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

#ifndef SRSLTE_CC_WORKER_H
#define SRSLTE_CC_WORKER_H

#include "phy_common.h"
#include "srslte/srslte.h"

namespace srsue {

class cc_worker
{
public:
  cc_worker(uint32_t cc_idx, uint32_t max_prb, phy_common* phy, srslte::log* log);
  ~cc_worker();
  void reset();

  bool set_cell(srslte_cell_t cell);

  /* Functions used by main PHY thread */
  cf_t* get_rx_buffer(uint32_t antenna_idx);
  cf_t* get_tx_buffer(uint32_t antenna_idx);
  uint32_t get_buffer_len();

  void  set_tti(uint32_t tti);
  void  set_cfo(float cfo);
  float get_ref_cfo();

  void set_tdd_config(srslte_tdd_config_t config);
  void set_pcell_config(phy_interface_rrc_lte::phy_cfg_t* phy_cfg);
  void set_scell_config(asn1::rrc::scell_to_add_mod_r10_s* phy_cfg);
  void set_crnti(uint16_t rnti);
  void enable_pregen_signals(bool enabled);

  bool work_dl_regular();
  bool work_dl_mbsfn(srslte_mbsfn_cfg_t mbsfn_cfg);
  bool work_ul(srslte_uci_data_t* uci_data);

  bool work_sl_rx();
  bool work_sl_tx();

  int read_ce_abs(float* ce_abs, uint32_t tx_antenna, uint32_t rx_antenna);
  int read_pdsch_d(cf_t* pdsch_d);

  int read_pssch_d(cf_t* pdsch_d);

  cf_t* read_td_samples(uint32_t* n_samples);
  float get_last_agc();

  void update_measurements();
  bool dump_subframe();
  void set_receive_time(srslte_timestamp_t tx_time);
  void set_receiver_gain(float rx_gain_from_sf_worker);

private:
  void dl_phy_to_mac_grant(srslte_pdsch_grant_t*                  phy_grant,
                           srslte_dci_dl_t*                       dl_dci,
                           mac_interface_phy_lte::mac_grant_dl_t* mac_grant);
  void ul_phy_to_mac_grant(srslte_pusch_grant_t*                  phy_grant,
                           srslte_dci_ul_t*                       ul_dci,
                           uint32_t                               pid,
                           bool                                   ul_grant_available,
                           mac_interface_phy_lte::mac_grant_ul_t* mac_grant);
  void fill_dci_cfg(srslte_dci_cfg_t* cfg, bool rel10 = false);

  // Cross-carried grants scheduled from PCell
  void set_dl_pending_grant(uint32_t cc_idx, srslte_dci_dl_t* dl_dci);
  bool get_dl_pending_grant(uint32_t cc_idx, srslte_dci_dl_t* dl_dci);
  typedef struct {
    bool            enable;
    srslte_dci_dl_t dl_dci;
  } pending_dl_grant_t;
  pending_dl_grant_t pending_dl_grant[SRSLTE_MAX_CARRIERS]; // Only for the current TTI

  /* Methods for DL... */
  int decode_pdcch_ul();
  int decode_pdcch_dl();

  void decode_phich();
  int  decode_pdsch(srslte_pdsch_ack_resource_t            ack_resource,
                    mac_interface_phy_lte::tb_action_dl_t* action,
                    bool                                   acks[SRSLTE_MAX_CODEWORDS]);
  int  decode_pmch(mac_interface_phy_lte::tb_action_dl_t* action, srslte_mbsfn_cfg_t* mbsfn_cfg);

  /* Methods for UL */
  bool     encode_uplink(mac_interface_phy_lte::tb_action_ul_t* action, srslte_uci_data_t* uci_data);
  void     set_uci_sr(srslte_uci_data_t* uci_data);
  void     set_uci_periodic_cqi(srslte_uci_data_t* uci_data);
  void     set_uci_aperiodic_cqi(srslte_uci_data_t* uci_data);
  void     set_uci_ack(srslte_uci_data_t* uci_data, bool is_grant_available, uint32_t dai_ul, bool is_pusch_available);
  uint32_t get_wideband_cqi();
  srslte_cqi_report_mode_t aperiodic_mode(asn1::rrc::cqi_report_mode_aperiodic_e mode);
  void                     parse_antenna_info(asn1::rrc::phys_cfg_ded_s* dedicated);
  void                     parse_pucch_config(phy_interface_rrc_lte::phy_cfg_t* phy_cfg);

  /* Methods for sidleink */
  bool decode_pscch_dl(srsue::mac_interface_phy_lte::mac_grant_dl_t* grant);
  int decode_pssch(srslte_ra_sl_sci_t *grant,
                    uint8_t *payload[SRSLTE_MAX_CODEWORDS],
                    srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                    uint32_t rv[SRSLTE_MAX_CODEWORDS],
                    uint16_t rnti, uint32_t harq_pid, bool acks[SRSLTE_MAX_CODEWORDS]);
  
  typedef struct {
    bool            enable;
    srslte_ra_sl_sci_t sl_dci;
  } pending_sl_grant_t;
  pending_sl_grant_t pending_sl_grant[SRSLTE_MAX_CARRIERS]; // Only for the current TTI

  /* Common objects */
  phy_common*  phy;
  srslte::log* log_h;

  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t sf_cfg_dl;
  srslte_ul_sf_cfg_t sf_cfg_ul;

  uint32_t cc_idx;
  bool     pregen_enabled;
  bool     cell_initiated;
  cf_t*    signal_buffer_rx[SRSLTE_MAX_PORTS] = {};
  cf_t*    signal_buffer_tx[SRSLTE_MAX_PORTS] = {};
  uint32_t signal_buffer_max_samples          = 0;

  /* Objects for DL */
  srslte_ue_dl_t     ue_dl;
  srslte_ue_dl_cfg_t ue_dl_cfg;
  srslte_pmch_cfg_t  pmch_cfg;

  srslte_chest_dl_cfg_t chest_mbsfn_cfg;
  srslte_chest_dl_cfg_t chest_default_cfg;

  /* Objects for UL */
  srslte_ue_ul_t     ue_ul;
  srslte_ue_ul_cfg_t ue_ul_cfg;

  /* Objects for SL */
  srslte_ue_sl_mib_t ue_sl;
  srslte_ue_sl_tx_t ue_sl_tx;

  /* SL */
  srslte_timestamp_t rx_time;
  static uint32_t sps_rsrp_read_cnt;
  static uint32_t sps_rssi_read_cnt;
  bool last_decoding_successful;
  bool last_decoding_successful_high_rsrp;
  float agc_max_value;
  // this is the rx_gain
  float curr_rx_gain;

  // Metrics
  dl_metrics_t dl_metrics;
  ul_metrics_t ul_metrics;
};

} // namespace srsue

#endif // SRSLTE_CC_WORKER_H
