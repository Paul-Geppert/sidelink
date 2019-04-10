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
 * This file is part of the srsUE library.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <assert.h>
#include <string.h>
#include <sstream>
#include "srslte/srslte.h"
#include "srssl/hdr/phy/phch_common.h"
#include "srssl/hdr/upper/rest.h"

#define Error(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (SRSLTE_DEBUG_ENABLED) log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (SRSLTE_DEBUG_ENABLED) log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->debug(fmt, ##__VA_ARGS__)

namespace srsue {

cf_t zeros[50000];

phch_common::phch_common(uint32_t max_mutex_) : tx_mutex(max_mutex_)
{
  config    = NULL; 
  args      = NULL; 
  log_h     = NULL; 
  radio_h   = NULL; 
  mac       = NULL; 
  max_mutex = max_mutex_;
  nof_mutex = 0;
  rx_gain_offset = 0;
  last_ri = 0;
  last_pmi = 0;
  //have_mtch_stop = false;
  
  bzero(&dl_metrics, sizeof(dl_metrics_t));
  dl_metrics_read = true;
  dl_metrics_count = 0;
  bzero(&ul_metrics, sizeof(ul_metrics_t));
  ul_metrics_read = true;
  ul_metrics_count = 0;
  bzero(&sync_metrics, sizeof(sync_metrics_t));
  sync_metrics_read = true;
  sync_metrics_count = 0;

  bzero(zeros, 50000*sizeof(cf_t));

  bzero(snr_pssch_per_ue, sizeof(snr_pssch_per_ue));

  // FIXME: This is an ugly fix to avoid the TX filters to empty
  /*
  for (int i=0;i<50000;i++) {
    zeros[i] = 0.01*cexpf(((float) i/50000)*0.1*_Complex_I);
  }*/

  reset();

  sib13_configured = false;
  mcch_configured  = false;
}

#ifdef ENABLE_REST

static int phch_rest_get_metrics (const struct _u_request * request, struct _u_response * response, void * user_data) {
  phch_common * _this = (phch_common *)user_data;

  json_t * json_body = json_pack("{sfsfsfsfsfsf}",
                                  "snr_psbch", _this->snr_psbch,
                                  "snr_ue_0", _this->snr_pssch_per_ue[0],
                                  "snr_ue_1", _this->snr_pssch_per_ue[1],
                                  "snr_ue_2", _this->snr_pssch_per_ue[2],
                                  "snr_ue_3", _this->snr_pssch_per_ue[3],
                                  "snr_ue_4", _this->snr_pssch_per_ue[4]);
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);

  return U_CALLBACK_CONTINUE;
}

static int phch_rest_get_repo_cb (const struct _u_request * request, struct _u_response * response, void * user_data) {
  phch_common * _this = (phch_common *)user_data;

  json_t * json_body = json_pack("{sisisisisisisi}",
                                  "numSubchannel_r14",      _this->ue_repo.rp.numSubchannel_r14,
                                  "sizeSubchannel_r14",     _this->ue_repo.rp.sizeSubchannel_r14,
                                  "sl_OffsetIndicator_r14", _this->ue_repo.rp.sl_OffsetIndicator_r14,
                                  "sl_Subframe_r14_len",    _this->ue_repo.rp.sl_Subframe_r14_len,
                                  "startRB_PSCCH_Pool_r14", _this->ue_repo.rp.startRB_PSCCH_Pool_r14,
                                  "startRB_Subchannel_r14", _this->ue_repo.rp.startRB_Subchannel_r14,
                                  "sizeSubchannel_r14",     _this->ue_repo.rp.sizeSubchannel_r14);
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);

  return U_CALLBACK_CONTINUE;
}


static int phch_rest_put_repo_cb (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phch_common * _this = (phch_common *)user_data;

  json_error_t json_error;
  json_t * req = ulfius_get_json_body_request(request, &json_error);

  if(NULL == req) {
    ulfius_set_string_body_response(response, 400, json_error.text);
    return U_CALLBACK_CONTINUE;
  }

  char *jd = json_dumps(req, 0);
  printf("Received JSON: %s\n", jd);

  SL_CommResourcePoolV2X_r14 new_repo;
  memcpy(&new_repo, &_this->ue_repo.rp, sizeof(SL_CommResourcePoolV2X_r14));

  // parse received repo
  json_unpack(req, "{s?i,s?i,s?i,s?i,s?i,s?i,s?i}",
              "numSubchannel_r14",      &new_repo.numSubchannel_r14,
              "sizeSubchannel_r14",     &new_repo.sizeSubchannel_r14,
              "sl_OffsetIndicator_r14", &new_repo.sl_OffsetIndicator_r14,
              "sl_Subframe_r14_len",    &new_repo.sl_Subframe_r14_len,
              "startRB_PSCCH_Pool_r14", &new_repo.startRB_PSCCH_Pool_r14,
              "startRB_Subchannel_r14", &new_repo.startRB_Subchannel_r14,
              "sizeSubchannel_r14",     &new_repo.sizeSubchannel_r14);

  json_decref(req);

  // check if repo has changed, if so apply changes
  if(0 != memcmp(&new_repo, &_this->ue_repo.rp, sizeof(SL_CommResourcePoolV2X_r14))) {
    printf("Apply new repo settings\n");

    // @todo: when we change the setting while the system is running we may loose some pakets
    memcpy(&_this->ue_repo.rp, &new_repo, sizeof(SL_CommResourcePoolV2X_r14));
  }

  // send current setting to user

  return phch_rest_get_repo_cb(request, response, user_data);
}





static int phch_rest_get_gain (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phch_common * _this = (phch_common *)user_data;

  json_t * json_body = json_pack("{sfsf}",
                                  "tx_gain", _this->get_radio()->get_tx_gain(),
                                  "rx_gain", _this->get_radio()->get_rx_gain());
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);

  return U_CALLBACK_CONTINUE;
}  


static int phch_rest_put_gain (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phch_common * _this = (phch_common *)user_data;
  const int GAIN_NO_CHANGE = -99;

  json_error_t json_error;
  json_t * req = ulfius_get_json_body_request(request, &json_error);

  if(NULL == req) {
    ulfius_set_string_body_response(response, 400, json_error.text);
    return U_CALLBACK_CONTINUE;
  }

  char *jd = json_dumps(req, 0);
  printf("Received JSON: %s\n", jd);

  int tx_gain = GAIN_NO_CHANGE;
  int rx_gain = GAIN_NO_CHANGE;

  // parse received repo
  json_unpack(req, "{s?i,s?i}",
              "tx_gain", &tx_gain,
              "rx_gain", &rx_gain);

  json_decref(req);

  if(GAIN_NO_CHANGE != tx_gain) {
    printf("Apply new tx gain: %i\n", tx_gain);
    _this->get_radio()->set_tx_gain((float)tx_gain);
  }

  if(GAIN_NO_CHANGE != rx_gain) {
    printf("Apply new rx gain: %i\n", rx_gain);
    _this->get_radio()->set_rx_gain((float)rx_gain);
  }

  // send current setting to user
  return phch_rest_get_gain(request, response, user_data);
}

#endif

void phch_common::init(phy_interface_rrc::phy_cfg_t *_config, phy_args_t *_args, srslte::log *_log, srslte::radio *_radio, rrc_interface_phy *_rrc, mac_interface_phy *_mac)
{
  log_h     = _log; 
  radio_h   = _radio;
  rrc       = _rrc;
  mac       = _mac; 
  config    = _config;     
  args      = _args; 
  is_first_tx = true; 
  sr_last_tx_tti = -1;
  
  for (uint32_t i=0;i<nof_mutex;i++) {
    pthread_mutex_init(&tx_mutex[i], NULL);
  }

  #ifdef ENABLE_REST
  int ret;
  ret = ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/repo", NULL, 0, &srsue::phch_rest_get_repo_cb, this);
  ret = ulfius_add_endpoint_by_val(&g_restapi.instance, "PUT", "/phy/repo", NULL, 0, &srsue::phch_rest_put_repo_cb, this);

  ret = ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/metrics", NULL, 0, &srsue::phch_rest_get_metrics, this);

  ret = ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/gain", NULL, 0, &srsue::phch_rest_get_gain, this);
  ret = ulfius_add_endpoint_by_val(&g_restapi.instance, "PUT", "/phy/gain", NULL, 0, &srsue::phch_rest_put_gain, this);

  #endif

}

void phch_common::set_nof_mutex(uint32_t nof_mutex_) {
  nof_mutex = nof_mutex_; 
  assert(nof_mutex <= max_mutex);
}

bool phch_common::ul_rnti_active(uint32_t tti) {
  if ((((int)tti >= ul_rnti_start && ul_rnti_start >= 0) || ul_rnti_start < 0) &&
      (((int)tti <  ul_rnti_end   && ul_rnti_end   >= 0) || ul_rnti_end   < 0))
  {
    return true; 
  } else {
    return false; 
  }
}

bool phch_common::dl_rnti_active(uint32_t tti) {
  Debug("tti=%d, dl_rnti_start=%d, dl_rnti_end=%d, dl_rnti=0x%x\n", tti, dl_rnti_start, dl_rnti_end, dl_rnti);
  if ((((int)tti >= dl_rnti_start && dl_rnti_start >= 0)  || dl_rnti_start < 0) &&
      (((int)tti <  dl_rnti_end   && dl_rnti_end   >= 0)  || dl_rnti_end   < 0))
  {
    bool ret = true; 
    // FIXME: This scheduling decision belongs to RRC
    if (dl_rnti_type == SRSLTE_RNTI_SI) {
      if (dl_rnti_end - dl_rnti_start > 1) { // This is not a SIB1        
        if ((tti/10)%2 == 0 && (tti%10) == 5) { // Skip subframe #5 for which SFN mod 2 = 0
          ret = false; 
        }
      }
    }
    return ret; 
  } else {
    return false; 
  }
}

srslte::radio* phch_common::get_radio()
{
  return radio_h;
}

// Unpack RAR grant as defined in Section 6.2 of 36.213 
void phch_common::set_rar_grant(uint32_t tti, uint8_t grant_payload[SRSLTE_RAR_GRANT_LEN])
{
  srslte_dci_rar_grant_unpack(&rar_grant, grant_payload);
  rar_grant_pending = true;
  if (MSG3_DELAY_MS < 0) {
    fprintf(stderr, "Error MSG3_DELAY_MS can't be negative\n");
  }
  if (rar_grant.ul_delay) {
    rar_grant_tti     = (tti + MSG3_DELAY_MS + 1) % 10240;
  } else {
    rar_grant_tti     = (tti + MSG3_DELAY_MS) % 10240;
  }
}

bool phch_common::get_pending_rar(uint32_t tti, srslte_dci_rar_grant_t *rar_grant_)
{
  if (rar_grant_pending && tti >= rar_grant_tti) {
    if (rar_grant_) {
      rar_grant_pending = false; 
      memcpy(rar_grant_, &rar_grant, sizeof(srslte_dci_rar_grant_t));
    }
    return true; 
  }
  return false; 
}

/* Common variables used by all phy workers */
uint16_t phch_common::get_ul_rnti(uint32_t tti) {
  if (ul_rnti_active(tti)) {
    return ul_rnti; 
  } else {
    return 0; 
  }
}
srslte_rnti_type_t phch_common::get_ul_rnti_type() {
  return ul_rnti_type; 
}
void phch_common::set_ul_rnti(srslte_rnti_type_t type, uint16_t rnti_value, int tti_start, int tti_end) {
  ul_rnti = rnti_value;
  ul_rnti_type = type;
  ul_rnti_start = tti_start;
  ul_rnti_end   = tti_end;
}
uint16_t phch_common::get_dl_rnti(uint32_t tti) {
  if (dl_rnti_active(tti)) {
    return dl_rnti; 
  } else {
    return 0; 
  }
}
srslte_rnti_type_t phch_common::get_dl_rnti_type() {
  return dl_rnti_type; 
}
void phch_common::set_dl_rnti(srslte_rnti_type_t type, uint16_t rnti_value, int tti_start, int tti_end) {
  dl_rnti       = rnti_value;
  dl_rnti_type  = type;
  dl_rnti_start = tti_start;
  dl_rnti_end   = tti_end;
  if (log_h) {
    Debug("Set DL rnti: start=%d, end=%d, value=0x%x\n", tti_start, tti_end, rnti_value);
  }
}

void phch_common::reset_pending_ack(uint32_t tti) {
  pending_ack[TTIMOD(tti)].enabled = false;
}

void phch_common::set_pending_ack(uint32_t tti, uint32_t I_lowest, uint32_t n_dmrs) {
  pending_ack[TTIMOD(tti)].enabled  = true;
  pending_ack[TTIMOD(tti)].I_lowest = I_lowest;
  pending_ack[TTIMOD(tti)].n_dmrs = n_dmrs;
  Debug("Set pending ACK for tti=%d I_lowest=%d, n_dmrs=%d\n", tti, I_lowest, n_dmrs);
}

bool phch_common::get_pending_ack(uint32_t tti) {
  return get_pending_ack(tti, NULL, NULL); 
}

bool phch_common::get_pending_ack(uint32_t tti, uint32_t *I_lowest, uint32_t *n_dmrs) {
  if (I_lowest) {
    *I_lowest = pending_ack[TTIMOD(tti)].I_lowest;
  }
  if (n_dmrs) {
    *n_dmrs = pending_ack[TTIMOD(tti)].n_dmrs;
  }
  return pending_ack[TTIMOD(tti)].enabled;
}

bool phch_common::is_any_pending_ack() {
  for (int i=0;i<TTIMOD_SZ;i++) {
    if (pending_ack[i].enabled) {
      return true;
    }
  }
  return false;
}

/* The transmisison of UL subframes must be in sequence. Each worker uses this function to indicate
 * that all processing is done and data is ready for transmission or there is no transmission at all (tx_enable). 
 * In that case, the end of burst message will be send to the radio 
 */
void phch_common::worker_end(uint32_t tti, bool tx_enable, 
                                   cf_t *buffer, uint32_t nof_samples, 
                                   srslte_timestamp_t tx_time) 
{

  // Wait previous TTIs to be transmitted 
  if (is_first_tx) {
    is_first_tx = false; 
  } else {
    pthread_mutex_lock(&tx_mutex[tti%nof_mutex]);
  }

  radio_h->set_tti(tti); 
  if (tx_enable) {
    radio_h->tx_single(buffer, nof_samples, tx_time);
    is_first_of_burst = false; 
  } else {
    if (radio_h->is_continuous_tx()) {
      if (!is_first_of_burst) {
        radio_h->tx_single(zeros, nof_samples, tx_time);
      }
    } else {
      if (!is_first_of_burst) {
        radio_h->tx_end();
        is_first_of_burst = true;   
      }
    }
  }
  // Trigger next transmission 
  pthread_mutex_unlock(&tx_mutex[(tti+1)%nof_mutex]);
}    


void phch_common::set_cell(const srslte_cell_t &c) {
  cell = c;

  // setup resouce pool with a default configuration
  SL_CommResourcePoolV2X_r14 respool = {
    0, //uint8_t sl_OffsetIndicator_r14;

    // subframe bitmap
    {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0},//uint8_t sl_Subframe_r14[100];
    20, //uint8_t sl_Subframe_r14_len;

    true,//bool adjacencyPSCCH_PSSCH_r14;

    // indicated the number of PRBs of each subchannel in the corresponding resource pool
    5,//uint8_t sizeSubchannel_r14;

    // indicates the number of subchannels in the corresponding resource pool
    5,//uint8_t numSubchannel_r14;

    // indicates the lowest RB index of the subchannel with the lowest index
    0,//uint8_t startRB_Subchannel_r14;

    // indicates the lowest RB index of PSCCH pool, this field is absent whe a pool is (pre)configured
    // such tha a UE always transmits SC and data in adjacent RBs in the same subframe
    0,//uint8_t startRB_PSCCH_Pool_r14;
  };

  if (srslte_repo_init(&ue_repo, cell, respool)) {
    fprintf(stderr, "Error creating Ressource Pool object\n");
  }

  // set sync period to 5, as this is what the current sync implementations expects
  ue_repo.syncPeriod = 5;
  srslte_repo_build_resource_pool(&ue_repo);
}

uint32_t phch_common::get_nof_prb() {
  return cell.nof_prb;
}

void phch_common::set_dl_metrics(const dl_metrics_t &m) {
  if(dl_metrics_read) {
    dl_metrics       = m;
    dl_metrics_count = 1;
    dl_metrics_read  = false;
  } else {
    dl_metrics_count++;
    dl_metrics.mcs  = dl_metrics.mcs + (m.mcs - dl_metrics.mcs)/dl_metrics_count;
    dl_metrics.n    = dl_metrics.n + (m.n - dl_metrics.n)/dl_metrics_count;
    dl_metrics.rsrp = dl_metrics.rsrp + (m.rsrp - dl_metrics.rsrp)/dl_metrics_count;
    dl_metrics.rsrq = dl_metrics.rsrq + (m.rsrq - dl_metrics.rsrq)/dl_metrics_count;
    dl_metrics.rssi = dl_metrics.rssi + (m.rssi - dl_metrics.rssi)/dl_metrics_count;
    dl_metrics.sinr = dl_metrics.sinr + (m.sinr - dl_metrics.sinr)/dl_metrics_count;
    dl_metrics.pathloss = dl_metrics.pathloss + (m.pathloss - dl_metrics.pathloss)/dl_metrics_count;
    dl_metrics.turbo_iters = dl_metrics.turbo_iters + (m.turbo_iters - dl_metrics.turbo_iters)/dl_metrics_count;
  }
}

void phch_common::get_dl_metrics(dl_metrics_t &m) {
  m = dl_metrics;
  dl_metrics_read = true;
}

void phch_common::set_ul_metrics(const ul_metrics_t &m) {
  if(ul_metrics_read) {
    ul_metrics       = m;
    ul_metrics_count = 1;
    ul_metrics_read  = false;
  } else {
    ul_metrics_count++;
    ul_metrics.mcs   = ul_metrics.mcs + (m.mcs - ul_metrics.mcs)/ul_metrics_count;
    ul_metrics.power = ul_metrics.power + (m.power - ul_metrics.power)/ul_metrics_count;
  }
}

void phch_common::get_ul_metrics(ul_metrics_t &m) {
  m = ul_metrics;
  ul_metrics_read = true;
}

void phch_common::set_sync_metrics(const sync_metrics_t &m) {

  if(sync_metrics_read) {
    sync_metrics = m;
    sync_metrics_count = 1;
    sync_metrics_read  = false;
  } else {
    sync_metrics_count++;
    sync_metrics.cfo = sync_metrics.cfo + (m.cfo - sync_metrics.cfo)/sync_metrics_count;
    sync_metrics.sfo = sync_metrics.sfo + (m.sfo - sync_metrics.sfo)/sync_metrics_count;
  }
}

void phch_common::get_sync_metrics(sync_metrics_t &m) {
  m = sync_metrics;
  sync_metrics_read = true;
}

void phch_common::reset() {
  sr_enabled        = false;
  is_first_of_burst = true;
  is_first_tx       = true;
  rar_grant_pending = false;
  pathloss = 0;
  cur_pathloss = 0;
  cur_pusch_power = 0;
  p0_preamble = 0;
  cur_radio_power = 0;
  sr_last_tx_tti = -1;
  cur_pusch_power = 0;
  avg_snr_db_cqi = 0;
  avg_rsrp = 0;
  avg_rsrp_dbm = 0;
  avg_rsrq_db = 0;

  pcell_report_period = 20;

  bzero(pending_ack, sizeof(pending_ack_t)*TTIMOD_SZ);

}

void phch_common::reset_ul()
{
  /*
  is_first_tx = true;
  is_first_of_burst = true;

  for (uint32_t i=0;i<nof_mutex;i++) {
    pthread_mutex_trylock(&tx_mutex[i]);
    pthread_mutex_unlock(&tx_mutex[i]);
  }
  radio_h->tx_end();
   */
}

/*  Convert 6-bit maps to 10-element subframe tables
    bitmap         = |0|0|0|0|0|0|
    subframe index = |1|2|3|6|7|8|
*/
void phch_common::build_mch_table()
{
  // First reset tables
  bzero(&mch_table[0], sizeof(uint8_t)*40);

  // 40 element table represents 4 frames (40 subframes)
  generate_mch_table(&mch_table[0], config->mbsfn.mbsfn_subfr_cnfg.subfr_alloc,(LIBLTE_RRC_SUBFRAME_ALLOCATION_NUM_FRAMES_ONE == config->mbsfn.mbsfn_subfr_cnfg.subfr_alloc_num_frames)?1:4);
  // Debug

  
  std::stringstream ss;
  ss << "|";
  for(uint32_t j=0; j<40; j++) {
    ss << (int) mch_table[j] << "|";
  }
  Info("MCH table: %s\n", ss.str().c_str());
}

void phch_common::build_mcch_table()
{
  // First reset tables
  bzero(&mcch_table[0], sizeof(uint8_t)*10);
  generate_mcch_table(&mcch_table[0], config->mbsfn.mbsfn_area_info.sf_alloc_info_r9);
  // Debug
  std::stringstream ss;
  ss << "|";
  for(uint32_t j=0; j<10; j++) {
    ss << (int) mcch_table[j] << "|";
  }
  Info("MCCH table: %s\n", ss.str().c_str());
  sib13_configured = true;
}

void phch_common::set_mcch()
{
  mcch_configured = true;
}

void phch_common::set_mch_period_stop(uint32_t stop)
{
  pthread_mutex_lock(&mtch_mutex);
  have_mtch_stop = true;
  mch_period_stop = stop;
  pthread_cond_signal(&mtch_cvar);
  pthread_mutex_unlock(&mtch_mutex);

}

bool phch_common::is_mch_subframe(subframe_cfg_t *cfg, uint32_t phy_tti)
{
  uint32_t sfn;   // System Frame Number
  uint8_t  sf;    // Subframe
  uint8_t  offset;
  uint8_t  period;

  sfn = phy_tti/10;
  sf  = phy_tti%10;

  // Set some defaults
  cfg->mbsfn_area_id            = 0;
  cfg->non_mbsfn_region_length  = 1;
  cfg->mbsfn_mcs                = 2;
  cfg->mbsfn_decode             = false;
  cfg->is_mcch                  = false;
  // Check for MCCH
  if(is_mcch_subframe(cfg, phy_tti)) {
    cfg->is_mcch = true;
    return true;
  }
  
  // Not MCCH, check for MCH
  LIBLTE_RRC_MBSFN_SUBFRAME_CONFIG_STRUCT *subfr_cnfg = &config->mbsfn.mbsfn_subfr_cnfg;
  LIBLTE_RRC_MBSFN_AREA_INFO_STRUCT *area_info = &config->mbsfn.mbsfn_area_info;
  offset = subfr_cnfg->radio_fr_alloc_offset;
  period = liblte_rrc_radio_frame_allocation_period_num[subfr_cnfg->radio_fr_alloc_period];

  if(LIBLTE_RRC_SUBFRAME_ALLOCATION_NUM_FRAMES_ONE == subfr_cnfg->subfr_alloc_num_frames) {
    if((sfn%period == offset) && (mch_table[sf] > 0)) {
      if(sib13_configured) {
        cfg->mbsfn_area_id = area_info->mbsfn_area_id_r9;
        cfg->non_mbsfn_region_length = liblte_rrc_non_mbsfn_region_length_num[area_info->non_mbsfn_region_length];
        if(mcch_configured) {
          // Iterate through PMCH configs to see which one applies in the current frame
          LIBLTE_RRC_MCCH_MSG_STRUCT *mcch = &config->mbsfn.mcch;
          uint32_t mbsfn_per_frame = mcch->pmch_infolist_r9[0].pmch_config_r9.sf_alloc_end_r9/liblte_rrc_mch_scheduling_period_r9_num[mcch->pmch_infolist_r9[0].pmch_config_r9.mch_schedulingperiod_r9];
          uint32_t frame_alloc_idx = sfn%liblte_rrc_mbsfn_common_sf_alloc_period_r9_num[mcch->commonsf_allocperiod_r9];        
          uint32_t sf_alloc_idx = frame_alloc_idx*mbsfn_per_frame + ((sf<4)?sf-1:sf-3); 
          pthread_mutex_lock(&mtch_mutex);
          while(!have_mtch_stop) {
            pthread_cond_wait(&mtch_cvar, &mtch_mutex);
          }
          pthread_mutex_unlock(&mtch_mutex);

          for(uint32_t i=0; i<mcch->pmch_infolist_r9_size; i++) {
            if(sf_alloc_idx <= mch_period_stop) {
              //trigger conditional variable, has ot be untriggered by mtch stop location
              cfg->mbsfn_mcs = mcch->pmch_infolist_r9[i].pmch_config_r9.datamcs_r9;
              cfg->mbsfn_decode = true;
            } else {
              //have_mtch_stop = false;
            }
          }
          Debug("MCH subframe TTI:%d\n", phy_tti);
        }
      }
      return true;
    }
  }else if(LIBLTE_RRC_SUBFRAME_ALLOCATION_NUM_FRAMES_FOUR == subfr_cnfg->subfr_alloc_num_frames) {
    uint8_t idx = sfn%period;
    if((idx >= offset) && (idx < offset+4)) {
      if(mch_table[(idx*10)+sf] > 0){
        if(sib13_configured) {
          cfg->mbsfn_area_id = area_info->mbsfn_area_id_r9;
          cfg->non_mbsfn_region_length = liblte_rrc_non_mbsfn_region_length_num[area_info->non_mbsfn_region_length];
         // TODO: check for MCCH configuration, set MCS and decode

        }
        return true;
      }
    }
  }

  return false;
}

bool phch_common::is_mcch_subframe(subframe_cfg_t *cfg, uint32_t phy_tti)
{
  uint32_t sfn;   // System Frame Number
  uint8_t  sf;    // Subframe
  uint8_t  offset;
  uint8_t  period;

  sfn = phy_tti/10;
  sf  = phy_tti%10;

  if(sib13_configured) {
    LIBLTE_RRC_MBSFN_SUBFRAME_CONFIG_STRUCT *subfr_cnfg = &config->mbsfn.mbsfn_subfr_cnfg;
    LIBLTE_RRC_MBSFN_AREA_INFO_STRUCT *area_info = &config->mbsfn.mbsfn_area_info;

    offset = area_info->mcch_offset_r9;
    period = liblte_rrc_mcch_repetition_period_r9_num[area_info->mcch_repetition_period_r9];

    if((sfn%period == offset) && mcch_table[sf] > 0) {
      cfg->mbsfn_area_id = area_info->mbsfn_area_id_r9;
      cfg->non_mbsfn_region_length = liblte_rrc_non_mbsfn_region_length_num[area_info->non_mbsfn_region_length];
      cfg->mbsfn_mcs = liblte_rrc_mcch_signalling_mcs_r9_num[area_info->signalling_mcs_r9];
      cfg->mbsfn_decode = true;
      have_mtch_stop = false;
      Debug("MCCH subframe TTI:%d\n", phy_tti);
      return true;
    }
  }
  return false;
}

void phch_common::get_sf_config(subframe_cfg_t *cfg, uint32_t phy_tti)
{
  if(is_mch_subframe(cfg, phy_tti)) {
    cfg->sf_type = SUBFRAME_TYPE_MBSFN;
  }else{
    cfg->sf_type = SUBFRAME_TYPE_REGULAR;
  }
}

}
