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

#include <unistd.h>
#include <string.h>
#include "srssl/hdr/phy/phch_worker.h"
#include "srslte/srslte.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/asn1/liblte_rrc.h"

#define Error(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (SRSLTE_DEBUG_ENABLED) log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (SRSLTE_DEBUG_ENABLED) log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->debug(fmt, ##__VA_ARGS__)


/* This is to visualize the channel response */
#ifdef ENABLE_GUI
#include "srsgui/srsgui.h"
#include <semaphore.h>

void init_plots(srsue::phch_worker *worker);
pthread_t plot_thread; 
sem_t plot_sem; 
static int plot_worker_id = -1;
#else
#warning Compiling without srsGUI support
#endif
/*********************************************/


namespace srsue {


phch_worker::phch_worker() : tr_exec(10240)
{
  phy = NULL;
  chest_loop = NULL;

  bzero(signal_buffer, sizeof(cf_t*)*SRSLTE_MAX_PORTS);

  mem_initiated   = false;
  cell_initiated  = false; 
  pregen_enabled  = false; 
  trace_enabled   = false;
  reset();
}


phch_worker::~phch_worker()
{
  if (mem_initiated) {
    for (uint32_t i=0;i<phy->args->nof_rx_ant;i++) {
      if (signal_buffer[i]) {
        free(signal_buffer[i]);
      }
    }
    srslte_ue_dl_free(&ue_dl);
    srslte_ue_ul_free(&ue_ul);
    mem_initiated = false;
  }
}

void phch_worker::reset()
{
  bzero(&dl_metrics, sizeof(dl_metrics_t));
  bzero(&ul_metrics, sizeof(ul_metrics_t));
  bzero(&dmrs_cfg, sizeof(srslte_refsignal_dmrs_pusch_cfg_t));    
  bzero(&pusch_hopping, sizeof(srslte_pusch_hopping_cfg_t));
  bzero(&uci_cfg, sizeof(srslte_uci_cfg_t));
  bzero(&pucch_cfg, sizeof(srslte_pucch_cfg_t));
  bzero(&pucch_sched, sizeof(srslte_pucch_sched_t));
  bzero(&srs_cfg, sizeof(srslte_refsignal_srs_cfg_t));
  bzero(&period_cqi, sizeof(srslte_cqi_periodic_cfg_t));
  sr_configured   = false;
  rnti_is_set     = false; 
  rar_cqi_request = false;
  I_sr = 0;
  cfi  = 0;
  rssi_read_cnt = 0;
}

void phch_worker::enable_pdsch_coworker() {
  srslte_pdsch_enable_coworker(&ue_dl.pdsch);
}

void phch_worker::set_common(phch_common* phy_)
{
  phy = phy_;   
}

bool phch_worker::init(uint32_t max_prb, srslte::log *log_h, srslte::log *log_phy_lib_h , chest_feedback_itf *chest_loop)
{
  this->log_h = log_h;
  this->log_phy_lib_h =  log_phy_lib_h;
  this->chest_loop = chest_loop;

  // ue_sync in phy.cc requires a buffer for 3 subframes
  // why do we need a buffer for 3 subframes? 
  // added one CP, allow us to move our fft into CP
  for (uint32_t i=0;i<phy->args->nof_rx_ant;i++) {
    signal_buffer[i] = (cf_t*) srslte_vec_malloc(3 * sizeof(cf_t) * SRSLTE_SF_LEN_PRB(max_prb) + SRSLTE_CP_LEN_EXT(srslte_symbol_sz(max_prb)));
    if (!signal_buffer[i]) {
      Error("Allocating memory\n");
      return false;
    }
  }

  if (srslte_ue_dl_init(&ue_dl, signal_buffer, max_prb, phy->args->nof_rx_ant)) {
    Error("Initiating UE DL\n");
    return false;
  }

  if (srslte_ue_ul_init(&ue_ul, signal_buffer[0], max_prb)) {
    Error("Initiating UE UL\n");
    return false;
  }

  if (srslte_ue_sl_mib_init(&ue_sl, signal_buffer, max_prb)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    return false;
  }

  if(srslte_ue_sl_tx_init(&ue_sl_tx, signal_buffer[0], max_prb)) {
    fprintf(stderr, "Error initiating ue_sl_tx object\n");
    return false;
  }






  srslte_chest_dl_set_rsrp_neighbour(&ue_dl.chest, true);
  srslte_chest_dl_average_subframe(&ue_dl.chest, phy->args->average_subframe_enabled);
  srslte_chest_dl_cfo_estimate_enable(&ue_dl.chest, phy->args->cfo_ref_mask!=0, phy->args->cfo_ref_mask);
  srslte_ue_ul_set_normalization(&ue_ul, true);
  srslte_ue_ul_set_cfo_enable(&ue_ul, true);
  srslte_pdsch_enable_csi(&ue_dl.pdsch, phy->args->pdsch_csi_enabled);

  srslte_ue_sl_tx_set_cfo_enable(&ue_sl_tx, true);

  mem_initiated = true;

  pthread_mutex_init(&mutex, NULL);
  return true;
}

bool phch_worker::set_cell(srslte_cell_t cell_)
{
  bool ret = false;
  pthread_mutex_lock(&mutex);
  if (cell.id != cell_.id || !cell_initiated) {
    memcpy(&cell, &cell_, sizeof(srslte_cell_t));

    if (srslte_ue_dl_set_cell(&ue_dl, cell)) {
      Error("Initiating UE DL\n");
      goto unlock;
    }

    if (srslte_ue_sl_mib_set_cell(&ue_sl, cell)) {
      fprintf(stderr, "Error initaiting UE MIB decoder\n");
      goto unlock;
    }

    if(srslte_ue_sl_tx_set_cell(&ue_sl_tx, cell)) {
      fprintf(stderr, "Error setting cell for ue_sl_tx object\n");
      goto unlock;
    }

    if(srslte_ue_dl_set_mbsfn_area_id(&ue_dl, 1)){
      Error("Setting mbsfn id\n");
      goto unlock;
    }
    
    if (srslte_ue_ul_set_cell(&ue_ul, cell)) {
      Error("Initiating UE UL\n");
      goto unlock;
    }
    srslte_ue_ul_set_normalization(&ue_ul, true);
    srslte_ue_ul_set_cfo_enable(&ue_ul, true);

    srslte_ue_sl_tx_set_cfo_enable(&ue_sl_tx, true);

    cell_initiated = true;
  }
  ret = true;
unlock:
  pthread_mutex_unlock(&mutex);
  return ret;
}

cf_t* phch_worker::get_buffer(uint32_t antenna_idx)
{
  // by shifting our inputbuffer while receiving, we move our fft window into the CP
  return signal_buffer[antenna_idx] + SRSLTE_CP_LEN(srslte_symbol_sz(cell.nof_prb), SRSLTE_CP_NORM_LEN) / 8;
}

void phch_worker::set_tti(uint32_t tti_, uint32_t tx_tti_)
{
  tti    = tti_; 
  tx_tti = tx_tti_;
  log_h->step(tti);
  if (log_phy_lib_h) {
    log_phy_lib_h->step(tti);
  }
}

void phch_worker::set_prach(cf_t *prach_ptr, float prach_power) {
  this->prach_ptr   = prach_ptr;
  this->prach_power = prach_power;
}

void phch_worker::set_cfo(float cfo_)
{
  cfo = cfo_;
}

void phch_worker::set_crnti(uint16_t rnti)
{
  srslte_ue_dl_set_rnti(&ue_dl, rnti);
  srslte_ue_ul_set_rnti(&ue_ul, rnti);
  rnti_is_set = true;
}

float phch_worker::get_ref_cfo()
{
  return srslte_chest_dl_get_cfo(&ue_dl.chest);
}

float phch_worker::get_snr()
{
  return 10*log10(srslte_chest_dl_get_snr(&ue_dl.chest));
}

float phch_worker::get_rsrp()
{
  return 10*log10(srslte_chest_dl_get_rsrp(&ue_dl.chest));
}

float phch_worker::get_noise()
{
  return 10*log10(srslte_chest_dl_get_noise_estimate(&ue_dl.chest));
}


float phch_worker::get_cfo()
{
  return cfo;
}

void phch_worker::work_imp()
{
  if (!cell_initiated) {
    return; 
  }

  pthread_mutex_lock(&mutex);

  Debug("TTI %d running\n", tti);

#ifdef LOG_EXECTIME
  gettimeofday(&logtime_start[1], NULL);
#endif

  tr_log_start();
  
  reset_uci();

  subframe_cfg_t sf_cfg;
  phy->get_sf_config(&sf_cfg, tti);
  Debug("TTI: %d, Subframe type: %s\n", tti, subframe_type_text[sf_cfg.sf_type]);

  bool dl_grant_available = false; 
  bool ul_grant_available = false;
  bool dl_ack[SRSLTE_MAX_CODEWORDS] = {false};

  mac_interface_phy::mac_grant_t    dl_mac_grant;
  mac_interface_phy::tb_action_dl_t dl_action;

  mac_interface_phy::mac_grant_t    ul_mac_grant;
  mac_interface_phy::tb_action_ul_t ul_action;

  ZERO_OBJECT(dl_mac_grant);
  ZERO_OBJECT(dl_action);
  ZERO_OBJECT(ul_mac_grant);
  ZERO_OBJECT(ul_action);

  /** Calculate RSSI on the input signal before generating the output */

  // Average RSSI over all symbols (make sure SF length is non-zero)
  float rssi_dbm = SRSLTE_SF_LEN_PRB(cell.nof_prb) > 0 ? (10*log10(srslte_vec_avg_power_cf(get_buffer(0), SRSLTE_SF_LEN_PRB(cell.nof_prb))) + 30) : 0;
  if (isnormal(rssi_dbm)) {
    phy->avg_rssi_dbm = SRSLTE_VEC_EMA(rssi_dbm, phy->avg_rssi_dbm, phy->args->snr_ema_coeff);
  }

  bool mch_decoded = false;
  srslte_ra_dl_grant_t mch_grant;


  // Call feedback loop for chest
  // if (chest_loop && ((1<<(tti%10)) & phy->args->cfo_ref_mask)) {
  //   chest_loop->set_cfo(srslte_chest_dl_get_cfo(&ue_dl.chest));
  // }
  bool chest_ok = false;
  bool snr_th_ok = false;

  
  /***** Downlink Processing *******/


  /* Run FFT for the slot symbols */
  srslte_ofdm_rx_sf(&ue_sl.fft);

  // in each sync symobol we also decode mib
  if(!phy->args->sidelink_master && (tti%5 == 0)) {
    // get channel estimates for psbch
    int ret = srslte_chest_sl_estimate_psbch(&ue_sl.chest, ue_sl.sf_symbols, ue_sl.ce, SRSLTE_SL_MODE_4);
    if (ret < 0) {
      printf("srslte_chest_sl_estimate_psbch failed with %d", ret);
    }

    // calculate snr psbch
    float snr = 10*log10(ue_sl.chest.pilot_power / ue_sl.chest.noise_estimate);

    phy->snr_psbch = SRSLTE_VEC_EMA(snr, phy->snr_psbch, 0.1);

    uint8_t bch_payload[SRSLTE_SL_BCH_PAYLOAD_LEN];
    /* Decode PSBCH */
    ret = srslte_psbch_decode(&ue_sl.psbch, ue_sl.sf_symbols, ue_sl.ce, ue_sl.chest.noise_estimate, bch_payload);

    if (ret < 0) {
      printf("Error decoding PSBCH (%d) in phch worker snr: %f / %f @tti %d\n", ret, snr, phy->snr_psbch, tti);
      ret = SRSLTE_UE_SL_MIB_NOTFOUND; 
    } else if (ret == 1) {
      uint32_t dfn;
      uint32_t dsfn;
      srslte_cell_t cell;
      srslte_psbch_mib_unpack(bch_payload, &cell, &dfn, &dsfn);

      if(tti != 10*dfn + dsfn) {
        printf("-----------------Decoded differing MIB tti(%d/%d) numbers during camping---------\n", tti, 10*dfn + dsfn);
        // @todo: we need to find a way to either run run_sfn_sync or to update the tti manually
      }

      srslte_ue_sl_mib_reset(&ue_sl);
    } else {
      printf("MIB not decoded: %u in phch worker\n", ue_sl.frame_cnt);
      ue_sl.frame_cnt++;
    }
  }



  dl_grant_available = decode_pscch_dl(&dl_mac_grant); 


  // int32_t decoded_bytes = 0;
  // uint8_t data[100];
  // srslte_ue_sl_pscch_decode(&ue_sl,
  //                           &ue_repo,
  //                           data,
  //                           &decoded_bytes);

  if(dl_grant_available) {
    //printf("dl_grant_available %d\n", my_id);
    /* Send grant to MAC and get action for this TB */
    phy->mac->new_grant_dl(dl_mac_grant, &dl_action);

    /* Decode PDSCH if instructed to do so */
    if (dl_action.decode_enabled[0] || dl_action.decode_enabled[1]) {
      
      // make space for snr insertion
      dl_action.payload_ptr[0] += sizeof(float);

      decode_pssch(&dl_action.phy_grant.sl, dl_action.payload_ptr,
                    dl_action.softbuffers, dl_action.rv, dl_action.rnti,
                    dl_mac_grant.pid, dl_ack);
      
      // calculate snr for possibly decoded pssch
      float snr = 10*log10(ue_sl.chest.pilot_power / ue_sl.chest.noise_estimate);

      int ue_id = srslte_repo_get_t_SL_k(&phy->ue_repo, tti % 10240);

      if(ue_id < 0) {
        // this should not happen
        printf("Decoded in none sidelink subframe.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      } else {
        // calulate exponential moving average
        ue_id = ue_id % 5;
        phy->snr_pssch_per_ue[ue_id] = SRSLTE_VEC_EMA(snr, phy->snr_pssch_per_ue[ue_id], 0.1);
        //printf("moving snr[%d]: %f\n", ue_id, phy->snr_pssch_per_ue[ue_id]);

        // if(phy->args->sidelink_id == ue_id) {
        //   printf("decoded data in my own time slot????? tti: %d\n", tti);
        // }
      }

      // reset pointer and prepend SNR value
      dl_action.payload_ptr[0] -= sizeof(float);
      *((float *)dl_action.payload_ptr[0]) = snr;//ue_sl.chest.noise_estimate;
    }
    
    phy->mac->tb_decoded(dl_ack[0], 0, dl_mac_grant.rnti_type, dl_mac_grant.pid);

  }





  /***** Uplink Processing + Transmission *******/

  bool signal_ready = false;
  cf_t *signal_ptr = NULL;


  // we are the master node and need to send sync sequences
  if(phy->args->sidelink_master && ((tti + HARQ_DELAY_MS) % phy->ue_repo.syncPeriod == phy->ue_repo.syncOffsetIndicator_r12)){

    // clear sf buffer
    bzero(ue_sl_tx.sf_symbols, sizeof(cf_t)*SRSLTE_SF_LEN_RE(ue_sl_tx.cell.nof_prb, ue_sl_tx.cell.cp));
    srslte_psss_put_sf(ue_sl_tx.psss_signal, ue_sl_tx.sf_symbols, cell.nof_prb, SRSLTE_CP_NORM);
    srslte_ssss_put_sf(ue_sl_tx.ssss_signal, ue_sl_tx.sf_symbols, cell.nof_prb, SRSLTE_CP_NORM);

    srslte_refsignal_sl_dmrs_psbch_put(&ue_sl_tx.signals, SRSLTE_SL_MODE_4, ue_sl_tx.psbch_dmrs, ue_sl_tx.sf_symbols);

    srslte_psbch_mib_pack(&cell,
                          ((tti + HARQ_DELAY_MS) % 10240)/10, //sfn,
                          (tti + HARQ_DELAY_MS) % 10,//sf_idx,
                          ue_sl_tx.bch_payload);

    cf_t *sf_tmp[2];
    sf_tmp[0] = ue_sl_tx.sf_symbols;
    sf_tmp[1] = NULL;

    srslte_psbch_encode(&ue_sl_tx.psbch, ue_sl_tx.bch_payload, sf_tmp);

    srslte_ofdm_tx_sf(&ue_sl_tx.fft);

    signal_ready = true;
    signal_ptr = signal_buffer[0];
  }


  int t_SL_k = srslte_repo_get_t_SL_k(&phy->ue_repo, (tti + HARQ_DELAY_MS) % 10240);

  // determine it next transmit tti belongs to ressource pool and we are statically assigned to it
  if(t_SL_k >0 && phy->args->sidelink_id == (t_SL_k%5)) {
  //if(0){

    /* Check if we have UL grant. ul_phy_grant will be overwritten by new grant */
    // ul_grant_available = decode_pdcch_ul(&ul_mac_grant);

    ul_mac_grant.rnti_type = SRSLTE_RNTI_SL_RANDOM;
    ul_mac_grant.tti = t_SL_k;//tti;

    ul_mac_grant.n_bytes[0] = 101;

    // set retransmission gap
    ul_mac_grant.phy_grant.sl.time_gap = 5;

    /* TTI offset for UL */
    ul_action.tti_offset = 5;//HARQ_DELAY_MS;

    phy->mac->new_grant_ul(ul_mac_grant, &ul_action);

    // /* Set UL CFO before transmission */
    // srslte_ue_ul_set_cfo(&ue_ul, cfo);

    //if(0)
    if(ul_action.payload_ptr[0])
    {
      // fill in required values for sci
      srslte_ra_sl_sci_t sci;
      sci.priority = 0;
      sci.resource_reservation = 0;

      sci.time_gap = ul_mac_grant.phy_grant.sl.time_gap;
      sci.rti = ul_action.rv[0] == 0 ? 0 : 1;


      // payload size including mac headers
      uint32_t sdu_size = ul_mac_grant.n_bytes[0]*8;//5*8;// 88; 
      bool found_mcs = false;

      uint8_t L_subch = 1;
      uint8_t n_subCH_start = 0;

      // get valid prb sizes by checking all subchannel sizes
      for(L_subch = 1; L_subch <= phy->ue_repo.rp.numSubchannel_r14; L_subch++) {
        int n_prb = L_subch*phy->ue_repo.rp.sizeSubchannel_r14 - 2;

        // check if number of prb fullfills 2^a2*3^a3*5^a5, see14.1.1.4C
        if(!srslte_dft_precoding_valid_prb(n_prb)) {
          continue;
        }

        for(int mcs=0; mcs<12; mcs++) {
          sci.mcs.idx = mcs;
          srslte_sl_fill_ra_mcs(&sci.mcs, n_prb);

          if(sci.mcs.tbs > sdu_size) {
            found_mcs = true;
            break;
          }
        }

        if(found_mcs) {
          break;
        }
      }


      if(!found_mcs) {
        printf("Failed to aquired a valid MCS for sdu size %d\n", sdu_size);
        //continue;
      }
      
      float c_bits = 9*12*srslte_mod_bits_x_symbol(sci.mcs.mod)*(L_subch*phy->ue_repo.rp.sizeSubchannel_r14 - 2);
      float i_bits = sci.mcs.tbs;

      printf("TBS: %d PRB: %d Channel-Bits: %.0f Coderate: %f (+CRC: %f)\n",
              sci.mcs.tbs,
              (L_subch*phy->ue_repo.rp.sizeSubchannel_r14 - 2),
              c_bits,
              i_bits/c_bits,
              (i_bits+24)/c_bits);
      
      // select random between valid values
      n_subCH_start = rand() % (phy->ue_repo.rp.numSubchannel_r14 - L_subch + 1);

      // set RIV
      srslte_repo_encode_frl(&phy->ue_repo, &sci, L_subch, n_subCH_start);

      uint8_t sci_buffer[SRSLTE_SCI1_MAX_BITS + 16];

      memset(sci_buffer, 0x00, SRSLTE_SCI1_MAX_BITS);

      srslte_repo_sci_encode(&phy->ue_repo, sci_buffer, &sci);

      printf("TTI: %d t_SL_k: %d ENCODED SCI-1: prio: %d rr: 0x%x frl(%d): 0x%x gap: %d mcs: %x rti: %d\n",
              tti,
              t_SL_k,
              sci.priority,
              sci.resource_reservation,
              srslte_repo_frl_len(&phy->ue_repo),
              sci.frl,
              sci.time_gap,
              sci.mcs.idx,
              sci.rti);


      cf_t *ta[2];
      ta[0] = ue_sl_tx.sf_symbols;

      bzero(ue_sl_tx.sf_symbols, sizeof(cf_t)*SRSLTE_SF_LEN_RE(ue_sl_tx.cell.nof_prb, ue_sl_tx.cell.cp));

      if(SRSLTE_SUCCESS != srslte_pscch_encode(&ue_sl_tx.pscch, sci_buffer, phy->ue_repo.rp.startRB_Subchannel_r14 + n_subCH_start*phy->ue_repo.rp.sizeSubchannel_r14, ta)) {
        printf("Failed to encode PSCCH for PRB offset: %d \n", phy->ue_repo.rp.startRB_Subchannel_r14 + n_subCH_start*phy->ue_repo.rp.sizeSubchannel_r14);
        exit(-1);
      }

      srslte_refsignal_sl_dmrs_psxch_put(&ue_sl_tx.signals, SRSLTE_SL_MODE_4,
                                          phy->ue_repo.rp.startRB_Subchannel_r14 + n_subCH_start*phy->ue_repo.rp.sizeSubchannel_r14,
                                          2,
                                          ue_sl_tx.pscch_dmrs, ue_sl_tx.sf_symbols);


      // retrieve crc for pssch dmrs generation
      uint8_t *x = &sci_buffer[SRSLTE_SCI1_MAX_BITS];
      uint16_t pscch_crc = (uint16_t) srslte_bit_pack(&x, 16);

      srslte_vec_fprint_hex(stdout, sci_buffer, SRSLTE_SCI1_MAX_BITS + 16);
      //printf("CRC: %x\n", pscch_crc);



      ue_sl_tx.pssch.n_PSSCH_ssf = 0;//ssf;
      ue_sl_tx.pssch.n_X_ID = pscch_crc;

      // rest softbuffer only on initial transmission, retransmissions will use the same buffer
      // keep in mind, that actually the w_buff used in rate-matching will be persistent and is
      // only populated when RV=0 which is only fulfilled on initial transmission
      if(sci.rti == 0) {
        srslte_softbuffer_tx_reset_tbs(&ul_action.softbuffers[0], (uint32_t) sci.mcs.tbs);
      }

      int encode_ret = srslte_pssch_encode_simple(&ue_sl_tx.pssch,
                            &sci,
                            &ul_action.softbuffers,//softbuffers,//srslte_softbuffer_tx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                            ta,//cf_t *sf_symbols[SRSLTE_MAX_PORTS],
                            phy->ue_repo.rp.startRB_Subchannel_r14 + n_subCH_start*phy->ue_repo.rp.sizeSubchannel_r14 + 2,//repo.rp.startRB_Subchannel_r14 + rbp*repo.rp.sizeSubchannel_r14 + 2,//uint32_t prb_offset,
                            L_subch * phy->ue_repo.rp.sizeSubchannel_r14 - 2,//sci.frl_L_subCH*repo.rp.sizeSubchannel_r14 - 2,//uint32_t n_prb,
                            ul_action.payload_ptr);//uint8_t *data[SRSLTE_MAX_CODEWORDS])


      // generate pssch dmrs
      srslte_refsignal_sl_dmrs_pssch_gen(&ue_sl_tx.signals,
                                          L_subch * phy->ue_repo.rp.sizeSubchannel_r14 - 2,
                                          ue_sl_tx.pssch.n_PSSCH_ssf,
                                          ue_sl_tx.pssch.n_X_ID, // CRC checksum
                                          ue_sl_tx.pssch_dmrs);

      srslte_refsignal_sl_dmrs_psxch_put(&ue_sl_tx.signals, SRSLTE_SL_MODE_4,
                                          phy->ue_repo.rp.startRB_Subchannel_r14 + n_subCH_start*phy->ue_repo.rp.sizeSubchannel_r14 + 2,
                                          L_subch * phy->ue_repo.rp.sizeSubchannel_r14 - 2,
                                          ue_sl_tx.pssch_dmrs, ue_sl_tx.sf_symbols);  


      srslte_ofdm_tx_sf(&ue_sl_tx.fft);

      // apply same cfo which we have to the master node, by doing this, each other node
      // also needs to be cfo-sync to the master to be automatically sync with all nodes
      if (ue_sl_tx.cfo_en) {
        srslte_cfo_correct(&ue_sl_tx.cfo, signal_buffer[0], signal_buffer[0], get_cfo() / srslte_symbol_sz(ue_sl_tx.cell.nof_prb));
      }

      signal_ready = true;
      signal_ptr = signal_buffer[0];
    }
  }



#if 0
  if(SUBFRAME_TYPE_REGULAR == sf_cfg.sf_type) {
    /* Do FFT and extract PDCCH LLR, or quit if no actions are required in this subframe */
    chest_ok = extract_fft_and_pdcch_llr(sf_cfg);

    snr_th_ok = 10*log10(srslte_chest_dl_get_snr(&ue_dl.chest))>1.0;

    if (chest_ok && snr_th_ok) {
      
      /***** Downlink Processing *******/
      
      /* PDCCH DL + PDSCH */
       dl_grant_available = decode_pdcch_dl(&dl_mac_grant); 
       if(dl_grant_available) {
         /* Send grant to MAC and get action for this TB */
         phy->mac->new_grant_dl(dl_mac_grant, &dl_action);

         /* Set DL ACKs to default */
         for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
           dl_ack[tb] = dl_action.default_ack[tb];
         }

         /* Decode PDSCH if instructed to do so */
         if (dl_action.decode_enabled[0] || dl_action.decode_enabled[1]) {
           decode_pdsch(&dl_action.phy_grant.dl, dl_action.payload_ptr,
                         dl_action.softbuffers, dl_action.rv, dl_action.rnti,
                         dl_mac_grant.pid, dl_ack);
         }
         if (dl_action.generate_ack_callback) {
           for (uint32_t tb = 0; tb < SRSLTE_MAX_TB; tb++) {
             if (dl_action.decode_enabled[tb]) {
               phy->mac->tb_decoded(dl_ack[tb], tb, dl_mac_grant.rnti_type, dl_mac_grant.pid);
               dl_ack[tb] = dl_action.generate_ack_callback(dl_action.generate_ack_callback_arg);
               Debug("Calling generate ACK callback for TB %d returned=%d\n", tb, dl_ack[tb]);
             }
           }
         }
         Debug("dl_ack={%d, %d}, generate_ack=%d\n", dl_ack[0], dl_ack[1], dl_action.generate_ack);
         if (dl_action.generate_ack) {
           set_uci_ack(dl_ack, dl_mac_grant.tb_en);
         }
       }
     }
    
  } else if(SUBFRAME_TYPE_MBSFN == sf_cfg.sf_type) {
    srslte_ue_dl_set_non_mbsfn_region(&ue_dl, sf_cfg.non_mbsfn_region_length);

    /* Do FFT and extract PDCCH LLR, or quit if no actions are required in this subframe */
    if (extract_fft_and_pdcch_llr(sf_cfg)) {

      dl_grant_available = decode_pdcch_dl(&dl_mac_grant); 
      phy->mac->new_grant_dl(dl_mac_grant, &dl_action);

      /* Set DL ACKs to default */
      for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
        dl_ack[tb] = dl_action.default_ack[tb];
      }      
      if(sf_cfg.mbsfn_decode) {
      
        mch_grant.sf_type = SRSLTE_SF_MBSFN;
        mch_grant.mcs[0].idx = sf_cfg.mbsfn_mcs;
        mch_grant.tb_en[0] = true;
        for(uint32_t i=1;i<SRSLTE_MAX_CODEWORDS;i++) {
          mch_grant.tb_en[i] = false;
        }
        mch_grant.nof_prb = ue_dl.pmch.cell.nof_prb;
        srslte_dl_fill_ra_mcs(&mch_grant.mcs[0], mch_grant.nof_prb);
        for(int j = 0; j < 2; j++){
          for(uint32_t f = 0; f < mch_grant.nof_prb; f++){
            mch_grant.prb_idx[j][f] = true;
          }
        }
        mch_grant.Qm[0] = srslte_mod_bits_x_symbol(mch_grant.mcs[0].mod);

        /* Get MCH action for this TB */
        phy->mac->new_mch_dl(mch_grant, &dl_action);
        srslte_softbuffer_rx_reset_tbs(dl_action.softbuffers[0], mch_grant.mcs[0].tbs);
        Debug("TBS=%d, Softbuffer max_cb=%d\n", mch_grant.mcs[0].tbs, dl_action.softbuffers[0]->max_cb);
        if(dl_action.decode_enabled[0]) {
          mch_decoded = decode_pmch(&mch_grant, dl_action.payload_ptr[0], dl_action.softbuffers[0], sf_cfg.mbsfn_area_id);
        }
      }
    }
  }

  // Process RAR before UL to enable zero-delay Msg3
  bool rar_delivered = false;
  if (HARQ_DELAY_MS == MSG3_DELAY_MS && dl_mac_grant.rnti_type == SRSLTE_RNTI_RAR) {
    rar_delivered = true;
    phy->mac->tb_decoded(dl_ack[0], 0, dl_mac_grant.rnti_type, dl_mac_grant.pid);
  }
  
  // Decode PHICH 
  bool ul_ack = false;
  bool ul_ack_available = decode_phich(&ul_ack);


  /***** Uplink Processing + Transmission *******/

  bool signal_ready = false;
  cf_t *signal_ptr = NULL;

  /* Transmit PRACH if pending, or PUSCH/PUCCH otherwise */
  if (prach_ptr) {
    signal_ready = true;
    signal_ptr   = prach_ptr;
  } else {
    /* Generate SR if required*/
    set_uci_sr();

    /* Check if we have UL grant. ul_phy_grant will be overwritten by new grant */
    ul_grant_available = decode_pdcch_ul(&ul_mac_grant);

    /* Generate CQI reports if required, note that in case both aperiodic
        and periodic ones present, only aperiodic is sent (36.213 section 7.2) */
    if (ul_grant_available && ul_mac_grant.has_cqi_request) {
      set_uci_aperiodic_cqi();
    } else {
      set_uci_periodic_cqi();
    }

    /* TTI offset for UL */
    ul_action.tti_offset = HARQ_DELAY_MS;

    /* Send UL grant or HARQ information (from PHICH) to MAC */
    if (ul_grant_available         && ul_ack_available)  {
      phy->mac->new_grant_ul_ack(ul_mac_grant, ul_ack, &ul_action);
    } else if (ul_grant_available  && !ul_ack_available) {
      phy->mac->new_grant_ul(ul_mac_grant, &ul_action);
    } else if (!ul_grant_available && ul_ack_available)  {
      phy->mac->harq_recv(tti, ul_ack, &ul_action);
    }

    /* Set UL CFO before transmission */
    srslte_ue_ul_set_cfo(&ue_ul, cfo);

    /* Transmit PUSCH, PUCCH or SRS */
    if (ul_action.tx_enabled) {
      encode_pusch(&ul_action.phy_grant.ul, ul_action.payload_ptr[0], ul_action.current_tx_nb,
                   &ul_action.softbuffers[0], ul_action.rv[0], ul_action.rnti, ul_mac_grant.is_from_rar);
      signal_ready = true;
      if (ul_action.expect_ack) {
        phy->set_pending_ack(TTI_RX_ACK(tti), ue_ul.pusch_cfg.grant.n_prb_tilde[0], ul_action.phy_grant.ul.ncs_dmrs);
      }

    } else if (dl_action.generate_ack || uci_data.scheduling_request || uci_data.uci_cqi_len > 0 || uci_data.uci_ri_len > 0) {
      encode_pucch();
      signal_ready = true;
    } else if (srs_is_ready_to_send()) {
      encode_srs();
      signal_ready = true;
    }
    signal_ptr = signal_buffer[0];
  }
#endif

  tr_log_end();

  if (next_offset > 0) {
    phy->worker_end(tx_tti, signal_ready, signal_ptr, SRSLTE_SF_LEN_PRB(cell.nof_prb)+next_offset, tx_time);
  } else {
    phy->worker_end(tx_tti, signal_ready, &signal_ptr[-next_offset], SRSLTE_SF_LEN_PRB(cell.nof_prb)+next_offset, tx_time);
  }

  // if(SUBFRAME_TYPE_REGULAR == sf_cfg.sf_type) {
  //   if (!dl_action.generate_ack_callback) {
  //     if (dl_mac_grant.rnti_type == SRSLTE_RNTI_PCH && dl_action.decode_enabled[0]) {
  //       if (dl_ack[0]) {
  //         phy->mac->pch_decoded_ok(dl_mac_grant.n_bytes[0]);
  //       }
  //     } else if (!rar_delivered) {
  //       for (uint32_t tb = 0; tb < SRSLTE_MAX_TB; tb++) {
  //         if (dl_action.decode_enabled[tb]) {
  //           phy->mac->tb_decoded(dl_ack[tb], tb, dl_mac_grant.rnti_type, dl_mac_grant.pid);
  //         }
  //       }
  //     }
  //   }
  // } else if (SUBFRAME_TYPE_MBSFN == sf_cfg.sf_type && sf_cfg.mbsfn_decode) {
  //   if(mch_decoded) {
  //     phy->mac->mch_decoded_ok(mch_grant.mcs[0].tbs/8);
  //   } else if(sf_cfg.is_mcch) {
  //     //release lock in phch_common
  //     phy->set_mch_period_stop(0);
  //   }
  // }
  if(SUBFRAME_TYPE_REGULAR == sf_cfg.sf_type){
    update_measurements();
  }

  // if (chest_ok) {
  //   if (phy->avg_rsrp_dbm > -130.0 && phy->avg_snr_db_cqi > -6.0) {
  //     log_h->debug("SNR=%.1f dB, RSRP=%.1f dBm sync=in-sync from channel estimator\n",
  //                  phy->avg_snr_db_cqi, phy->avg_rsrp_dbm);
  //     chest_loop->in_sync();
  //   } else {
  //     log_h->warning("SNR=%.1f dB RSRP=%.1f dBm, sync=out-of-sync from channel estimator\n",
  //                    phy->avg_snr_db_cqi, phy->avg_rsrp_dbm);
  //     chest_loop->out_of_sync();
  //   }
  // }

  pthread_mutex_unlock(&mutex);

  /* Tell the plotting thread to draw the plots */
#ifdef ENABLE_GUI
  if ((int) get_id() == plot_worker_id) {
    sem_post(&plot_sem);    
  }
#endif
}

void phch_worker::compute_ri(uint8_t *ri, uint8_t *pmi, float *sinr) {
  if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_3) {
    if (ue_dl.nof_rx_antennas > 1) {
      /* If 2 ort more receiving antennas, select RI */
      float cn = 0.0f;
      srslte_ue_dl_ri_select(&ue_dl, ri, &cn);
      if (ri) {
        Debug("TM3 RI select %d layers, κ=%fdB\n", (*ri) + 1, cn);
      }
    } else {
      /* If only one receiving antenna, force RI for 1 layer */
      if (ri) {
        *ri = 0;
      }
    }
    uci_data.uci_ri_len = 1;
  } else if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_4) {
    if (sinr) {
      srslte_ue_dl_ri_pmi_select(&ue_dl, ri, pmi, sinr);
      Debug("TM4 ri=%d; pmi=%d; SINR=%.1fdB\n", ue_dl.ri, ue_dl.pmi[ue_dl.ri], 10*log10f(ue_dl.sinr[ue_dl.ri][ue_dl.pmi[ue_dl.ri]]));
    }
  }
}



bool phch_worker::extract_fft_and_pdcch_llr(subframe_cfg_t sf_cfg) {
  bool decode_pdcch = true;

  // Do always channel estimation to keep track of out-of-sync and send measurements to RRC

  // Setup estimator filter
  srslte_chest_dl_set_smooth_filter_gauss(&ue_dl.chest,
                                          phy->args->estimator_fil_order,
                                          phy->args->estimator_fil_stddev);
  srslte_chest_dl_set_smooth_filter_auto(&ue_dl.chest, phy->args->estimator_fil_auto);

  if (!phy->args->snr_estim_alg.compare("refs")) {
    srslte_chest_dl_set_noise_alg(&ue_dl.chest, SRSLTE_NOISE_ALG_REFS);
  } else if (!phy->args->snr_estim_alg.compare("empty")) {
    srslte_chest_dl_set_noise_alg(&ue_dl.chest, SRSLTE_NOISE_ALG_EMPTY);
  } else {
    srslte_chest_dl_set_noise_alg(&ue_dl.chest, SRSLTE_NOISE_ALG_PSS);
  }


    int decode_fft = 0;
    if(SUBFRAME_TYPE_MBSFN == sf_cfg.sf_type) {
      srslte_ue_dl_set_non_mbsfn_region(&ue_dl, sf_cfg.non_mbsfn_region_length);
      decode_fft = srslte_ue_dl_decode_fft_estimate_mbsfn(&ue_dl, tti%10, &cfi, SRSLTE_SF_MBSFN);
    }else{
      decode_fft = srslte_ue_dl_decode_fft_estimate(&ue_dl, tti%10, &cfi);
    }
    if (decode_fft < 0) {
      Error("Getting PDCCH FFT estimate\n");
      return false;
    }

    chest_done = true; 


  if (chest_done && decode_pdcch) { /* and not in DRX mode */
    
    float noise_estimate = phy->avg_noise;
    
    if (!phy->args->equalizer_mode.compare("zf")) {
      noise_estimate = 0; 
    }

    if (srslte_pdcch_extract_llr_multi(&ue_dl.pdcch, ue_dl.sf_symbols_m, ue_dl.ce_m, noise_estimate, tti%10, cfi)) {
      Error("Extracting PDCCH LLR\n");
      return false; 
    }
  }
  return (decode_pdcch || phy->get_pending_ack(tti));
}
  




/********************* Sidelink processing functions ****************************/

bool phch_worker::decode_pscch_dl(srsue::mac_interface_phy::mac_grant_t* grant)
{
  char timestr[64];
  timestr[0]='\0';

  Debug("decode_pscch_dl TTI: %d t_SL: %d\n", tti, phy->ue_repo.subframe_rp[tti]);

  // early completion in case of this subframe is not in resource pool
  if(phy->ue_repo.subframe_rp[tti] == -1) {
    return false;
  }

  uint8_t mdata[SRSLTE_SCI1_MAX_BITS + 16];
  uint16_t crc_rem = 0xdead;
  srslte_ra_sl_sci_t sci;
  cf_t * ce[4];
  srslte_ue_sl_mib_t * q = &ue_sl;


  ce[0] = q->ce;


    // try to decode PSCCH for each subchannel
  for(int rbp=0; rbp < phy->ue_repo.rp.numSubchannel_r14; rbp++) {

    uint32_t prb_offset = phy->ue_repo.rp.startRB_Subchannel_r14 + rbp*phy->ue_repo.rp.sizeSubchannel_r14;

    srslte_chest_sl_estimate_pscch(&q->chest, q->sf_symbols, q->ce, SRSLTE_SL_MODE_4, prb_offset);

    if (srslte_pscch_extract_llr(&q->pscch, q->sf_symbols, 
                        ce,
                        q->chest.noise_estimate,
                        0 %10, prb_offset)) {
      fprintf(stderr, "Error extracting LLRs\n");
      return -1;
    }

    if(SRSLTE_SUCCESS != srslte_pscch_dci_decode(&q->pscch, q->pscch.llr, mdata, q->pscch.max_bits, SRSLTE_SCI1_MAX_BITS, &crc_rem)) {
      continue;
    }

    if(SRSLTE_SUCCESS != srslte_repo_sci_decode(&phy->ue_repo, mdata, &sci)) {
      continue;
    }

    printf("DECODED PSCCH  N_X_ID: %x on tti %d t_SL_k: %d (rbp:%d)\n",
            crc_rem, tti,
            srslte_repo_get_t_SL_k(&phy->ue_repo, tti),
            rbp);

    if(prb_offset != phy->ue_repo.rp.startRB_Subchannel_r14 + sci.frl_n_subCH*phy->ue_repo.rp.sizeSubchannel_r14) {
      printf("Detected different Ressourcepool configurations.\n");
      continue;
    }

    q->pssch.n_X_ID = crc_rem;

    memcpy(&grant->phy_grant.sl, &sci, sizeof(srslte_ra_sl_sci_t));


    /* Fill MAC grant structure */
    grant->ndi[0] = 0;
    grant->ndi[1] = 0;
    grant->n_bytes[0] = sci.mcs.tbs / (uint32_t) 8;
    grant->n_bytes[1] = 0;
    grant->tti = phy->ue_repo.subframe_rp[tti];//tti;
    grant->rv[0] = 0; // @todo change if this is a retransmission
    grant->rv[1] = 0;
    grant->rnti = 0;
    grant->rnti_type = SRSLTE_RNTI_SL_PLACEHOLDER;
    grant->last_tti = 0;
    grant->tb_en[0] = true;
    grant->tb_en[1] = false;
    grant->tb_cw_swap = false;

    // here we select which harq process should handle this tb
    // todo: this may still be wrong, as i assume need at least 16 different processes
    grant->pid = (grant->tti + (8 - sci.rti * sci.time_gap)) % 8;


    // char hexstr[512];
    // hexstr[0]='\0';
    // if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
    //   srslte_vec_sprint_hex(hexstr, sizeof(hexstr), dci_msg.data, dci_msg.nof_bits);
    // }
    // Info("PDCCH: DL DCI %s cce_index=%2d, L=%d, n_data_bits=%d, tpc_pucch=%d, hex=%s\n", srslte_dci_format_string(dci_msg.format),
    //      last_dl_pdcch_ncce, (1<<ue_dl.last_location.L), dci_msg.nof_bits, dci_unpacked.tpc_pucch, hexstr);
    
    return true; 



  }

  // dl_rnti = phy->get_dl_rnti(tti);

  // if (dl_rnti) {
    
  //   srslte_rnti_type_t type = phy->get_dl_rnti_type();

  //   srslte_dci_msg_t dci_msg; 
  //   srslte_ra_dl_dci_t dci_unpacked;

  //   if (type == SRSLTE_RNTI_RAR) {
  //     Debug("Looking for RNTI=0x%x\n", dl_rnti);
  //   }

  //   if (srslte_ue_dl_find_dl_dci_type(&ue_dl, phy->config->dedicated.antenna_info_explicit_value.tx_mode, cfi, tti%10,
  //                                     dl_rnti, type, &dci_msg) != 1) {
  //     if (type == SRSLTE_RNTI_RAR) {
  //       Debug("RAR not found, SNR=%.1f dB, tti=%d, cfi=%d, tx_mode=%d, cell_id=%d\n",
  //            10*log10(srslte_chest_dl_get_snr(&ue_dl.chest)), tti, cfi,
  //            phy->config->dedicated.antenna_info_explicit_value.tx_mode, cell.id);
  //     }
  //     return false;
  //   }
    
  //   if (srslte_dci_msg_to_dl_grant(&dci_msg, dl_rnti, cell.nof_prb, cell.nof_ports, &dci_unpacked, &grant->phy_grant.dl)) {
  //     Error("Converting DCI message to DL grant\n");
  //     return false;   
  //   }

  //   grant->pid = ASYNC_DL_SCHED?dci_unpacked.harq_process:(UL_PIDOF(TTI_TX(tti)));

  //   // Set last TBS for this TB (pid) in case of mcs>28 (7.1.7.2 of 36.213)
  //   for (int i=0;i<SRSLTE_MAX_CODEWORDS;i++) {
  //     if (grant->phy_grant.dl.mcs[i].idx > 28) {
  //       grant->phy_grant.dl.mcs[i].tbs = phy->last_dl_tbs[grant->pid][i];
  //     }
  //     if(grant->phy_grant.dl.mcs[i].tbs < 0) {
  //       Info("Invalid TBS size for PDSCH grant\n");
  //       grant->phy_grant.dl.mcs[i].tbs = 0;
  //     }
  //     // save it
  //     phy->last_dl_tbs[grant->pid][i] = grant->phy_grant.dl.mcs[i].tbs;
  //   }

  //   /* Fill MAC grant structure */
  //   grant->ndi[0] = dci_unpacked.ndi;
  //   grant->ndi[1] = dci_unpacked.ndi_1;
  //   grant->n_bytes[0] = grant->phy_grant.dl.mcs[0].tbs / (uint32_t) 8;
  //   grant->n_bytes[1] = grant->phy_grant.dl.mcs[1].tbs / (uint32_t) 8;
  //   grant->tti = tti;
  //   grant->rv[0] = dci_unpacked.rv_idx;
  //   grant->rv[1] = dci_unpacked.rv_idx_1;
  //   grant->rnti = dl_rnti;
  //   grant->rnti_type = type;
  //   grant->last_tti = 0;
  //   grant->tb_en[0] = dci_unpacked.tb_en[0];
  //   grant->tb_en[1] = dci_unpacked.tb_en[1];
  //   grant->tb_cw_swap = dci_unpacked.tb_cw_swap; // FIXME: tb_cw_swap not supported

  //   last_dl_pdcch_ncce = srslte_ue_dl_get_ncce(&ue_dl);

  //   char hexstr[512];
  //   hexstr[0]='\0';
  //   if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
  //     srslte_vec_sprint_hex(hexstr, sizeof(hexstr), dci_msg.data, dci_msg.nof_bits);
  //   }
  //   Info("PDCCH: DL DCI %s cce_index=%2d, L=%d, n_data_bits=%d, tpc_pucch=%d, hex=%s\n", srslte_dci_format_string(dci_msg.format),
  //        last_dl_pdcch_ncce, (1<<ue_dl.last_location.L), dci_msg.nof_bits, dci_unpacked.tpc_pucch, hexstr);
    
  //   return true; 
  // } else {
  //   return false; 
  // }
  return false; 
}


int phch_worker::decode_pssch(srslte_ra_sl_sci_t *grant, uint8_t *payload[SRSLTE_MAX_CODEWORDS],
                                     srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                     int rv[SRSLTE_MAX_CODEWORDS],
                                     uint16_t rnti, uint32_t harq_pid, bool acks[SRSLTE_MAX_CODEWORDS]) {
  char timestr[64];
  char commonstr[128];
  char tbstr[2][128];
  bool valid_config = true;
  timestr[0]='\0';
  srslte_mimo_type_t mimo_type = SRSLTE_MIMO_TYPE_SINGLE_ANTENNA;
  int ret = SRSLTE_SUCCESS;

  // for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
  //   if (grant->tb_en[tb] && (rv[tb] < 0 || rv[tb] > 3)) {
  //     valid_config = false;
  //     Error("Wrong RV (%d) for TB index %d\n", rv[tb], tb);
  //   }
  // }


  /* Set power allocation according to 3GPP 36.213 clause 5.2 Downlink power allocation */


  Debug("DL Buffer TTI %d: Decoding PDSCH\n", tti);

  // we use this parameter to indicate successful decoding
  acks[0] = false;

  /* Setup PDSCH configuration for this CFI, SFIDX and RVIDX */
  if (valid_config) {
    /*if (!srslte_ue_dl_cfg_grant(&ue_dl, grant, cfi, tti%10, rv, mimo_type)) */
    if(1) {
      // if ((ue_dl.pdsch_cfg.grant.mcs[0].mod > 0 && ue_dl.pdsch_cfg.grant.mcs[0].tbs >= 0) ||
      //     (ue_dl.pdsch_cfg.grant.mcs[1].mod > 0 && ue_dl.pdsch_cfg.grant.mcs[1].tbs >= 0)) {
      if(grant->mcs.mod > 0 && grant->mcs.tbs >0) {
        
        float noise_estimate = srslte_chest_sl_get_noise_estimate(&ue_sl.chest);
        
        if (!phy->args->equalizer_mode.compare("zf")) {
          noise_estimate = 0; 
        }

        /* Set decoder iterations */
        if (phy->args->pdsch_max_its > 0) {
          srslte_pssch_set_max_noi(&ue_sl.pssch, phy->args->pdsch_max_its);
        }

        /** maybe export into seperate function */
        {

          // workaround for copy and paste
          srslte_ue_sl_mib_t *q = &ue_sl;

          uint32_t prb_offset = phy->ue_repo.rp.startRB_Subchannel_r14 + grant->frl_n_subCH*phy->ue_repo.rp.sizeSubchannel_r14;
          cf_t * ce[4];

          ce[0] = q->ce;

          // set parameters for pssch
          //q->pssch.n_X_ID = crc_rem;
          q->pssch.n_PSSCH_ssf = 0; //@todo, make dynamically when sfn is detected
          
          // generate pssch dmrs
          srslte_refsignal_sl_dmrs_pssch_gen(&q->chest.dmrs_signal,
                                              grant->frl_L_subCH*phy->ue_repo.rp.sizeSubchannel_r14 - 2,
                                              q->pssch.n_PSSCH_ssf,
                                              q->pssch.n_X_ID, // CRC checksum
                                              q->chest.pilot_known_signal);


          srslte_chest_sl_estimate_pssch(&q->chest, q->sf_symbols, ce[0], SRSLTE_SL_MODE_4,
                                          prb_offset + 2,
                                          grant->frl_L_subCH*phy->ue_repo.rp.sizeSubchannel_r14 - 2);

          cf_t *_sf_symbols[SRSLTE_MAX_PORTS]; 
          cf_t *_ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];

          _sf_symbols[0] = q->sf_symbols; 
          for (int i=0;i<q->pscch.cell.nof_ports;i++) {
            _ce[i][0] = ce[i]; 
          }

          //srslte_softbuffer_rx_reset_tbs(q->softbuffers[0], (uint32_t) grant->mcs.tbs);

          // // allocate buffer to store decoded data, tbs size in bytes plus crc 
          // uint8_t *data_rx[SRSLTE_MAX_CODEWORDS];
          // data_rx[0] = decoded;
          // memset(data_rx[0], 0xFF, sizeof(uint8_t) * sci.mcs.tbs/8 + 3);


          int decode_ret = srslte_pssch_decode_simple(&q->pssch,
                                                      grant,
                                                      NULL,//&pdsch_cfg,//srslte_pssch_cfg_t *cfg,
                                                      softbuffers,//srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                                      _sf_symbols,//cf_t *sf_symbols[SRSLTE_MAX_PORTS],
                                                      _ce,//_ce,//cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS],
                                                      q->chest.noise_estimate,//float noise_estimate,
                                                      prb_offset + 2,
                                                      grant->frl_L_subCH*phy->ue_repo.rp.sizeSubchannel_r14 - 2,
                                                      payload);//uint8_t *data[SRSLTE_MAX_CODEWORDS],


          if(SRSLTE_SUCCESS == decode_ret) {
            ret = SRSLTE_SUCCESS;
            acks[0] = true;

            srslte_vec_fprint_byte(stdout, payload[0], grant->mcs.tbs/8 + 3);
          } else {
            Error("ERROR: Decoding PDSCH\n");
            printf("ERROR: Decoding PSSCH\n");
          }

        }


        // #ifdef LOG_EXECTIME
        //       struct timeval t[3];
        //       gettimeofday(&t[1], NULL);
        // #endif
        //       ret = srslte_pssch_decode(&ue_dl.pdsch, &ue_dl.pdsch_cfg, softbuffers, ue_dl.sf_symbols_m,
        //                                 ue_dl.ce_m, noise_estimate, rnti, payload, acks);
        //       if (ret) {
        //         Error("ERROR: Decoding PDSCH\n");
        //       }
        // #ifdef LOG_EXECTIME
        //       gettimeofday(&t[2], NULL);
        //       get_time_interval(t);
        //       snprintf(timestr, 64, ", dec_time=%4d us", (int) t[0].tv_usec);
        // #endif

        // char pinfo_str[16] = {0};
        // if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_4) {
        //   snprintf(pinfo_str, 15, ", pinfo=%x", grant->pinfo);
        // }

        // snprintf(commonstr, 128, "PDSCH: l_crb=%2d, harq=%d, snr=%.1f dB, tx_scheme=%s%s", grant->nof_prb, harq_pid,
        //          10 * log10(srslte_chest_dl_get_snr(&ue_dl.chest)), srslte_mimotype2str(mimo_type), pinfo_str);

        // for (int i=0;i<SRSLTE_MAX_CODEWORDS;i++) {
        //   if (grant->tb_en[i]) {
        //     snprintf(tbstr[i], 128, ", CW%d: tbs=%d, mcs=%d, rv=%d, crc=%s, it=%d",
        //              i, grant->mcs[i].tbs/8, grant->mcs[i].idx, rv[i], acks[i] ? "OK" : "KO",
        //              srslte_pdsch_last_noi_cw(&ue_dl.pdsch, i));
        //   }
        // }

        // Info("%s%s%s%s\n", commonstr, grant->tb_en[0]?tbstr[0]:"", grant->tb_en[1]?tbstr[1]:"", timestr);

        // Store metrics
        dl_metrics.mcs    = grant->mcs.idx;
        float niters = srslte_pssch_last_noi(&ue_sl.pssch);
        if (niters) {
          dl_metrics.turbo_iters = niters;
        }
      } else {
        Warning("Received grant for TBS=0\n");
      }
    } else {
      Error("Error configuring DL grant\n");
      ret = SRSLTE_ERROR;
    }
  } else {
    Error("Error invalid DL config\n");
    ret = SRSLTE_ERROR;
  }
  return ret;
}




/********************* Downlink processing functions ****************************/

bool phch_worker::decode_pdcch_dl(srsue::mac_interface_phy::mac_grant_t* grant)
{
  char timestr[64];
  timestr[0]='\0';

  dl_rnti = phy->get_dl_rnti(tti); 
  if (dl_rnti) {
    
    srslte_rnti_type_t type = phy->get_dl_rnti_type();

    srslte_dci_msg_t dci_msg; 
    srslte_ra_dl_dci_t dci_unpacked;

    if (type == SRSLTE_RNTI_RAR) {
      Debug("Looking for RNTI=0x%x\n", dl_rnti);
    }

    if (srslte_ue_dl_find_dl_dci_type(&ue_dl, phy->config->dedicated.antenna_info_explicit_value.tx_mode, cfi, tti%10,
                                      dl_rnti, type, &dci_msg) != 1) {
      if (type == SRSLTE_RNTI_RAR) {
        Debug("RAR not found, SNR=%.1f dB, tti=%d, cfi=%d, tx_mode=%d, cell_id=%d\n",
             10*log10(srslte_chest_dl_get_snr(&ue_dl.chest)), tti, cfi,
             phy->config->dedicated.antenna_info_explicit_value.tx_mode, cell.id);
      }
      return false;
    }
    
    if (srslte_dci_msg_to_dl_grant(&dci_msg, dl_rnti, cell.nof_prb, cell.nof_ports, &dci_unpacked, &grant->phy_grant.dl)) {
      Error("Converting DCI message to DL grant\n");
      return false;   
    }

    grant->pid = ASYNC_DL_SCHED?dci_unpacked.harq_process:(UL_PIDOF(TTI_TX(tti)));

    // Set last TBS for this TB (pid) in case of mcs>28 (7.1.7.2 of 36.213)
    for (int i=0;i<SRSLTE_MAX_CODEWORDS;i++) {
      if (grant->phy_grant.dl.mcs[i].idx > 28) {
        grant->phy_grant.dl.mcs[i].tbs = phy->last_dl_tbs[grant->pid][i];
      }
      if(grant->phy_grant.dl.mcs[i].tbs < 0) {
        Info("Invalid TBS size for PDSCH grant\n");
        grant->phy_grant.dl.mcs[i].tbs = 0;
      }
      // save it
      phy->last_dl_tbs[grant->pid][i] = grant->phy_grant.dl.mcs[i].tbs;
    }

    /* Fill MAC grant structure */
    grant->ndi[0] = dci_unpacked.ndi;
    grant->ndi[1] = dci_unpacked.ndi_1;
    grant->n_bytes[0] = grant->phy_grant.dl.mcs[0].tbs / (uint32_t) 8;
    grant->n_bytes[1] = grant->phy_grant.dl.mcs[1].tbs / (uint32_t) 8;
    grant->tti = tti;
    grant->rv[0] = dci_unpacked.rv_idx;
    grant->rv[1] = dci_unpacked.rv_idx_1;
    grant->rnti = dl_rnti;
    grant->rnti_type = type;
    grant->last_tti = 0;
    grant->tb_en[0] = dci_unpacked.tb_en[0];
    grant->tb_en[1] = dci_unpacked.tb_en[1];
    grant->tb_cw_swap = dci_unpacked.tb_cw_swap; // FIXME: tb_cw_swap not supported

    last_dl_pdcch_ncce = srslte_ue_dl_get_ncce(&ue_dl);

    char hexstr[512];
    hexstr[0]='\0';
    if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
      srslte_vec_sprint_hex(hexstr, sizeof(hexstr), dci_msg.data, dci_msg.nof_bits);
    }
    Info("PDCCH: DL DCI %s cce_index=%2d, L=%d, n_data_bits=%d, tpc_pucch=%d, hex=%s\n", srslte_dci_format_string(dci_msg.format),
         last_dl_pdcch_ncce, (1<<ue_dl.last_location.L), dci_msg.nof_bits, dci_unpacked.tpc_pucch, hexstr);
    
    return true; 
  } else {
    return false; 
  }
}

int phch_worker::decode_pdsch(srslte_ra_dl_grant_t *grant, uint8_t *payload[SRSLTE_MAX_CODEWORDS],
                                     srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS],
                                     int rv[SRSLTE_MAX_CODEWORDS],
                                     uint16_t rnti, uint32_t harq_pid, bool acks[SRSLTE_MAX_CODEWORDS]) {
  char timestr[64];
  char commonstr[128];
  char tbstr[2][128];
  bool valid_config = true;
  timestr[0]='\0';
  srslte_mimo_type_t mimo_type = SRSLTE_MIMO_TYPE_SINGLE_ANTENNA;
  int ret = SRSLTE_SUCCESS;

  for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
    if (grant->tb_en[tb] && (rv[tb] < 0 || rv[tb] > 3)) {
      valid_config = false;
      Error("Wrong RV (%d) for TB index %d\n", rv[tb], tb);
    }
  }

  uint32_t nof_tb = SRSLTE_RA_DL_GRANT_NOF_TB(grant);
  switch(phy->config->dedicated.antenna_info_explicit_value.tx_mode) {
    /* Implemented Tx Modes */
    case LIBLTE_RRC_TRANSMISSION_MODE_1:
      mimo_type = SRSLTE_MIMO_TYPE_SINGLE_ANTENNA;
      if (nof_tb != 1) {
        Error("Wrong number of transport blocks (%d) for single antenna.", nof_tb);
        valid_config = false;
      }
      break;
    case LIBLTE_RRC_TRANSMISSION_MODE_2:
      if (cell.nof_ports > 1) {
        mimo_type = SRSLTE_MIMO_TYPE_TX_DIVERSITY;
      } else {
        mimo_type = SRSLTE_MIMO_TYPE_SINGLE_ANTENNA;
      }
      if (nof_tb != 1) {
        Error("Wrong number of transport blocks (%d) for transmit diversity.", nof_tb);
        valid_config = false; 
      }
      break;
    case LIBLTE_RRC_TRANSMISSION_MODE_3:
      if (nof_tb == 1) {
        mimo_type = SRSLTE_MIMO_TYPE_TX_DIVERSITY;
      } else if (ue_dl.nof_rx_antennas > 1 && nof_tb == 2) {
        mimo_type = SRSLTE_MIMO_TYPE_CDD;
      } else {
        Error("Wrong combination of antennas (%d) or transport blocks (%d) for TM3\n", ue_dl.nof_rx_antennas,
              nof_tb);
        valid_config = false;
      }
      break;
    case LIBLTE_RRC_TRANSMISSION_MODE_4:
      if (nof_tb == 1) {
        mimo_type = (grant->pinfo == 0) ? SRSLTE_MIMO_TYPE_TX_DIVERSITY : SRSLTE_MIMO_TYPE_SPATIAL_MULTIPLEX;
      } else if (ue_dl.nof_rx_antennas > 1 && nof_tb == 2) {
        mimo_type = SRSLTE_MIMO_TYPE_SPATIAL_MULTIPLEX;
      } else {
        Error("Wrong combination of antennas (%d) or transport blocks (%d) for TM3\n", ue_dl.nof_rx_antennas,
              nof_tb);
        valid_config = false;
      }
    break;

    /* Not implemented cases */
    case LIBLTE_RRC_TRANSMISSION_MODE_5:
    case LIBLTE_RRC_TRANSMISSION_MODE_6:
    case LIBLTE_RRC_TRANSMISSION_MODE_7:
    case LIBLTE_RRC_TRANSMISSION_MODE_8:
      Error("Not implemented Tx mode (%d)\n", phy->config->dedicated.antenna_info_explicit_value.tx_mode);
      break;

    /* Error cases */
    case LIBLTE_RRC_TRANSMISSION_MODE_N_ITEMS:
    default:
      Error("Wrong Tx mode (%d)\n", phy->config->dedicated.antenna_info_explicit_value.tx_mode);
      valid_config = false;
  }

  /* Set power allocation according to 3GPP 36.213 clause 5.2 Downlink power allocation */
  float rho_a = 1.0f, rho_b = 1.0f;
  if (phy->config->dedicated.pdsch_cnfg_ded < LIBLTE_RRC_PDSCH_CONFIG_P_A_N_ITEMS) {
    float rho_a_db = liblte_rrc_pdsch_config_p_a_num[(int) phy->config->dedicated.pdsch_cnfg_ded];
    rho_a = powf(10.0f, rho_a_db / 20.0f) * ((cell.nof_ports == 1) ? 1.0f : sqrtf(2.0f));
  }
  if (phy->config->common.pdsch_cnfg.p_b < 4) {
    uint32_t idx0 = (cell.nof_ports == 1) ? 0 : 1;
    float cell_specific_ratio = pdsch_cfg_cell_specific_ratio_table[idx0][phy->config->common.pdsch_cnfg.p_b];
    rho_b = sqrtf(cell_specific_ratio);
  }
  srslte_ue_dl_set_power_alloc(&ue_dl, rho_a, rho_b);

  Debug("DL Buffer TTI %d: Decoding PDSCH\n", tti);

  /* Setup PDSCH configuration for this CFI, SFIDX and RVIDX */
  if (valid_config) {
    if (!srslte_ue_dl_cfg_grant(&ue_dl, grant, cfi, tti%10, rv, mimo_type)) {
      if ((ue_dl.pdsch_cfg.grant.mcs[0].mod > 0 && ue_dl.pdsch_cfg.grant.mcs[0].tbs >= 0) ||
          (ue_dl.pdsch_cfg.grant.mcs[1].mod > 0 && ue_dl.pdsch_cfg.grant.mcs[1].tbs >= 0)) {
        
        float noise_estimate = srslte_chest_dl_get_noise_estimate(&ue_dl.chest);
        
        if (!phy->args->equalizer_mode.compare("zf")) {
          noise_estimate = 0; 
        }
        
        /* Set decoder iterations */
        if (phy->args->pdsch_max_its > 0) {
          srslte_pdsch_set_max_noi(&ue_dl.pdsch, phy->args->pdsch_max_its);
        }


  #ifdef LOG_EXECTIME
        struct timeval t[3];
        gettimeofday(&t[1], NULL);
  #endif
        ret = srslte_pdsch_decode(&ue_dl.pdsch, &ue_dl.pdsch_cfg, softbuffers, ue_dl.sf_symbols_m,
                                  ue_dl.ce_m, noise_estimate, rnti, payload, acks);
        if (ret) {
          Error("ERROR: Decoding PDSCH\n");
        }
  #ifdef LOG_EXECTIME
        gettimeofday(&t[2], NULL);
        get_time_interval(t);
        snprintf(timestr, 64, ", dec_time=%4d us", (int) t[0].tv_usec);
  #endif

        char pinfo_str[16] = {0};
        if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_4) {
          snprintf(pinfo_str, 15, ", pinfo=%x", grant->pinfo);
        }

        snprintf(commonstr, 128, "PDSCH: l_crb=%2d, harq=%d, snr=%.1f dB, tx_scheme=%s%s", grant->nof_prb, harq_pid,
                 10 * log10(srslte_chest_dl_get_snr(&ue_dl.chest)), srslte_mimotype2str(mimo_type), pinfo_str);

        for (int i=0;i<SRSLTE_MAX_CODEWORDS;i++) {
          if (grant->tb_en[i]) {
            snprintf(tbstr[i], 128, ", CW%d: tbs=%d, mcs=%d, rv=%d, crc=%s, it=%d",
                     i, grant->mcs[i].tbs/8, grant->mcs[i].idx, rv[i], acks[i] ? "OK" : "KO",
                     srslte_pdsch_last_noi_cw(&ue_dl.pdsch, i));
          }
        }

        Info("%s%s%s%s\n", commonstr, grant->tb_en[0]?tbstr[0]:"", grant->tb_en[1]?tbstr[1]:"", timestr);

        // Store metrics
        dl_metrics.mcs    = grant->mcs[0].idx;
        float niters = srslte_pdsch_last_noi(&ue_dl.pdsch);
        if (niters) {
          dl_metrics.turbo_iters = niters;
        }
      } else {
        Warning("Received grant for TBS=0\n");
      }
    } else {
      Error("Error configuring DL grant\n");
      ret = SRSLTE_ERROR;
    }
  } else {
    Error("Error invalid DL config\n");
    ret = SRSLTE_ERROR;
  }
  return ret;
}

bool phch_worker::decode_pmch(srslte_ra_dl_grant_t *grant, uint8_t *payload,
                              srslte_softbuffer_rx_t* softbuffer, uint16_t mbsfn_area_id)
{
  char timestr[64];
  timestr[0]='\0';

  Debug("DL Buffer TTI %d: Decoding PMCH\n", tti);
  /* Setup PMCH configuration */
  srslte_ue_dl_set_mbsfn_area_id(&ue_dl, mbsfn_area_id);

  if (!srslte_ue_dl_cfg_grant(&ue_dl, grant, cfi, tti%10, SRSLTE_PMCH_RV, SRSLTE_MIMO_TYPE_SINGLE_ANTENNA)) {
    if (ue_dl.pmch_cfg.grant.mcs[0].mod > 0 && ue_dl.pmch_cfg.grant.mcs[0].tbs >= 0) {

    Debug("Decoding PMCH SF: %d, MBSFN area ID: 0x%x, Mod %s, TBS: %d, NofSymbols: %d, NofBitsE: %d, rv_idx: %d, C_prb=%d, cfi=%d\n",
        ue_dl.pmch_cfg.sf_idx, mbsfn_area_id, srslte_mod_string(ue_dl.pmch_cfg.grant.mcs[0].mod), ue_dl.pmch_cfg.grant.mcs[0].tbs, ue_dl.pmch_cfg.nbits[0].nof_re,
         ue_dl.pmch_cfg.nbits[0].nof_bits, 0, ue_dl.pmch_cfg.grant.nof_prb, ue_dl.pmch_cfg.nbits[0].lstart-1);

      float noise_estimate = srslte_chest_dl_get_noise_estimate(&ue_dl.chest);

      if (!phy->args->equalizer_mode.compare("zf")) {
        noise_estimate = 0;
      }

      /* Set decoder iterations */
      // TODO: Add separate arg for pmch_max_its
      if (phy->args->pdsch_max_its > 0) {
        srslte_sch_set_max_noi(&ue_dl.pmch.dl_sch, phy->args->pdsch_max_its);
      }

#ifdef LOG_EXECTIME
      struct timeval t[3];
      gettimeofday(&t[1], NULL);
#endif

      bool ack = srslte_pmch_decode_multi(&ue_dl.pmch, &ue_dl.pmch_cfg, softbuffer, ue_dl.sf_symbols_m,
                                    ue_dl.ce_m, noise_estimate, mbsfn_area_id, payload) == 0;
  
#ifdef LOG_EXECTIME
      gettimeofday(&t[2], NULL);
      get_time_interval(t);
      snprintf(timestr, 64, ", dec_time=%4d us", (int) t[0].tv_usec);
#endif

      Info("PMCH: l_crb=%2d, tbs=%d, mcs=%d, crc=%s, snr=%.1f dB, n_iter=%d%s\n",
            grant->nof_prb,
            grant->mcs[0].tbs/8, grant->mcs[0].idx,
            ack?"OK":"KO",
            10*log10(srslte_chest_dl_get_snr(&ue_dl.chest)),
            srslte_pmch_last_noi(&ue_dl.pmch),
            timestr);

      //printf("tti=%d, cfo=%f\n", tti, cfo*15000);
      //srslte_vec_save_file("pdsch", signal_buffer, sizeof(cf_t)*SRSLTE_SF_LEN_PRB(cell.nof_prb));

      // Store metrics
      dl_metrics.mcs = grant->mcs[0].idx;

      return ack;
    } else {
      Warning("Received grant for TBS=0\n");
    }
  } else {
    Error("Error configuring DL grant\n");
  }
  return true;
}

bool phch_worker::decode_phich(bool *ack)
{
  uint32_t I_lowest, n_dmrs; 
  if (phy->get_pending_ack(tti, &I_lowest, &n_dmrs)) {
    if (ack) {
      *ack = srslte_ue_dl_decode_phich(&ue_dl, tti%10, I_lowest, n_dmrs);     
      Info("PHICH: hi=%d, I_lowest=%d, n_dmrs=%d\n", *ack, I_lowest, n_dmrs);
    }
    phy->reset_pending_ack(tti);
    return true; 
  } else {
    return false; 
  }
}




/********************* Uplink processing functions ****************************/

bool phch_worker::decode_pdcch_ul(mac_interface_phy::mac_grant_t* grant)
{
  char timestr[64];
  timestr[0]='\0';

  phy->reset_pending_ack(TTI_RX_ACK(tti));

  srslte_dci_msg_t dci_msg; 
  srslte_ra_ul_dci_t dci_unpacked;
  srslte_dci_rar_grant_t rar_grant;
  srslte_rnti_type_t type = phy->get_ul_rnti_type();
  
  bool ret = false; 
  if (phy->get_pending_rar(tti, &rar_grant)) {

    if (srslte_dci_rar_to_ul_grant(&rar_grant, cell.nof_prb, pusch_hopping.hopping_offset, 
      &dci_unpacked, &grant->phy_grant.ul)) 
    {
      Error("Converting RAR message to UL grant\n");
      return false; 
    }
    grant->rnti_type = SRSLTE_RNTI_TEMP;
    grant->is_from_rar = true; 
    grant->has_cqi_request = false; // In contention-based Random Access CQI request bit is reserved
    Debug("RAR grant found for TTI=%d\n", tti);
    ret = true;  
  } else {
    ul_rnti = phy->get_ul_rnti(tti);
    if (ul_rnti) {
      if (srslte_ue_dl_find_ul_dci(&ue_dl, cfi, tti%10, ul_rnti, &dci_msg) != 1) {
        return false;
      }
      
      if (srslte_dci_msg_to_ul_grant(&dci_msg, cell.nof_prb, pusch_hopping.hopping_offset, 
        &dci_unpacked, &grant->phy_grant.ul, tti)) 
      {
        Error("Converting DCI message to UL grant\n");
        return false;   
      }
      grant->rnti_type = type; 
      grant->is_from_rar = false;
      grant->has_cqi_request = dci_unpacked.cqi_request;
      ret = true; 
      
      char hexstr[512];
      hexstr[0]='\0';
      if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
        srslte_vec_sprint_hex(hexstr, sizeof(hexstr), dci_msg.data, dci_msg.nof_bits);
      }
      // Change to last_location_ul
      Info("PDCCH: UL DCI Format0  cce_index=%d, L=%d, n_data_bits=%d, tpc_pusch=%d, hex=%s\n",
           ue_dl.last_location_ul.ncce, (1<<ue_dl.last_location_ul.L), dci_msg.nof_bits, dci_unpacked.tpc_pusch, hexstr);
      
      if (grant->phy_grant.ul.mcs.tbs==0) {
        Info("Received PUSCH grant with empty data\n");
      }
    }
  }


  // Handle Format0 adaptive retx
  if (ret) {
    // Use last TBS for this TB in case of mcs>28
    if (grant->phy_grant.ul.mcs.idx > 28 && grant->phy_grant.ul.mcs.mod == SRSLTE_MOD_LAST) {
      // Make sure we received a grant in the previous TTI for this PID
      grant->phy_grant.ul.mcs.tbs = phy->last_ul_tbs[UL_PIDOF(TTI_TX(tti))];
      grant->phy_grant.ul.mcs.mod = phy->last_ul_mod[UL_PIDOF(TTI_TX(tti))];
      grant->phy_grant.ul.mcs.idx = phy->last_ul_idx[UL_PIDOF(TTI_TX(tti))];
      grant->phy_grant.ul.Qm      = srslte_mod_bits_x_symbol(grant->phy_grant.ul.mcs.mod);
    }
  }
  if (ret) {
    phy->last_ul_tbs[UL_PIDOF(TTI_TX(tti))] = grant->phy_grant.ul.mcs.tbs;
    phy->last_ul_mod[UL_PIDOF(TTI_TX(tti))] = grant->phy_grant.ul.mcs.mod;
    phy->last_ul_idx[UL_PIDOF(TTI_TX(tti))] = grant->phy_grant.ul.mcs.idx;
    phy->last_ul_tti[UL_PIDOF(TTI_TX(tti))] = TTI_RX_ACK(tti);
    /* Limit UL modulation if not supported by the UE or disabled by higher layers */
    if (!phy->config->enable_64qam) {
      if (grant->phy_grant.ul.mcs.mod >= SRSLTE_MOD_64QAM) {
        grant->phy_grant.ul.mcs.mod = SRSLTE_MOD_16QAM;
        grant->phy_grant.ul.Qm      = 4;
      }
    }
  }

  /* Make sure the grant is valid */
  if (ret && !srslte_dft_precoding_valid_prb(grant->phy_grant.ul.L_prb) && grant->phy_grant.ul.L_prb <= cell.nof_prb) {
    Warning("Received invalid UL grant. L=%d\n", grant->phy_grant.ul.L_prb);
    ret = false; 
  }
  
  if (ret) {    
    grant->ndi[0] = dci_unpacked.ndi;
    grant->pid = 0; // This is computed by MAC from TTI 
    grant->n_bytes[0] = grant->phy_grant.ul.mcs.tbs / (uint32_t) 8;
    grant->tti = tti; 
    grant->rnti = ul_rnti; 
    grant->rv[0] = dci_unpacked.rv_idx;
    if (SRSLTE_VERBOSE_ISINFO()) {
      srslte_ra_pusch_fprint(stdout, &dci_unpacked, cell.nof_prb);
    }
  }

  return ret;
}

void phch_worker::reset_uci()
{
  ZERO_OBJECT(uci_data);
  ZERO_OBJECT(cqi_report);
}

void phch_worker::set_uci_ack(bool ack[SRSLTE_MAX_CODEWORDS], bool tb_en[SRSLTE_MAX_CODEWORDS])
{
  /* Map ACK according to 3GPP 36.212 clause 5.2.3.1 */
  uint32_t nof_ack = 0;
  for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
    if (tb_en[tb]) {
      ((nof_ack == 0)?uci_data.uci_ack:uci_data.uci_ack_2) = (uint8_t)(ack[tb]?1:0);
      nof_ack++;
    }
  }
  uci_data.uci_ack_len = nof_ack;
}

void phch_worker::set_uci_sr()
{
  uci_data.scheduling_request = false; 
  if (phy->sr_enabled && sr_configured) {
    uint32_t sr_tx_tti = TTI_TX(tti);
    // Get I_sr parameter   
    if (srslte_ue_ul_sr_send_tti(I_sr, sr_tx_tti)) {
      Info("PUCCH: SR transmission at TTI=%d, I_sr=%d\n", sr_tx_tti, I_sr);
      uci_data.scheduling_request = true; 
      phy->sr_last_tx_tti = sr_tx_tti; 
      phy->sr_enabled = false;
    }
  } 
}

void phch_worker::set_uci_periodic_cqi()
{
  int cqi_fixed     = phy->args->cqi_fixed;
  int cqi_max       = phy->args->cqi_max;

  float sinr = ue_dl.sinr[phy->last_ri & SRSLTE_MAX_LAYERS][phy->last_pmi % SRSLTE_MAX_CODEBOOKS];

  if (period_cqi.configured && rnti_is_set) {
    if (period_cqi.ri_idx_present && srslte_ri_send(period_cqi.pmi_idx, period_cqi.ri_idx, TTI_TX(tti))) {
      /* Compute RI, PMI and SINR */
      compute_ri(&phy->last_ri, &phy->last_pmi, &sinr);
      uci_data.uci_ri = phy->last_ri;
      uci_data.uci_ri_len = 1;
      uci_data.ri_periodic_report = true;
      Debug("PUCCH: Periodic ri=%d, SINR=%.1f\n", phy->last_ri, sinr);
    } else if (srslte_cqi_send(period_cqi.pmi_idx, TTI_TX(tti))) {
      compute_ri(NULL, NULL, NULL);
      phy->last_pmi = (uint8_t) ue_dl.pmi[phy->last_ri % SRSLTE_MAX_LAYERS];

      ZERO_OBJECT(cqi_report);

      if (period_cqi.format_is_subband) {
        // TODO: Implement subband periodic reports
        cqi_report.type = SRSLTE_CQI_TYPE_SUBBAND;
        cqi_report.subband.subband_cqi = srslte_cqi_from_snr(phy->avg_snr_db_cqi);
        cqi_report.subband.subband_label = 0;
        log_h->console("Warning: Subband CQI periodic reports not implemented\n");
        Debug("PUCCH: Periodic CQI=%d, SNR=%.1f dB\n", cqi_report.subband.subband_cqi, phy->avg_snr_db_cqi);
      } else {
        cqi_report.type = SRSLTE_CQI_TYPE_WIDEBAND;
        if (cqi_fixed >= 0) {
          cqi_report.wideband.wideband_cqi = cqi_fixed;
        } else {
          cqi_report.wideband.wideband_cqi = srslte_cqi_from_snr(phy->avg_snr_db_cqi);      
        }
        if (cqi_max >= 0 && cqi_report.wideband.wideband_cqi > cqi_max) {
          cqi_report.wideband.wideband_cqi = cqi_max; 
        }
        if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_4) {
          cqi_report.wideband.pmi_present = true;
          cqi_report.wideband.pmi = phy->last_pmi;
          cqi_report.wideband.rank_is_not_one = (phy->last_ri != 0);
        }
        Debug("PUCCH: Periodic CQI=%d, SNR=%.1f dB\n", cqi_report.wideband.wideband_cqi, phy->avg_snr_db_cqi);
      }
      uci_data.uci_cqi_len = (uint32_t) srslte_cqi_value_pack(&cqi_report, uci_data.uci_cqi);
      uci_data.uci_ri = phy->last_ri;
      uci_data.uci_ri_len = 0;
      rar_cqi_request = false;       
    }
  }
}

void phch_worker::set_uci_aperiodic_cqi()
{
  float sinr_db = ue_dl.sinr[phy->last_ri % SRSLTE_MAX_LAYERS][phy->last_pmi%SRSLTE_MAX_CODEBOOKS];

  if (phy->config->dedicated.cqi_report_cnfg.report_mode_aperiodic_present) {
    /* Compute RI, PMI and SINR */
    compute_ri(&phy->last_ri, &phy->last_pmi, &sinr_db);

    switch(phy->config->dedicated.cqi_report_cnfg.report_mode_aperiodic) {
      case LIBLTE_RRC_CQI_REPORT_MODE_APERIODIC_RM30:
        /* only Higher Layer-configured subband feedback support right now, according to TS36.213 section 7.2.1
          - A UE shall report a wideband CQI value which is calculated assuming transmission on set S subbands
          - The UE shall also report one subband CQI value for each set S subband. The subband CQI
            value is calculated assuming transmission only in the subband
          - Both the wideband and subband CQI represent channel quality for the first codeword,
            even when RI>1
          - For transmission mode 3 the reported CQI values are calculated conditioned on the
            reported RI. For other transmission modes they are reported conditioned on rank 1.
        */
        if (rnti_is_set) {
          ZERO_OBJECT(cqi_report);

          cqi_report.type = SRSLTE_CQI_TYPE_SUBBAND_HL;
          cqi_report.subband_hl.wideband_cqi_cw0 = srslte_cqi_from_snr(phy->avg_snr_db_cqi);

          // TODO: implement subband CQI properly
          cqi_report.subband_hl.subband_diff_cqi_cw0 = 0; // Always report zero offset on all subbands
          cqi_report.subband_hl.N = (cell.nof_prb > 7) ? (uint32_t) srslte_cqi_hl_get_no_subbands(cell.nof_prb) : 0;

          int cqi_len = srslte_cqi_value_pack(&cqi_report, uci_data.uci_cqi);
          if (cqi_len < 0) {
            Error("Error packing CQI value (Aperiodic reporting mode RM30).");
            return;
          }
          uci_data.uci_cqi_len = (uint32_t) cqi_len;

          char cqi_str[SRSLTE_CQI_STR_MAX_CHAR] = {0};
          srslte_cqi_to_str(uci_data.uci_cqi, uci_data.uci_cqi_len, cqi_str, SRSLTE_CQI_STR_MAX_CHAR);

          /* Set RI = 1 */
          if (phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_3 ||
              phy->config->dedicated.antenna_info_explicit_value.tx_mode == LIBLTE_RRC_TRANSMISSION_MODE_4) {
            uci_data.uci_ri = phy->last_ri;
            uci_data.uci_ri_len = 1;
          } else {
            uci_data.uci_ri_len = 0;
          }

          Info("PUSCH: Aperiodic RM30 CQI=%s, %sSNR=%.1f dB, for %d subbands\n",
               cqi_str, (uci_data.uci_ri_len)?((uci_data.uci_ri == 0)?"ri=0, ":"ri=1, "):"", phy->avg_snr_db_cqi, cqi_report.subband_hl.N);
        }
        break;
      case LIBLTE_RRC_CQI_REPORT_MODE_APERIODIC_RM31:
        /* only Higher Layer-configured subband feedback support right now, according to TS36.213 section 7.2.1
          - A single precoding matrix is selected from the codebook subset assuming transmission on set S subbands
          - A UE shall report one subband CQI value per codeword for each set S subband which are calculated assuming
            the use of the single precoding matrix in all subbands and assuming transmission in the corresponding
            subband.
          - A UE shall report a wideband CQI value per codeword which is calculated assuming the use of the single
            precoding matrix in all subbands and transmission on set S subbands
          - The UE shall report the single selected precoding matrix indicator.
          - For transmission mode 4 the reported PMI and CQI values are calculated conditioned on the reported RI. For
            other transmission modes they are reported conditioned on rank 1.
        */
        if (rnti_is_set) {
          /* Fill CQI Report */
          srslte_cqi_value_t cqi_report;
          ZERO_OBJECT(cqi_report);

          cqi_report.type = SRSLTE_CQI_TYPE_SUBBAND_HL;

          cqi_report.subband_hl.wideband_cqi_cw0 = srslte_cqi_from_snr(sinr_db);
          cqi_report.subband_hl.subband_diff_cqi_cw0 = 0; // Always report zero offset on all subbands

          if (phy->last_ri > 0) {
            cqi_report.subband_hl.rank_is_not_one = true;
            cqi_report.subband_hl.wideband_cqi_cw1 = srslte_cqi_from_snr(sinr_db);
            cqi_report.subband_hl.subband_diff_cqi_cw1 = 0; // Always report zero offset on all subbands
          }

          cqi_report.subband_hl.pmi = phy->last_pmi;
          cqi_report.subband_hl.pmi_present = true;
          cqi_report.subband_hl.four_antenna_ports = (cell.nof_ports == 4);

          // TODO: implement subband CQI properly
          cqi_report.subband_hl.N = (uint32_t) ((cell.nof_prb > 7) ? srslte_cqi_hl_get_no_subbands(cell.nof_prb) : 0);

          int cqi_len = srslte_cqi_value_pack(&cqi_report, uci_data.uci_cqi);
          if (cqi_len < 0) {
            Error("Error packing CQI value (Aperiodic reporting mode RM31).");
            return;
          }
          uci_data.uci_cqi_len = (uint32_t) cqi_len;
          uci_data.uci_ri_len = 1;
          uci_data.uci_ri = phy->last_ri;

          char cqi_str[SRSLTE_CQI_STR_MAX_CHAR] = {0};
          srslte_cqi_to_str(uci_data.uci_cqi, uci_data.uci_cqi_len, cqi_str, SRSLTE_CQI_STR_MAX_CHAR);

          if (cqi_report.subband_hl.rank_is_not_one) {
            Info("PUSCH: Aperiodic RM31 ri~1, CQI=%02d/%02d, SINR=%2.1f/%2.1fdB, pmi=%d for %d subbands\n",
                 cqi_report.subband_hl.wideband_cqi_cw0, cqi_report.subband_hl.wideband_cqi_cw1,
                 sinr_db, sinr_db, phy->last_pmi, cqi_report.subband_hl.N);
          } else {
            Info("PUSCH: Aperiodic RM31 ri=1, CQI=%02d, SINR=%2.1f, pmi=%d for %d subbands\n",
                 cqi_report.subband_hl.wideband_cqi_cw0,
                 sinr_db, phy->last_pmi, cqi_report.subband_hl.N);
          }
        }
        break;
      default:
        Warning("Received CQI request but mode %s is not supported\n", 
                liblte_rrc_cqi_report_mode_aperiodic_text[phy->config->dedicated.cqi_report_cnfg.report_mode_aperiodic]);
        break;
    }
  } else {
    Warning("Received CQI request but aperiodic mode is not configured\n");    
  }
}

bool phch_worker::srs_is_ready_to_send() {
  if (srs_cfg.configured) {
    if (srslte_refsignal_srs_send_cs(srs_cfg.subframe_config, TTI_TX(tti)%10) == 1 &&
        srslte_refsignal_srs_send_ue(srs_cfg.I_srs, TTI_TX(tti))              == 1)
    {
      return true; 
    }
  }
  return false; 
}

void phch_worker::set_tx_time(srslte_timestamp_t _tx_time, uint32_t next_offset)
{
  this->next_offset = next_offset;
  memcpy(&tx_time, &_tx_time, sizeof(srslte_timestamp_t));
}

void phch_worker::encode_pusch(srslte_ra_ul_grant_t *grant, uint8_t *payload, uint32_t current_tx_nb, 
                               srslte_softbuffer_tx_t* softbuffer, uint32_t rv, uint16_t rnti, bool is_from_rar)
{
  char timestr[64];
  timestr[0]='\0';
  
  if (srslte_ue_ul_cfg_grant(&ue_ul, grant, TTI_TX(tti), rv, current_tx_nb)) {
    Error("Configuring UL grant\n");
  }

  if (srslte_ue_ul_pusch_encode_rnti_softbuffer(&ue_ul, 
                                                payload, uci_data, 
                                                softbuffer,
                                                rnti, 
                                                signal_buffer[0])) 
  {
    Error("Encoding PUSCH\n");
  }
    
  float p0_preamble = 0; 
  if (is_from_rar) {
    p0_preamble = phy->p0_preamble;
  }
  float tx_power = srslte_ue_ul_pusch_power(&ue_ul, phy->pathloss, p0_preamble);
  float gain = set_power(tx_power);

  // Save PUSCH power for PHR calculation  
  phy->cur_pusch_power = tx_power; 
  
#ifdef LOG_EXECTIME
  gettimeofday(&logtime_start[2], NULL);
  get_time_interval(logtime_start);
  snprintf(timestr, 64, ", tot_time=%4d us", (int) logtime_start[0].tv_usec);
#endif

  char cqi_str[SRSLTE_CQI_STR_MAX_CHAR] = "";
  if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
    srslte_cqi_value_tostring(&cqi_report, cqi_str, SRSLTE_CQI_STR_MAX_CHAR);
  }

  uint8_t dummy[2] = {0,0};
  log_h->info_hex(payload, grant->mcs.tbs/8,
              "PUSCH: tti_tx=%d, amp=%.2f, alloc=(%d,%d), tbs=%d, mcs=%d, rv=%d%s%s%s, cfo=%.1f KHz%s%s\n",
              (tti + HARQ_DELAY_MS) % 10240, srslte_ue_ul_get_last_amplitude(&ue_ul),
              grant->n_prb[0], grant->n_prb[0] + grant->L_prb,
              grant->mcs.tbs / 8, grant->mcs.idx, rv,
              uci_data.uci_ack_len > 0 ? (uci_data.uci_ack ? ", ack=1" : ", ack=0") : "",
              uci_data.uci_ack_len > 1 ? (uci_data.uci_ack_2 ? "1" : "0") : "",
              uci_data.uci_ri_len > 0 ? (uci_data.uci_ri ? ", ri=1" : ", ri=0") : "",
              cfo * 15, timestr,
              uci_data.uci_cqi_len > 0 ? cqi_str : "");

  // Store metrics
  ul_metrics.mcs   = grant->mcs.idx;
  ul_metrics.power = tx_power;
  phy->set_ul_metrics(ul_metrics);
}

void phch_worker::encode_pucch()
{
  char timestr[64];
  timestr[0]='\0';

  if (uci_data.scheduling_request || uci_data.uci_ack_len > 0 || uci_data.uci_cqi_len > 0 || uci_data.uci_ri_len > 0)
  {
    
    // Drop CQI if there is collision with ACK 
    if (!period_cqi.simul_cqi_ack && uci_data.uci_ack_len > 0 && uci_data.uci_cqi_len > 0) {
      uci_data.uci_cqi_len = 0; 
    }

#ifdef LOG_EXECTIME
    struct timeval t[3];
    gettimeofday(&t[1], NULL);
#endif

    if (srslte_ue_ul_pucch_encode(&ue_ul, uci_data, last_dl_pdcch_ncce, TTI_TX(tti), signal_buffer[0])) {
      Error("Encoding PUCCH\n");
    }

#ifdef LOG_EXECTIME
  gettimeofday(&logtime_start[2], NULL);
  memcpy(&t[2], &logtime_start[2], sizeof(struct timeval));
  get_time_interval(logtime_start);
  get_time_interval(t);
  snprintf(timestr, 64, ", pucch_time=%d us, tot_time=%d us", (int) t[0].tv_usec, (int) logtime_start[0].tv_usec);
#endif

  float tx_power = srslte_ue_ul_pucch_power(&ue_ul, phy->pathloss, ue_ul.last_pucch_format, uci_data.uci_cqi_len, uci_data.uci_ack_len);
  float gain = set_power(tx_power);  

    char cqi_str[SRSLTE_CQI_STR_MAX_CHAR] = "";
    if (log_h->get_level() >= srslte::LOG_LEVEL_INFO) {
      srslte_cqi_value_tostring(&cqi_report, cqi_str, SRSLTE_CQI_STR_MAX_CHAR);
    }

    Info("PUCCH: tti_tx=%d, amp=%.2f, n_pucch=%d, n_prb=%d, ack=%s%s%s%s, sr=%s, cfo=%.1f KHz%s\n",
         (tti + 4) % 10240, srslte_ue_ul_get_last_amplitude(&ue_ul),
         ue_ul.pucch.last_n_pucch, ue_ul.pucch.last_n_prb,
         uci_data.uci_ack_len > 0 ? (uci_data.uci_ack ? "1" : "0") : "no",
         uci_data.uci_ack_len > 1 ? (uci_data.uci_ack_2 ? "1" : "0") : "",
         uci_data.uci_ri_len > 0 ? (uci_data.uci_ri ? ", ri=1" : ", ri=0") : "",
         uci_data.uci_cqi_len > 0 ? cqi_str : "",
         uci_data.scheduling_request ? "yes" : "no",
         cfo * 15, timestr);
  }
  
  if (uci_data.scheduling_request) {
    phy->sr_enabled = false; 
  }
}

void phch_worker::encode_srs()
{
  char timestr[64];
  timestr[0]='\0';
  
  if (srslte_ue_ul_srs_encode(&ue_ul, TTI_TX(tti), signal_buffer[0]))
  {
    Error("Encoding SRS\n");
  }

#ifdef LOG_EXECTIME
  gettimeofday(&logtime_start[2], NULL);
  get_time_interval(logtime_start);
  snprintf(timestr, 64, ", tot_time=%4d us", (int) logtime_start[0].tv_usec);
#endif
  
  float tx_power = srslte_ue_ul_srs_power(&ue_ul, phy->pathloss);  
  float gain = set_power(tx_power);

  Info("SRS:   power=%.2f dBm, amp=%.2f, tti_tx=%d%s\n", tx_power, srslte_ue_ul_get_last_amplitude(&ue_ul), TTI_TX(tti), timestr);
}

void phch_worker::enable_pregen_signals(bool enabled)
{
  pregen_enabled = enabled; 
  if (enabled) {
    Info("Pre-generating UL signals worker=%d\n", get_id());
    srslte_ue_ul_pregen_signals(&ue_ul);
    Info("Done pre-generating signals worker=%d\n", get_id());
  }
}

void phch_worker::set_ul_params(bool pregen_disabled)
{

  phy_interface_rrc::phy_cfg_common_t         *common    = &phy->config->common;
  LIBLTE_RRC_PHYSICAL_CONFIG_DEDICATED_STRUCT *dedicated = &phy->config->dedicated;
  
  Info("Setting new params worker_id=%d, pregen_disabled=%d\n", get_id(), pregen_disabled);
  
  /* PUSCH DMRS signal configuration */
  bzero(&dmrs_cfg, sizeof(srslte_refsignal_dmrs_pusch_cfg_t));    
  dmrs_cfg.group_hopping_en    = common->pusch_cnfg.ul_rs.group_hopping_enabled;
  dmrs_cfg.sequence_hopping_en = common->pusch_cnfg.ul_rs.sequence_hopping_enabled;
  dmrs_cfg.cyclic_shift        = common->pusch_cnfg.ul_rs.cyclic_shift;
  dmrs_cfg.delta_ss            = common->pusch_cnfg.ul_rs.group_assignment_pusch;
  
  /* PUSCH Hopping configuration */
  bzero(&pusch_hopping, sizeof(srslte_pusch_hopping_cfg_t));
  pusch_hopping.n_sb           = common->pusch_cnfg.n_sb;
  pusch_hopping.hop_mode       = common->pusch_cnfg.hopping_mode == LIBLTE_RRC_HOPPING_MODE_INTRA_AND_INTER_SUBFRAME ? 
                                  pusch_hopping.SRSLTE_PUSCH_HOP_MODE_INTRA_SF : 
                                  pusch_hopping.SRSLTE_PUSCH_HOP_MODE_INTER_SF; 
  pusch_hopping.hopping_offset = common->pusch_cnfg.pusch_hopping_offset;

  /* PUSCH UCI configuration */
  bzero(&uci_cfg, sizeof(srslte_uci_cfg_t));
  uci_cfg.I_offset_ack         = dedicated->pusch_cnfg_ded.beta_offset_ack_idx;
  uci_cfg.I_offset_cqi         = dedicated->pusch_cnfg_ded.beta_offset_cqi_idx;
  uci_cfg.I_offset_ri          = dedicated->pusch_cnfg_ded.beta_offset_ri_idx;
  
  /* PUCCH configuration */  
  bzero(&pucch_cfg, sizeof(srslte_pucch_cfg_t));
  pucch_cfg.delta_pucch_shift  = liblte_rrc_delta_pucch_shift_num[common->pucch_cnfg.delta_pucch_shift%LIBLTE_RRC_DELTA_PUCCH_SHIFT_N_ITEMS];
  pucch_cfg.N_cs               = common->pucch_cnfg.n_cs_an;
  pucch_cfg.n_rb_2             = common->pucch_cnfg.n_rb_cqi;
  pucch_cfg.srs_configured     = dedicated->srs_ul_cnfg_ded.setup_present;
  if (pucch_cfg.srs_configured) {
    pucch_cfg.srs_cs_subf_cfg    = liblte_rrc_srs_subfr_config_num[common->srs_ul_cnfg.subfr_cnfg%LIBLTE_RRC_SRS_SUBFR_CONFIG_N_ITEMS];
    pucch_cfg.srs_simul_ack      = common->srs_ul_cnfg.ack_nack_simul_tx;
  }
  
  /* PUCCH Scheduling configuration */
  bzero(&pucch_sched, sizeof(srslte_pucch_sched_t));
  pucch_sched.n_pucch_1[0]     = 0; // TODO: n_pucch_1 for SPS
  pucch_sched.n_pucch_1[1]     = 0;
  pucch_sched.n_pucch_1[2]     = 0;
  pucch_sched.n_pucch_1[3]     = 0;
  pucch_sched.N_pucch_1        = common->pucch_cnfg.n1_pucch_an;
  pucch_sched.n_pucch_2        = dedicated->cqi_report_cnfg.report_periodic.pucch_resource_idx;
  pucch_sched.n_pucch_sr       = dedicated->sched_request_cnfg.sr_pucch_resource_idx;

  /* SRS Configuration */
  bzero(&srs_cfg, sizeof(srslte_refsignal_srs_cfg_t));
  srs_cfg.configured           = dedicated->srs_ul_cnfg_ded.setup_present;
  if (pucch_cfg.srs_configured) {
    srs_cfg.subframe_config      = liblte_rrc_srs_subfr_config_num[common->srs_ul_cnfg.subfr_cnfg%LIBLTE_RRC_SRS_SUBFR_CONFIG_N_ITEMS];
    srs_cfg.bw_cfg               = liblte_rrc_srs_bw_config_num[common->srs_ul_cnfg.bw_cnfg%LIBLTE_RRC_SRS_BW_CONFIG_N_ITEMS];
    srs_cfg.I_srs                = dedicated->srs_ul_cnfg_ded.srs_cnfg_idx;
    srs_cfg.B                    = dedicated->srs_ul_cnfg_ded.srs_bandwidth;
    srs_cfg.b_hop                = dedicated->srs_ul_cnfg_ded.srs_hopping_bandwidth;
    srs_cfg.n_rrc                = dedicated->srs_ul_cnfg_ded.freq_domain_pos;
    srs_cfg.k_tc                 = dedicated->srs_ul_cnfg_ded.tx_comb;
    srs_cfg.n_srs                = dedicated->srs_ul_cnfg_ded.cyclic_shift;
  }
  
  /* UL power control configuration */
  bzero(&power_ctrl, sizeof(srslte_ue_ul_powerctrl_t));
  power_ctrl.p0_nominal_pusch  = common->ul_pwr_ctrl.p0_nominal_pusch;
  power_ctrl.alpha             = liblte_rrc_ul_power_control_alpha_num[common->ul_pwr_ctrl.alpha%LIBLTE_RRC_UL_POWER_CONTROL_ALPHA_N_ITEMS];
  power_ctrl.p0_nominal_pucch  = common->ul_pwr_ctrl.p0_nominal_pucch;
  power_ctrl.delta_f_pucch[0]  = liblte_rrc_delta_f_pucch_format_1_num[common->ul_pwr_ctrl.delta_flist_pucch.format_1%LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1_N_ITEMS];
  power_ctrl.delta_f_pucch[1]  = liblte_rrc_delta_f_pucch_format_1b_num[common->ul_pwr_ctrl.delta_flist_pucch.format_1b%LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_1B_N_ITEMS];
  power_ctrl.delta_f_pucch[2]  = liblte_rrc_delta_f_pucch_format_2_num[common->ul_pwr_ctrl.delta_flist_pucch.format_2%LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2_N_ITEMS];
  power_ctrl.delta_f_pucch[3]  = liblte_rrc_delta_f_pucch_format_2a_num[common->ul_pwr_ctrl.delta_flist_pucch.format_2a%LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2A_N_ITEMS];
  power_ctrl.delta_f_pucch[4]  = liblte_rrc_delta_f_pucch_format_2b_num[common->ul_pwr_ctrl.delta_flist_pucch.format_2b%LIBLTE_RRC_DELTA_F_PUCCH_FORMAT_2B_N_ITEMS];
  
  power_ctrl.delta_preamble_msg3 = common->ul_pwr_ctrl.delta_preamble_msg3;
  
  power_ctrl.p0_ue_pusch       = dedicated->ul_pwr_ctrl_ded.p0_ue_pusch;
  power_ctrl.delta_mcs_based   = dedicated->ul_pwr_ctrl_ded.delta_mcs_en==LIBLTE_RRC_DELTA_MCS_ENABLED_EN0;
  power_ctrl.acc_enabled       = dedicated->ul_pwr_ctrl_ded.accumulation_en;
  power_ctrl.p0_ue_pucch       = dedicated->ul_pwr_ctrl_ded.p0_ue_pucch;
  power_ctrl.p_srs_offset      = dedicated->ul_pwr_ctrl_ded.p_srs_offset;
  
  srslte_ue_ul_set_cfg(&ue_ul, &dmrs_cfg, &srs_cfg, &pucch_cfg, &pucch_sched, &uci_cfg, &pusch_hopping, &power_ctrl);

  /* CQI configuration */
  bzero(&period_cqi, sizeof(srslte_cqi_periodic_cfg_t));
  period_cqi.configured        = dedicated->cqi_report_cnfg.report_periodic_setup_present;
  period_cqi.pmi_idx           = dedicated->cqi_report_cnfg.report_periodic.pmi_cnfg_idx;
  period_cqi.simul_cqi_ack     = dedicated->cqi_report_cnfg.report_periodic.simult_ack_nack_and_cqi;
  period_cqi.format_is_subband = dedicated->cqi_report_cnfg.report_periodic.format_ind_periodic ==
                                 LIBLTE_RRC_CQI_FORMAT_INDICATOR_PERIODIC_SUBBAND_CQI;
  period_cqi.subband_size      = dedicated->cqi_report_cnfg.report_periodic.format_ind_periodic_subband_k;

  if (dedicated->cqi_report_cnfg.report_periodic.ri_cnfg_idx_present) {
    period_cqi.ri_idx = dedicated->cqi_report_cnfg.report_periodic.ri_cnfg_idx;
    period_cqi.ri_idx_present = true;
  } else {
    period_cqi.ri_idx_present = false;
  }

  /* SR configuration */
  I_sr                         = dedicated->sched_request_cnfg.sr_cnfg_idx;
  sr_configured                = true;
  
  if (pregen_enabled && !pregen_disabled) { 
    Info("Pre-generating UL signals worker=%d\n", get_id());
    srslte_ue_ul_pregen_signals(&ue_ul);
    Info("Done pre-generating signals worker=%d\n", get_id());
  } 
}

float phch_worker::set_power(float tx_power) {
  float gain = 0; 
  /* Check if UL power control is enabled */
  if(phy->args->ul_pwr_ctrl_en) {    
    /* Adjust maximum power if it changes significantly */
    if (tx_power < phy->cur_radio_power - 5 || tx_power > phy->cur_radio_power + 5) {
      phy->cur_radio_power = tx_power; 
      float radio_tx_power = phy->cur_radio_power;
      gain = phy->get_radio()->set_tx_power(radio_tx_power);  
   }    
  }
  return gain;
}

void phch_worker::start_plot() {
#ifdef ENABLE_GUI
  if (plot_worker_id == -1) {
    plot_worker_id = get_id();
    log_h->console("Starting plot for worker_id=%d\n", plot_worker_id);
    init_plots(this);
  } else {
    log_h->console("Trying to start a plot but already started by worker_id=%d\n", plot_worker_id);
  }
#else 
    log_h->console("Trying to start a plot but plots are disabled (ENABLE_GUI constant in phch_worker.cc)\n");
#endif
}

int phch_worker::read_ce_abs(float *ce_abs, uint32_t tx_antenna, uint32_t rx_antenna) {
  uint32_t i=0;
  int sz = srslte_symbol_sz(cell.nof_prb);
  bzero(ce_abs, sizeof(float)*sz);
  int g = (sz - 12*cell.nof_prb)/2;
  for (i = 0; i < 12*cell.nof_prb; i++) {
    ce_abs[g+i] = 20 * log10f(cabsf(ue_dl.ce_m[tx_antenna][rx_antenna][i]));
    if (isinf(ce_abs[g+i])) {
      ce_abs[g+i] = -80;
    }
  }
  return sz;
}

int phch_worker::read_pdsch_d(cf_t* pdsch_d)
{

  memcpy(pdsch_d, ue_dl.pdsch.d[0], ue_dl.pdsch_cfg.nbits[0].nof_re*sizeof(cf_t));
  return ue_dl.pdsch_cfg.nbits[0].nof_re;
}



/**************************** Measurements **************************/

void phch_worker::update_measurements() 
{
  float snr_ema_coeff = phy->args->snr_ema_coeff;
  if (chest_done) {

    /* Only worker 0 reads the RSSI sensor every ~1-nof_cores s */
    if (get_id() == 0) {
      if (!rssi_read_cnt) {
        if (phy->get_radio()->has_rssi() && phy->args->rssi_sensor_enabled) {
          phy->last_radio_rssi = phy->get_radio()->get_rssi();
          phy->rx_gain_offset = phy->avg_rssi_dbm - phy->last_radio_rssi + 30;
        } else {
          phy->rx_gain_offset = phy->get_radio()->get_rx_gain() + phy->args->rx_gain_offset;
        }
      }
      rssi_read_cnt++;
      if (rssi_read_cnt == 1000) {
        rssi_read_cnt = 0;
      }
    }
    
    // Average RSRQ over DEFAULT_MEAS_PERIOD_MS then sent to RRC
    float rsrq_db = 10*log10(srslte_chest_dl_get_rsrq(&ue_dl.chest));
    if (isnormal(rsrq_db)) {
      if (!(tti%phy->pcell_report_period) || !phy->avg_rsrq_db) {
        phy->avg_rsrq_db = rsrq_db;
      } else {
        phy->avg_rsrq_db = SRSLTE_VEC_CMA(rsrq_db, phy->avg_rsrq_db, tti%phy->pcell_report_period);
      }
    }

    // Average RSRP taken from CRS
    float rsrp_lin = srslte_chest_dl_get_rsrp(&ue_dl.chest);
    if (isnormal(rsrp_lin)) {
      if (!phy->avg_rsrp) {
        phy->avg_rsrp = SRSLTE_VEC_EMA(rsrp_lin, phy->avg_rsrp, snr_ema_coeff);
      } else {
        phy->avg_rsrp = rsrp_lin;
      }
    }
    
    /* Correct absolute power measurements by RX gain offset */
    float rsrp_dbm = 10*log10(rsrp_lin) + 30 - phy->rx_gain_offset;

    // Serving cell RSRP measurements are averaged over DEFAULT_MEAS_PERIOD_MS then sent to RRC
    if (isnormal(rsrp_dbm)) {
      if (!(tti%phy->pcell_report_period) || !phy->avg_rsrp_dbm) {
        phy->avg_rsrp_dbm = rsrp_dbm;
      } else {
        phy->avg_rsrp_dbm = SRSLTE_VEC_CMA(rsrp_dbm, phy->avg_rsrp_dbm, tti%phy->pcell_report_period);
      }
    }

    // Send PCell measurement
    if ((tti%phy->pcell_report_period) == phy->pcell_report_period-1) {
      phy->rrc->new_phy_meas(phy->avg_rsrp_dbm, phy->avg_rsrq_db, tti);
    }

    // Compute PL
    float tx_crs_power = phy->config->common.pdsch_cnfg.rs_power;
    phy->pathloss = tx_crs_power - phy->avg_rsrp_dbm;

    // Average noise 
    float cur_noise = srslte_chest_dl_get_noise_estimate(&ue_dl.chest);
    if (isnormal(cur_noise)) {
      if (!phy->avg_noise) {  
        phy->avg_noise = cur_noise;          
      } else {
        phy->avg_noise = SRSLTE_VEC_EMA(cur_noise, phy->avg_noise, snr_ema_coeff);
      }
    }

    phy->avg_snr_db_cqi  = 10*log10(phy->avg_rsrp/phy->avg_noise);

    // Store metrics
    dl_metrics.n      = phy->avg_noise;
    dl_metrics.rsrp   = phy->avg_rsrp_dbm;
    dl_metrics.rsrq   = phy->avg_rsrq_db;
    dl_metrics.rssi   = phy->avg_rssi_dbm;
    dl_metrics.pathloss = phy->pathloss;
    dl_metrics.sinr   = phy->avg_snr_db_cqi;
    phy->set_dl_metrics(dl_metrics);

  }
}


/********** Execution time trace function ************/

void phch_worker::start_trace() {
  trace_enabled = true; 
}

void phch_worker::write_trace(std::string filename) {
  tr_exec.writeToBinary(filename + ".exec");
}

void phch_worker::tr_log_start()
{
  if (trace_enabled) {
    gettimeofday(&tr_time[1], NULL);
  }
}

void phch_worker::tr_log_end()
{
  if (trace_enabled) {
    gettimeofday(&tr_time[2], NULL);
    get_time_interval(tr_time);
    tr_exec.push(tti, tr_time[0].tv_usec);
  }
}

}








/***********************************************************
 * 
 * PLOT TO VISUALIZE THE CHANNEL RESPONSEE 
 * 
 ***********************************************************/


#ifdef ENABLE_GUI
plot_real_t    pce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
plot_scatter_t pconst;
#define SCATTER_PDSCH_BUFFER_LEN   (20*6*SRSLTE_SF_LEN_RE(SRSLTE_MAX_PRB, SRSLTE_CP_NORM))
#define SCATTER_PDSCH_PLOT_LEN    4000
float tmp_plot[SCATTER_PDSCH_BUFFER_LEN];
cf_t  tmp_plot2[SRSLTE_SF_LEN_RE(SRSLTE_MAX_PRB, SRSLTE_CP_NORM)];

#define CFO_PLOT_LEN 0 /* Set to non zero for enabling CFO plot */
#if CFO_PLOT_LEN > 0
static plot_real_t    pcfo;
static uint32_t icfo = 0;
static float cfo_buffer[CFO_PLOT_LEN];
#endif /* CFO_PLOT_LEN > 0 */

void *plot_thread_run(void *arg) {
  srsue::phch_worker *worker = (srsue::phch_worker*) arg; 

  sdrgui_init();
  for (uint32_t tx = 0; tx < worker->get_cell_nof_ports(); tx++) {
    for (uint32_t rx = 0; rx < worker->get_rx_nof_antennas(); rx++) {
      char str_buf[64];
      snprintf(str_buf, 64, "|H%d%d|", rx, tx);
      plot_real_init(&pce[tx][rx]);
      plot_real_setTitle(&pce[tx][rx], str_buf);
      plot_real_setLabels(&pce[tx][rx], (char *) "Index", (char *) "dB");
      plot_real_setYAxisScale(&pce[tx][rx], -40, 40);

      plot_real_addToWindowGrid(&pce[tx][rx], (char*)"srsue", tx, rx);
    }
  }

  plot_scatter_init(&pconst);
  plot_scatter_setTitle(&pconst, (char*) "PDSCH - Equalized Symbols");
  plot_scatter_setXAxisScale(&pconst, -4, 4);
  plot_scatter_setYAxisScale(&pconst, -4, 4);

  plot_scatter_addToWindowGrid(&pconst, (char*)"srsue", 0, worker->get_rx_nof_antennas());

#if CFO_PLOT_LEN > 0
  plot_real_init(&pcfo);
  plot_real_setTitle(&pcfo, (char*) "CFO (Hz)");
  plot_real_setLabels(&pcfo, (char *) "Time", (char *) "Hz");
  plot_real_setYAxisScale(&pcfo, -4000, 4000);

  plot_scatter_addToWindowGrid(&pcfo, (char*)"srsue", 1, worker->get_rx_nof_antennas());
#endif /* CFO_PLOT_LEN > 0 */

  int n; 
  int readed_pdsch_re=0; 
  while(1) {
    sem_wait(&plot_sem);    
    
    if (readed_pdsch_re < SCATTER_PDSCH_PLOT_LEN) {
      n = worker->read_pdsch_d(&tmp_plot2[readed_pdsch_re]);
      readed_pdsch_re += n;           
    } else {
      for (uint32_t tx = 0; tx < worker->get_cell_nof_ports(); tx++) {
        for (uint32_t rx = 0; rx < worker->get_rx_nof_antennas(); rx++) {
          n = worker->read_ce_abs(tmp_plot, tx, rx);
          if (n > 0) {
            plot_real_setNewData(&pce[tx][rx], tmp_plot, n);
          }
        }
      }
      if (readed_pdsch_re > 0) {
        plot_scatter_setNewData(&pconst, tmp_plot2, readed_pdsch_re);
      }
      readed_pdsch_re = 0; 
    }

#if CFO_PLOT_LEN > 0
    cfo_buffer[icfo] = worker->get_cfo() * 15000.0f;
    icfo = (icfo + 1)%CFO_PLOT_LEN;
    plot_real_setNewData(&pcfo, cfo_buffer, CFO_PLOT_LEN);
#endif /* CFO_PLOT_LEN > 0 */

  }
  return NULL;
}


void init_plots(srsue::phch_worker *worker) {

  if (sem_init(&plot_sem, 0, 0)) {
    perror("sem_init");
    exit(-1);
  }
  
  pthread_attr_t attr;
  struct sched_param param;
  param.sched_priority = 0;  
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
  pthread_attr_setschedparam(&attr, &param);
  if (pthread_create(&plot_thread, &attr, plot_thread_run, worker)) {
    perror("pthread_create");
    exit(-1);
  }  
}
#endif

