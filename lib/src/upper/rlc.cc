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
 * MERCHANTABILITY or FITNESS FOR A PARTICRXAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */


#include "srslte/upper/rlc.h"
#include "srslte/upper/rlc_tm.h"
#include "srslte/upper/rlc_um.h"
#include "srslte/upper/rlc_am.h"

namespace srslte {

rlc::rlc()
{
  pool = byte_buffer_pool::get_instance();
  rlc_log = NULL;
  pdcp = NULL;
  rrc = NULL;
  mac_timers = NULL;
  ue = NULL;
  default_lcid = 0;
  bzero(metrics_time, sizeof(metrics_time));
  bzero(ul_tput_bytes, sizeof(ul_tput_bytes));
  bzero(dl_tput_bytes, sizeof(dl_tput_bytes));
}

void rlc::init(srsue::pdcp_interface_rlc *pdcp_,
               srsue::rrc_interface_rlc  *rrc_,
               srsue::ue_interface       *ue_,
               log                       *rlc_log_, 
               mac_interface_timers      *mac_timers_,
               uint32_t                  lcid_,
               int                       buffer_size_)
{
  pdcp    = pdcp_;
  rrc     = rrc_;
  ue      = ue_;
  rlc_log = rlc_log_;
  mac_timers = mac_timers_;
  default_lcid = lcid_;
  buffer_size  = buffer_size_;

  gettimeofday(&metrics_time[1], NULL);
  reset_metrics(); 

  rlc_array[0].init(RLC_MODE_TM, rlc_log, default_lcid, pdcp, rrc, mac_timers, buffer_size); // SRB0
}

void rlc::init(srsue::pdcp_interface_rlc *pdcp_,
               srsue::rrc_interface_rlc  *rrc_,
               srsue::ue_interface       *ue_,
               log                       *rlc_log_, 
               mac_interface_timers      *mac_timers_,
               uint32_t                  lcid_,
               int                       buffer_size_,
               int port)
{

  rlc::init(pdcp_,rrc_,ue_,rlc_log_,mac_timers_,lcid_,buffer_size_);
  tcp_process_thread.init(port);
}

void rlc::reset_metrics() 
{
  bzero(dl_tput_bytes, sizeof(long)*SRSLTE_N_RADIO_BEARERS);
  bzero(ul_tput_bytes, sizeof(long)*SRSLTE_N_RADIO_BEARERS);
}

void rlc::stop()
{
  for(uint32_t i=0; i<SRSLTE_N_RADIO_BEARERS; i++) {
    if(rlc_array[i].active()) {
      rlc_array[i].stop();
    }
  }
}

void rlc::get_metrics(rlc_metrics_t &m)
{
  
  gettimeofday(&metrics_time[2], NULL);
  get_time_interval(metrics_time);
  double secs = (double)metrics_time[0].tv_sec + metrics_time[0].tv_usec*1e-6;
  
  m.dl_tput_mbps = 0; 
  m.ul_tput_mbps = 0; 
  for (int i=0;i<SRSLTE_N_RADIO_BEARERS;i++) {
    m.dl_tput_mbps += (dl_tput_bytes[i]*8/(double)1e6)/secs;
    m.ul_tput_mbps += (ul_tput_bytes[i]*8/(double)1e6)/secs;    
    if(rlc_array[i].active()) {
      rlc_log->info("LCID=%d, RX throughput: %4.6f Mbps. TX throughput: %4.6f Mbps.\n",
                    i,
                    (dl_tput_bytes[i]*8/(double)1e6)/secs,
                    (ul_tput_bytes[i]*8/(double)1e6)/secs);
    }
  }

  // Add multicast metrics
  for (int i=0;i<SRSLTE_N_MCH_LCIDS;i++) {
    m.dl_tput_mbps += (dl_tput_bytes_mrb[i]*8/(double)1e6)/secs;
    if(rlc_array_mrb[i].is_mrb()) {
      rlc_log->info("MCH_LCID=%d, RX throughput: %4.6f Mbps.\n",
                    i,
                    (dl_tput_bytes_mrb[i]*8/(double)1e6)/secs);
    }
  }

  memcpy(&metrics_time[1], &metrics_time[2], sizeof(struct timeval));
  reset_metrics();
}

// A call to reestablish stops all lcids but does not delete the instances. The mapping lcid to rlc mode can not change
void rlc::reestablish() {
  for(uint32_t i=0; i<SRSLTE_N_RADIO_BEARERS; i++) {
    if(rlc_array[i].active()) {
      rlc_array[i].reestablish();
    }
  }
}

// Resetting the RLC layer returns the object to the state after the call to init(): All lcids are stopped and
// defaul lcid=0 is created
void rlc::reset()
{
  for(uint32_t i=0; i<SRSLTE_N_RADIO_BEARERS; i++) {
    if(rlc_array[i].active())
      rlc_array[i].stop();
  }

  rlc_array[0].init(RLC_MODE_TM, rlc_log, default_lcid, pdcp, rrc, mac_timers, buffer_size); // SRB0
}

void rlc::empty_queue()
{
  for(uint32_t i=0; i<SRSLTE_N_RADIO_BEARERS; i++) {
    if(rlc_array[i].active())
      rlc_array[i].empty_queue();
  }
}

/*******************************************************************************
  PDCP interface
*******************************************************************************/
void rlc::write_sdu(uint32_t lcid, byte_buffer_t *sdu)
{
  if(valid_lcid(lcid)) {
    rlc_array[lcid].write_sdu(sdu);
  }
}
void rlc::write_sdu_nb(uint32_t lcid, byte_buffer_t *sdu)
{
  if(valid_lcid(lcid)) {
    rlc_array[lcid].write_sdu_nb(sdu);
  }
}
void rlc::write_sdu_mch(uint32_t lcid, byte_buffer_t *sdu)
{
  if(valid_lcid_mrb(lcid)) {
    rlc_array_mrb[lcid].write_sdu(sdu);
  }
}

bool rlc::rb_is_um(uint32_t lcid) {
  return rlc_array[lcid].get_mode()==RLC_MODE_UM;
}

/*******************************************************************************
  MAC interface
*******************************************************************************/
uint32_t rlc::get_buffer_state(uint32_t lcid)
{
  if(valid_lcid(lcid)) {
    return rlc_array[lcid].get_buffer_state();
  } else {
    return 0;
  }
}

uint32_t rlc::get_total_buffer_state(uint32_t lcid)
{
  if(valid_lcid(lcid)) {
    return rlc_array[lcid].get_total_buffer_state();
  } else {
    return 0;
  }
}

uint32_t rlc::get_total_mch_buffer_state(uint32_t lcid)
{
  if(valid_lcid_mrb(lcid)) {
    return rlc_array_mrb[lcid].get_total_buffer_state();
  } else {
    return 0;
  }
}

int rlc::read_pdu_sl(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  return tcp_process_thread.get_packet(payload, nof_bytes);
}

int rlc::read_pdu(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  if(valid_lcid(lcid)) {
    ul_tput_bytes[lcid] += nof_bytes;
    return rlc_array[lcid].read_pdu(payload, nof_bytes);
  }
  return 0;
}

int rlc::read_pdu_mch(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  if(valid_lcid_mrb(lcid)) {
    ul_tput_bytes[lcid] += nof_bytes;
    return rlc_array_mrb[lcid].read_pdu(payload, nof_bytes);
  }
  return 0;
}

void rlc::write_pdu_sl(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  dl_tput_bytes[lcid] += nof_bytes;
  tcp_process_thread.send_packet(payload, nof_bytes);
}

void rlc::write_pdu(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  if(valid_lcid(lcid)) {
    dl_tput_bytes[lcid] += nof_bytes;
    rlc_array[lcid].write_pdu(payload, nof_bytes);
  }
}

void rlc::write_pdu_bcch_bch(uint8_t *payload, uint32_t nof_bytes)
{
  rlc_log->info_hex(payload, nof_bytes, "BCCH BCH message received.");
  dl_tput_bytes[0] += nof_bytes;
  byte_buffer_t *buf = pool_allocate;
  if (buf) {
    memcpy(buf->msg, payload, nof_bytes);
    buf->N_bytes = nof_bytes;
    buf->set_timestamp();
    pdcp->write_pdu_bcch_bch(buf);
  } else {
    rlc_log->error("Fatal error: Out of buffers from the pool in write_pdu_bcch_bch()\n");
  }
}

void rlc::write_pdu_bcch_dlsch(uint8_t *payload, uint32_t nof_bytes)
{
  rlc_log->info_hex(payload, nof_bytes, "BCCH TXSCH message received.");
  dl_tput_bytes[0] += nof_bytes;
  byte_buffer_t *buf = pool_allocate;
  if (buf) {
    memcpy(buf->msg, payload, nof_bytes);
    buf->N_bytes = nof_bytes;
    buf->set_timestamp();
    pdcp->write_pdu_bcch_dlsch(buf);
  } else {
    rlc_log->error("Fatal error: Out of buffers from the pool in write_pdu_bcch_dlsch()\n");
  }
}

void rlc::write_pdu_pcch(uint8_t *payload, uint32_t nof_bytes)
{
  rlc_log->info_hex(payload, nof_bytes, "PCCH message received.");
  dl_tput_bytes[0] += nof_bytes;
  byte_buffer_t *buf = pool_allocate;
  if (buf) {
    memcpy(buf->msg, payload, nof_bytes);
    buf->N_bytes = nof_bytes;
    buf->set_timestamp();
    pdcp->write_pdu_pcch(buf);
  } else {
    rlc_log->error("Fatal error: Out of buffers from the pool in write_pdu_pcch()\n");
  }
}

void rlc::write_pdu_mch(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes)
{
  if(valid_lcid_mrb(lcid)) {
    dl_tput_bytes_mrb[lcid] += nof_bytes;
    rlc_array_mrb[lcid].write_pdu(payload, nof_bytes);
  }
}

/*******************************************************************************
  RRC interface
*******************************************************************************/
void rlc::add_bearer(uint32_t lcid)
{
  // No config provided - use defaults for SRB1 and SRB2
  if(lcid < 3) {
    if (!rlc_array[lcid].active()) {
      LIBLTE_RRC_RLC_CONFIG_STRUCT cnfg;
      cnfg.rlc_mode                     = LIBLTE_RRC_RLC_MODE_AM;
      cnfg.ul_am_rlc.t_poll_retx        = LIBLTE_RRC_T_POLL_RETRANSMIT_MS45;
      cnfg.ul_am_rlc.poll_pdu           = LIBLTE_RRC_POLL_PDU_INFINITY;
      cnfg.ul_am_rlc.poll_byte          = LIBLTE_RRC_POLL_BYTE_INFINITY;
      cnfg.ul_am_rlc.max_retx_thresh    = LIBLTE_RRC_MAX_RETX_THRESHOLD_T4;
      cnfg.dl_am_rlc.t_reordering       = LIBLTE_RRC_T_REORDERING_MS35;
      cnfg.dl_am_rlc.t_status_prohibit  = LIBLTE_RRC_T_STATUS_PROHIBIT_MS0;
      add_bearer(lcid, srslte_rlc_config_t(&cnfg));
    } else {
      rlc_log->warning("Bearer %s already configured. Reconfiguration not supported\n", rrc->get_rb_name(lcid).c_str());
    }
  }else{
    rlc_log->error("Radio bearer %s does not support default RLC configuration.\n", rrc->get_rb_name(lcid).c_str());
  }
}

void rlc::add_bearer(uint32_t lcid, srslte_rlc_config_t cnfg)
{
  if(lcid >= SRSLTE_N_RADIO_BEARERS) {
    rlc_log->error("Radio bearer id must be in [0:%d] - %d\n", SRSLTE_N_RADIO_BEARERS, lcid);
    return;
  }

  if (!rlc_array[lcid].active()) {
    rlc_log->warning("Adding radio bearer %s with mode %s\n",
                  rrc->get_rb_name(lcid).c_str(), liblte_rrc_rlc_mode_text[cnfg.rlc_mode]);
    switch(cnfg.rlc_mode)
    {
    case LIBLTE_RRC_RLC_MODE_AM:
      rlc_array[lcid].init(RLC_MODE_AM, rlc_log, lcid, pdcp, rrc, mac_timers, buffer_size);
      break;
    case LIBLTE_RRC_RLC_MODE_UM_BI:
      rlc_array[lcid].init(RLC_MODE_UM, rlc_log, lcid, pdcp, rrc, mac_timers, buffer_size);
      break;
    case LIBLTE_RRC_RLC_MODE_UM_UNI_DL:
      rlc_array[lcid].init(RLC_MODE_UM, rlc_log, lcid, pdcp, rrc, mac_timers, buffer_size);
      break;
    case LIBLTE_RRC_RLC_MODE_UM_UNI_UL:
      rlc_array[lcid].init(RLC_MODE_UM, rlc_log, lcid, pdcp, rrc, mac_timers, buffer_size);
      break;
    default:
      rlc_log->error("Cannot add RLC entity - invalid mode\n");
      return;
    }
  } else {
    rlc_log->warning("Bearer %s already created.\n", rrc->get_rb_name(lcid).c_str());
  }
  rlc_array[lcid].configure(cnfg);

}

void rlc::add_bearer_mrb(uint32_t lcid)
{
  // 36.321 Table 6.2.1-4
  if(lcid >= SRSLTE_N_MCH_LCIDS) {
    rlc_log->error("Radio bearer id must be in [0:%d] - %d\n", SRSLTE_N_MCH_LCIDS, lcid);
    return;
  }
  rlc_array_mrb[lcid].init(rlc_log, lcid, pdcp, rrc, mac_timers);
  rlc_array_mrb[lcid].configure(srslte_rlc_config_t::mch_config());
}

void rlc::add_bearer_mrb_enb(uint32_t lcid)
{
   if(lcid >= SRSLTE_N_MCH_LCIDS) {
    rlc_log->error("Radio bearer id must be in [0:%d] - %d\n", SRSLTE_N_MCH_LCIDS, lcid);
    return;
  }
  rlc_array_mrb[lcid].init(rlc_log,lcid,pdcp,rrc,mac_timers);
  rlc_array_mrb[lcid].configure(srslte_rlc_config_t::mch_config());
}

/*******************************************************************************
  Helpers
*******************************************************************************/
bool rlc::valid_lcid(uint32_t lcid)
{
  if(lcid >= SRSLTE_N_RADIO_BEARERS) {
    rlc_log->warning("Invalid LCID=%d\n", lcid);
    return false;
  } else if(!rlc_array[lcid].active()) {
    return false;
  }
  return true;
}

bool rlc::valid_lcid_mrb(uint32_t lcid)
{
  if(lcid >= SRSLTE_N_MCH_LCIDS) {
    return false;
  }
  if(!rlc_array_mrb[lcid].is_mrb()) {
    return false;
  }
  return true;
}


/********************
 * tcp socket class implementation
 * *****************************/

rlc::tcp_process::tcp_process()
{

}

void rlc::tcp_process::init(int net_port) {
  // mac_unit = mac_unit_;
  pthread_mutex_init(&mutex, NULL);
  pthread_cond_init(&cvar, NULL);
  have_data = false; 

  printf("TCP: setting up as client on port %d\n", net_port);
  if (srslte_netsink_init(&net_source, "127.0.0.1", net_port, SRSLTE_NETSINK_TCP)) {
    fprintf(stderr, "Error source as client\n");
    exit(-1);
  }
  srslte_netsink_set_nonblocking(&net_source);
  start(MAC_PDU_THREAD_PRIO);
}


void rlc::tcp_process::stop()
{
  pthread_mutex_lock(&mutex);
  running = false; 
  pthread_cond_signal(&cvar);
  pthread_mutex_unlock(&mutex);
  
  wait_thread_finish();
}

void rlc::tcp_process::notify()
{
  pthread_mutex_lock(&mutex);
  have_data = true; 
  pthread_cond_signal(&cvar);
  pthread_mutex_unlock(&mutex);
}

int rlc::tcp_process::get_packet(uint8_t *p_, uint32_t len_) {
  if(recv_len > 0) {
    if((int32_t)len_ > recv_len) {
      printf("MAC requested %d bytes but RLC has only %d\n", len_, recv_len);
      len_ = recv_len;
    }
    memcpy(p_, recv_buffer, len_);
    recv_len = 0;
    return len_;
  }
  return 0;
}

// @todo: only copy buffer and let it the task send out
void rlc::tcp_process::send_packet(uint8_t *p_, uint32_t len_) {
  pthread_mutex_lock(&mutex);
  srslte_netsink_write(&net_source, p_, len_);
  pthread_mutex_unlock(&mutex);
}

void rlc::tcp_process::run_thread()
{
  pthread_setname_np(pthread_self(), "rlc::tcp_proc");
  
  running = true; 
  while(running) {
    
    pthread_mutex_lock(&mutex);

    if(recv_len <= 0) {
      int b = srslte_netsink_read( &net_source, recv_buffer, 100);
      if(b > 0) {
        // printf("tcp_process: got %d bytes\n", b);
        recv_len = b;

      }
    } else {
      //make a dummy read to setup connection
      srslte_netsink_read( &net_source, NULL, 0);
    }
    

    // if(b>0) {
    //   if(0 != recv_len){
    //     printf("TCP: Packet overwritten %d bytes with %d bytes.\n", recv_len, b);
    //   }
    //   recv_len = b;
    // }
    pthread_mutex_unlock(&mutex);
    usleep(900);
  }
}

} // namespace srsue