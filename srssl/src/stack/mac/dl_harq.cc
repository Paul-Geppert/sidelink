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

#define Error(fmt, ...) log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...) log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...) log_h->debug(fmt, ##__VA_ARGS__)

#include "srssl/hdr/stack/mac/dl_harq.h"
#include "srslte/common/log.h"
#include "srslte/common/mac_pcap.h"
#include "srslte/common/timers.h"
#include "srslte/interfaces/ue_interfaces.h"

namespace srsue {

dl_harq_entity::dl_harq_entity() : proc(SRSLTE_MAX_HARQ_PROC)
{
  pcap                 = NULL;
  demux_unit           = NULL;
  log_h                = NULL;
  timer_aligment_timer = NULL;
  si_window_start      = 0;
  last_temporal_crnti  = 0;
  average_retx         = 0;
  nof_pkts             = 0;
}

bool dl_harq_entity::init(srslte::log*                  log_h,
                          mac_interface_rrc::ue_rnti_t* rntis,
                          srslte::timers::timer*        timer_aligment_timer,
                          demux*                        demux_unit)
{
  this->timer_aligment_timer = timer_aligment_timer;
  this->demux_unit           = demux_unit;
  this->log_h                = log_h;
  this->rntis                = rntis;

  for (uint32_t i = 0; i < SRSLTE_MAX_HARQ_PROC; i++) {
    if (!proc[i].init(i, this)) {
      return false;
    }
  }
  bcch_proc.init(-1, this);
  return true;
}

/***************** PHY->MAC interface for DL processes **************************/
void dl_harq_entity::new_grant_dl(mac_interface_phy_lte::mac_grant_dl_t  grant,
                                  mac_interface_phy_lte::tb_action_dl_t* action)
{
  bzero(action, sizeof(mac_interface_phy_lte::tb_action_dl_t));

  // sidelink extension
  if (grant.rnti == SRSLTE_RNTI_SL_PLACEHOLDER) {

    grant.pid = grant.pid % SRSLTE_MAX_HARQ_PROC;

    dl_harq_process* proc_ptr = NULL;

    if (grant.pid >= SRSLTE_MAX_HARQ_PROC) {
      Error("Invalid PID: %d\n", grant.pid);
      return;
    }
    proc_ptr = &proc[grant.pid];

    proc_ptr->new_grant_dl(grant, action);

    return;
  }

  if (grant.rnti != rntis->sps_rnti) {
    // Set BCCH PID for SI RNTI
    dl_harq_process* proc_ptr = NULL;
    if (grant.rnti == SRSLTE_SIRNTI) {
      proc_ptr = &bcch_proc;
    } else {
      if (grant.pid >= SRSLTE_MAX_HARQ_PROC) {
        Error("Invalid PID: %d\n", grant.pid);
        return;
      }
      proc_ptr = &proc[grant.pid];
    }
    // Consider the NDI to have been toggled
    if (grant.rnti == rntis->temp_rnti && last_temporal_crnti != rntis->temp_rnti) {
      last_temporal_crnti = rntis->temp_rnti;
      proc_ptr->reset_ndi();
      Info("Considering NDI in pid=%d to be toggled for first Temporal C-RNTI\n", grant.pid);
    }
    proc_ptr->new_grant_dl(grant, action);
  } else {
    Error("SPS not supported\n");
  }
}

void dl_harq_entity::tb_decoded(mac_interface_phy_lte::mac_grant_dl_t grant, bool ack[SRSLTE_MAX_CODEWORDS])
{
  if (grant.rnti == SRSLTE_SIRNTI) {
    bcch_proc.tb_decoded(grant, ack);
  } else {
    if (grant.pid >= SRSLTE_MAX_HARQ_PROC) {
      Error("Invalid PID: %d\n", grant.pid);
      return;
    }
    proc[grant.pid].tb_decoded(grant, ack);
  }
}

void dl_harq_entity::reset()
{
  for (uint32_t i = 0; i < SRSLTE_MAX_HARQ_PROC; i++) {
    proc[i].reset();
  }
  bcch_proc.reset();
  dl_sps_assig.clear();
}

void dl_harq_entity::start_pcap(srslte::mac_pcap* pcap_)
{
  pcap = pcap_;
}

void dl_harq_entity::set_si_window_start(int si_window_start_)
{
  si_window_start = si_window_start_;
}

float dl_harq_entity::get_average_retx()
{
  return average_retx;
}

dl_harq_entity::dl_harq_process::dl_harq_process() : subproc(SRSLTE_MAX_TB) {}

bool dl_harq_entity::dl_harq_process::init(int pid, dl_harq_entity* parent)
{
  bool ret = true;

  for (uint32_t tb = 0; tb < SRSLTE_MAX_TB; tb++) {
    ret &= subproc[tb].init(pid, parent, tb);
  }
  return ret;
}

void dl_harq_entity::dl_harq_process::reset(void)
{
  for (uint32_t tb = 0; tb < SRSLTE_MAX_TB; tb++) {
    subproc[tb].reset();
  }
}

void dl_harq_entity::dl_harq_process::reset_ndi()
{
  for (uint32_t tb = 0; tb < SRSLTE_MAX_TB; tb++) {
    subproc[tb].reset_ndi();
  }
}

void dl_harq_entity::dl_harq_process::new_grant_dl(mac_interface_phy_lte::mac_grant_dl_t  grant,
                                                   mac_interface_phy_lte::tb_action_dl_t* action)
{
  bzero(action, sizeof(mac_interface_phy_lte::tb_action_dl_t));
  /* For each subprocess... */
  for (uint32_t i = 0; i < SRSLTE_MAX_TB; i++) {
    if (grant.tb[i].tbs) {
      subproc[i].new_grant_dl(grant, action);
    }
  }
}

void dl_harq_entity::dl_harq_process::tb_decoded(mac_interface_phy_lte::mac_grant_dl_t grant,
                                                 bool                                  ack[SRSLTE_MAX_CODEWORDS])
{
  /* For each subprocess... */
  for (uint32_t i = 0; i < SRSLTE_MAX_TB; i++) {
    subproc[i].tb_decoded(grant, &ack[i]);
  }
}

bool dl_harq_entity::dl_harq_process::is_sps()
{
  return false;
}

dl_harq_entity::dl_harq_process::dl_tb_process::dl_tb_process()
{
  is_initiated = false;
  ack          = false;
  n_retx       = 0;
  bzero(&cur_grant, sizeof(mac_interface_phy_lte::mac_grant_dl_t));
  payload_buffer_ptr = NULL;
  pthread_mutex_init(&mutex, NULL);
}

dl_harq_entity::dl_harq_process::dl_tb_process::~dl_tb_process()
{
  if (is_initiated) {
    srslte_softbuffer_rx_free(&softbuffer);
  }
}

bool dl_harq_entity::dl_harq_process::dl_tb_process::init(int pid, dl_harq_entity* parent, uint32_t tb_idx)
{
  if (srslte_softbuffer_rx_init(&softbuffer, 110)) {
    Error("Error initiating soft buffer\n");
    return false;
  }

  if (pid < 0) {
    is_bcch   = true;
    this->pid = 0;
  } else {
    this->pid = (uint32_t)pid;
    is_bcch   = false;
  }

  tid          = tb_idx;
  is_first_tb  = true;
  is_initiated = true;
  harq_entity  = parent;
  log_h        = harq_entity->log_h;
  return true;
}

void dl_harq_entity::dl_harq_process::dl_tb_process::reset(bool lock)
{
  if (lock) {
    pthread_mutex_lock(&mutex);
  }

  bzero(&cur_grant, sizeof(mac_interface_phy_lte::mac_grant_dl_t));
  is_first_tb = true;
  ack         = false;
  n_retx      = 0;

  if (payload_buffer_ptr) {
    if (!is_bcch) {
      harq_entity->demux_unit->deallocate(payload_buffer_ptr);
    }
    payload_buffer_ptr = NULL;
  }

  if (is_initiated && lock) {
    srslte_softbuffer_rx_reset(&softbuffer);
  }

  if (lock) {
    pthread_mutex_unlock(&mutex);
  }
}

void dl_harq_entity::dl_harq_process::dl_tb_process::reset_ndi()
{
  is_first_tb = true;
}

void dl_harq_entity::dl_harq_process::dl_tb_process::new_grant_dl(mac_interface_phy_lte::mac_grant_dl_t  grant,
                                                                  mac_interface_phy_lte::tb_action_dl_t* action)
{

  pthread_mutex_lock(&mutex);

  // Compute RV for BCCH when not specified in PDCCH format
  if (is_bcch && grant.tb[tid].rv == -1) {
    uint32_t k;
    if ((grant.pid / 10) % 2 == 0 && grant.pid % 10 == 5) { // This is SIB1, k is different
      k                = (grant.pid / 20) % 4;
      grant.tb[tid].rv = ((uint32_t)ceilf((float)1.5 * k)) % 4;
    } else {
      k                = (grant.pid - harq_entity->si_window_start) % 4;
      grant.tb[tid].rv = ((uint32_t)ceilf((float)1.5 * k)) % 4;
    }
  }

  // sidelink extension
  if (grant.rnti != SRSLTE_RNTI_SL_PLACEHOLDER) {
    calc_is_new_transmission(grant);
  } else {
    // For sidelink we use the ndi indicator to tell us if this is a new transmission or not,
    // instead of toggling it.
    if (grant.tb[tid].ndi_present) {
      is_new_transmission = grant.tb[tid].ndi;
    } else {
      is_new_transmission = false;
    }
  }

  // If this is a new transmission or the size of the TB has changed
  if (is_new_transmission || (cur_grant.tb[tid].tbs != grant.tb[tid].tbs)) {
    if (!is_new_transmission) {
      Warning("DL PID %d: Size of dci changed during a retransmission %d!=%d\n",
              pid,
              cur_grant.tb[tid].tbs,
              grant.tb[tid].tbs);
    }
    ack    = false;
    n_retx = 0;
    srslte_softbuffer_rx_reset_tbs(&softbuffer, grant.tb[tid].tbs * 8);
  }

  // this is a re-transmission, check if it belongs to previous decoding
  if(!is_new_transmission && (cur_grant.sl_tti + cur_grant.sl_gap != grant.sl_tti)) {
    printf("--> grants seem to be unrelated, resetting\n");
    ack    = false;
    n_retx = 0;
    srslte_softbuffer_rx_reset_tbs(&softbuffer, grant.tb[tid].tbs * 8);
  }

  n_retx++;

  // If data has not yet been successfully decoded
  if (!ack) {

    // Save dci
    cur_grant = grant;

    if (payload_buffer_ptr) {
      Warning("DL PID %d: Allocating buffer already allocated. Deallocating.\n", pid);
      if (!is_bcch) {
        harq_entity->demux_unit->deallocate(payload_buffer_ptr);
      }
    }

    // Instruct the PHY To combine the received data and attempt to decode it
    if (is_bcch) {
      payload_buffer_ptr = harq_entity->demux_unit->request_buffer_bcch(cur_grant.tb[tid].tbs);
    } else {
      payload_buffer_ptr = harq_entity->demux_unit->request_buffer(cur_grant.tb[tid].tbs);
    }

    action->tb[tid].payload = payload_buffer_ptr;
    if (!action->tb[tid].payload) {
      Error("Can't get a buffer for TBS=%d\n", cur_grant.tb[tid].tbs);
      return;
    }

    action->tb[tid].enabled       = true;
    action->tb[tid].rv            = cur_grant.tb[tid].rv;
    action->tb[tid].softbuffer.rx = &softbuffer;
  } else {
    Warning("DL PID %d: Received duplicate TB%d. Discarting and retransmitting ACK (n_retx=%d, reset=%s)\n",
            pid,
            tid,
            n_retx,
            n_retx > RESET_DUPLICATE_TIMEOUT ? "yes" : "no");
    if (n_retx > RESET_DUPLICATE_TIMEOUT) {
      reset(false);
    }
  }

  if (is_bcch || harq_entity->timer_aligment_timer->is_expired()) {
    // Do not generate ACK
    action->generate_ack = false;
  } else {
    action->generate_ack = true;
  }
}

void dl_harq_entity::dl_harq_process::dl_tb_process::tb_decoded(mac_interface_phy_lte::mac_grant_dl_t grant,
                                                                bool*                                 ack_ptr)
{
  if (payload_buffer_ptr) {
    this->ack = *ack_ptr;
    if (ack) {
      if (is_bcch) {
        if (harq_entity->pcap) {
          harq_entity->pcap->write_dl_sirnti(payload_buffer_ptr, cur_grant.tb[tid].tbs, ack, 0);
        }
        Debug("Delivering PDU=%d bytes to Dissassemble and Demux unit (BCCH)\n", cur_grant.tb[tid].tbs);
        harq_entity->demux_unit->push_pdu_bcch(payload_buffer_ptr, cur_grant.tb[tid].tbs);
      } else {
        if (harq_entity->pcap) {
          harq_entity->pcap->write_dl_crnti(payload_buffer_ptr, cur_grant.tb[tid].tbs, cur_grant.rnti, ack, grant.sl_lte_tti,
                                            grant.sl_snr, grant.sl_rsrp, grant.sl_rssi, grant.sl_noise_power, grant.sl_rx_full_secs,
                                            grant.sl_rx_frac_secs, grant.sl_rx_gain, grant.sl_sci_frl);
        }
        if (cur_grant.rnti == harq_entity->rntis->temp_rnti) {
          Debug("Delivering PDU=%d bytes to Dissassemble and Demux unit (Temporal C-RNTI)\n", cur_grant.tb[tid].tbs);
          harq_entity->demux_unit->push_pdu_temp_crnti(payload_buffer_ptr, cur_grant.tb[tid].tbs);

          // If T-CRNTI, update ack value with result from contention resolution
          *ack_ptr = harq_entity->demux_unit->get_uecrid_successful();

        } else if (cur_grant.rnti == SRSLTE_RNTI_SL_PLACEHOLDER) {
          Debug("SL: Delivering PDU=%d bytes to Dissassemble and Demux unit\n", cur_grant.tb[tid].tbs);

          // @todo: fix this last parameter, is last version it was cur_grant.tti
          harq_entity->demux_unit->push_pdu_sl(payload_buffer_ptr, cur_grant.tb[tid].tbs, 0);

          // Compute average number of retransmissions per packet
          harq_entity->average_retx = SRSLTE_VEC_CMA((float)n_retx, harq_entity->average_retx, harq_entity->nof_pkts++);

        } else {
          Debug("Delivering PDU=%d bytes to Dissassemble and Demux unit\n", cur_grant.tb[tid].tbs);
          harq_entity->demux_unit->push_pdu(payload_buffer_ptr, cur_grant.tb[tid].tbs);

          // Compute average number of retransmissions per packet
          harq_entity->average_retx = SRSLTE_VEC_CMA((float)n_retx, harq_entity->average_retx, harq_entity->nof_pkts++);
        }
      }

    } else if (!is_bcch) {
      harq_entity->demux_unit->deallocate(payload_buffer_ptr);
    }

    payload_buffer_ptr = NULL;

    Info("DL %d (TB %d):  %s tbs=%d, rv=%d, ack=%s, ndi=%d\n",
         pid,
         tid,
         is_new_transmission ? "newTX" : "reTX ",
         cur_grant.tb[tid].tbs,
         cur_grant.tb[tid].rv,
         ack ? "OK" : "KO",
         cur_grant.tb[tid].ndi);
  }

  pthread_mutex_unlock(&mutex);

  if (ack && is_bcch) {
    reset();
  }
}

// Determine if it's a new transmission 5.3.2.2
bool dl_harq_entity::dl_harq_process::dl_tb_process::calc_is_new_transmission(
    mac_interface_phy_lte::mac_grant_dl_t grant)
{

  if (((grant.tb[tid].ndi_present &&
        grant.tb[tid].ndi != cur_grant.tb[tid].ndi) || // 1st condition (NDI provided and has changed)
       (is_bcch && grant.tb[tid].rv == 0) ||           // 2nd condition (Broadcast and 1st transmission)
       is_first_tb))                                   // 3rd condition (is first tx for this tb)
  {
    is_first_tb         = false;
    is_new_transmission = true;
  } else {
    is_new_transmission = false;
  }

  Debug("Set HARQ for %stransmission\n", is_new_transmission ? "new " : "re");

  return is_new_transmission;
}

} // namespace srsue
