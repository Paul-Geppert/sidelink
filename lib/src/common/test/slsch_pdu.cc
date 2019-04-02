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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

#include "srslte/common/pdu.h"
#include "srslte/srslte.h"


int main(int argc, char **argv) {

  printf("This is slsch_pdu test\n");


  srslte::slsch_pdu sl_mac_msg(20);
  srslte::slsch_pdu sl_mac_msg_tx(20);


  uint32_t nof_bytes = 50;
  uint8_t mac_pdu[200];


  memset(mac_pdu, 0xFF, sizeof(mac_pdu));
  sl_mac_msg_tx.init_tx(mac_pdu, nof_bytes, false);

  sl_mac_msg_tx.V = 3;
  sl_mac_msg_tx.SRC[0] = 0xdd;
  sl_mac_msg_tx.SRC[1] = 0xdd;
  sl_mac_msg_tx.SRC[2] = 0xdd;
  sl_mac_msg_tx.DST[0] = 0xee;
  sl_mac_msg_tx.DST[1] = 0xee;
  sl_mac_msg_tx.DST[2] = 0xee;


  uint32_t total_sdu_len = 10;
  uint8_t sdu_payload[10];
  memset(sdu_payload, 0xbb, 10);

  for(int l=0; l<5; l++) {
    int sdu_len = rand() % sizeof(sdu_payload);//SRSLTE_MIN(total_sdu_len, (uint32_t) sdu_space);

    uint8_t lcid = 0x0a + l;
    int sdu_space = sl_mac_msg_tx.get_sdu_space();

    if(sdu_len <= sdu_space) {


      memset(sdu_payload, l+1, sizeof(sdu_payload));

      if (sl_mac_msg_tx.new_subh()) { // there is space for a new subheader
        printf("SDU:   set_sdu(), lcid=%d, sdu_len=%d, sdu_space=%d rem_size=%d\n", lcid, sdu_len, sdu_space, sl_mac_msg_tx.rem_size());
        int n = sl_mac_msg_tx.get()->set_sdu(lcid, sdu_len, sdu_payload);
        if(n<0) {
          sl_mac_msg_tx.del_subh();
        }
      } else {
        printf("Could not add SDU lcid=%d nbytes=%d, space=%d\n", lcid, sdu_len, sdu_space);
        sl_mac_msg_tx.del_subh();
      }
      srslte_vec_fprint_byte(stdout, mac_pdu, sizeof(mac_pdu));

    }else {
      printf("not enough room for packet\n");
    }

  }

  
  uint8_t *ret2 = sl_mac_msg_tx.write_packet();

  srslte_vec_fprint_byte(stdout, ret2, nof_bytes);

  sl_mac_msg_tx.fprint(stdout);

  // now we are going to decode this mac pdu

  sl_mac_msg.init_rx(nof_bytes);
  sl_mac_msg.parse_packet(ret2);

  sl_mac_msg.fprint(stdout);

  exit(0);
}
