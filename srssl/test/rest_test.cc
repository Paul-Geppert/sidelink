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

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <cstring>
#include <unistd.h>

extern "C" {
  #include "ulfius.h"
}  

#include <srslte/phy/phch/repo.h>
#include <srssl/hdr/upper/rest.h>

  SL_CommResourcePoolV2X_r14 rp = {
    0, //uint8_t sl_OffsetIndicator_r14;

    // subframe bitmap
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},//uint8_t sl_Subframe_r14[100];
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

  // if (srslte_repo_init(&ue_repo, cell, respool)) {
  //   fprintf(stderr, "Error creating Ressource Pool object\n");
  // }

  // // set sync period to 5, as this is what the current sync implementations expects
  // ue_repo.syncPeriod = 5;
  // srslte_repo_build_resource_pool(&ue_repo);




static int callback_get_test (const struct _u_request * request, struct _u_response * response, void * user_data) {

  json_t *json_body = json_pack("{sisisisisisisi}",
                                "numSubchannel_r14",      rp.numSubchannel_r14,
                                "sizeSubchannel_r14",     rp.sizeSubchannel_r14,
                                "sl_OffsetIndicator_r14", rp.sl_OffsetIndicator_r14,
                                "sl_Subframe_r14_len",    rp.sl_Subframe_r14_len,
                                "startRB_PSCCH_Pool_r14", rp.startRB_PSCCH_Pool_r14,
                                "startRB_Subchannel_r14", rp.startRB_Subchannel_r14,
                                "sizeSubchannel_r14",     rp.sizeSubchannel_r14);

  printf("json_pack returned %x\n", json_body);
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);
  return U_CALLBACK_CONTINUE;
}


static int callback_put_test (const struct _u_request * request, struct _u_response * response, void * user_data) {

  json_error_t json_error;
  json_t * req = ulfius_get_json_body_request(request, &json_error);

  if(NULL == req) {
    ulfius_set_string_body_response(response, 400, json_error.text);
    return U_CALLBACK_CONTINUE;
  }

  printf("ulfius_get_json_body_request returned %s\n", json_error.text);
  printf("ulfius_get_json_body_request returned %d\n", req);

    char *jd = json_dumps(req, 0);
  printf("%s\n", jd);

  SL_CommResourcePoolV2X_r14 new_repo;
  memcpy(&new_repo, &rp, sizeof(SL_CommResourcePoolV2X_r14));

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
  if(0 != memcmp(&new_repo, &rp, sizeof(SL_CommResourcePoolV2X_r14))) {
    printf("Apply new repo settings\n");
  }


  return callback_get_test(request, response, user_data);

  json_t *json_body = json_pack("{sisisisisisisi}",
                                "numSubchannel_r14",      new_repo.numSubchannel_r14,
                                "sizeSubchannel_r14",     new_repo.sizeSubchannel_r14,
                                "sl_OffsetIndicator_r14", new_repo.sl_OffsetIndicator_r14,
                                "sl_Subframe_r14_len",    new_repo.sl_Subframe_r14_len,
                                "startRB_PSCCH_Pool_r14", new_repo.startRB_PSCCH_Pool_r14,
                                "startRB_Subchannel_r14", new_repo.startRB_Subchannel_r14,
                                "sizeSubchannel_r14",     new_repo.sizeSubchannel_r14);
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);

  //ulfius_set_string_body_response(response, 200, "Hello World! from /test");
  //mue->phy->workers_common
  return U_CALLBACK_CONTINUE;
}


int main(int argc, char **argv)
{
  srsue::rest r;
  //r.init();
  if (ulfius_init_instance(&r.instance, 9078, NULL, NULL) != U_OK) {
    printf("Error ulfius_init_instance, abort\n");
  }

  srslte_repo_t ue_repo;
  srslte_cell_t cell;

  // if (srslte_repo_init(&ue_repo, cell, respool)) {
  //   fprintf(stderr, "Error creating Ressource Pool object\n");
  // }

  json_t * json_body = NULL;
  
  json_body = json_object();
  json_object_set_new(json_body, "nbsheep", json_integer(1337));

  json_object_set_new(json_body, "e1", json_integer(2));
  json_object_set_new(json_body, "bitmap", json_array());

  json_t *arr = json_array();
  json_array_append_new(arr, json_integer(5));
  json_array_append_new(arr, json_integer(15));
  json_array_append_new(arr, json_integer(25));

  json_object_set_new(json_body, "bitmap2", arr);

  json_object_set_new(json_object_get(json_body, "bitmap"), "1", json_integer(1));

  

  char *jd = json_dumps(json_body, 0);
  printf("%s\n", jd);

  json_error_t error;
  json_t * jl = json_loads(jd, JSON_DECODE_ANY, &error);

  jd = json_dumps(jl, 0);
  printf("Decoded: %s\n", jd);

  int dec_sheep = 0;
  int dec_e1 = 0;
  json_unpack(jl, "{s?i,s?i}", "e1", &dec_e1, "nbsheep", &dec_sheep);

  printf("dec_sheep %d\n", dec_sheep);
  printf("dec_e1 %d\n", dec_e1);

    json_body = json_pack("{sssssi}",
        "error", "resource not found", "message", "no resource available at this address", "bal", 45);
        jd = json_dumps(json_body, 0);
  printf("%s\n", jd);




  json_body = json_pack("{sisisisisisisi}",
                                  "numSubchannel_r14",      rp.numSubchannel_r14,
                                  "sizeSubchannel_r14",     rp.sizeSubchannel_r14,
                                  "sl_OffsetIndicator_r14", rp.sl_OffsetIndicator_r14,
                                  "sl_Subframe_r14_len",    rp.sl_Subframe_r14_len,
                                  "startRB_PSCCH_Pool_r14", rp.startRB_PSCCH_Pool_r14,
                                  "startRB_Subchannel_r14", rp.startRB_Subchannel_r14,
                                  "sizeSubchannel_r14",     rp.sizeSubchannel_r14);
                                  
  // ulfius_set_json_body_response(response, 200, json_body);
  // json_decref(json_body);

  jd = json_dumps(json_body, 0);
  printf("%s\n", jd);


  int ret;

  ret = ulfius_add_endpoint_by_val(&r.instance, "GET", "/", NULL, 0, callback_get_test, NULL);
  if(ret != U_OK) {
    printf("Error starting callback_get_test %d\n", ret);
  }

  ret = ulfius_add_endpoint_by_val(&r.instance, "PUT", "/phy/repo", NULL, 0, callback_put_test, NULL);
  if(ret != U_OK) {
    printf("Error starting callback_put_test %d\n", ret);
  }



  ulfius_set_default_endpoint(&r.instance, &callback_get_test, NULL);

  ret = ulfius_start_framework(&r.instance);
  if(ret != U_OK) {
    printf("Error starting framework %d\n", ret);
  }

  std::cout << "REST test .." << std::endl;
  usleep(80e6);
  
  return 0;
}
