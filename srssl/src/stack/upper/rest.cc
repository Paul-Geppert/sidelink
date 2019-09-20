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

#include <srssl/hdr/upper/rest.h>

#include <jansson.h>

namespace srsue {

// global object, where our rest api lives
rest g_restapi;



/*****
 * 
 *  These are all REST endpoints
 * 
 */
 
static int rest_get_metrics (const struct _u_request * request, struct _u_response * response, void * user_data) {
  phy_common * _this = (phy_common *)user_data;

  json_t * json_body = json_pack("{sfsfsfsfsfsfsf}",
                                  "snr_psbch", _this->snr_psbch,
                                  "rsrp_psbch", _this->rsrp_psbch,
                                  "snr_ue_0", _this->snr_pssch_per_ue[0],
                                  "snr_ue_1", _this->snr_pssch_per_ue[1],
                                  "snr_ue_2", _this->snr_pssch_per_ue[2],
                                  "snr_ue_3", _this->snr_pssch_per_ue[3],
                                  "snr_ue_4", _this->snr_pssch_per_ue[4]);
                                  

  char *resp = json_dumps(json_body, JSON_REAL_PRECISION(2));
  ulfius_set_string_body_response(response, 200, resp);

  free(resp);
  json_decref(json_body);

  return U_CALLBACK_CONTINUE;
}


static int rest_get_repo_cb (const struct _u_request * request, struct _u_response * response, void * user_data) {
  phy_common * _this = (phy_common *)user_data;

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


static int rest_put_repo_cb (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phy_common * _this = (phy_common *)user_data;

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

  return rest_get_repo_cb(request, response, user_data);
}


static int rest_get_gain (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phy_common * _this = (phy_common *)user_data;

  json_t * json_body = json_pack("{sfsf}",
                                  "tx_gain", _this->get_radio()->get_tx_gain(0),
                                  "rx_gain", _this->get_radio()->get_rx_gain(0));
                                  
  ulfius_set_json_body_response(response, 200, json_body);
  json_decref(json_body);

  return U_CALLBACK_CONTINUE;
}  


static int rest_put_gain (const struct _u_request * request, struct _u_response * response, void * user_data) {

  phy_common * _this = (phy_common *)user_data;
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
  int ret = json_unpack(req, "{s?i,s?i}",
                        "tx_gain", &tx_gain,
                        "rx_gain", &rx_gain);

  json_decref(req);

  if(ret < 0) {
    ulfius_set_string_body_response(response, 400, "Failed to parse gain values(They need to be intergers).");
    return U_CALLBACK_CONTINUE;
  }

  if(GAIN_NO_CHANGE != tx_gain) {
    printf("Apply new tx gain: %i\n", tx_gain);
    _this->get_radio()->set_tx_gain(0, (float)tx_gain);
  }

  if(GAIN_NO_CHANGE != rx_gain) {
    printf("Apply new rx gain: %i\n", rx_gain);
    _this->get_radio()->set_rx_gain(0, (float)rx_gain);
  }

  // send current setting to user
  return rest_get_gain(request, response, user_data);
}

static int rest_get_SPS_rsrp(const struct _u_request* request, struct _u_response* response, void* user_data)
{
  phy_common* _this = (phy_common*)user_data;

  json_t* arr_sps_rsrp = json_array();
  int s = sizeof(_this->inst_sps_rsrp)/sizeof(_this->inst_sps_rsrp[0]);
  
  for (int i = 0; (i <= s) && (_this->inst_sps_rsrp[i] != 0); i++)
  {
    json_array_append_new(arr_sps_rsrp, json_real(_this->inst_sps_rsrp[i]));
  }
  
  ulfius_set_json_body_response(response, 200, arr_sps_rsrp);

  json_decref(arr_sps_rsrp);

  return U_CALLBACK_CONTINUE;
}

static int rest_get_SPS_rssi(const struct _u_request* request, struct _u_response* response, void* user_data)
{
  phy_common* _this = (phy_common*)user_data;

  json_t* arr_sps_rssi = json_array();
  int s = sizeof(_this->inst_sps_rssi)/sizeof(_this->inst_sps_rssi[0]);
  
  for (int i = 0; i < s; i++)
  {
    json_array_append_new(arr_sps_rssi, json_real(_this->inst_sps_rssi[i]));
  }

  char *resp = json_dumps(arr_sps_rssi, JSON_REAL_PRECISION(2));
  ulfius_set_string_body_response(response, 200, resp);

  free(resp);
  json_decref(arr_sps_rssi);

  return U_CALLBACK_CONTINUE;
}


/**
 * Default callback function called if no endpoint has a match
 */
int callback_default (const struct _u_request * request, struct _u_response * response, void * user_data) {
  ulfius_set_string_body_response(response, 404, "Page not found, do what you want");
  return U_CALLBACK_CONTINUE;
}


void rest::init(unsigned int port){
  //mue =_ue;

  if (ulfius_init_instance(&instance, port, NULL, NULL) != U_OK) {
    printf("Error ulfius_init_instance, abort\n");
  }

  // Maximum body size sent by the client is 1 Kb
  instance.max_post_body_size = 1024;

  // ulfius_add_endpoint_by_val(&instance, "GET", PREFIX, NULL, 0, &srsue::callback_get_test, NULL);

  // default_endpoint declaration
  int ret = ulfius_set_default_endpoint(&instance, &callback_default, NULL);
  if(U_OK != ret) {
    printf("Error: Failed to set default endpoint.\n");
  }
}


void rest::init_and_start(phy_common * this_){

  uint32_t port = 13000 + this_->args->sidelink_id;

  rest::init(port);

  printf("REST API is attached to port %d\n", port);

  int ret = U_OK;
  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/repo", NULL, 0, &srsue::rest_get_repo_cb, this_);
  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "PUT", "/phy/repo", NULL, 0, &srsue::rest_put_repo_cb, this_);

  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/metrics", NULL, 0, &srsue::rest_get_metrics, this_);

  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/gain", NULL, 0, &srsue::rest_get_gain, this_);
  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "PUT", "/phy/gain", NULL, 0, &srsue::rest_put_gain, this_);

  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/SPS_rsrp", NULL, 0, &srsue::rest_get_SPS_rsrp, this_);
  ret += ulfius_add_endpoint_by_val(&g_restapi.instance, "GET", "/phy/SPS_rssi", NULL, 0, &srsue::rest_get_SPS_rssi, this_);
  
  if(U_OK != ret) {
    printf("Error: failed to add endpoints for rest api\n");
  }

  rest::start();
}


void rest::start() {
  if(NULL == instance.endpoint_list) {
    // add a dummy endpoint
    ulfius_add_endpoint_by_val(&instance, "GET", "/dummy_rest_info", NULL, 1, &callback_default, NULL);
  }

  // Open an http connection
  int ret = ulfius_start_framework(&instance);
  if(ret != U_OK) {
    printf("Error starting REST framework, error code is: %d\n", ret);
  }
}

void rest::stop() {
  ulfius_stop_framework(&instance);
  ulfius_clean_instance(&instance);
}


} // namespace srsue