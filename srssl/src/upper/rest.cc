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

// extern "C" {
//   #include "ulfius.h"
// }  

rest g_restapi;

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
  ulfius_set_default_endpoint(&instance, &callback_default, NULL);

}

void rest::add(char *prefix, int (* callback_function)(const struct _u_request * request, // Input parameters (set by the framework)
                                                        struct _u_response * response,     // Output parameters (set by the user)
                                                        void * user_data)){

  ulfius_add_endpoint_by_val(&instance, "GET", prefix, NULL, 0, callback_function, NULL);
}

void rest::start() {
  // Open an http connection
  int ret = ulfius_start_framework(&instance);
  if(ret != U_OK) {
    printf("Error starting framework\n");
  }
}


} // namespace srsue
