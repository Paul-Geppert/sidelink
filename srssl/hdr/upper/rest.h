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

#ifndef SRSUE_REST_H
#define SRSUE_REST_H

// define must be set during configuration step
#ifdef ENABLE_REST

#include <jansson.h>

namespace srsue {


extern "C" {
  #include "ulfius.h"
}  



class rest// : public thread
{
public:

  rest(){
    printf("Constructed my ulfius\n");
  }

  void stop(){
    printf("Destructed my ulfius\n");
  }

  void init(unsigned int port);

  void add(char *prefix, int (* callback_function)(const struct _u_request * request, // Input parameters (set by the framework)
                                                         struct _u_response * response,     // Output parameters (set by the user)
                                                         void * user_data));

  void start();

  struct _u_instance instance;

private:


  // /**
  //  * Callback function that put a "Hello World!" string in the response
  //  */
  // static int callback_get_test (const struct _u_request * request, struct _u_response * response, void * user_data) {
  //   ulfius_set_string_body_response(response, 200, "Hello World!");
  //   //mue->phy->workers_common
  //   return U_CALLBACK_CONTINUE;
  // }


};

// make a global variable, which as accessed by each module which uses the REST API
extern rest g_restapi;

} // namespace srsue

#endif

#endif // SRSUE_REST_H
