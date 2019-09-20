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

#ifndef SRSLTE_BUILD_INFO_H_IN
#define SRSLTE_BUILD_INFO_H_IN

#include "srslte/config.h"

# ifdef __cplusplus
extern "C" {
# endif

#ifdef BUILD_TYPE_RELEASE
  static char build_mode[] = "Release";
#else
  #ifdef BUILD_TYPE_DEBUG
    static char build_mode[] = "Debug";
  #else
    #ifdef BUILD_TYPE_RELWITHDEBINFO
      static char build_mode[] = "RelWithDebInfo";
    #else
      static char build_mode[] = "unknown";
    #endif
  #endif
#endif

// the configured build options for srsLTE
static char build_info[] = "commit 998010b on branch master";

SRSLTE_API char* srslte_get_build_info() {
  return build_info;
};

SRSLTE_API char* srslte_get_build_mode() {
  return build_mode;
}

# ifdef __cplusplus
}
# endif

#endif // BUILD_INFO_