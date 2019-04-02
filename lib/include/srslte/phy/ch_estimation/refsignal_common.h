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

/**********************************************************************************************
 *  File:         refsignal_common.h
 *
 *  Description:  Object to manage common uplink and sidelink reference signals for channel estimation.
 *
 *  Reference:    3GPP TS 36.211 version 14.2.0
 *********************************************************************************************/

#ifndef SRSLTE_REFSIGNAL_COMMON_H
#define SRSLTE_REFSIGNAL_COMMON_H

#include "srslte/config.h"
#include "srslte/phy/phch/pucch.h"
#include "srslte/phy/common/phy_common.h"


SRSLTE_API void srslte_refsignal_r_uv_arg_1prb(float *arg, 
                                               uint32_t u); 

SRSLTE_API void srslte_refsignal_compute_r_uv_arg(float *arg, 
                                               uint32_t nof_prb, uint32_t u, uint32_t v);

#endif // SRSLTE_REFSIGNAL_COMMON_H
