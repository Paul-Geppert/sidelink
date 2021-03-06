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

#ifndef SRSUE_UE_STACK_BASE_H
#define SRSUE_UE_STACK_BASE_H

#include "srslte/common/logger.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srssl/hdr/ue_metrics_interface.h"

#include "rrc/rrc.h"
#include "upper/gw.h"
#include "upper/usim.h"

namespace srsue {

typedef struct {
  bool        enable;
  std::string filename;
  bool        nas_enable;
  std::string nas_filename;
} pcap_args_t;

typedef struct {
  std::string mac_level;
  std::string rlc_level;
  std::string pdcp_level;
  std::string rrc_level;
  std::string gw_level;
  std::string nas_level;
  std::string usim_level;

  int mac_hex_limit;
  int rlc_hex_limit;
  int pdcp_hex_limit;
  int rrc_hex_limit;
  int gw_hex_limit;
  int nas_hex_limit;
  int usim_hex_limit;
} stack_log_args_t;

typedef struct {
  std::string      type;
  pcap_args_t      pcap;
  stack_log_args_t log;
  usim_args_t      usim;
  rrc_args_t       rrc;
  std::string      ue_category_str;
  nas_args_t       nas;
} stack_args_t;

class ue_stack_base
{
public:
  ue_stack_base()          = default;
  virtual ~ue_stack_base() = default;

  virtual std::string get_type() = 0;

  virtual void stop()             = 0;
  virtual bool switch_on()        = 0;
  virtual bool switch_off()       = 0;

  // UE metrics interface
  virtual bool get_metrics(stack_metrics_t* metrics) = 0;
};

} // namespace srsue

#endif // SRSUE_UE_BASE_H
