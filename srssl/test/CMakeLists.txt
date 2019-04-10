#/**
#* Copyright 2013-2019 
#* Fraunhofer Institute for Telecommunications, Heinrich-Hertz-Institut (HHI)
#*
#* This file is part of the HHI Sidelink.
#*
#* HHI Sidelink is under the terms of the GNU Affero General Public License
#* as published by the Free Software Foundation version 3.
#*
#* HHI Sidelink is distributed WITHOUT ANY WARRANTY,
#* without even the implied warranty of
#* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#*
#* A copy of the GNU Affero General Public License can be found in
#* the LICENSE file in the top-level directory of this distribution
#* and at http://www.gnu.org/licenses/.
#*
#* The HHI Sidelink is based on srsLTE.
#* All necessary files and sources from srsLTE are part of HHI Sidelink.
#* srsLTE is under Copyright 2013-2017 by Software Radio Systems Limited.
#* srsLTE can be found under:
#* https://github.com/srsLTE/srsLTE
#*/

#
# Copyright 2013-2017 Software Radio Systems Limited
#
# This file is part of srsLTE
#
# srsLTE is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsLTE is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

#add_subdirectory(phy)
#add_subdirectory(mac)
#add_subdirectory(upper)

#add_executable(metrics_test metrics_test.cc ../src/metrics_stdout.cc ../src/metrics_csv.cc)
#target_link_libraries(metrics_test srslte_phy srslte_common)
#add_test(metrics_test metrics_test -o ${CMAKE_CURRENT_BINARY_DIR}/ue_metrics.csv)


if(ENABLE_REST)
    add_executable(rest_test rest_test.cc)
    target_link_libraries(rest_test ${ULFIUS_LIBRARIES} ${JANSSON_LIBRARIES})
    add_test(rest_test rest_test)
endif(ENABLE_REST)
