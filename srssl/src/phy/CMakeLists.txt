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

file(GLOB SOURCES "*.cc")
add_library(srssl_phy STATIC ${SOURCES})

if(ENABLE_GUI AND SRSGUI_FOUND)
  target_link_libraries(srssl_phy ${SRSGUI_LIBRARIES})
endif(ENABLE_GUI AND SRSGUI_FOUND)

install(TARGETS srssl_phy DESTINATION ${LIBRARY_DIR})
