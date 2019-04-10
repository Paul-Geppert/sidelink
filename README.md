sidelink
========

Copyright
--------

 Copyright 2013-2019
 Fraunhofer Institute for Telecommunications, Heinrich-Hertz-Institut (HHI)

 This file is part of the HHI Sidelink.
 
 HHI Sidelink is under the terms of the GNU Affero General Public License
 as published by the Free Software Foundation version 3.

 HHI Sidelink is distributed WITHOUT ANY WARRANTY, 
 without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 
 A copy of the GNU Affero General Public License can be found in
 the LICENSE file in the top-level directory of this distribution
 and at http://www.gnu.org/licenses/.

 The HHI Sidelink is based on srsLTE. 
 All necessary files and sources from srsLTE are part of HHI Sidelink.
 srsLTE is under Copyright 2013-2017 by Software Radio Systems Limited.
 srsLTE can be found under: 
 https://github.com/srsLTE/srsLTE

Hardware
--------

We have tested the following hardware: 
 * USRP B210
 * USRP X300
 * USRP N310

Build Instructions
------------------
Based on Ubuntu the following packages are needed for building.

```
sudo apt-get install build-essential cmake libfftw3-dev libmbedtls-dev libboost-program-options-dev libconfig++-dev libsctp-dev libboost-all-dev libusb-1.0-0-dev python-mako doxygen python-docutils libudev-dev python-numpy python-requests python-setuptools libulfius-dev libjansson-dev libmicrohttpd-dev cpufrequtils

```

Note that depending on your flavor and version of Linux, the actual package names may be different.

* RF front-end driver:
  * UHD:                 https://github.com/EttusResearch/uhd

Please have a working UHD driver installed!


actual build

```
mkdir build
cd build
cmake ../
make
```

Execution Instructions
----------------------

You need to modify the start.sh file to adapt to your local SDR configuration.
The script assumes a locally connected B210 via USB.

Easy start of the "master" node just use the start.sh script in the main folder.

There are various configuration options. Looking into the start.sh in the main directory can help to understand.

* --rf.device_args "type=b200" # UHD device arguments
* --log.all_level "info" # setting the log-level
* --rf.continuous_tx "no"  # needed for TDD operation
* --rf.dl_freq 2350000000 # downlink frequency set to 2350 MHz
* --rf.ul_freq 2350000000 # uplink frequency set to 2350 MHz
* --expert.phy.sidelink_id 1 # this parameter sets the node ID to 1 out of 4
* --rf.rx_gain 40 # some rx_gain factor, depending on the hardware this can be also increased
* --rf.tx_gain 30 # a tx_gain factor, depending on the hardware this can be increased
* --rf.burst_preamble_us 10 # tuning parameter
* --expert.phy.sidelink_master 1 # enables master_node operation so no cell_search is active

Runntime information
--------------------

Restapi is attached to port 1300 + given expert.phy.sidelink_id i.e. 13001

Date Socket is client listening on port 2200 + given expert.phy.sidelink_id i.e. 22001


