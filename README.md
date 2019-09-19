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
cmake -DUSE_LTE_RATES=y ../
make
```

Execution Instructions
----------------------

After building, the srssl/ue.conf.example can be modified to your needs.
Setting frequency, gain values and other options. The default configuration is for a B210 locally connected via USB.

Easy start of the "master" node just use the start.sh script in the main folder.
Additionally you can use the start_client.sh script in the main folder for client mode operation with sidelink_id=2.

Runntime information
--------------------

Restapi is attached to port 1300 + given expert.phy.sidelink_id i.e. 13001
The stack generates a tunnel interface called tun_srssl with IP: 10.0.2.10+sidelink_id i.e. 10.0.2.11
Any IP package is routed to air and can be decoded by any other node.


REST API
--------------------------
During runtime the REST API can be used to get status information and change settings.

* Gain settings
Readout:
```
curl -X GET localhost:13001/phy/gain
```
Example:
```
{"tx_gain":57.0,"rx_gain":41.0}
```

Set new values:
```
curl -X PUT -H "Content-Type: application/json" -d '{"rx_gain":60, "tx_gain":35}' localhost:13001/phy/gain
```

* Measurement data
To get SNR values in db of differnt sidelink_id UEs and PSBCH use:
```
curl -X GET localhost:13001/phy/metrics
```
Example:
```
{"snr_psbch":8.4800424575805664,"snr_ue_0":0.0,"snr_ue_1":8.6331901550292969,"snr_ue_2":21.439271926879883,"snr_ue_3":0.0,"snr_ue_4":0.0}
```
This can be used for estimating the link quality and find optimal gain values. If the SNR is near 30db the RX may saturated and lower values make more sense.

* Resource pool configuration
We can change the physical layer resource pool configuration via REST API.
Readout:
```
curl -X GET localhost:13001/phy/repo
```

Example:
```
{"numSubchannel_r14":10,"sizeSubchannel_r14":5,"sl_OffsetIndicator_r14":0,"sl_Subframe_r14_len":20,"startRB_PSCCH_Pool_r14":0,"startRB_Subchannel_r14":0}
```

The resource pool configuration should be the same on all nodes in the network, otherwise there will be an error information on stdout.

Any field can be set independent or all at once. To set the resource pool configuration:
```
curl -X PUT -H "Content-Type: application/json" -d '{"numSubchannel_r14":10,"sizeSubchannel_r14":5,"sl_OffsetIndicator_r14":0,"sl_Subframe_r14_len":20,"startRB_PSCCH_Pool_r14":0,"startRB_Subchannel_r14":0}' localhost:13001/phy/repo
```
