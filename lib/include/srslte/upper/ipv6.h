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

#ifndef SRSLTE_IPV6_H
#define SRSLTE_IPV6_H

#ifdef USE_GLIBC_IPV6
#include <linux/ipv6.h>
#else
// Some versions of glibc yield to a compile error with gcc
// complaining about a redefinition of struct in6_pktinfo. See [1].
// Since we only need two structs from that header we define them here and
// just don't include the entire file. See [1] for more information.
//
// [1] https://patchwork.ozlabs.org/patch/425881/
#include <asm/byteorder.h>
struct ipv6hdr {
#if defined(__LITTLE_ENDIAN_BITFIELD)
  __u8 priority : 4, version : 4;
#elif defined(__BIG_ENDIAN_BITFIELD)
  __u8 version : 4, priority : 4;
#else
#error "Please fix <asm/byteorder.h>"
#endif
  __u8 flow_lbl[3];

  __be16 payload_len;
  __u8   nexthdr;
  __u8   hop_limit;

  struct in6_addr saddr;
  struct in6_addr daddr;
};

struct in6_ifreq {
  struct in6_addr ifr6_addr;
  __u32           ifr6_prefixlen;
  int             ifr6_ifindex;
};
#endif

#endif // SRSLTE_IPV6_H
