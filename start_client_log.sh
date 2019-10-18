#!/bin/bash
TIMESTAMP=`date +"%Y%m%d%H%M%Sh"`
MEASUREFILE="robin_${TIMESTAMP}_sidelink_log.txt"
DATADIR="/home/vw/working/measurements"
OUTFILE="${DATADIR}/${MEASUREFILE}"
sudo build/srssl/src/srssl srssl/ue.conf.example --expert.phy.sidelink_id 2 --expert.phy.sidelink_master 0 --log.filename $OUTFILE --log.phy_level "info" --log.phy_lib_level "info"
