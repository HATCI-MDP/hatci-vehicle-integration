#!/bin/bash

ABS_PATH_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA=$1
#NRU=$(lsmod | grep nru | awk '{print $1}')
NRU=$(lsmod | grep nv_nru | awk '{print $1}' | head -n 1)

if [ -n "$NRU" ]; then
  echo nvidia | sudo -S rmmod $NRU 2>/dev/null 
fi

if [ "$CAMERA" = "IMX390" ]; then
  NRU="nv_nru_imx390.ko"
elif [ "$CAMERA" = "IMX490" ]; then
  NRU="nv_nru_imx490.ko"
elif [ "$CAMERA" = "ISX021" ]; then
  NRU="nv_nru_isx021.ko"
elif [ "$CAMERA" = "ISX031" ]; then
  NRU="nv_nru_isx031.ko"
else
  echo "Not support camera."
  exit 1
fi

echo "Initializing ..."
cd $ABS_PATH_WS
echo nvidia | sudo -S sudo insmod $NRU 2>/dev/null
echo nvidia | sudo -S sudo ./init_8xIMXCam.sh
echo nvidia | sudo -S sudo ./mv_addr.sh

# ./gst-launch_all.sh
# sleep 0.5
