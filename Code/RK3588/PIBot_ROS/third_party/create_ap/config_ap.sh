#!/bin/bash
sudo apt-get install -y hostapd

SERIAL_ID=`cat /proc/cpuinfo | grep Serial | awk -F ':' '{print \$2}'`
SHORT_SERIAL_ID=${SERIAL_ID: -8}

if [ "$SHORT_SERIAL_ID"_X!=""_X ]; then
    SHORT_SERIAL_ID=`udevadm info --name=mmcblk0 --query=property | grep ID_SERIAL | awk -F '=' '{print $2}'`
fi

if [ "$SHORT_SERIAL_ID"_X!=""_X ]; then
    SERIAL_ID=`udevadm info --name=sda --query=property | grep ID_SERIAL_SHORT | awk -F '=' '{print \$2}'`
    SHORT_SERIAL_ID=${SERIAL_ID: -8}
fi

cp create_ap.service.template create_ap.service
sed -i "s|SSID|pibot_ap_$SHORT_SERIAL_ID|g" create_ap.service

sudo make install

sudo systemctl daemon-reload

