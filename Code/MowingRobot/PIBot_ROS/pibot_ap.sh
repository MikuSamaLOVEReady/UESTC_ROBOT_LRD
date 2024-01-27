#!/bin/bash

# put this cmd to startup scritps
# create pibot ap
# if [ -f /home/pibot/.pibot_ap ]; then
#     create_ap wlan0 eth0 pibot_ap pibot_ap&
# else
#     create_ap --fix-unmanaged
#     ifconfig eth0 up
#     ifconfig wlan0 up
# fi

function start() {
	echo "start ap mode"
	touch ~/.pibot_ap
	# sudo nmcli c | awk -F' ' '{cmd="sudo nmcli c del "$1; if(NR>=2) system(cmd)}'
	sudo nmcli --fields UUID con | awk '{print $1}' | while read line; do sudo nmcli con delete uuid $line; done
	sudo systemctl restart create_ap
	sudo systemctl enable create_ap
}

function stop() {
	echo "stop ap mode"
	if [ -f ~/.pibot_ap ]; then
		rm ~/.pibot_ap
	fi
	sudo systemctl stop create_ap
	sudo systemctl disable create_ap
}

case "$1" in
	start )
	echo  "****************"
	start
	echo  "****************"
	;;
	stop )
	echo  "****************"
	stop
	echo  "****************"
	;;
	* )
	echo  "****************"
	echo  "$0 start/stop"
	echo  "****************"
esac
