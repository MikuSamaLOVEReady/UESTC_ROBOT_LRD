#!/bin/bash
if [ ! -z "$PIBOT_HOME" ]; then
    PIBOT_HOME_DIR=$PIBOT_HOME
else
    PIBOT_HOME_DIR=~/pibot_ros
fi

sudo ln -sf $PIBOT_HOME_DIR/pibot_upstart/pibotenv /etc/pibotenv
sudo ln -sf $PIBOT_HOME_DIR/pibot_upstart/pibot_start.sh /usr/bin/pibot_start
sudo ln -sf $PIBOT_HOME_DIR/pibot_upstart/pibot_stop.sh /usr/bin/pibot_stop
sudo ln -sf $PIBOT_HOME_DIR/pibot_upstart/pibot_restart.sh /usr/bin/pibot_restart
sudo cp $PIBOT_HOME_DIR/pibot_upstart/pibot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pibot
sudo systemctl is-enabled pibot
