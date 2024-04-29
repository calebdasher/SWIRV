#!/bin/bash
switch_to_client_mode() {
  echo "Switching back to client mode..."
  nmcli radio wifi off
  nmcli radio wifi on
}

stop_access_point() {
  echo "Stopping access point..."
  nmcli device disconnect wlan0
}

stop_access_point
switch_to_client_mode
sudo systemctl restart NetworkManager

sleep 7

sudo nmcli dev wifi connect "WCU-Guest"


