#!/bin/bash

restart_as_access_point() {
  echo "Starting the wireless connection..."
  nmcli radio wifi on

  
  echo "Starting the wireless access point..."
  nmcli device wifi hotspot ifname wlan0 ssid SWIRV_AP password SWIRV_AP
}

restart_as_access_point

