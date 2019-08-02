#!/usr/bin/env bash

set -eu

# trap "set +x; sleep 5; set -x" DEBUG

# Check whether we are running sudo
if [[ $EUID -ne 0 ]]; then
  	echo "This script must be run as root" 1>&2
  	exit 1
fi


echo '================================================================================ '
echo '|                                                                               |'
echo '|                           Waveshare Electronic Team                           |'
echo '|             Power-Management-HAT Installation Script - Raspbian only                 |'
echo '|                                                                               |'
echo '================================================================================ '

##Update and upgrade
#sudo apt-get update && sudo apt-get upgrade -y


echo 'Begin Installation ? (y/n) '
read ReadyInput
if [[ "$ReadyInput" == "Y" || "$ReadyInput" == "y" ]]; then
    echo "Beginning installation..."
else
    echo "Aborting installation"
    exit 0
fi

##-------------------------------------------------------------------------------------------------
##-------------------------------------------------------------------------------------------------
## Test Area
# echo every line 
set +x

# exit 0
## End Test Area

##-------------------------------------------------------------------------------------------------
##-------------------------------------------------------------------------------------------------


##-------------------------------------------------------------------------------------------------
## Getting Sleepy Pi to shutdown the Raspberry Pi
echo 'Setting up the shutdown...'
cd ~
if grep -q 'StatusDetection.py' /etc/rc.local; then
    echo 'StatusDetection.py is already setup - skipping...'
else
    mkdir -p /home/pi/bin
    mkdir -p /home/pi/bin/PowerManagementHAT
    wget https://raw.githubusercontent.com/waveshare/Power-Management-HAT/master/StatusDetection.py
    mv -f StatusDetection.py /home/pi/bin/PowerManagementHAT
    sed -i '/exit 0/i python /home/pi/bin/PowerManagementHAT/StatusDetection.py &' /etc/rc.local
    # echo "python /home/pi/bin/PowerManagementHAT/StatusDetection.py &" | sudo tee -a /etc/rc.local
fi



##-------------------------------------------------------------------------------------------------
# complete
echo "Sleepy Pi setup complete!"
echo  "Would you like to reboot now? y/n"
read RebootInput
if [[ "$RebootInput" == "Y" || "$RebootInput" == "y" ]]; then
    echo "Now rebooting..."
    sleep 3
    reboot
fi
exit 0
##-------------------------------------------------------------------------------------------------



