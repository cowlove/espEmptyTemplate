#!/bin/bash
# on host machine:
# sudo apt-get install vagrant apt-cacher-ng
if [ ! -d /vagrant ]; then 
	vagrant init hashicorp/bionic64
	vagrant up
	vagrant ssh -c /vagrant/setup.sh
	exit
fi

# TODO: check if proxy is up/exists
if [ -d /vagrant ]; then
	echo 'Acquire::http { Proxy "http://10.0.2.2:3142"; };' | sudo tee /etc/apt/apt.conf.d/01proxy
fi

sudo usermod -a -G dialout vagrant
sudo apt-get update
#sudo apt-get -y upgrade; sudo apt-get -y dist-upgrade
sudo apt-get install -y git curl build-essential 
#bash-completion gnuplot python3 python3-pip
#sudo apt-get install python-seriali freeglut3-dev

mkdir -p ${HOME}/bin
export BINDIR=${HOME}/bin
export PATH=$PATH:${HOME}/bin

arduino-cli version || curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

arduino-cli config init 
sed -i 's|additional_urls: \[\]|additional_urls: \[https://dl.espressif.com/dl/package_esp32_index.json,http://arduino.esp8266.com/stable/package_esp8266com_index.json\]|' ~/.arduino15/arduino-cli.yaml 
arduino-cli update
arduino-cli core install esp32:esp32@2.0.17
arduino-cli lib install "Adafruit ST7735 and ST7789 Library"

cd ${HOME}/Arduino/libraries 
git clone https://github.com/plerup/makeEspArduino.git

# makeEspArduino needs needs a preferences.txt file 
#echo sketchbook.path=${HOME}/Arduino >> ~/.arduino15/preferences.txt
   
 
