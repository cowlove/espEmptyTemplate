#!/bin/bash
# on host machine:
# sudo apt-get install vagrant apt-cacher-ng
if [ ! -d /vagrant ]; then 
	vagrant init ubuntu/jammy64
	vagrant up
	vagrant plugin list | grep vagrant-scp || vagrant plugin install vagrant-scp
	vagrant ssh -c "mkdir -p bin/ .arduino15/"
	vagrant scp ~/bin/arduino-cli bin/arduino-cli
	vagrant scp ~/.arduino15/staging .arduino15/
	vagrant ssh -c /vagrant/setup.sh
	exit
fi

# TODO: check if proxy is up/exists
if [ -d /vagrant ]; then
	echo 'Acquire::http { Proxy "http://10.0.2.3:3142"; };' | sudo tee /etc/apt/apt.conf.d/01proxy
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
arduino-cli core install esp32:esp32
arduino-cli lib install ArduinoOTA PubSubClient HTTPClient OneWireNg \
	ArduinoJson Arduino_CRC32 "DHT sensor library"

mkdir -p ${HOME}/Arduino/libraries 
cd ${HOME}/Arduino/libraries 
git config --global user.name "Jim Evans"
git config --global user.email "jim@vheavy.com"
printf "Host *\n StrictHostKeyChecking no" >> ~/.ssh/config
chmod 600 ~/.ssh/config

git clone git@github.com:cowlove/esp32jimlib.git
git clone git@github.com:cowlove/esp32csim.git


cd ${HOME}
SKETCH=$( echo `basename /vagrant/*.ino` | sed s/.ino// )

git clone git@github.com:cowlove/${SKETCH}.git

cd ${SKETCH}
make uc

# makeEspArduino needs needs a preferences.txt file 
#echo sketchbook.path=${HOME}/Arduino >> ~/.arduino15/preferences.txt
   
 
