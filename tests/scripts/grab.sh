#!/bin/bash

grab_data() {
	tail -n 1 $1 > /dev/ttyACM0
	echo >> /dev/ttyACM0
	dd if="/dev/ttyACM0" of="$2"
}

pause() {
   read -p "$*"
}

firstfile='_firstcrt.csv'
secondfile='_firstocclude.csv'
thirdfile='_secondcrt.csv'
forthfile='_secondocclude.csv'
stty -F /dev/ttyACM0 raw
echo 'Commencing refills'
grab_data settings_crt.txt "$1$firstfile" &
pause 'Press enter to kill data aquisition and move to occlusion stage of experiment'
kill %1
echo '#' > /dev/ttyACM0
sleep 5
grab_data settings_occlude.txt "$1$secondfile" &
pause 'Press enter to kill data aquisition and move to water bath stage of experiment'
kill %1
echo '#' > /dev/ttyACM0
pause 'Reset board then press enter to commence post water bath refills, wait for flashing LED'
grab_data settings_crt.txt "$1$thirdfile" &
pause 'Press enter to kill data aquisition and move to  bath stage of experiment'
kill %1
echo '#' > /dev/ttyACM0
sleep 5
grab_data settings_occlude.txt "$1$forthfile" &
pause 'Press enter to kill data aquisition, completing experiment'
kill %1
