#!/usr/bin/bash
tail -n 1 $1 > /dev/ttyACM0
echo >> /dev/ttyACM0
cat /dev/ttyACM0 > $2



grab_data() {
	tail -n 1 $1 > /dev/ttyACM0
	echo >> /dev/ttyACM0
	cat /dev/ttyACM0 > $2
}

pause() {
   read -p "$*"
}

firstfile='_firstcrt.csv'
secondfile='_firstocclude.csv'
thirdfile='_secondcrt.csv'
forthfile='_secondocclude.csv'
echo 'Commencing refills'
grab_data settings_crt.txt "$1$firstfile" &
pause 'Enter any key to kill data aquisition and move to occlusion stage of experiment'
kill %1
echo '#' > /dev/ttyACM0
sleep 5
grab_data settings_occlude.txt "$1$secondfile" &
pause 'Enter any key to kill data aquisition and move to water bath stage of experiment'
kill %2
echo '#' > /dev/ttyACM0
pause 'Reset board then enter any key to commence post water bath refills, wait for flashing LED'
grab_data settings_crt.txt "$1$thirdfile" &
pause 'Enter any key to kill data aquisition and move to  bath stage of experiment'
kill %3
echo '#' > /dev/ttyACM0
sleep 5
grab_data settings_occlude.txt "$1$forthfile" &
pause 'Enter any key to kill data aquisition, completing experiment'
kill %4
