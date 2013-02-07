tail -n 1 settings.txt > /dev/ttyACM0
echo >> /dev/ttyACM0
cat /dev/ttyACM0 > $1
