sudo ip link set can0 type can bitrate 250000
sudo ifconfig can0 up
sudo ifconfig can0 txqueuelen 1000

cansniffer -c -t 0 can0
