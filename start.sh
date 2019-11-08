# Kills the previously running driver
sudo pkill -f emucd
sleep 0.1
sudo rmmod emuccan.ko
sudo insmod src/emuccan.ko

# Start 2 CAN channels
# -sN option sets the bitrate
#       -s4  100 KBPS
#       -s5  125 KBPS
#       -s6  250 KBPS
#       -s7  500 KBPS
#       -s8  800 KBPS
#       -s9  1 MBPS 
sudo src/emucd -s7 ttyACM0 can0 can1
sudo sudo ip link set can0 up qlen 10
sudo sudo ip link set can1 up qlen 10

# Note about qlen: It was found that 10 works
# better than the default of 10000
