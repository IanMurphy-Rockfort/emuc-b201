# Script edited 2019-11-08 by Ian Murphy

cd # ensure we are 'home', we expect a sub-folder 'emuc-b201'

# Kills the daemon & driver if already running
sudo pkill -f emucd
sleep 0.1
sudo rmmod emuccan.ko

# Install the kernel module
sudo insmod ./emuc-b201/src/emuccan.ko

# Start daemon and specify CAN interfaces
# -sN option sets the bitrate NOTE: PDF & command line are correct, previous script was not...
#       -s3  50 KBPS
#       -s4  125 KBPS
#       -s5  250 KBPS
#       -s6  500 KBPS
#       -s7  1 MBPS 
sudo ./emuc-b201/src/emucd -s6 ttyACM0 emuccan0 emuccan1 # use default names to avoid conflict with installed PEAK hardware
sudo ip link set up emuccan0 qlen 10
sudo ip link set up emuccan1 qlen 10

# NOTE: the following statement needs validating
# Note about qlen: It was found that 10 works
# better than the default of 10000
