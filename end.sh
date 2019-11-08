# First bring down the CAN interfaces
sudo ip link set down emuccan0
sudo ip link set down emuccan1

# Kill the daemon & driver
sudo pkill -f emucd
sleep 0.1
sudo rmmod emuccan.ko
