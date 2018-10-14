systemctl stop networking
/etc/init.d/network-manager stop
ip addr change 192.168.11.10/24 dev eth0
ip addr change 192.168.12.10/24 dev ens2

