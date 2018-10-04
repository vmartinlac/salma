systemctl stop networking
ip addr change 192.168.11.10/24 dev eth0
ip addr change 192.168.12.10/24 dev ens2

