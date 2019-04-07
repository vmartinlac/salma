sysctl -w net.core.rmem_max=1048576
sysctl -w net.core.rmem_default=1048576
ip link set mtu 8228 dev ens2
ip link set mtu 8228 dev enp6s0
export ARV_DEBUG=stream-thread:3,stream:3,device:3

