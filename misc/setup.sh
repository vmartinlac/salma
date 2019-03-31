sysctl -w net.core.rmem_max=1048576
sysctl -w net.core.rmem_default=1048576
ifconfig ens2 mtu 8228
ifconfig eth0 mtu 8228
export ARV_DEBUG=stream-thread:3,stream:3,device:3

