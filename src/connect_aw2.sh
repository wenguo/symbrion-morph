#!/bin/bash
sudo route del -net 192.168.2.0 netmask 255.255.255.0 gw 164.11.73.214 
sudo route add -net 192.168.2.0 netmask 255.255.255.0 gw 164.11.73.220 
telnet 192.168.2.52
