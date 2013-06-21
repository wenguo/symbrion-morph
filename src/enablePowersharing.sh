#!/usr/bin/expect -f
set timeout 10

set arg1 [lindex $argv 0]

spawn telnet 192.168.52.$arg1
expect "login:"
send -- "root\r"
expect  ">"
send -- "/flash/robotest -c 0 67 1\r"
expect  ">"
send -- "/flash/robotest -c 2 67 1\r"
expect  ">"
return
