#!/usr/bin/expect -f
set timeout 10

set arg1 [lindex $argv 0]

spawn telnet 192.168.52.$arg1
expect "login:"
send -- "root\r"
expect  ">"
send -- "killall morph\r"
expect  ">"
send -- "sleep 2\r"
expect  ">"
send -- "cd /flash/morph\r"
expect  ">"
send -- "./run.sh\r"
interact +++ return
