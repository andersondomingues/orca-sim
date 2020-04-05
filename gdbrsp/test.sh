#!/bin/bash
echo "foo" | nc -w1 -u 127.0.0.1 5000
gdb --command=gdbcmd

