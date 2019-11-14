#!/bin/bash

#generate one hex dump foreach bin file in the callee folder
for f in *.bin ; do xxd "$f" > "${f%}.hex" ; done 