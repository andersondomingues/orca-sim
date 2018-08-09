#!/bin/bash

#1) copy profile and layout to terminator's folder if not there already
HAS_PROFILE="$(grep 'LAYOUTNOC3X3' ~/.config/ -R)"

#if [[ $HAS_PROFILE != *"LAYOUTNOC3X3"* ]]; then

	read -p "Could not find a suitable layout for opening multiple terminals. Do you want to wipe your terminator configuration and create a new one? If you do not made any configuration to terminator, just type (Y):" -n 1 -r
	echo    # (optional) move to a new line
	if [[ $REPLY =~ ^[Yy]$ ]]; then
		cp -f config ~/.config/terminator/
	fi
#fi

#2) start terminator using the selected layout and profile
terminator -l LAYOUTNOC3X3 -p PROFILE_NOC_3X3

