#-------------------------------------------------------------------------
# This file is part of Project URSA. More information on the project
# can be found at URSA's repository at GitHub.
# 
# http://https://github.com/andersondomingues/ursa
# 
# Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#-------------------------------------------------------------------------

# This platform tests the router model. A single router is used. 
# The test consists of sending a few flits into ports and comparing
# with the expected behaviour. Please help adding new tests if possible.
PLATFORM_NAME:=tb_netif

# Path to dependencies. This platform is simulated using
# URSA, so libsim and libmod are required. 
LIBSIM_PATH:=../../libs/simulator
LIBMOD_PATH:=../../libs/models

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

# Flags to be used during compilation of the platform.
GCCFLAGS  := -g -I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include -Wall -Werror -O3 
LDFLAGS   := -L$(LIBSIM_PATH)/bin -L$(LIBMOD_PATH)/bin -lmod -lsim 
OUTPUTDIR :=./bin
SOURCEDIR :=./src

# This is the master recipie.
all: $(LIBSIM) $(LIBMOD) $(OUTPUTDIR)/$(PLATFORM_NAME).exe
	@echo "$'\e[032mStarting... \e[0m"
	$(OUTPUTDIR)/$(PLATFORM_NAME).exe

#Recipies for compiling dependencies.
$(LIBSIM):
	@echo "$'\e[036mBuilding simulation library... \e[0m"
	@make -C $(LIBSIM_PATH)

$(LIBMOD):
	@echo "$'\e[036mBuilding models library... \e[0m"
	@make -C $(LIBMOD_PATH)

$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCEDIR)/platform.cpp
	@echo "$'\e[036mBuilding target platform ($(PLATFORM_NAME))... \e[0m"
	g++ $(SOURCEDIR)/platform.cpp $(GCCFLAGS) $(LDFLAGS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe

#The old and gold cleanup
clean:
	@echo "$'\e[033mCleaning simulation library... \e[0m"
	make -C $(LIBSIM_PATH) clean
	@echo "$'\e[033mCleaning models library... \e[0m"
	make -C $(LIBMOD_PATH) clean
	@echo "$'\e[033mCleaning target platform $(PLATFORM_NAME)... \e[0m"
	rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME)
	