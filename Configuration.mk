# ===============================================================#
# SAFETY AND DEBUGGING                                           #
# ===============================================================#
# Check whether some event has been schedule to run in some point
# of time prior to the beggining (t < 1). First event must always
# be schedule to time=1, otherwise the simulation will crash.
# Turning this option on depletes simulation performance.
URSA_ZERO_TIME_CHECKING := NO

# Check whether the queue is not empty before simulating the next
# event. In some cases, hardware models can manipulate the queue 
# to add or remove elements from the simulation. Although manipu-
# lating the queue at the runtime can increase simulation perfor-
# mance, bad coding can lock down the simulation when no more ele-
# ments are there to be simulated. For most situations, the number
# of events in the queue is constant, so checking the size of the
# queue is unecessary. Set this option to YES to force checking 
# the queue size at every cycle. Turning this option on depletes
# simulation performance.
URSA_QUEUE_SIZE_CHECKING := NO

# Size of the scheduling queue. Adjust the value to match the 
# number of events being simultaneously simulated. If set below 
# the number of elements, simulation will crash as some elements
# will not be scheduled. Larger numbers may represent more time to
# spent when sort elements at the end of each epoch.
URSA_MAX_QUEUE_SIZE := 30

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 # 
#================================================================#

URSA_COMPLINE := $(URSA_COMPLINE) \
	-DURSA_MAX_QUEUE_SIZE=$(URSA_MAX_QUEUE_SIZE) 

ifeq ($(URSA_ZERO_TIME_CHECKING), YES)
	URSA_COMPLINE := $(URSA_COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif

ifeq ($(URSA_QUEUE_SIZE_CHECKING), YES)
	URSA_COMPLINE := $(URSA_COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

export URSA_COMPLINE
