# ========================================================================
# GLOBAL SETTINGS
# ========================================================================
URSA_COMPLINE := -Wall -Wextra -Werror -g3 -O3 
#-lasan -fsanitize=address

# ========================================================================
# SAFETY AND DEBUGGING
# ========================================================================
# Check whether some event has been schedule to run in some point of 
# time prior to the beggining. First event must always be schedule to 
# time=1, otherwise the simulation will crash (depletes performance).
URSA_ZERO_TIME_CHECKING := NO

# Check whether the queue has some event before simulating the next event.
# When simulating hardware, the number of events in the queue is constant,
# so checking the size of the queue is unecessary. Set this option to 
# YES to force checking (depletes performance).
URSA_QUEUE_SIZE_CHECKING := NO

# Max number of schedulable elements. Adjust the value to match the number 
# of simulated events.
URSA_MAX_QUEUE_SIZE := 20

# ========================================================================
# !
# !  DO NOT MODIFY BELOW LINES
# !
ifeq ($(URSA_ZERO_TIME_CHECKING), YES)
	URSA_COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(URSA_QUEUE_SIZE_CHECKING), YES)
	URSA_COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

URSA_COMPLINE := $(URSA_COMPLINE) -DURSA_MAX_QUEUE_SIZE=$(URSA_MAX_QUEUE_SIZE) 

export URSA_COMPLINE