# ========================================================================
# GLOBAL SETTINGS
# ========================================================================
COMPLINE := -Wall -Wextra -Werror -g3 -O3 
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

# Max number of schedulable elements. More slots represent more 
URSA_MAX_QUEUE_SIZE := 20

# ========================================================================
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.
# DO NOT MODIFY BELOW THIS LINE!
# ========================================================================
# Check whether the buffer is full before pushing data (depletes performance).
BUFFER_OVERFLOW_CHECKING := NO

# Check whether the buffer is empty before popping data (depletes performance).
BUFFER_UNDERFLOW_CHECKING := NO
