# ===============================================================[ MODELS ]
# -DOPT_URSA									#enable cheats
# -DOPT_MEMORY_SKIP_WRITE_ADDRESS_CHECKING		#disable address checking when wipeing the memory
# -DOPT_MEMORY_SKIP_READ_ADDRESS_CHECKING		#disable address checking when reading from the memory
# -DOPT_MEMORY_SKIP_WIPE_ADDRESS_CHECKING		#disable address checking when writing to the memory
# -DOPT_BUFFER_SKIP_OVERFLOW_CHECKING			#disable overflow checking when push to the buffer
# -DOPT_BUFFER_SKIP_UNDERFLOW_CHECKING			#disable underflow checking when popping from the buffer
# -DOPT_ROUTER_DISABLE_METRICS					#remove data sampling code from compilation (affects router model only)
# -DOPT_NETSOCKET_DISABLE_OUTGOING_PACKETS_LOG 	#disable output log for outgoing packets
# -DOPT_NETSOCKET_DISABLE_INCOMING_PACKETS_LOG 	#disable output log for outgoing packets
# -DOPT_HFRISC_DISABLE_READ_ADDRESS_CHECKING	#disable checking on unmapped addressess when reading from memory
# -DOPT_HFRISC_DISABLE_WRITE_ADDRESS_CHECKING	#disable checking on unmapped addressess when writing to memory
# -DOPT_HFRISC_DISABLE_METRICS					#disable data sampling code from compilation (affects hfrisc model only)
# -DCFG_NETSOCKET_CLIENT_ADDR="127.0.0.1"		#ip address for the external network interface client
# -DCFG_NETSOCKET_CLIENT_PORT=8888				#client port
# -DCFG_NETSOCKET_SERVER_ADDR="127.0.0.1"		#ip address for the external network interface server
# -DCFG_NETSOCKET_SERVER_PORT=9999				#server port
OPT_MODEL_FLAGS := -DOPT_URSA \
	-DOPT_MEMORY_SKIP_WRITE_ADDRESS_CHECKING \
	-DOPT_MEMORY_SKIP_READ_ADDRESS_CHECKING \
	-DOPT_MEMORY_SKIP_WIPE_ADDRESS_CHECKING \
	-DOPT_BUFFER_SKIP_OVERFLOW_CHECKING \
	-DOPT_BUFFER_SKIP_UNDERFLOW_CHECKING \
	-DOPT_ROUTER_DISABLE_METRICS \
	-DOPT_ROUTER_DISABLE_GHOST_ROUTER_CHECKING \
	-DOPT_NETSOCKET_DISABLE_OUTGOING_PACKETS_LOG \
	-DOPT_NETSOCKET_DISABLE_INCOMING_PACKETS_LOG \
	-DOPT_HFRISC_DISABLE_READ_ADDRESS_CHECKING \
	-DOPT_HFRISC_DISABLE_WRITE_ADDRESS_CHECKING \
	-D___OPT_HFRISC_DISABLE_METRICS \
	-DCFG_NETSOCKET_CLIENT_ADDR=\"127.0.0.1\" \
	-DCFG_NETSOCKET_CLIENT_PORT=8888 \
	-DCFG_NETSOCKET_SERVER_ADDR=\"127.0.0.1\" \
	-DCFG_NETSOCKET_SERVER_PORT=9999

# ==============================================================[ SIMULATOR ]
# -DOPT_URSA									#enable cheats
# -DOPT_SIMULATOR_SKIP_ZERO_TIME_CHECKING		#ignore events scheduled behind initial time (crash if such events exist).
OPT_SIMUL_FLAGS := -DOPT_URSA \
	-DOPT_SIMULATOR_SKIP_ZERO_TIME_CHECKING

# ===============================================================[ PLATFORM ]
OPT_PLAT_FLAGS := $(OPT_MODEL_FLAGS) $(OPT_SIMUL_FLAGS)

SYSTEM_FLAGS := -DNOC_H_SIZE=4 -DNOC_W_SIZE=4 -DCYCLES_TO_SIM=1000000

# -03: optmize to run as faster as possible
# -g: generate debugging information
# -Wall: enable warnings for most operations
# -Werror: treat warning as errors
# -Wextra: enable extra warnings
# -fsanitize=address: compile with leak-checking analysis
COMPILER_FLAGS := -O3 -Wall -Wextra -Werror -g 

#export variables so that other makefiles can use them
export COMPILER_FLAGS
export OPT_SIMUL_FLAGS
export OPT_MODEL_FLAGS
export OPT_PLAT_FLAGS
export SYSTEM_FLAGS