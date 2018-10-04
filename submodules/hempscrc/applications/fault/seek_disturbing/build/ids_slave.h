#define MAX_PROCESSORS		35	/* Number of slave processors available in the platform */
#define MAXLOCALTASKS		2	/* Number of task which can be allocated simultaneously in one processor */
#define MAX_GLOBAL_TASKS	MAXLOCALTASKS * MAX_PROCESSORS	/* Number of task which can be allocated simultaneously in the platform */
#define KERNELPAGECOUNT	2
#define PAGESIZE			16384
#define MASTERADDRESS		0x00
#define XDIMENSION		3
#define YDIMENSION		3
