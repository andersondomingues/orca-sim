#ifndef __PUBSUB_SHARED_H
#define __PUBSUB_SHARED_H

#include "hellfire.h"

//default port for broker and client process
#define PS_BROKER_DEFAULT_PORT 2000
#define PS_CLIENT_DEFAULT_PORT 2001

//maximum length of publish-subscribe tables.
#define PUBSUBLIST_SIZE   40

//RT param for the client task, cannot be changed by users
#define PS_CLIENT_PERIOD    0
#define PS_CLIENT_CAPACITY  0
#define PS_CLIENT_DEADLINE  0
#define PS_CLIENT_STACKSIZE 8192

//enable/disable debug prints
#define PS_DEBUG_ENABLE 1

#if PS_DEBUG_ENABLE != 0
#define PS_DEBUG(...) printf(__VA_ARGS__)
#else
#define PS_DEBUG(...) /* nothing */
#endif

//list of message macroses
enum PSMSG{
	PSMSG_SUBSCRIBE     = 0x1,
	PSMSG_UNSUBSCRIBE   = 0x2, 
	PSMSG_ADVERTISE     = 0x3,
	PSMSG_UNADVERTISE   = 0x4	
};

//the system support up to (2^16)-1 topics if using uint16_t
typedef uint16_t topic_t;

//an entry of publishers/subscribers lists.
typedef struct pubsub_entry_t_struct {
	uint16_t opcode;   //is this entry valid? (value fell when entry gets discarded)
						    //this field also represents the opcode when used as message format
	topic_t topic;     //unique identifier to the topic
	uint16_t cpu;      //cpu in which the subscriber/pubilsher resides 
	uint16_t port;     //port to which the subscriber/publisher task is bound to
	uint16_t channel;  //specific channel to identify the message as a pubsub message
	
	uint16_t __padding0; //!necessary?
	
} pubsub_entry_t;

//identification of a broker process (addres + port)
typedef struct pubsub_node_info_t_struct{
	uint16_t address;  //address (number of the tile) in which the broker is running
	uint16_t port;     //port in which the broker process in running
} pubsub_node_info_t;

/**
 * @brief Adds a new entry to a list of publishers or subscribers
 * @param list List to receive the new entry
 * @param entry Entry to be added to the list
 * @return 0 if the entry was succefully added, 1 if the list if full */
int pubsublist_add(pubsub_entry_t* list, pubsub_entry_t entry);

/**
 * @brief remove an entry from a list of publishers or subscribers. only 
 * entries with opcode != 0 are considered to be valid.
 * @param list List to remove the entry from
 * @param entry Entry to be removed
 * @return 0 if the entry was sucefully removed, 1 otherwise */
int pubsublist_remove(pubsub_entry_t* list, pubsub_entry_t entry);

/**
 * @brief Compares two entries from a table of publishers or subscribers
 * @param a The first entry
 * @param b The second entry
 * @return 0 if entries does not match, 1 otherwise. */
int pubsub_entry_cmp(pubsub_entry_t a, pubsub_entry_t b);

/**
 * @brief Displays the valid content of a list of publishers or subscribers
 * @param list List to be scanned for valid entries
 */
void pubsublist_print(pubsub_entry_t* list);

/**
 * @brief Initialize a new list of publishers or subscribers
 * @param list The list to be initialized
 */
void pubsublist_init(pubsub_entry_t* list);

/**
 * @brief Search for an entry in the list 
 */
int pubsublist_has(pubsub_entry_t* list, pubsub_entry_t entry);


#endif /* __PUBSUB_SHARED_H*/