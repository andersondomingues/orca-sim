#ifndef __PUBSUB_SHARED_H
#define __PUBSUB_SHARED_H

#define PS_BROKER_PORT    55
#define PS_BROKER_CHANNEL 55

#define PUBSUBLIST_SIZE   40

#include "hellfire.h"

//list of message macroses
enum PSMSG{
	PSMSG_SUBSCRIBE,
	PSMSG_UNSUBSCRIBE,
	PSMSG_ADVERTISE,
	PSMSG_UNADVERTISE	
};

//the system support up to (2^16)-1 topics if using uint16_t
typedef uint16_t topic_t;

//an entry of publishers/subscribers lists.
typedef struct{
	uint8_t opcode;    //is this entry valid? (value fell when entry gets discarded)
						    //this field also represents the opcode when used as message format
	topic_t topic;     //unique identifier to the topic
	uint16_t cpu;      //cpu in which the subscriber/pubilsher resides 
	uint16_t port;     //port to which the subscriber/publisher task is bound to
	uint16_t channel;  //specific channel to identify the message as a pubsub message	
	
} pubsub_entry_t;

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
 * @brief Initialize a new list of publishers or subscribers
 * @param list The list to be initialized
 */
void pubsublist_init(pubsub_entry_t* list);


#endif /* __PUBSUB_SHARED_H*/