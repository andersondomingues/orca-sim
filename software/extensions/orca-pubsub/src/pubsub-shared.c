#include "pubsub-shared.h"

/**
 * @brief adds an entry to a list of publishers or subscribers.
 * @param list List to add the entry to
 * @param entry The entry to be added 
 * @return 0 if the entry was succefully added, 1 if the list was full
 * or the entry was in there already */
int pubsublist_remove(pubsub_entry_t* list, pubsub_entry_t entry){
	
	pubsub_entry_t e;
	
	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = list[i];
		
		if(!pubsub_entry_cmp(entry, e)){
			list[i].opcode = 0;
			return 0;
		}
	}
		
	return 1; //entry wasn't found
}

int pubsublist_has(pubsub_entry_t* list, pubsub_entry_t entry){
	
	pubsub_entry_t e;

	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = list[i];
		
		if(e.opcode != 0){

			//PS_DEBUG("e: op=%d cpu=%d ch=%d port=%d\n", e.opcode, e.cpu, e.channel, e.port);
			if(!pubsub_entry_cmp(entry, e))
				return 1;
		}
	}
	
	return 0; //entry wasn't found
}


void pubsublist_print(pubsub_entry_t* list){
	
	pubsub_entry_t e;
	
	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = list[i];
		
		if(e.opcode)
			PS_DEBUG("lst entry %d: op: %d, cpu %d, channel %d, port %d, topic %d\n",
				i, e.opcode, e.cpu, e.channel, e.port, e.topic);
	}
}


/**
 * @brief Adds a new entry to a list of publishers or subscribers
 * @param list List to receive the new entry
 * @param entry Entry to be added to the list
 * @return 0 if the entry was succefully added, 1 if the list if full */
int pubsublist_add(pubsub_entry_t* list, pubsub_entry_t entry){
	
	//is faster to remove than search then add
	pubsublist_remove(list, entry);
	
	//PS_DEBUG("lst: add \n");
	
	pubsub_entry_t* e;
	
	for(int i = 0; i < PUBSUBLIST_SIZE; i++){
		
		e = &(list[i]);
		
		if(e->opcode == 0){
			*e = entry;
			//PS_DEBUG("lst: added\n");
			return 0;
		}
	}
	
	//PS_DEBUG("lst: could not add, full \n");
	
	return 1;
}

/**
 * @brief Compares two entries from a table of publishers or subscribers
 * @param a The first entry
 * @param b The second entry
 * @return 0 if entries does not match, 1 otherwise. */
int pubsub_entry_cmp(pubsub_entry_t a, pubsub_entry_t b){
	
	//PS_DEBUG("a: op=%d cpu=%d ch=%d port=%d\n", a.opcode, a.cpu, a.channel, a.port);
	//PS_DEBUG("b: op=%d cpu=%d ch=%d port=%d\n", b.opcode, b.cpu, b.channel, b.port);
	return !((a.cpu == b.cpu) && (a.channel == b.channel) && (a.port == b.port));
}

/**
 * @brief Initialize a new list of publishers or subscribers
 * @param list The list to be initialized
 */
void pubsublist_init(pubsub_entry_t* list){
		
	for(int i = 0; i < PUBSUBLIST_SIZE; i++)
		list[i].opcode = 0;
}
