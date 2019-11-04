#ifndef __PUBSUB_SUBSCRIBER_H
#define __PUBSUB_SUBSCRIBER_H

#include "hellfire.h"
#include "pubsub-shared.h"

/**
 * @brief Subscribes to a topic.
 * @param subinfo Network information of the subscriber
 * @param brokerinfo Network information of the broker
 * @param topic_t Topic to which to subscribe to
 */
void pubsub_subscribe(pubsub_node_info_t subinfo, pubsub_node_info_t brokerinfo, topic_t topic);

/**
 * @brief Unsubscribes from a topic. 
 * @param subinfo Information on the subscriber node
 * @param brokerinfo Information on the broker node 
 * @param topic_t Topic to which to unsubscribe
 */
void pubsub_unsubscribe(pubsub_node_info_t subinfo, pubsub_node_info_t brokerinfo, topic_t topic);

#endif /* __PUBSUB_SUBSCRIBER_H */



