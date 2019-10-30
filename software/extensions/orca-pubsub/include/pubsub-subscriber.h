#ifndef __PUBSUB_SUBSCRIBER_H
#define __PUBSUB_SUBSCRIBER_H

#include "hellfire.h"
#include "pubsub-shared.h"

/**
 * @brief Subscribes to a topic;
 * @param subinfo Network information of the subscriber
 * @param brokerinfo Network information of the broker
 * @param TOPIC_01 Topic to which to subscribe to
 */
void pubsub_subscribe(pubsub_node_info_t subinfo, pubsub_node_info_t brokerinfo, topic_t topic);

#endif /* __PUBSUB_SUBSCRIBER_H */



