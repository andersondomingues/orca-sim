/*
 * Implementation file for ORCA-LIB library.
 * Copyright (C) 2018-2019 Anderson Domingues, <ti.andersondomingues@gmail.com>
 * This file is part of project URSA (http://https://github.com/andersondomingues/ursa).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. */
#ifndef __PUBSUB_PUBLISHER_H
#define __PUBSUB_PUBLISHER_H

#include "hellfire.h"
#include "pubsub-shared.h"

/**
 * @brief Advertises to a topic
 * @param pubinfo Information to be stored in the table of publishers at the broker
 * @param brokerinfo Information about the broker (required since many brokers can run at once)
 * @param topic_name Name of the topic to which the publisher will publish
 */
void pubsub_advertise(pubsub_node_info_t pubinfo, pubsub_node_info_t brokerinfo, topic_t topic_name);

/**
 * @brief Unadvertises a topic
 * @param pubinfo Information removed from the table of publishers at the broker
 * @param brokerinfo Information about the broker (required since many brokers can run at once)
 * @param topic_name Name of the topic to which the publisher will stop publishing
 */
void pubsub_unadvertise(pubsub_node_info_t pubinfo, pubsub_node_info_t brokerinfo, topic_t topic_name);

/**
 * @brief Publish a message to some topic
 * @param topic Topic to which the message will be published
 * @param msg Message to be published
 * @param size Size of the message (in bytes) */
void pubsub_publish(topic_t topic, int8_t* msg, uint16_t size);


#endif /* __PUBSUB_PUBLISHER_H */



