#ifndef MAIN_H
#define MAIN_H

/**
 * ===================================================================
 * @warning VERY IMPORTANT MACROS !!!
 * DO NOT CHANGE THE THINGS BELOW WITHOUT KNOWING WHAT YOU ARE DOING
 * IT AFFECTS ACROSS THE WHOLE CODEBASE. YOU HAVE BEEN WARNED !!!
 * ===================================================================
 */
#define IS_GWY 1
#define CLIENT_RELEASE 1
/*====================================================================*/

#define LTE_THREAD_STACK_SIZE 4096
#define IR_THREAD_STACK_SIZE 8192

#if(IS_GWY)
#define MAJ_VERSION 1
#define MIN_VERSION 1
#define PATCH_VERSION 8
#endif

#if(!IS_GWY)
#define MAJ_VERSION 1
#define MIN_VERSION 0
#define PATCH_VERSION 3
#endif

#if(!CLIENT_RELEASE)
#define MQTT_SERVER_IP "54.215.188.103"
#define MQTT_PORT 1883
#define MQTT_BROKER_USERNAME "QmaxSystems"
#define MQTT_BROKER_PASSWORD "Qmax_mosquitto_!@#"
#endif

#if(CLIENT_RELEASE)
#define MQTT_SERVER_IP "195.250.30.213"
#define MQTT_PORT 1883
#define MQTT_BROKER_USERNAME "unimationuser"
#define MQTT_BROKER_PASSWORD "RpGf+zeWueXSOq23E0"
#endif

#endif