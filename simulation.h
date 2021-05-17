#ifndef SIMULATION_TYPES_H
#define SIMULATION_TYPES_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>

#include "writeOutput.h"
#include "helper.h"

// ******************* DRONE *********************
typedef enum DroneOrderType {
    DRONE_DELIVER_PACKAGE,
    DRONE_HELP_HUB
} DroneOrderType;

typedef struct DroneOrderNode {
    DroneOrderType type;
    struct DroneOrderNode *next;
} DroneOrderNode;

typedef struct Drone {
    int speed;
    int current_hub_id;
    int max_range;
    int current_range;

    PackageInfo *order_package;
    int order_hub;
    sem_t order_ready;
    DroneOrderNode *order_dummy;

    int total_delivered;
    int total_help;
    int total_received_job;
    long long last_timestamp;

} Drone;
//**********************************************

// ******************* HUB *********************
typedef struct StorageNode {
    PackageInfo *package;
    struct StorageNode *next;
} StorageNode;

typedef struct Storage {
    sem_t empty;
    sem_t full;
    StorageNode *dummy;
} Storage;

typedef struct ChargingSpaceNode{
    Drone *drone;
    struct ChargingSpaceNode *next;
} ChargingSpaceNode;

typedef struct ChargingSpace {
    sem_t empty;
    sem_t full;
    ChargingSpaceNode *dummy;
} ChargingSpace;

typedef struct DistanceNode {
    int distance;
    int id;
    struct DistanceNode *next;
} DistanceNode;


typedef struct Hub {
    sem_t incoming_storage_mutex;
    Storage incoming_storage;

    sem_t outgoing_storage_mutex;
    Storage outgoing_storage;

    sem_t charging_space_mutex;
    ChargingSpace charging_space;

    DistanceNode *distance_dummy;

    int total_assigned;
} Hub;
//**********************************************

// ******************* RECEIVER *********************
typedef struct Receiver {
    int wait_time;
    int assigned_hub_id;
    sem_t have_package;

    int total_received;
} Receiver;
//**********************************************

// ******************* SENDER *********************
typedef struct Sender {
    int wait_time;
    int remaining_packages;
    int assigned_hub_id;

    int total_sent;
} Sender;
//**********************************************




#endif // SIMULATION_TYPES_H