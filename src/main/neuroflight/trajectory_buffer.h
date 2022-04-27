#ifndef TRAJECTORY_BUFFER_H
#define TRAJECTORY_BUFFER_H

#include "crc.h"

typedef struct rpy_t {
    float roll;
    float pitch;
    float yaw;
} __attribute__((packed)) rpy_t;

typedef struct action_t {
    float top_left;
    float top_right;
    float bottom_left;
    float bottom_right;
} __attribute__((packed)) action_t;

typedef struct observation_t {
    rpy_t error;
    rpy_t ang_vel;
    rpy_t ang_acc;
    action_t prev_action;
    uint16_t iter;
    uint16_t delta_micros;
} __attribute__((packed)) observation_t;


typedef struct checked_observation_t {
    observation_t observation;
    crc_t crc;
} __attribute__((packed)) checked_observation_t;

typedef enum TRAJ_BUFFER_STATE_t {
    PRODUCING,
    TRANSMITTING
} TRAJ_BUFFER_STATE_t;


void traj_transmission_handler(observation_t curr_state);
void reset_trajectory();

#endif