#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/printf_serial.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"

#include "sensors/gyro.h"

#include "neuroflight/neuro.h"
#include "neuroflight/byte_utils.h"
#include "neuroflight/crc.h"
#include "neuroflight/graph_dim.h"
#include "neuroflight/graph_interface.h"
#include "neuroflight/tflite_model.h"
#include "neuroflight/trajectory_buffer.h"

#include "drivers/time.h"

#define MAX_BUFFER_SIZE 30000

typedef uint16_t buffer_size_t;
#define NUM_SIZE_BYTES (sizeof(buffer_size_t))
#define NUM_META_BYTES (NUM_SIZE_BYTES + NUM_CRC_BYTES)

uint8_t block_at(unsigned int index);
uint8_t *block_ptr();
buffer_size_t expected_block_size();
crc_t expected_crc();
buffer_size_t block_size();
crc_t block_crc();
void print_block();
void update_nn();
crc_t block_crc();

/* An array containing inputs for the neural network
 * where the first element is the oldest
 */
static float graphInput[GRAPH_INPUT_SIZE];
static float graphOutput[GRAPH_OUTPUT_SIZE];
static float controlOutput[GRAPH_OUTPUT_SIZE];
static float previousOutput[GRAPH_OUTPUT_SIZE];

typedef enum TRANSMISSION_STATE_t
{
	RECEIVING_NN,
	WAIT_FOR_COMMAND,
	SENDING_OBS,
	DEAD
} TRANSMISSION_STATE_t;

static bool initFlag = true;
static TRANSMISSION_STATE_t trans_state = SENDING_OBS;

void neuroInit(const pidProfile_t *pidProfile)
{
	for (unsigned int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
	{
		previousOutput[i] = 0.0;
	}
}

void evaluateGraphWithErrorStateDeltaStateAct(timeUs_t currentTimeUs)
{
	static timeUs_t previousTime;
	static float previousState[3];
	const float deltaT = ((float)(currentTimeUs - previousTime)) / 1000000.0f;

	rpy_t ang_vel;
	ang_vel.roll = gyro.gyroADCf[FD_ROLL];
	ang_vel.pitch = gyro.gyroADCf[FD_PITCH];
	ang_vel.yaw = gyro.gyroADCf[FD_YAW];

	rpy_t ang_acc;
	ang_acc.roll = ang_vel.roll - previousState[FD_ROLL];
	ang_acc.pitch = ang_vel.pitch - previousState[FD_PITCH];
	ang_acc.yaw = ang_vel.yaw - previousState[FD_YAW];

	rpy_t error;
	error.roll = getSetpointRate(FD_ROLL) - ang_vel.roll;
	error.pitch = getSetpointRate(FD_PITCH) - ang_vel.pitch;
	error.yaw = getSetpointRate(FD_YAW) - ang_vel.yaw;

	action_t prev_action;
	prev_action.top_left = previousOutput[0];
	prev_action.top_right = previousOutput[1];
	prev_action.bottom_left = previousOutput[2];
	prev_action.bottom_right = previousOutput[3];

	observation_t obs = {
		.error = error,
		.ang_vel = ang_vel,
		.ang_acc = ang_acc,
		.prev_action = prev_action,
		.delta_micros = currentTimeUs - previousTime,
		// .delta_micros = infer_time,
		.iter = 0};
	// if(trans_state == SENDING_OBS && ARMING_FLAG(ARMED))
	// 	traj_transmission_handler(obs);

	// Prepare the neural network inputs
	//  Set the current error and deriviate
	for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
	{

		float currentSetpoint = getSetpointRate(axis);
		float state = gyro.gyroADCf[axis];
		const float deltaState = state - previousState[axis];
		float error = currentSetpoint - state;
		graphInput[axis] = error;

		if (debugMode == DEBUG_NN_GYDELTA)
		{
			debug[axis] = (int16_t)(1000 * deltaState);
		}

		if (debugMode == DEBUG_NN_SP)
		{
			debug[axis] = (int16_t)(1000 * currentSetpoint);
		}

		if (debugMode == DEBUG_NN_GYRATE)
		{
			debug[axis] = (int16_t)(10 * state);
		}

		if (debugMode == DEBUG_NN_ERR_RATE)
		{
			debug[axis] = (int16_t)(10 * error);
		}

		graphInput[axis + 3] = state;
		// TODO We need to include delta time because the loop is not fixed
		graphInput[axis + 6] = deltaState;

		previousState[axis] = state;
	}

	for (unsigned int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
	{
		graphInput[i + 9] = previousOutput[i];
	}

	// if (debugMode == DEBUG_NN_OUT) {
	//     for (unsigned int i = 0; i<GRAPH_INPUT_SIZE; i++){
	//         debug[i] = (int16_t)(graphInput[i] * 1000.0);
	//     }
	// }

	if (debugMode == DEBUG_NN_ACT_IN)
	{
		for (unsigned int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
		{
			debug[i] = (int16_t)(previousOutput[i] * 1000.0);
		}
	}

	// Evaluate the neural network graph and convert to range [-1,1]->[0,1]
	infer(graphInput, GRAPH_INPUT_SIZE, graphOutput, memory_trick(), GRAPH_OUTPUT_SIZE);

	for (unsigned int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
	{
		float new_output = graphOutput[i];
		new_output = constrainf(new_output, -1.0f, 1.0f);
		controlOutput[i] = transformScale(new_output, -1.0f, 1.0f, 0.0f, 1.0f);
		previousOutput[i] = new_output;
	}

	if (debugMode == DEBUG_NN_OUT)
	{
		for (unsigned int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
		{
			debug[i] = (int16_t)(previousOutput[i] * 1000.0);
		}
	}
	previousTime = currentTimeUs;
}

uint8_t buffer[NUM_META_BYTES + MAX_BUFFER_SIZE];
buffer_size_t buffer_size = 0;

buffer_size_t expected_block_size()
{
	if (buffer_size < NUM_SIZE_BYTES)
		return 0;
	buffer_size_t num_bytes = 0;
	for (unsigned int i = 0; i < NUM_SIZE_BYTES; i++)
		num_bytes += ((buffer_size_t)block_at(i)) << (i * 8);
	return num_bytes;
}

crc_t expected_crc()
{
	buffer_size_t crc = 0;
	for (unsigned int i = 0; i < NUM_CRC_BYTES; i++)
		crc += ((buffer_size_t)buffer[NUM_SIZE_BYTES + i]) << (i * 8);
	return crc;
}

inline buffer_size_t block_size()
{
	return (buffer_size > NUM_META_BYTES) ? (buffer_size - NUM_META_BYTES) : 0;
}

inline void add_to_buffer(uint8_t add_me)
{
	buffer[buffer_size] = add_me;
	buffer_size++;
}

inline uint8_t block_at(unsigned int index)
{
	return buffer[index + NUM_META_BYTES];
}

inline uint8_t *block_ptr()
{
	return buffer + NUM_META_BYTES;
}

void print_block()
{
	for (unsigned int i = 0; i < block_size(); i++)
	{
		tfp_printf("%02x", block_at(i));
	}
	tfp_printf("\n");
}

void update_nn()
{
	for (unsigned int i = 0; i < block_size(); i++)
	{
		memory_trick()[i] = block_at(i);
	}
}

inline crc_t block_crc()
{
	return compute_crc(block_ptr(), block_size());
}

bool was_armed = true;

void neuroController(timeUs_t currentTimeUs, const pidProfile_t *pidProfile)
{
	static uint32_t time_since_last_byte = 0;
	if (initFlag)
	{
		neuroInit(pidProfile);
		initFlag = false;
	}
	else
	{
		evaluateGraphWithErrorStateDeltaStateAct(currentTimeUs);
		mixGraphOutput(currentTimeUs, controlOutput);
	}
}

float transformScale(float value, float oldLow, float oldHigh, float newLow, float newHigh)
{
	return ((value - oldLow) / (oldHigh - oldLow)) * (newHigh - newLow) + newLow;
}
