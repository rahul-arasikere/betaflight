extern "C"
{
#include <platform.h>

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/time.h"
}

#include "neuroflight/graph_dim.h"
#include "neuroflight/graph_interface.h"
#include "neuroflight/inference.h"

void infer(float *input, float *output, const uint8_t *model_data)
{
	static constexpr int tensor_arena_size = 30 * 1024; // limit of 80kb
	static uint8_t tensor_arena[tensor_arena_size] = {0};
	tflite::MicroMutableOpResolver<5> resolver;
	tflite::MicroErrorReporter micro_error_reporter;
	resolver.AddFullyConnected();
	resolver.AddSub();
	// resolver.AddMul();
	resolver.AddAdd();
	resolver.AddTanh();
	// Pending model update
	const tflite::Model *model = tflite::GetModel(model_data);
	tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter);
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	if (allocate_status != kTfLiteOk)
	{
		while (1)
			;
	}
	const long before_reading = micros();
	TfLiteTensor *input_ptr = interpreter.input(0);
	// Copy the input into the buffer
	std::copy(input + 0, input + GRAPH_INPUT_SIZE, input_ptr->data.f);
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk)
	{
		while (1)
			;
	}
	infer_time = micros() - before_reading;
	// The output of the neural network is in range [-1:1] for each motor output
	TfLiteTensor *output_ptr = interpreter.output(0);
	std::copy(output_ptr->data.f + 0, output_ptr->data.f + GRAPH_OUTPUT_SIZE, output);
}