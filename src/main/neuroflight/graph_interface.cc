extern "C"
{
#include <platform.h>

#include "common/utils.h"
#include "common/maths.h"

#include "neuroflight/defines.h"

#include "io/xbee.h"
}
#include "neuroflight/graph_interface.h"
#include "neuroflight/inference.h"
#include "neuroflight/tflite_model.h"

#include "debug_log_callback.h"

// TFLite globals
namespace
{
	tflite::ErrorReporter *error_reporter = nullptr;
	const tflite::Model *model = nullptr;
	tflite::MicroInterpreter *interpreter = nullptr;
	TfLiteTensor *model_input = nullptr;
	TfLiteTensor *model_output = nullptr;
	// Create an area of memory to use for input, output, and other TensorFlow
	// arrays. You'll need to adjust this by compiling, running, and looking
	// for errors.
	constexpr int kTensorArenaSize = 30 * 1024;
	alignas(16) uint8_t tensor_arena[kTensorArenaSize];
} // namespace

void tflite_log_print_callback(const char *str)
{
	xprintf(str);
}

void graphSetup()
{
	tflite::InitializeTarget();
	RegisterDebugLogCallback(tflite_log_print_callback);
	static tflite::MicroErrorReporter micro_error_reporter;
	error_reporter = &micro_error_reporter;
	doModelUpdate();
	error_reporter->Report("aaa");
}

FAST_CODE_NOINLINE void infer(float *input, float *output)
{
	// Copy the input into the buffer
	for (int i = 0; i < GRAPH_INPUT_SIZE; i++)
	{
		model_input->data.f[i] = input[i];
	}
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk)
	{
		TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed with %d\n", invoke_status);
		return;
	}
	// The output of the neural network is in range [-1:1] for each motor output
	for (int i = 0; i < GRAPH_OUTPUT_SIZE; i++)
	{
		output[i] = model_output->data.f[i];
	}
}

void doModelUpdate()
{
	static tflite::MicroMutableOpResolver<5> resolver(error_reporter);
	resolver.AddFullyConnected();
	resolver.AddSub();
	resolver.AddMul();
	resolver.AddAdd();
	resolver.AddTanh();
	model = tflite::GetModel(model_bytes);
	if (model->version() != TFLITE_SCHEMA_VERSION)
	{
		TF_LITE_REPORT_ERROR(error_reporter,
							 "Model provided is schema version %d not equal "
							 "to supported version %d.",
							 model->version(), TFLITE_SCHEMA_VERSION);
		return;
	}
	static tflite::MicroInterpreter static_interpreter(model,
													   resolver,
													   tensor_arena,
													   kTensorArenaSize,
													   error_reporter);
	interpreter = &static_interpreter;
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk)
	{
		TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
		return;
	}
	model_input = interpreter->input(0);
	model_output = interpreter->output(0);
}

void update_nn(const uint16_t block_size, const uint8_t *buffer)
{
	for (uint16_t i = 0; i < block_size; i++)
	{
		model_bytes[i] = buffer[i + NUM_META_BYTES];
	}
	doModelUpdate();
}
