#include <platform.h>
#include "common/maths.h"
#include "graph_interface.h"
#include "tflite/inference.h"
#include "io/uart4Serial.h"
#include "drivers/system.h"

uint32_t infer_time = 0;


void infer(float *input, int input_size, float *output, const uint8_t* model_data, int output_size) {
	// failureMode(0);
	const long before_reading = micros();
	tflite::MicroErrorReporter micro_error_reporter;
	const tflite::Model* model = ::tflite::GetModel(model_data);
	// if (model->version() != TFLITE_SCHEMA_VERSION) {
	// 	serialPrint(getUART4(), "Model version does not match Schema");
	// 	while(1);
	// }
	tflite::MicroMutableOpResolver<5> resolver;
	resolver.AddFullyConnected();
	resolver.AddSub();
	resolver.AddMul();
	resolver.AddAdd();
	resolver.AddTanh();
	static constexpr int tensor_arena_size = 80 * 1024; //limit of 100kb
	static uint8_t tensor_arena[tensor_arena_size];
	tflite::MicroInterpreter interpreter(
		model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter
	);
	
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	infer_time = micros() - before_reading;
	asm volatile("mov r0, sp\n");
	// if (allocate_status != kTfLiteOk) {
	// 	serialPrint(getUART4(), "AllocateTensors() failed");
	// 	while(1);
	// }
	TfLiteTensor* input_ptr = interpreter.input(0);
	// // // //Copy the input into the buffer
	std::copy(input + 0, input + input_size, input_ptr->data.f);
	TfLiteStatus invoke_status = interpreter.Invoke();
	// if (invoke_status != kTfLiteOk) {
	// 	serialPrint(getUART4(), "Invoke failed on input");
	// 	while(1);
	// }
	// //The output of the neural network is in rage [-1:1] for each motor output
	TfLiteTensor* output_ptr = interpreter.output(0);
	std::copy(output_ptr->data.f + 0, output_ptr->data.f + output_size, output);
}