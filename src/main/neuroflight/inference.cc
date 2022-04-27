#include "neuroflight/inference.h"


tflite::MicroInterpreter* init_model(const void *buf, bool update) {
	static tflite::MicroErrorReporter micro_error_reporter;
	static const int tensor_arena_size = 20 * 1024; //limit of 100kb
	static const uint8_t tensor_arena[tensor_arena_size];
	const tflite::Model* model = ::tflite::GetModel(buf);
	static tflite::MicroMutableOpResolver<3> resolver;
	resolver.AddFullyConnected();
	resolver.AddReshape();
	resolver.AddSoftmax();

	static tflite::MicroInterpreter interpreter(
		model, resolver, tensor_arena, tensor_arena_size, nullptr);
	// if(update) {
	// 	interpreter = tflite::MicroInterpreter::MicroInterpreter(
	// 		model, resolver, tensor_arena, tensor_arena_size, &micro_error_reporter);
	// }
	TfLiteStatus allocate_status = interpreter.AllocateTensors();
	return &interpreter;
}