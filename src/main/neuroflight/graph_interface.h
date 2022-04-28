#ifndef GRAPH_INTERFACE_H
#define GRAPH_INTERFACE_H
#ifdef __cplusplus
#define EXTERNC extern "C"

EXTERNC uint32_t micros(void);

#else
#define EXTERNC
#endif

EXTERNC uint32_t infer_time;

EXTERNC void infer(float *input, int input_size, float *output, const uint8_t* model_data, int output_size);

#undef EXTERNC

#endif