#ifndef GRAPH_INTERFACE_H
#define GRAPH_INTERFACE_H
#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC uint32_t infer_time;
EXTERNC void graphSetup();
EXTERNC void infer(float *input, float *output);
EXTERNC void doModelUpdate();
EXTERNC void update_nn(const uint16_t block_size, const uint8_t *buffer);

#undef EXTERNC

#endif