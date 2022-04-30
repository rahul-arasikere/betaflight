#pragma once

#ifdef __GNU_C__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_utils.h"
#include "tensorflow/lite/schema/schema_generated.h"

#ifdef __GNU_C__
#pragma GCC diagnostic pop
#endif
// DEPRACTED
// tflite::MicroInterpreter* init_model(const void *buf, bool update);
