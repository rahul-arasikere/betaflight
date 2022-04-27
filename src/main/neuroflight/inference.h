#pragma once
#include "tflite_micro/tensorflow/lite/micro/micro_interpreter.h"
#include "tflite_micro/tensorflow/lite/micro/all_ops_resolver.h"
#include "tflite_micro/tensorflow/lite/micro/micro_error_reporter.h"
#include "tflite_micro/tensorflow/lite/micro/micro_utils.h"
#include "tflite_micro/tensorflow/lite/schema/schema_generated.h"

tflite::MicroInterpreter* init_model(const void *buf, bool update);
