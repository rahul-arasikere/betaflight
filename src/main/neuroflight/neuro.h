#include "flight/pid.h"

void neuroInit(const pidProfile_t *pidProfile);
void neuroController(timeUs_t currentTimeUs, const pidProfile_t *pidProfile);
void evaluateGraphWithErroeDeltaTargetDeltaStateAct(timeUs_t currentTimeUs);
float transformScale(float value, float oldLow, float oldHigh, float newLow, float newHigh);
