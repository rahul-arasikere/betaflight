#pragma once

void neuroInit();
void neuroController(timeUs_t currentTimeUs);
void evaluateGraphWithErroeDeltaTargetDeltaStateAct(timeUs_t currentTimeUs);
float transformScale(float value, float oldLow, float oldHigh, float newLow, float newHigh);
