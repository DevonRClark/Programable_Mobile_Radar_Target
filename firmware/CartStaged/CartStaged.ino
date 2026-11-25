// Onboard logging: stage 6, protocol-v3 bench commands and right-wheel odometry.
// For a fresh board, start at 1 and work up to 6. Change only this stage number.
// Flash the matching protocol-v3 Dongle sketch; motor power stays off for validation.
#ifndef CART_TEST_STAGE
#define CART_TEST_STAGE 6
#endif

#include "src/CartRuntime.h"

void setup() { firmware::setupCart(); }
void loop() { firmware::loopCart(); }
