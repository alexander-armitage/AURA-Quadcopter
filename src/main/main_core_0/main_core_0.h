#pragma once

#include "BMP280.h"
#include "FSA8S.h"
#include "I2C.h"
#include "ICM20948.h"
#include "INA219.h"
#include "PCA9685.h"
#include "PID.h"
#include "Quat.h"
#include "esp_timer.h"
#include "utility.h"

namespace main {

constexpr uint16_t MAX_ANGLE = 50;
constexpr uint16_t TRANSMITTER_ZERO = 1500;
constexpr uint16_t TRANSMITTER_RANGE = 500;
constexpr uint16_t TRANSMITTER_MAX = 2000;
constexpr uint16_t TRANSMITTER_MIN = 1000;
constexpr uint16_t ARMED_THROTTLE = 10;

constexpr float RAD_TO_DEG = 57.3;
constexpr float DEG_TO_RAD = 0.017452F;
constexpr float INPUT_TO_MAX_ANGLE = DEG_TO_RAD * MAX_ANGLE / TRANSMITTER_RANGE;
constexpr float INPUT_TO_PRECENT = 0.1F;

constexpr float PI = 3.1415;

void core_0_task(void* args);

}  // namespace main
