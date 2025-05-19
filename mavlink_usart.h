#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

int init_uart(void);

void SendHeartbeat(void);
void SendServo(uint8_t servo, uint16_t pwm);

#ifdef __cplusplus
}
#endif