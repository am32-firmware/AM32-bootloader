#pragma once

#include <stdbool.h>

#if DRONECAN_SUPPORT
void DroneCAN_Init(void);
bool DroneCAN_update();

#endif // DRONECAN_SUPPORT
