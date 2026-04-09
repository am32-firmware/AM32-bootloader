#pragma once

#include <stdbool.h>

#if DRONECAN_SUPPORT
void DroneCAN_Init(void);
bool DroneCAN_update();
bool DroneCAN_boot_ok(void);
void DroneCAN_set_have_signal(void);

#endif // DRONECAN_SUPPORT
