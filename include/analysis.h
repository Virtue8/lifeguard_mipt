#ifndef ANALYSIS_H
#define ANALYSIS_H

bool is_pulsing (struct DeviceData * data);
bool is_moving (struct DeviceData * data);

void IRAM_ATTR movement_check (struct DeviceData * data);
void IRAM_ATTR battery_check (struct DeviceData * data);
void emergency (struct DeviceData * data);

#endif
