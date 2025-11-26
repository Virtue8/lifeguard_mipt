#ifndef ANALYSIS_H
#define ANALYSIS_H

bool is_pulsing ();
bool is_moving ();

void button_interrupt ();
void start_buzzing ();
void end_buzzing ();

void IRAM_ATTR movement_check ();
void battery_check ();
void emergency ();

#endif
