#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <stdint.h>

typedef struct {
    float value;
    uint32_t time_ms;
} Sample;

// Функция анализа экстремумов и производных
void analyze_series_with_window(Sample *samples, int n);
bool is_moving ();

#endif
