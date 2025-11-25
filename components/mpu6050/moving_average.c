#include "moving_average.h"
#include <string.h>

void movavg_init(movavg_t *m, float *buffer, size_t size) {
    m->buf = buffer;
    m->size = size;
    m->index = 0;
    m->sum = 0.0f;
    m->filled = 0;
    if (buffer) memset(buffer, 0, sizeof(float)*size);
}

float movavg_update(movavg_t *m, float x) {
    if (m->size == 0) return x;
    m->sum -= m->buf[m->index];
    m->buf[m->index] = x;
    m->sum += x;
    m->index = (m->index + 1) % m->size;
    if (m->filled < (int)m->size) m->filled++;
    return m->sum / (float)(m->filled ? m->filled : 1);
}

float lowpass_update(float prev, float input, float alpha) {
    return prev + alpha * (input - prev);
}
