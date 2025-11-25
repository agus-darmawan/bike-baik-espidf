/**
 * @file moving_average.h
 * @author @agus-darmawan
 * @brief This file contains the interface for moving average and low-pass filter functions
 * @version 0.1
 * @date 2025-11-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t; 

/**
 * @brief Moving average filter structure
 */
typedef struct {
    float *buf; 
    size_t size;
    size_t index;
    float sum;
    int filled;
} movavg_t;

/**
 * @brief  moving average filter.
 * 
 * @param m Pointer to moving average structure
 * @param buffer Pointer to float buffer
 * @param size Size of buffer
 */
void movavg_init(movavg_t *m, float *buffer, size_t size);

/**
 * @brief Update moving average with new sample.
 * 
 * @param m Pointer to moving average structure
 * @param x New sample value
 * @return Filtered output
 */
float32_t movavg_update(movavg_t *m, float x);

/**
 * @brief First-order low-pass filter update.
 * 
 * @param prev Previous filtered value
 * @param input Current input value
 * @param alpha Filter coefficient (0..1), higher = less filtering
 * @return Filtered output
 */
float32_t lowpass_update(float prev, float input, float alpha);

#ifdef __cplusplus
}
#endif