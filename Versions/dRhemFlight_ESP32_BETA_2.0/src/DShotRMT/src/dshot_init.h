/**
 * @file dshot_init.h
 * @brief RMT configuration and initialization function declarations for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

#include "dshot_definitions.h"

// Function to initialize the RMT TX channel
dshot_result_t init_rmt_tx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, bool is_bidirectional);

// Function to initialize the RMT RX channel
dshot_result_t init_rmt_rx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, rmt_rx_event_callbacks_t *rx_event_callbacks, void *user_data);

// Function to initialize the DShot RMT encoder
dshot_result_t init_dshot_encoder(rmt_encoder_handle_t *out_encoder, const rmt_ticks_t &rmt_ticks, uint16_t pulse_level, uint16_t idle_level);
