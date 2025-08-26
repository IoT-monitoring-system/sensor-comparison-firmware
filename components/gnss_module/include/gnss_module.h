#pragma once
#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include "esp_err.h"

#include "driver/uart.h"
#include "soc/gpio_num.h"

#include "gnss_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
gnss_module_init(const uart_config_t *uart_cfg, uart_port_t uart_port, gpio_num_t rx_pin, gpio_num_t tx_pin,
                 gnss_module_handle *out_module);
esp_err_t
gnss_module_del(gnss_module_handle module);

esp_err_t
gnss_module_start(gnss_module_handle module);
esp_err_t
gnss_module_stop(gnss_module_handle module);

esp_err_t
gnss_module_read_sentence(gnss_module_handle module, struct gnss_module_sentence *out_sentence, uint32_t timeout_ms);
esp_err_t
gnss_module_write_sentence(gnss_module_handle module, const struct gnss_module_sentence sentence);

esp_err_t
gnss_module_register_event_handler(gnss_module_handle module, gnss_module_event_handler handler);

esp_err_t
gnss_module_update_parse_sentences(gnss_module_handle module, enum minmea_sentence_id *parse_sentences,
                                   uint8_t parse_sentences_len);

#ifdef __cplusplus
}
#endif
#endif