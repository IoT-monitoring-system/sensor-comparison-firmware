#pragma once
#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include "mqtt_module_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
mqtt_module_init(mqtt_module_handle *out_module); // Initial configuration?
esp_err_t
mqtt_module_del(mqtt_module_handle module);

/**
 * Connect with preconfigured connection information if available
 */
esp_err_t
mqtt_module_connect(mqtt_module_handle module, uint32_t timeout_ms);
esp_err_t
mqtt_module_connect_with_cred(mqtt_module_handle module, const char *username, const char *password, uint32_t timeout_ms);
esp_err_t
mqtt_module_connect_with_uri(mqtt_module_handle module, const char *uri, uint32_t timeout_ms);

esp_err_t mqtt_module_reconnect(mqtt_module_handle module, uint32_t timeout_ms);

esp_err_t
mqtt_module_disconnect(mqtt_module_handle module);

esp_err_t
mqtt_module_publish(mqtt_module_handle module, const char *topic, const char *payload, uint32_t payload_len, uint8_t qos, bool retain);
esp_err_t
mqtt_module_enqueue(mqtt_module_handle module, const char *topic, const char *payload, uint8_t qos, bool retain);

esp_err_t
mqtt_module_subscribe(mqtt_module_handle module, const char *topic, uint8_t qos);
esp_err_t
mqtt_module_unsubscribe(mqtt_module_handle module, const char *topic);

esp_err_t
mqtt_module_set_config(mqtt_module_handle module, const esp_mqtt_client_config_t *config);

esp_err_t
mqtt_module_set_address_cfg(mqtt_module_handle module, const struct mqtt_address_config *addr_cfg);
esp_err_t
mqtt_module_set_verification_cfg(mqtt_module_handle module, const struct mqtt_verification_config *verif_cfg);
esp_err_t
mqtt_module_set_credentials_cfg(mqtt_module_handle module, const struct mqtt_credentials_config *cred_cfg);
esp_err_t
mqtt_module_set_authentication_cfg(mqtt_module_handle module, const struct mqtt_auth_config *auth_cfg);
esp_err_t
mqtt_module_set_session_cfg(mqtt_module_handle module, const struct mqtt_session_config *session_cfg);
esp_err_t
mqtt_module_set_last_will_cfg(mqtt_module_handle module, const struct mqtt_last_will_config *lwt_cfg);
esp_err_t
mqtt_module_set_network_cfg(mqtt_module_handle module, const struct mqtt_network_config *net_cfg);
esp_err_t
mqtt_module_set_task_cfg(mqtt_module_handle module, const struct mqtt_task_config *task_cfg);
esp_err_t
mqtt_module_set_buffer_cfg(mqtt_module_handle module, const struct mqtt_buffer_config *buffer_cfg);
esp_err_t
mqtt_module_set_outbox_cfg(mqtt_module_handle module, const struct mqtt_outbox_config *outbox_cfg);

esp_err_t
mqtt_module_register_event_handler(mqtt_module_handle module, enum mqtt_event_type event, mqtt_event_handler_t event_handler);
esp_err_t
mqtt_module_unregister_event_handler(mqtt_module_handle module, enum mqtt_event_type event);

enum mqtt_module_state
mqtt_module_get_status(mqtt_module_handle module);

#ifdef __cplusplus
}
#endif
#endif