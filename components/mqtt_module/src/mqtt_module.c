#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>

#include "esp_check.h"
#include "esp_err.h"

#include "mqtt_client.h"

#include "mqtt_module.h"
#include "private/mqtt_module_private.h"

static const char *TAG = "mqtt_module";

struct mqtt_module_instance {
  enum mqtt_module_state state;

  esp_mqtt_client_handle_t mqtt_client;
  esp_mqtt_client_config_t mqtt_client_cfg;
  EventGroupHandle_t mqtt_event_group;

  struct event_handlers_t {
    mqtt_event_handler_t connection_event;
    mqtt_event_handler_t subscription_event;
    mqtt_event_handler_t published_event;
    mqtt_event_handler_t data_event;
    mqtt_event_handler_t deleted_event;
    mqtt_event_handler_t error_event;
    mqtt_event_handler_t custom_event;
    // status update event?
    mqtt_event_handler_t status_event;
  } event_handlers;
};

static void
generic_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static esp_err_t
on_connection_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
on_subscription_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
on_published_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
mqtt_data_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
on_deleted_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
on_error_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);
static esp_err_t
on_custom_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle);

static void
update_module_state(mqtt_module_handle mqtt_module, enum mqtt_module_state new_state);

esp_err_t
mqtt_module_init(mqtt_module_handle *out_module) {
  esp_err_t ret = ESP_OK;
  struct mqtt_module_instance *module = NULL;

  ESP_GOTO_ON_FALSE(out_module, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

  module = calloc(1, sizeof(struct mqtt_module_instance));
  ESP_GOTO_ON_FALSE(module, ESP_ERR_NO_MEM, err, TAG, "no memory for mqtt_module_instance");

  module->event_handlers.connection_event = on_connection_event_handler;
  module->event_handlers.subscription_event = on_subscription_event_handler;
  module->event_handlers.published_event = on_published_event_handler;
  module->event_handlers.data_event = mqtt_data_event_handler;
  module->event_handlers.deleted_event = on_deleted_event_handler;
  module->event_handlers.error_event = on_error_event_handler;
  module->event_handlers.custom_event = on_custom_event_handler;

  module->mqtt_event_group = xEventGroupCreate();

  // esp_event_loop_create_default();

  module->state = MQTT_MODULE_STATE_DISCONNECTED;

  module->mqtt_client = esp_mqtt_client_init(&module->mqtt_client_cfg);
  ESP_GOTO_ON_FALSE(module->mqtt_client, ESP_ERR_MQTT_MODULE_INIT_FAIL, err, TAG, "failed to init mqtt module");

  ESP_GOTO_ON_ERROR(esp_mqtt_client_register_event(module->mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                                   generic_event_handler, (mqtt_module_handle)module),
                    err, TAG, "failed to register generic event handler");

  *out_module = module;
  return ESP_OK;
err:
  if (module) {
    if (module->mqtt_client) {
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_destroy(module->mqtt_client));
    }
    free(module);
  }
  return ret;
}
esp_err_t
mqtt_module_del(mqtt_module_handle module) {
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_mqtt_client_unregister_event(module->mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, generic_event_handler));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_disconnect(module->mqtt_client));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(module->mqtt_client));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_destroy(module->mqtt_client));
  free(module);
  return ESP_OK;
}

/**
 * Connect with preconfigured connection information if available
 */
esp_err_t
mqtt_module_connect(mqtt_module_handle module, uint32_t timeout_ms) {
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is already connected to a broker");

  ESP_RETURN_ON_ERROR(esp_mqtt_client_start(module->mqtt_client), TAG, "failed to start mqtt client");
  EventBits_t bits =
      xEventGroupWaitBits(module->mqtt_event_group, MQTT_MODULE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

  if (!bits) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(module->mqtt_client));
    return ESP_ERR_MQTT_MODULE_CONNECT_FAILED;
  }
  return ESP_OK;
}
esp_err_t
mqtt_module_connect_with_cred(mqtt_module_handle module, const char *username, const char *password, uint32_t timeout_ms) {
  ESP_RETURN_ON_FALSE(username && password && module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is already connected to a broker");

  module->mqtt_client_cfg.credentials.username = username;
  module->mqtt_client_cfg.credentials.authentication.password = password;
  ESP_RETURN_ON_ERROR(esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg), TAG, "failed to set mqtt config");

  ESP_RETURN_ON_ERROR(esp_mqtt_client_start(module->mqtt_client), TAG, "failed to start mqtt client");

  EventBits_t bits =
      xEventGroupWaitBits(module->mqtt_event_group, MQTT_MODULE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

  if (!bits) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(module->mqtt_client));
    return ESP_ERR_MQTT_MODULE_CONNECT_FAILED;
  }
  return ESP_OK;
}
esp_err_t
mqtt_module_connect_with_uri(mqtt_module_handle module, const char *uri, uint32_t timeout_ms) {
  ESP_RETURN_ON_FALSE(uri && module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid client state, client is already connected to a broker");

  module->mqtt_client_cfg.broker.address.uri = uri;
  ESP_RETURN_ON_ERROR(esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg), TAG, "failed to set mqtt config");

  ESP_RETURN_ON_ERROR(esp_mqtt_client_start(module->mqtt_client), TAG, "failed to start mqtt client");

  EventBits_t bits =
      xEventGroupWaitBits(module->mqtt_event_group, MQTT_MODULE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

  if (!bits) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(module->mqtt_client));
    return ESP_ERR_MQTT_MODULE_CONNECT_FAILED;
  }
  return ESP_OK;
}

esp_err_t
mqtt_module_reconnect(mqtt_module_handle module, uint32_t timeout_ms) {
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_ERROR(esp_mqtt_client_reconnect(module->mqtt_client), TAG, "failed to reconnect a mqtt client");

  EventBits_t bits =
      xEventGroupWaitBits(module->mqtt_event_group, MQTT_MODULE_CONNECTED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));

  if (!bits) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(module->mqtt_client));
    return ESP_ERR_MQTT_MODULE_CONNECT_FAILED;
  }
  return ESP_OK;
}

esp_err_t
mqtt_module_disconnect(mqtt_module_handle module) {
  ESP_RETURN_ON_FALSE(module, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state == MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is not connected to a broker");

  ESP_RETURN_ON_ERROR(esp_mqtt_client_disconnect(module->mqtt_client), TAG, "failed to disconnect from a broker");

  update_module_state(module, MQTT_MODULE_STATE_DISCONNECTED);

  return ESP_OK;
}

esp_err_t
mqtt_module_publish(mqtt_module_handle module, const char *topic, const char *payload, uint32_t payload_len, uint8_t qos,
                    bool retain) {
  ESP_RETURN_ON_FALSE(module && topic && payload, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state == MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is not connected to a broker");

  ESP_RETURN_ON_FALSE(esp_mqtt_client_publish(module->mqtt_client, topic, payload, payload_len, qos, retain) >= 0,
                      ESP_ERR_MQTT_PUBLISH_FAILED, TAG, "failed to publish a message");
  return ESP_OK;
}
esp_err_t
mqtt_module_enqueue(mqtt_module_handle module, const char *topic, const char *payload, uint8_t qos, bool retain) {
  ESP_RETURN_ON_FALSE(module && topic && payload, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state == MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is not connected to a broker");

  ESP_RETURN_ON_FALSE(esp_mqtt_client_enqueue(module->mqtt_client, topic, payload, 0, qos, retain, (qos & 0x01) | 1) >= 0,
                      ESP_ERR_MQTT_ENQUEUE_FAILED, TAG, "failed to enqueue a message");
  return ESP_OK;
}

esp_err_t
mqtt_module_subscribe(mqtt_module_handle module, const char *topic, uint8_t qos) {
  ESP_RETURN_ON_FALSE(module && topic, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state == MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is not connected to a broker");

  ESP_RETURN_ON_FALSE(esp_mqtt_client_subscribe_single(module->mqtt_client, topic, qos) >= 0, ESP_ERR_MQTT_SUBSCRIBE_FAILED, TAG,
                      "failed to subscribe to a topic");
  return ESP_OK;
}
esp_err_t
mqtt_module_unsubscribe(mqtt_module_handle module, const char *topic) {
  ESP_RETURN_ON_FALSE(module && topic, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state == MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, module is not connected to a broker");

  ESP_RETURN_ON_FALSE(esp_mqtt_client_unsubscribe(module->mqtt_client, topic) >= 0, ESP_ERR_MQTT_UNSUBSCRIBE_FAILED, TAG,
                      "failed to unsubscribe");
  return ESP_OK;
}

esp_err_t
mqtt_module_set_config(mqtt_module_handle module, const esp_mqtt_client_config_t *config) {
  ESP_RETURN_ON_FALSE(module && config, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg = *config;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}

esp_err_t
mqtt_module_set_address_cfg(mqtt_module_handle module, const struct mqtt_address_config *addr_cfg) {
  ESP_RETURN_ON_FALSE(module && addr_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.broker.address.hostname = addr_cfg->hostname;
  module->mqtt_client_cfg.broker.address.path = addr_cfg->path;
  module->mqtt_client_cfg.broker.address.port = addr_cfg->port;
  module->mqtt_client_cfg.broker.address.transport = addr_cfg->transport;
  module->mqtt_client_cfg.broker.address.uri = addr_cfg->uri;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_verification_cfg(mqtt_module_handle module, const struct mqtt_verification_config *verif_cfg) {
  ESP_RETURN_ON_FALSE(module && verif_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.broker.verification.alpn_protos = verif_cfg->alpn_protos;
  module->mqtt_client_cfg.broker.verification.certificate = verif_cfg->certificate;
  module->mqtt_client_cfg.broker.verification.certificate_len = verif_cfg->certificate_len;
  module->mqtt_client_cfg.broker.verification.common_name = verif_cfg->common_name;
  module->mqtt_client_cfg.broker.verification.crt_bundle_attach = verif_cfg->crt_bundle_attach;
  module->mqtt_client_cfg.broker.verification.psk_hint_key = verif_cfg->psk_hint_key;
  module->mqtt_client_cfg.broker.verification.skip_cert_common_name_check = verif_cfg->alpn_protos;
  module->mqtt_client_cfg.broker.verification.use_global_ca_store = verif_cfg->use_global_ca_store;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_credentials_cfg(mqtt_module_handle module, const struct mqtt_credentials_config *cred_cfg) {
  ESP_RETURN_ON_FALSE(module && cred_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.credentials.client_id = cred_cfg->client_id;
  module->mqtt_client_cfg.credentials.set_null_client_id = cred_cfg->set_null_client_id;
  module->mqtt_client_cfg.credentials.username = cred_cfg->username;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_authentication_cfg(mqtt_module_handle module, const struct mqtt_auth_config *auth_cfg) {
  ESP_RETURN_ON_FALSE(module && auth_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.credentials.authentication.certificate = auth_cfg->certificate;
  module->mqtt_client_cfg.credentials.authentication.certificate_len = auth_cfg->certificate_len;
  module->mqtt_client_cfg.credentials.authentication.ds_data = auth_cfg->ds_data;
  module->mqtt_client_cfg.credentials.authentication.key = auth_cfg->key;
  module->mqtt_client_cfg.credentials.authentication.key_len = auth_cfg->key_len;
  module->mqtt_client_cfg.credentials.authentication.key_password = auth_cfg->key_password;
  module->mqtt_client_cfg.credentials.authentication.key_password_len = auth_cfg->key_password_len;
  module->mqtt_client_cfg.credentials.authentication.password = auth_cfg->password;
  module->mqtt_client_cfg.credentials.authentication.use_secure_element = auth_cfg->use_secure_element;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_session_cfg(mqtt_module_handle module, const struct mqtt_session_config *session_cfg) {
  ESP_RETURN_ON_FALSE(module && session_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.session.disable_clean_session = session_cfg->disable_clean_session;
  module->mqtt_client_cfg.session.disable_keepalive = session_cfg->disable_keepalive;
  module->mqtt_client_cfg.session.keepalive = session_cfg->keepalive;
  module->mqtt_client_cfg.session.message_retransmit_timeout = session_cfg->message_retransmit_timeout;
  module->mqtt_client_cfg.session.protocol_ver = session_cfg->protocol_ver;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_last_will_cfg(mqtt_module_handle module, const struct mqtt_last_will_config *lwt_cfg) {
  ESP_RETURN_ON_FALSE(module && lwt_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.session.last_will.msg = lwt_cfg->msg;
  module->mqtt_client_cfg.session.last_will.msg_len = lwt_cfg->msg_len;
  module->mqtt_client_cfg.session.last_will.qos = lwt_cfg->qos;
  module->mqtt_client_cfg.session.last_will.retain = lwt_cfg->retain;
  module->mqtt_client_cfg.session.last_will.topic = lwt_cfg->topic;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_network_cfg(mqtt_module_handle module, const struct mqtt_network_config *net_cfg) {
  ESP_RETURN_ON_FALSE(module && net_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.network.disable_auto_reconnect = net_cfg->disable_auto_reconnect;
  module->mqtt_client_cfg.network.if_name = net_cfg->if_name;
  module->mqtt_client_cfg.network.reconnect_timeout_ms = net_cfg->reconnect_timeout_ms;
  module->mqtt_client_cfg.network.refresh_connection_after_ms = net_cfg->refresh_connection_after_ms;
  module->mqtt_client_cfg.network.timeout_ms = net_cfg->timeout_ms;
  module->mqtt_client_cfg.network.transport = net_cfg->transport;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_task_cfg(mqtt_module_handle module, const struct mqtt_task_config *task_cfg) {
  ESP_RETURN_ON_FALSE(module && task_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.task.priority = task_cfg->priority;
  module->mqtt_client_cfg.task.stack_size = task_cfg->stack_size;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_buffer_cfg(mqtt_module_handle module, const struct mqtt_buffer_config *buffer_cfg) {
  ESP_RETURN_ON_FALSE(module && buffer_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.buffer.out_size = buffer_cfg->out_size;
  module->mqtt_client_cfg.buffer.size = buffer_cfg->size;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}
esp_err_t
mqtt_module_set_outbox_cfg(mqtt_module_handle module, const struct mqtt_outbox_config *outbox_cfg) {
  ESP_RETURN_ON_FALSE(module && outbox_cfg, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(module->state != MQTT_MODULE_STATE_CONNECTED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state, disconnect from a broker first");

  module->mqtt_client_cfg.outbox.limit = outbox_cfg->limit;

  return esp_mqtt_set_config(module->mqtt_client, &module->mqtt_client_cfg);
}

esp_err_t
mqtt_module_register_event_handler(mqtt_module_handle module, enum mqtt_event_type event, mqtt_event_handler_t event_handler) {
  switch (event) {
  case MQTT_MODULE_DATA_EVENT: {
    module->event_handlers.data_event = event_handler;
    break;
  }
  case MQTT_MODULE_PUBLISH_EVENT: {
    module->event_handlers.published_event = event_handler;
    break;
  }
  case MQTT_MODULE_SUBSCRIBE_EVENT: {
    module->event_handlers.subscription_event = event_handler;
    break;
  }
  case MQTT_MODULE_DELETED_EVENT: {
    module->event_handlers.deleted_event = event_handler;
    break;
  }
  case MQTT_MODULE_CUSTOM_EVENT: {
    module->event_handlers.custom_event = event_handler;
    break;
  }

  case MQTT_MODULE_CONNECTION_EVENT:
  case MQTT_MODULE_ERROR_EVENT:
    ESP_LOGW(TAG, "Redefining this callback will break module's state management");
  default: {
    return ESP_ERR_INVALID_ARG;
  }
  }

  return ESP_OK;
}
esp_err_t
mqtt_module_unregister_event_handler(mqtt_module_handle module, enum mqtt_event_type event) {
  switch (event) {
  case MQTT_MODULE_DATA_EVENT: {
    module->event_handlers.data_event = NULL;
    break;
  }
  case MQTT_MODULE_PUBLISH_EVENT: {
    module->event_handlers.published_event = NULL;
    break;
  }
  case MQTT_MODULE_SUBSCRIBE_EVENT: {
    module->event_handlers.subscription_event = NULL;
    break;
  }
  case MQTT_MODULE_DELETED_EVENT: {
    module->event_handlers.deleted_event = NULL;
    break;
  }
  case MQTT_MODULE_CUSTOM_EVENT: {
    module->event_handlers.custom_event = NULL;
    break;
  }

  case MQTT_MODULE_CONNECTION_EVENT:
  case MQTT_MODULE_ERROR_EVENT:
    ESP_LOGW(TAG, "Unregistering this callback will break module's state management");
  default: {
    return ESP_ERR_INVALID_ARG;
  }
  }

  return ESP_OK;
}

enum mqtt_module_state
mqtt_module_get_status(mqtt_module_handle module) {
  return module->state;
}

static void
update_module_state(mqtt_module_handle mqtt_module, enum mqtt_module_state new_state) {
  ESP_LOGD(TAG, "switching module state, from:%lu to:%lu", (uint32_t)mqtt_module->state, (uint32_t)new_state);
  mqtt_module->state = new_state;
};

static void
generic_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  esp_err_t ret = ESP_OK;
  if (NULL == event_handler_arg) {
    ret = ESP_ERR_INVALID_ARG;
    goto err;
  }
  mqtt_module_handle mqtt_module = (mqtt_module_handle)event_handler_arg;
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  ret = ESP_ERR_INVALID_EVENT_HANDLER;
  switch (event->event_id) {
  case MQTT_EVENT_DISCONNECTED:
  case MQTT_EVENT_CONNECTED:
  case MQTT_EVENT_BEFORE_CONNECT: {
    if (NULL != mqtt_module->event_handlers.connection_event) {
      ret = mqtt_module->event_handlers.connection_event(mqtt_module, event_id, event);
    }
    break;
  }
  case MQTT_EVENT_SUBSCRIBED:
  case MQTT_EVENT_UNSUBSCRIBED: {
    if (NULL != mqtt_module->event_handlers.subscription_event) {
      ret = mqtt_module->event_handlers.subscription_event(mqtt_module, event_id, event);
    }
    break;
  }
  case MQTT_EVENT_PUBLISHED: {
    if (NULL != mqtt_module->event_handlers.published_event) {
      ret = mqtt_module->event_handlers.published_event(mqtt_module, event_id, event);
    }
    break;
  }
  case MQTT_EVENT_DATA: {
    if (NULL != mqtt_module->event_handlers.data_event) {
      ret = mqtt_module->event_handlers.data_event(mqtt_module, event_id, event);
    }
    break;
  }
  case MQTT_EVENT_ERROR: {
    if (NULL != mqtt_module->event_handlers.error_event) {
      ret = mqtt_module->event_handlers.error_event(mqtt_module, event_id, event);
    }
    break;
  }
  case MQTT_EVENT_DELETED: {
    if (NULL != mqtt_module->event_handlers.deleted_event) {
      ret = mqtt_module->event_handlers.deleted_event(mqtt_module, event_id, event);
    }
    break;
  }
  default: {
    if (NULL != mqtt_module->event_handlers.custom_event) {
      ret = mqtt_module->event_handlers.custom_event(mqtt_module, event_id, event);
    }
  }
  }
err:
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
};

static esp_err_t
on_connection_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  esp_err_t ret = ESP_OK;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_DISCONNECTED: {
    update_module_state(mqtt_module, MQTT_MODULE_STATE_DISCONNECTED);
    break;
  }
  case MQTT_EVENT_BEFORE_CONNECT: {
    break;
  }
  case MQTT_EVENT_CONNECTED: {
    update_module_state(mqtt_module, MQTT_MODULE_STATE_CONNECTED);
    xEventGroupSetBits(mqtt_module->mqtt_event_group, MQTT_MODULE_CONNECTED_BIT);
    break;
  }
  default: {
    ESP_LOGE(TAG, "Unexpected connection event, %li", event_id);
  }
  }
  return ret;
}
static esp_err_t
on_subscription_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  esp_err_t ret = ESP_OK;
  int msgId = event_handle->msg_id;
  esp_mqtt_error_type_t errorType = event_handle->error_handle->error_type;

  if (errorType == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED || msgId < 0) {
    if (event_handle->topic_len)
      ESP_LOGE(TAG, "Failed to subscribe to %s; Broker message: %s", event_handle->topic,
               event_handle->data_len ? event_handle->data : "None");
    return ret;
  }

  if (event_handle->topic_len)
    ESP_LOGI(TAG, "Subscribed to: %s", event_handle->topic);
  return ret;
}
static esp_err_t
on_published_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  esp_err_t ret = ESP_OK;
  int msg_id = event_handle->msg_id;

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Failed to publish a message; Error: %i", msg_id);
    return ret;
  }
  ESP_LOGD(TAG, "Published a message");
  return ret;
}
static esp_err_t
mqtt_data_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  //   - msg_id               message id
  //   - topic                pointer to the received topic
  //   - topic_len            length of the topic
  //   - data                 pointer to the received data
  //   - data_len             length of the data for this event
  //   - current_data_offset  offset of the current data for this event
  //   - total_data_len       total length of the data received
  //   - retain               retain flag of the message
  //   - qos                  QoS level of the message
  //   - dup                  dup flag of the message
  //   Note: Multiple MQTT_EVENT_DATA could be fired for one
  // message, if it is longer than internal buffer. In that
  // case only first event contains topic pointer and length,
  // other contain data only with current data length and
  // current data offset updating.
  esp_err_t ret = ESP_OK;
  int msg_id = event_handle->msg_id;

  if (msg_id < 0) {
    ESP_LOGE(TAG, "Error receiving a message on %s; Error: %i", event_handle->topic_len ? event_handle->topic : "None", msg_id);
    return ret;
  }

  ESP_LOGD(TAG, "Received on %.*s; Content: %.*s", event_handle->topic_len, event_handle->topic, event_handle->data_len,
           event_handle->data);
  return ret;
}
static esp_err_t
on_deleted_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  ESP_LOGD(TAG, "An expired message was deleted from the internal outbox");
  return ESP_OK;
}
static esp_err_t
on_error_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  esp_err_t ret = ESP_OK;
  esp_mqtt_error_type_t error_type = event_handle->error_handle->error_type;
  esp_mqtt_connect_return_code_t connect_code = event_handle->error_handle->connect_return_code;

  switch (error_type) {
  case MQTT_ERROR_TYPE_NONE: {
    break;
  }
  case MQTT_ERROR_TYPE_CONNECTION_REFUSED: {
    update_module_state(mqtt_module, (enum mqtt_module_state)connect_code);
    ESP_LOGE(TAG, "Failed to connect, reason: %u", connect_code);
    break;
  }
  case MQTT_ERROR_TYPE_TCP_TRANSPORT: {
    update_module_state(mqtt_module, MQTT_MODULE_STATE_CONNECT_TRANSPORT_ERROR);
    ESP_LOGE(TAG, "Transport error: %i, %i, %i, %i", event_handle->error_handle->esp_tls_last_esp_err,
             event_handle->error_handle->esp_tls_stack_err, event_handle->error_handle->esp_tls_cert_verify_flags,
             event_handle->error_handle->esp_transport_sock_errno);
    break;
  }
  case MQTT_ERROR_TYPE_SUBSCRIBE_FAILED: {
    if (event_handle->topic_len)
      ESP_LOGE(TAG, "Failed to subscribe to %s", event_handle->topic);
  }
  };
  return ret;
}
static esp_err_t
on_custom_event_handler(mqtt_module_handle mqtt_module, int32_t event_id, esp_mqtt_event_handle_t event_handle) {
  esp_err_t ret = ESP_OK;
  ESP_LOGD(TAG, "Custom event handler triggered");
  return ret;
}
