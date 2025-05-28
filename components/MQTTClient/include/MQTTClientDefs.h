#include "mqtt_client.h"

class MQTTClient;

typedef esp_mqtt_client_config_t MQTTClientConfig;
typedef esp_mqtt_client_config_t::broker_t::address_t MQTTAddressConfig;
typedef esp_mqtt_client_config_t::broker_t::verification_t MQTTVerificationConfig;
typedef esp_mqtt_client_config_t::credentials_t MQTTCredentialsConfig;
typedef esp_mqtt_client_config_t::credentials_t::authentication_t MQTTAuthenticationConfig;
typedef esp_mqtt_client_config_t::session_t MQTTSessionConfig;
typedef esp_mqtt_client_config_t::session_t::last_will_t MQTTLastWillConfig;
typedef esp_mqtt_client_config_t::network_t MQTTNetworkConfig;
typedef esp_mqtt_client_config_t::task_t MQTTTaskConfig;
typedef esp_mqtt_client_config_t::buffer_t MQTTBufferConfig;
typedef esp_mqtt_client_config_t::outbox_config_t MQTTOutboxConfig;

typedef esp_mqtt_topic_t MQTTTopic;

typedef esp_mqtt_protocol_ver_t MQTTVersion;

enum class MQTTClientState : int8_t {
  CONNECTION_TIMEOUT = -4,
  CONNECTION_LOST,
  CONNECT_FAILED,
  DISCONNECTED,
  CONNECTED,
  CONNECT_BAD_PROTOCOL,
  CONNECT_BAD_CLIENT_ID,
  CONNECT_UNAVAILABLE,
  CONNECT_BAD_CREDENTIALS,
  CONNECT_UNAUTHORIZED,
  CONNECT_TRANSPORT_ERROR
};

enum class MQTTEventType : uint8_t {
  DATA_EVENT,
  ERROR_EVENT,
  CONNECTION_EVENT,
  PUBLISH_EVENT,
  SUBSCRIBE_EVENT,
  DELETED_EVENT,
  CUSTOM_EVENT
};

#define MQTT_CLIENT_CONNECTED_BIT (1 << 0)

#define MQTT_EVENT_HANDLER_SIGNATURE std::function<void(MQTTClient &, int32_t, esp_mqtt_event_handle_t)>

#define ESP_ERR_MQTT_CLIENT_BASE 0x7B00U

#define ESP_ERR_MQTT_CLIENT_INIT_FAIL (ESP_ERR_MQTT_CLIENT_BASE + 1U)
#define ESP_ERR_MQTT_CLIENT_NOT_INIT (ESP_ERR_MQTT_CLIENT_BASE + 2U)
#define ESP_ERR_INVALID_EVENT_HANDLER (ESP_ERR_MQTT_CLIENT_BASE + 3U)
#define ESP_ERR_MQTT_PUBLISH_FAILED (ESP_ERR_MQTT_CLIENT_BASE + 4U)
#define ESP_ERR_MQTT_ENQUEUE_FAILED (ESP_ERR_MQTT_CLIENT_BASE + 5U)
#define ESP_ERR_MQTT_SUBSCRIBE_FAILED (ESP_ERR_MQTT_CLIENT_BASE + 6U)
#define ESP_ERR_MQTT_UNSUBSCRIBE_FAILED (ESP_ERR_MQTT_CLIENT_BASE + 7U)
#define ESP_ERR_MQTT_CLIENT_INVALID_STATE (ESP_ERR_MQTT_CLIENT_BASE + 8U)
#define ESP_ERR_MQTT_CLIENT_CONNECT_FAILED (ESP_ERR_MQTT_CLIENT_BASE + 9U)

#define ESP_ERR_BME_MNGR_END 0x7C00U
