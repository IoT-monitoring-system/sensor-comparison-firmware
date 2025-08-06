#include "esp_check.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "lwip/apps/sntp.h"

#include "private/sntp_module_private.h"
#include "sntp_module.h"
#include "sntp_module_defs.h"

typedef struct {
  enum sntp_module_sync_state state;
  uint8_t last_add_server_idx;
  const char *sntp_servers[SNTP_MODULE_MAX_SNTP_SERVERS]; // Not freed
} sntp_module_instance_t;

static const char *TAG = "sntp_module";

static sntp_module_instance_t sntp_module_instance;

// Cant use the callback, since it will overwrite the esp_netif internal one, which may break sync_wait
static void
sntp_module_time_callback(struct timeval *tv);

static esp_err_t
push_server(sntp_module_instance_t *sntp_module_instance, const char *server);

static esp_err_t
pop_server(sntp_module_instance_t *sntp_module_instance);

static esp_err_t
sntp_stop_api(void *ctx);

esp_err_t
sntp_module_init(struct sntp_module_config *sntp_cfg) {
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_FALSE(sntp_cfg, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(sntp_module_instance.state == SNTP_MODULE_STATE_INVALID, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state");

  esp_sntp_config_t esp_sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_cfg->server);
  esp_sntp_cfg.start = false;
  ESP_GOTO_ON_ERROR(esp_netif_sntp_init(&esp_sntp_cfg), err, TAG, "failed to initialize esp_netif_sntp");
  sntp_module_instance.last_add_server_idx = 0;

  ESP_GOTO_ON_ERROR(push_server(&sntp_module_instance, sntp_cfg->server), err, TAG, "failed adding an sntp server");
  sntp_module_instance.state = SNTP_MODULE_STATE_INITIALIZED;

  return ESP_OK;
err:
  esp_netif_sntp_deinit();
  return ret;
}
esp_err_t
sntp_module_del(void) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(sntp_module_instance.state != SNTP_MODULE_STATE_INVALID, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state");

  esp_netif_sntp_deinit();
  sntp_module_instance.state = SNTP_MODULE_STATE_INVALID;

  return ret;
}

esp_err_t
sntp_module_start(void) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(sntp_module_instance.state == SNTP_MODULE_STATE_INITIALIZED, ESP_ERR_INVALID_STATE, TAG,
                      "invalid module state");
  ESP_RETURN_ON_ERROR(esp_netif_sntp_start(), TAG, "failed to start esp_netif_sntp");

  sntp_module_instance.state = SNTP_MODULE_STATE_IN_PROGRESS;

  return ret;
}
esp_err_t
sntp_module_stop(void) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_ERROR(esp_netif_tcpip_exec(sntp_stop_api, NULL), TAG, "failed to stop esp_netif_sntp");
  sntp_module_instance.state = SNTP_MODULE_STATE_INITIALIZED;

  return ret;
}

enum sntp_module_sync_state
sntp_module_get_state(void) {
  return sntp_module_instance.state;
}

esp_err_t
sntp_module_push_server(const char *server) {
  return push_server(&sntp_module_instance, server);
}
esp_err_t
sntp_module_pop_server(void) {
  return pop_server(&sntp_module_instance);
}

esp_err_t
sntp_module_sync_wait(uint32_t time_to_wait_ms) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_ERROR(esp_netif_sntp_sync_wait(pdMS_TO_TICKS(time_to_wait_ms)), TAG, "failed to sync sntp");
  sntp_module_instance.state = SNTP_MODULE_STATE_SYNCHRONIZED;

  return ret;
}

static void
sntp_module_time_callback(struct timeval *tv) {
}

static esp_err_t
push_server(sntp_module_instance_t *sntp_module_instance, const char *server) {
  ESP_RETURN_ON_FALSE(server && sntp_module_instance, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(sntp_module_instance->last_add_server_idx < SNTP_MODULE_MAX_SNTP_SERVERS, ESP_FAIL, TAG,
                      "failed to push a server, max number servers reached");
  sntp_module_instance->sntp_servers[sntp_module_instance->last_add_server_idx++] = server;

  return ESP_OK;
}

static esp_err_t
pop_server(sntp_module_instance_t *sntp_module_instance) {
  ESP_RETURN_ON_FALSE(sntp_module_instance, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(sntp_module_instance->last_add_server_idx > 0, ESP_FAIL, TAG, "failed to pop a server, invalid index");
  sntp_module_instance->sntp_servers[--sntp_module_instance->last_add_server_idx] = NULL;

  return ESP_OK;
}

static esp_err_t
sntp_stop_api(void *ctx) {
  sntp_stop();
  return ESP_OK;
}