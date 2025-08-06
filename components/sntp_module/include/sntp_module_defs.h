#pragma once
#ifndef SNTP_MODULE_DEFS_H
#define SNTP_MODULE_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*sntp_module_time_cb_t)(struct timeval *tv);

enum sntp_module_sync_type {
  SNTP_MODULE_SYNC_IMMED,
  SNTP_MODULE_SYNC_SMOOTH,
};

enum sntp_module_sync_state {
  SNTP_MODULE_STATE_INVALID,
  SNTP_MODULE_STATE_INITIALIZED,
  SNTP_MODULE_STATE_IN_PROGRESS,
  SNTP_MODULE_STATE_SYNCHRONIZED,
};

struct sntp_module_config {
  // enum sntp_module_sync_type sync_type;
  // sntp_module_time_cb_t time_sync_cb;
  const char *server;
};

#ifdef __cplusplus
}
#endif

#endif