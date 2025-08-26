#pragma once
#ifndef GNSS_MODULE_DEFS_H
#define GNSS_MODULE_DEFS_H

#include "minmea/minmea.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gnss_module_instance *gnss_module_handle;

struct gnss_module_sentence {
  enum minmea_sentence_id sentence_id;
  union {
    struct minmea_sentence_gbs gbs;
    struct minmea_sentence_rmc rmc;
    struct minmea_sentence_gga gga;
    struct minmea_sentence_gll gll;
    struct minmea_sentence_gst gst;
    struct minmea_sentence_gsa gsa;
    struct minmea_sentence_gsv gsv;
    struct minmea_sentence_vtg vtg;
    struct minmea_sentence_zda zda;
  } sentence;
};

typedef void (*gnss_module_event_handler)(struct gnss_module_sentence);

#ifdef __cplusplus
}
#endif
#endif