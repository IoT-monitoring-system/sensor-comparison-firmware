#pragma once
#ifndef ADC_MODULE_PRIVATE_H
#define ADC_MODULE_PRIVATE_H


#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ADC_ATTEN ADC_ATTEN_DB_12

#define ADC_MAX_MV 3160
#define ADC_MIN_MV 0

#define ADC_GET_IO_NUM(unit, channel) (adc_channel_io_map[unit][channel])

#define ADC_CONT_FLUSH_POOL_IF_FULL 1
#define ADC_CONT_MAX_FRAMES_LENGTH 1200
#define ADC_CONT_CONV_FRAME_SIZE 800

#ifdef __cplusplus
}
#endif

#endif