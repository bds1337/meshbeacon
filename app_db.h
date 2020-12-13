/**
 * @brief Module for handling the BLE whitelist.
 */

#ifndef APP_DB_H__
#define APP_DB_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ble.h"
#include "ble_gap.h"
#include "app_util.h"
#include "sdk_errors.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct 
{
   	uint8_t smartband_id[6];
	uint8_t smartband_data[3];
	uint8_t rssi;
    bool is_ready; //if data modified
} db_t; 

ret_code_t app_db_add(db_t * device);
int8_t app_db_find(uint8_t * smartband_id);
bool app_db_read( db_t * dev );

#ifdef __cplusplus
}
#endif

#endif // APP_DB_H__

/** @} */