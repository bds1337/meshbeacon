/**
 * @brief Module for handling the BLE whitelist.
 */

#ifndef APP_WHITELIST_H__
#define APP_WHITELIST_H__

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

ret_code_t wr_ble_app_whitelist_add(ble_gap_addr_t *addr, uint8_t * count);

ret_code_t wr_ble_app_whitelist_enable(void);

ret_code_t wr_ble_app_whitelist_clear(void);

bool wr_ble_app_whitelist_is_running(void);

uint8_t wr_ble_app_whitelist_count(void);

void wr_ble_app_whitelist_logshow(void);


#ifdef __cplusplus
}
#endif

#endif // APP_WHITELIST_H__

/** @} */