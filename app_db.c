/**
 * @brief Addrlist (wishlist) addon for ibeacon project
 */

#include "app_db.h"
#include "sdk_common.h"

#include "sdk_config.h"
#include <stdlib.h>

#include <string.h>
#include "app_error.h"
#include "nrf_assert.h"
#include "sdk_macros.h"
#include "ble_advdata.h"

//for debug
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define  DB_DEVICES_SIZE 1

static db_t db_device;

uint8_t app_db_add( db_t * device )
{
    db_device.rssi = device->rssi;
    for ( int j = 0 ; j < 6; j++ )
    {
        db_device.smartband_id[j] = device->smartband_id[j];
    } 
    for ( int j = 0 ; j < 3; j++ )
    {
        db_device.smartband_data[j] = device->smartband_data[j];
    } 

    db_device.is_ready = true;
    return 0;
}

void app_db_add_pulse( uint8_t data )
{
    db_device.smartband_data[0] = data;
    db_device.is_ready = true;
}

void app_db_add_pressure( uint8_t up, uint8_t down)
{
    db_device.smartband_data[1] = up;
    db_device.smartband_data[2] = down;
    db_device.is_ready = true;
}

bool app_db_read( db_t * dev )
{
    bool result = false;

    if ( db_device.is_ready )
    {
       result = true;
       for (int j = 0; j<6; j++ )
       {
           dev->smartband_id[j] = db_device.smartband_id[j];
       }
       for (int j = 0; j<3; j++ )
       {
           dev->smartband_data[j] = db_device.smartband_data[j];
       }
       dev->rssi = db_device.rssi;
       db_device.is_ready = false;
    }

    return result;
}
