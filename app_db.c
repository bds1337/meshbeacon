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

static db_t db_devices[DB_DEVICES_SIZE];
static uint8_t db_lenght;

ret_code_t app_db_add( db_t * device )
{
    ret_code_t ret;

    int8_t index = app_db_find(device->smartband_id);
    if (index == -1)
    {
        if ( db_lenght >= DB_DEVICES_SIZE )
            return 1;
        
        db_devices[db_lenght].rssi = device->rssi;
        for ( int j = 0 ; j < 6; j++ )
        {
            db_devices[db_lenght].smartband_id[j] = device->smartband_id[j];
        } 
        for ( int j = 0 ; j < 3; j++ )
        {
            db_devices[db_lenght].smartband_data[j] = device->smartband_data[j];
        } 
        db_devices[db_lenght].is_ready = true;
        db_lenght++;
    } 
    else 
    {
        db_devices[index].rssi = device->rssi;
        for ( int j = 0 ; j < 6; j++ )
        {
            db_devices[index].smartband_id[j] = device->smartband_id[j];
        } 
        for ( int j = 0 ; j < 3; j++ )
        {
            db_devices[index].smartband_data[j] = device->smartband_data[j];
        } 
        db_devices[index].is_ready = true;
    }

    return NRF_SUCCESS;
}

bool app_db_read( db_t * dev )
{
    bool result = false;

    for ( int i = 0; i<db_lenght; i++ )
    {
        if ( db_devices[i].is_ready )
        {
            result = true;
            for (int j = 0; j<6; j++ )
            {
                dev->smartband_id[j] = db_devices[i].smartband_id[j];
            }
            for (int j = 0; j<3; j++ )
            {
                dev->smartband_data[j] = db_devices[i].smartband_data[j];
            }
            dev->rssi = db_devices[i].rssi;
            db_devices[i].is_ready = false;
            break;
        }
    }

    return result;
}

//ret index
int8_t app_db_find(uint8_t * smartband_id)
{
    bool result = true;
    for ( int i = 0; i<db_lenght; i++ )
    {
        for (int j = 0; j<6; j++ )
        {
            if (db_devices[i].smartband_id[j] != smartband_id[j])
            {
                result = false;
                break;
            } 
        }
        if (result == true)
            return i;
        result = true;
    }
    
    return -1;
}
