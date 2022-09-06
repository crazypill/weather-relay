//
//  co2_sensor.c
//  weather-relay
//
//  Created by Alex Lelievre on 8/12/22.
//  Copyright © 2022 Far Out Labs. All rights reserved.
//

#include <wiringPiI2C.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "co2_sensor.h"


#define SCD30_I2CADDR_DEFAULT       0x61   ///< SCD30 default i2c address
#define SCD30_CMD_READ_MEASUREMENT  0x0300 ///< Main data register


static int s_sensor_fd = -1;


// CRC-8 formula from page 14 of SHT spec pdf
// Test data 0xBE, 0xEF should yield 0x92
// Initialization data 0xFF
// Polynomial 0x31 (x8 + x5 +x4 +1)
// Final XOR 0x00
static uint8_t crc8( const uint8_t* data, int len )
{
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;
    
    for( int j = len; j; --j )
    {
        crc ^= *data++;
        for( int i = 8; i; --i )
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
    return crc;
}


static bool co2_lazy_init( void )
{
    if( s_sensor_fd < 0 )
        s_sensor_fd = wiringPiI2CSetup( SCD30_I2CADDR_DEFAULT );
    if( s_sensor_fd < 0 )
        return false;
    
    return true;
}


bool co2_sensor_data_ready( void )
{
    return co2_read_register( SCD30_CMD_GET_DATA_READY ) == 1;
}


float co2_read_sensor( float* tempPtr, float* humidityPtr )
{
    ssize_t result;
    uint8_t buffer[18];
    
    buffer[0] = (SCD30_CMD_READ_MEASUREMENT >> 8) & 0xFF;
    buffer[1] = SCD30_CMD_READ_MEASUREMENT & 0xFF;
    
    if( !co2_lazy_init() )
    {
        log_error( "co2 sensor failed to initialize sensor...\n" );
        return 0.0f;
    }
        
    // wait a second if the data isn't ready and try again
    while( !co2_sensor_data_ready() )
    {
        log_error( "co2 sensor data not ready...\n" );
        sleep( 1 );
    }
    
    write( s_sensor_fd, buffer, 2 );
    usleep( 4 * 1000 );  // 4 milliseconds

    memset( buffer, 0, sizeof( buffer ) );
    result = read( s_sensor_fd, buffer, 18  );
    
    if( result != 18 )
    {
        log_error( "co2 sensor read failed\n" );
        return 0.0f;
    }
    
    // loop through the bytes we read, 3 at a time for i=MSB, i+1=LSB, i+2=CRC
    for( uint8_t i = 0; i < 18; i += 3 )
    {
        if( crc8( buffer + i, 2 ) != buffer[i + 2] )
        {
            // we got a bad CRC, fail out
            log_error( "co2 sensor bad CRC\n" );
            return 0.0f;
        }
    }
    
    // CRCs are good, unpack floats
    uint32_t co2  = (buffer[0] << 24)  | (buffer[1] << 16)  | (buffer[3] << 8)  | buffer[4];
    uint32_t temp = (buffer[6] << 24)  | (buffer[7] << 16)  | (buffer[9] << 8)  | buffer[10];
    uint32_t hum  = (buffer[12] << 24) | (buffer[13] << 16) | (buffer[15] << 8) | buffer[16];
    
    if( co2 == 0xE0000000 )
        log_error( "co2 sensor bad value: 0x%lx, buffer[0]: 0x%x\n", co2, buffer[0] );

    // coerce values into floats instead of using memcpy
    float CO2         = *((float*)&co2);
    float temperature = *((float*)&temp);
    float humidity    = *((float*)&hum);

    if( tempPtr )
        *tempPtr = temperature;
    if( humidityPtr )
        *humidityPtr = humidity;

    return CO2;
}


uint16_t co2_read_register( uint16_t reg_address )
{
    if( !co2_lazy_init() )
        return false;

    uint8_t buffer[2];
    buffer[0] = (reg_address >> 8) & 0xFF;
    buffer[1] = reg_address & 0xFF;
    
    // the SCD30 really wants a stop before the read!
    write( s_sensor_fd, buffer, 2 );
    
    usleep( 4 * 1000 );  // 4 milliseconds delay between write and read specified by the datasheet
    
    read( s_sensor_fd, buffer, 2 );
    return (uint16_t)(buffer[0] << 8 | (buffer[1] & 0xFF));
}


bool co2_send_command( uint16_t command )
{
    if( !co2_lazy_init() )
        return false;
    
    uint8_t buffer[2];
    buffer[0] = (command >> 8) & 0xFF;
    buffer[1] = command & 0xFF;

    return write( s_sensor_fd, buffer, sizeof( buffer ) );
}


bool co2_send_command_with_arg( uint16_t command, uint16_t argument )
{
    if( !co2_lazy_init() )
        return false;

    uint8_t buffer[5];
    buffer[0] = (command >> 8) & 0xFF;
    buffer[1] = command & 0xFF;
    buffer[2] = argument >> 8;
    buffer[3] = argument & 0xFF;
    buffer[4] = crc8( buffer + 2, 2 );
    return write( s_sensor_fd, buffer, sizeof( buffer ) );
}


bool co2_set_measurement_interval( uint16_t interval )
{
    if( !co2_lazy_init() )
        return false;

    if( (interval < 2) || (interval > 1800) )
        return false;
    
    return co2_send_command_with_arg( SCD30_CMD_SET_MEASUREMENT_INTERVAL, interval );
}


uint16_t co2_get_measurement_interval( void )
{
    if( !co2_lazy_init() )
        return 0;

    return co2_read_register( SCD30_CMD_SET_MEASUREMENT_INTERVAL );
}
