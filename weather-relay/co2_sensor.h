//
//  co2_sensor.h
//  weather-relay
//
//  Created by Alex Lelievre on 8/12/22.
//  Copyright © 2022 Far Out Labs. All rights reserved.
//

#ifndef _H_co2_sensor
#define _H_co2_sensor

// much of this code is derived from Adafruit SCD30 driver code just stripped and made simpler (no C++)

#define SCD30_CMD_CONTINUOUS_MEASUREMENT       0x0010 ///< Command to start continuous measurement
#define SCD30_CMD_STOP_MEASUREMENTS            0x0104 ///< Command to stop measurements
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL     0x4600 ///< Command to set measurement interval
#define SCD30_CMD_GET_DATA_READY               0x0202 ///< Data ready reg
#define SCD30_CMD_AUTOMATIC_SELF_CALIBRATION   0x5306 ///< enables/disables auto calibration
#define SCD30_CMD_SET_FORCED_RECALIBRATION_REF 0x5204 ///< Forces calibration with given value
#define SCD30_CMD_SET_TEMPERATURE_OFFSET       0x5403 ///< Specifies the temp offset
#define SCD30_CMD_SET_ALTITUDE_COMPENSATION    0x5102 ///< Specifies altitude offset
#define SCD30_CMD_SOFT_RESET                   0xD304 ///< Soft reset!
#define SCD30_CMD_READ_REVISION                0xD100 ///< Firmware revision number

bool     co2_sensor_data_ready( void );
float    co2_read_sensor( float* temp, float* humidity );
uint16_t co2_read_register( uint16_t reg_address );
bool     co2_send_command( uint16_t command );
bool     co2_send_command_with_arg( uint16_t command, uint16_t argument );
bool     co2_set_measurement_interval( uint16_t interval );
uint16_t co2_get_measurement_interval( void );

#endif // !_H_co2_sensor
