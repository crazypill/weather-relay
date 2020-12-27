//
//  rain_sensor.h
//  weather-relay
//
//  Created by Alex Lelievre on 12/26/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#ifndef _H_rain_sensor
#define _H_rain_sensor

#include <stdbool.h>
#include <stdint.h>

wx_thread_return_t rain_sensor_thread( void* args );
void               rain_sensor_thread_quit( void );
int                rain_sensor_raw_count( void );



#endif // !_H_rain_sensor
