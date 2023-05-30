//
//  rain_sensor.c
//  wx-relay
//
//  Created by Alex Lelievre on 12/26/20.
//  Copyright © 2020 Far Out Labs, LLC. All rights reserved.
//
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/termios.h>
#include <arpa/inet.h>
#include <time.h>
#include <signal.h>

#include "main.h"
#include "wx_thread.h"
#include "TXDecoderFrame.h"


static sig_atomic_t s_quit = 0;
static sig_atomic_t s_raw_rain_count = 0;

 
void process_rain_frame( RainFrame* frame )
{
    if( !frame )
        return;
    
    s_raw_rain_count = frame->raw_rain_count;
    
    if( debug_mode() )
        log_error( "rain_sensor: raw rain message: %d\n\n", frame->raw_rain_count );
}



wx_thread_return_t rain_sensor_thread( void* args )
{
    if( !args )
    {
        log_error( "rain_sensor_thread failed to startup, no device...\n" );
        wx_thread_return();
    }
    
    int fd = open_serial_port( (const char*)args, B115200 );
    if( fd < 0 )
    {
        log_error( "rain_sensor_thread failed to open serial port: %s...\n", (const char*)args );
        wx_thread_return();
    }

    log_error( "rain_sensor_thread running: %s...\n", (const char*)args );

    ssize_t result = 0;
    while( !s_quit )
    {
        RainFrame frame;

        result = read( fd, &frame, sizeof( frame ) );
        if( result == sizeof( frame ) )
            process_rain_frame( &frame );
        else if( result )
        {
            if( debug_mode() )
                log_error( " partial incoming rain sensor data %d (%d)\n", result, sizeof( frame )  );

            // we most likely have received a partially transmitted frame, try to do another read now to get the remainder of it
            sleep( 1 );
            uint8_t* partialFrame = (uint8_t*)&frame;
            uint8_t  lastRead = result;
            result = read( fd, &partialFrame[lastRead], sizeof( frame ) - lastRead );
            if( result + lastRead == sizeof( frame ) )
                process_rain_frame( &frame );
            else
                log_error( " bad frame size on incoming rain sensor data %d != %d\n", result, sizeof( frame )  );
        }
        
        sleep( 1 );
    }
        
    wx_thread_return();
}


void rain_sensor_thread_quit( void )
{
    s_quit = 1;
}


int rain_sensor_raw_count( void )
{
    return s_raw_rain_count;
}
