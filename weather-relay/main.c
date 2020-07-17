//
//  main.c
//  weather-relay
//
//  Created by Alex Lelievre on 6/16/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#include <stdio.h>
#include <stdbool.h>
#include <strings.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#include "TXDecoderFrame.h"


//#define PORT_DEVICE "/dev/cu.usbserial-0001"
#define PORT_DEVICE "/dev/serial0"
//#define PORT_DEVICE "/dev/serial1"

#define PORT_ERROR -1

#define pascal2inchHg    0.0002953
#define millibar2inchHg  0.02953

#define kLocalOffsetInHg 0.33
#define c2f( a ) (((a) * 1.8000) + 32)
#define ms2mph( a ) ((a) * 2.23694)


void buffer_input_flush()
{
    int c;
     // This will eat up all other characters
    while( (c = getchar()) != EOF && c != '\n' )
        ;
}

void printTime()
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

int main(int argc, const char * argv[]) {
    
    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;
    
    int fd = open( PORT_DEVICE, O_RDWR | O_NOCTTY | (blocking ? 0 : O_NDELAY) );
    if( fd < 0 )
    {
        printf( "Failed to open serial port: %s\n", PORT_DEVICE );
        return PORT_ERROR;
    }

    bzero( &newtio, sizeof( newtio ) );
    
    if( cfsetispeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;
    if( cfsetospeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;
    
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CRTSCTS;
    
    // Hardware control of port
    newtio.c_cc[VTIME] = blocking ? 1 : 0; // Read-timout 100ms when blocking
    newtio.c_cc[VMIN] = 0;
    
    tcflush( fd, TCIFLUSH );
    
    // Acquire new port settings
    if( tcsetattr( fd, TCSANOW, &newtio ) != 0 )
        puts( strerror( errno ) );

    if( fd == -1 )
        return PORT_ERROR;

    printf( "port open, reading from serial...\n\n" );
    
    uint8_t receivedFlags = 0;
    ssize_t result = 0;
    while( 1 )
    {
        Frame frame = {};

        result = read( fd, &frame, sizeof( frame ) );
//        if( result )
//            printf( "read size: %ld\n", result );
        
        if( result == sizeof( frame ) )
        {
            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to average data over the 5 minute period...
            receivedFlags |= frame.flags;
            if( (receivedFlags & 0x7F) == 0x7F )
            {
                printf( "Have full weather info...\n" );
                receivedFlags = 0;
            }

            
            printf( "\nstation_id: 0x%x\n", frame.station_id );
//            printf( "flags:      0x%x\n", frame.flags );
            printTime();

            // read flags
            if( frame.flags & kDataFlag_temp )
                printf( "temp:       %0.2f°F\n", c2f( frame.tempC ) );

            if( frame.flags & kDataFlag_humidity )
                printf( "humidity:   %d%%\n", frame.humidity );

            if( frame.flags & kDataFlag_wind )
            {
                printf( "wind:       %0.2f mph\n", ms2mph( frame.windSpeedMs ) );
                printf( "dir:        %0.2f degrees\n", frame.windDirection );
            }
            
            if( frame.flags & kDataFlag_gust )
                printf( "gust:       %0.2f mph\n", ms2mph( frame.windGustMs ) );
            
            if( frame.flags & kDataFlag_rain )
                printf( "rain:       %g\n", frame.rain );

            if( frame.flags & kDataFlag_intTemp )
                printf( "int temp:   %0.2f°F\n", c2f( frame.intTempC ) );

            if( frame.flags & kDataFlag_pressure )
                printf( "pressure:   %g InHg\n\n", (frame.pressure * millibar2inchHg) + kLocalOffsetInHg );
        }
        sleep( 1 );
    }
    
    return 0;
}
