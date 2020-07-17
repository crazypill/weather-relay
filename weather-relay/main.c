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


enum
{
    kDataFlag_temp     = 1 << 0,
    kDataFlag_humidity = 1 << 1,
    kDataFlag_rain     = 1 << 2,
    kDataFlag_wind     = 1 << 3,
    kDataFlag_gust     = 1 << 4
};


typedef struct
{
    uint8_t   magic;
    uint8_t   station_id;
    uint8_t   flags;         // the flags tell you which fields are valid in this message
    uint8_t   humidity;      // 0 - 100
    float     tempC;         // celsius
    float     windSpeedMs;   // meters/sec
    float     windGustMs;    // meters/sec
    float     rain;
    float     windDirection; // in degrees
    uint8_t   CRC;           // will be zero if did not match
    uint8_t   frameLength;
} __attribute__ ((__packed__)) Frame;


#define PORT_ERROR -1


void buffer_input_flush()
{
    int c;
     // This will eat up all other characters
    while( (c = getchar()) != EOF && c != '\n' )
        ;
}


int main(int argc, const char * argv[]) {
    
    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;
    
    int fd = open( "/dev/cu.usbserial-0001", O_RDWR | O_NOCTTY | (blocking ? 0 : O_NDELAY) );
    if( fd < 0 )
        return PORT_ERROR;

    bzero( &newtio, sizeof( newtio ) );
    
    if( cfsetispeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;
    if( cfsetospeed( &newtio, B9600 ) != 0 )
        return PORT_ERROR;
    
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;
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

    printf( "port open, reading from serial\n" );
    
    ssize_t result = 0;
    while( 1 )
    {
        Frame frame = {};

        result = read( fd, &frame, sizeof( frame ) );
        if( result == sizeof( frame ) )
        {
//            printf( "magic: %x\n", frame.magic );
            printf( "station_id: %x\n", frame.station_id );
            printf( "flags: %x\n", frame.flags );
            printf( "temp: %0.2f°C\n", frame.tempC );
            printf( "humidity: %d%%\n", frame.humidity );
            printf( "wind: %0.2f m/s\n", frame.windSpeedMs );
            printf( "dir: %0.2f degrees\n", frame.windDirection );
            printf( "gust: %0.2f m/s\n", frame.windGustMs );
            printf( "rain: %g\n\n", frame.rain );
        }
        sleep( 1 );
    }
    
    return 0;
}
