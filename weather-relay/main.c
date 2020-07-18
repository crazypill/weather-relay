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
#include <math.h>
#include <assert.h>

#include "TXDecoderFrame.h"
#include "aprs-wx.h"
#include "aprs-is.h"


//#define PORT_DEVICE "/dev/cu.usbserial-0001"
#define PORT_DEVICE "/dev/serial0"
//#define PORT_DEVICE "/dev/serial1"

#define PORT_ERROR -1

#define pascal2inchHg    0.0002953
#define millibar2inchHg  0.02953

#define kLocalOffsetInHg 0.33
#define c2f( a ) (((a) * 1.8000) + 32)
#define ms2mph( a ) ((a) * 2.23694)
#define inHg2millibars( a ) ((a) * 33.8639)

#define kSendInterval    60 * 5   // 5 minutes
//#define kSendInterval    30 // debug


#ifndef BUFSIZE
#define BUFSIZE 1025
#endif


static time_t s_lastTime = 0;


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

void printTimePlus5()
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min + 5, tm.tm_sec);
}

time_t timeGetTimeSec()
{
    time_t rawtime = 0;
    return time( &rawtime );
}


uint8_t imax( uint8_t a, uint8_t b )
{
    return a > b ? a : b;
}


uint8_t imin( uint8_t a, uint8_t b )
{
    return a < b ? a : b;
}


void updateStats( Frame* data, Frame* min, Frame* max, Frame* ave )
{
    if( data->flags & kDataFlag_temp )
    {
        max->tempC = fmax( data->tempC, max->tempC );
        min->tempC = fmin( data->tempC, min->tempC );
        ave->tempC = (data->tempC + ave->tempC) * 0.5f;
        printf( "temperature min: %0.2f°F, max: %0.2f°F, average: %0.2f°F\n", c2f( min->tempC ), c2f( max->tempC ), c2f( ave->tempC ) );
    }
    
    if( data->flags & kDataFlag_humidity )
    {
        max->humidity = imax( data->humidity, max->humidity );
        min->humidity = imin( data->humidity, min->humidity );
        ave->humidity = (data->humidity + ave->humidity) / 2;
        printf( "humidity min: %d%%, max:%d%%, average: %d%%\n", min->humidity, max->humidity, ave->humidity );
    }

    if( data->flags & kDataFlag_wind )
    {
        max->windSpeedMs = fmax( data->windSpeedMs, max->windSpeedMs );
        min->windSpeedMs = fmin( data->windSpeedMs, min->windSpeedMs );
        ave->windSpeedMs = (data->windSpeedMs + ave->windSpeedMs) * 0.5f;

        max->windDirection = fmax( data->windDirection, max->windDirection );
        min->windDirection = fmin( data->windDirection, min->windDirection );
        ave->windDirection = (data->windDirection + ave->windDirection) * 0.5f;

        printf( "wind min: %0.2f mph, max: %0.2f mph, average: %0.2f mph\n", ms2mph( min->windSpeedMs ), ms2mph( max->windSpeedMs ), ms2mph( ave->windSpeedMs ) );
        printf( "dir  min: %0.2f deg, max: %0.2f deg, average: %0.2f deg\n", min->windDirection, max->windDirection, ave->windDirection );
    }

    if( data->flags & kDataFlag_gust )
    {
        max->windGustMs = fmax( data->windGustMs, max->windGustMs );
        min->windGustMs = fmin( data->windGustMs, min->windGustMs );
        ave->windGustMs = (data->windGustMs + ave->windGustMs) * 0.5f;
        printf( "gust min: %0.2f mph, max: %0.2f mph, average: %0.2f mph\n", ms2mph( min->windGustMs ), ms2mph( max->windGustMs ), ms2mph( ave->windGustMs ) );
    }


    if( data->flags & kDataFlag_pressure )
    {
        max->pressure = fmax( data->pressure, max->pressure );
        min->pressure = fmin( data->pressure, min->pressure );
        ave->pressure = (data->pressure + ave->pressure) * 0.5f;
        printf( "pressure min: %0.2f InHg, max: %0.2f InHg, average: %0.2f InHg\n",(min->pressure * millibar2inchHg) + kLocalOffsetInHg, (max->pressure * millibar2inchHg) + kLocalOffsetInHg, (ave->pressure * millibar2inchHg) + kLocalOffsetInHg );
    }

//    if( data.flags & kDataFlag_rain )
//        printf( "rain:       %g\n", data.rain );

//    if( data.flags & kDataFlag_intTemp )
//        printf( "int temp:   %0.2f°F\n", c2f( data.intTempC ) );

}

int main(int argc, const char * argv[])
{
    char         packetToSend[BUFSIZE] = "";
    char         packetFormat = UNCOMPRESSED_PACKET;

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

    printf( "Far Out Labs wx-relay v1.0: reading from serial port: %s...\n\n", PORT_DEVICE );
    
    // this holds all the min/max/averages
//    Frame minFrame = {};
//    Frame maxFrame = {};
//    Frame aveFrame = {};
    
    // master weather frame that is used to create APRS message
    Frame wxFrame = {};

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
            printf( "\n" );
            printTime();
            printf( "station_id: 0x%x\n", frame.station_id );

            // read flags
            if( frame.flags & kDataFlag_temp )
            {
                printf( "temp:       %0.2f°F\n", c2f( frame.tempC ) );
                wxFrame.tempC = frame.tempC;
            }

            if( frame.flags & kDataFlag_humidity )
            {
                printf( "humidity:   %d%%\n", frame.humidity );
                wxFrame.humidity = frame.humidity;
            }

            if( frame.flags & kDataFlag_wind )
            {
                printf( "wind:       %0.2f mph\n", ms2mph( frame.windSpeedMs ) );
                printf( "dir:        %0.2f degrees\n", frame.windDirection );
                wxFrame.windSpeedMs = frame.windSpeedMs;
                wxFrame.windDirection = frame.windDirection;
            }
            
            if( frame.flags & kDataFlag_gust )
            {
                printf( "gust:       %0.2f mph\n", ms2mph( frame.windGustMs ) );
                wxFrame.windGustMs = frame.windGustMs;
            }
            
            if( frame.flags & kDataFlag_rain )
            {
                printf( "rain:       %g\n", frame.rain );
                wxFrame.rain = frame.rain;
            }

            if( frame.flags & kDataFlag_intTemp )
            {
                printf( "int temp:   %0.2f°F\n", c2f( frame.intTempC ) );
                wxFrame.intTempC = frame.intTempC;
            }

            if( frame.flags & kDataFlag_pressure )
            {
                printf( "pressure:   %g InHg\n\n", (frame.pressure * millibar2inchHg) + kLocalOffsetInHg );
                wxFrame.pressure = frame.pressure;
            }

//            updateStats( &frame, &minFrame, &maxFrame, &aveFrame );

            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to average data over the 5 minute period...
            receivedFlags |= frame.flags;
            if( (receivedFlags & 0x7F) == 0x7F )
            {
//                printf( "Have full weather info...  " );
//                printTime();
//                receivedFlags = 0;        // once we have a full set, just go with it-  we only update every five minutes anyway...

                // check the time
                if( timeGetTimeSec() > s_lastTime + kSendInterval )
                {
                    printTime();
                    printf( "Sending weather info to APRS-IS...  next send: " );
                    printTimePlus5();

                    APRSPacket wx;
                    packetConstructor( &wx );

                    uncompressedPosition( wx.latitude,    34.108,     IS_LATITUDE );
                    uncompressedPosition( wx.longitude, -118.3349371, IS_LONGITUDE );

                    int formatTruncationCheck = snprintf( wx.callsign, 10, "K6LOT-13" );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.windDirection, 4, "%03d", (int)(round(wxFrame.windDirection)) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.windSpeed, 4, "%03d", (int)(round(ms2mph(wxFrame.windSpeedMs))) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.gust, 4, "%03d", (int)(round(ms2mph(wxFrame.windGustMs))) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.temperature, 4, "%03d", (int)(round(c2f(wxFrame.tempC))) );
                    assert( formatTruncationCheck >= 0 );

                    unsigned short int h = wxFrame.humidity;
                    // APRS only supports values 1-100. Round 0% up to 1%.
                    if( h == 0 )
                        h = 1;
                    
                    // APRS requires us to encode 100% as "00".
                    else if( h >= 100 )
                        h = 0;
                    
                    formatTruncationCheck = snprintf( wx.humidity, 3, "%.2d", h );
                    assert( formatTruncationCheck >= 0 );
                    
                    // we are converting back from InHg because that's the offset we know based on airport data! (this means we go from millibars -> InHg + offset -> millibars)
                    formatTruncationCheck = snprintf( wx.pressure, 6, "%.5d", (int)(round(inHg2millibars((wxFrame.pressure * millibar2inchHg) + kLocalOffsetInHg) * 10)) );
                    assert( formatTruncationCheck >= 0 );

                    printAPRSPacket( &wx, packetToSend, packetFormat, 0);
                    sendPacket( "noam.aprs2.net", 10152, "K6LOT-13", "8347", packetToSend );
                    
                    printf( "%s\n", packetToSend );
                    s_lastTime = timeGetTimeSec();
                }
                
            }
        }
        sleep( 1 );
    }
    
    return 0;
}
