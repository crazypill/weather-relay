//
//  main.c
//  weather-relay
//
//  Created by Alex Lelievre on 6/16/20.
//  Copyright © 2020 Far Out Labs. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
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
#include <string.h>
#include <netdb.h>

#include "TXDecoderFrame.h"
#include "aprs-wx.h"
#include "aprs-is.h"

#include "ax25_pad.h"
#include "kiss_frame.h"


#define DEVICE_NAME_V "folabs-wx-relay100"

//#define PORT_DEVICE "/dev/cu.usbserial-0001"
#define PORT_DEVICE "/dev/serial0"
//#define PORT_DEVICE "/dev/serial1"

#define SEND_TO_RADIO

#define PORT_ERROR -1

#define pascal2inchHg    0.0002953
#define millibar2inchHg  0.02953

#define kLocalOffsetInHg 0.33
#define kLocalTempErrorC 2.033333333333333

#define c2f( a ) (((a) * 1.8000) + 32)
#define ms2mph( a ) ((a) * 2.23694)
#define inHg2millibars( a ) ((a) * 33.8639)

#define kSendInterval    60 * 5   // 5 minutes
#define kWindInterval    60 * 2   // every 2 minutes we reset the average wind speed and direction
#define kGustInterval    60 * 10  // every 10 minutes we reset the max wind gust to 0
#define kBaroInterval    60

//#define kSendInterval    30 // debug


#ifndef BUFSIZE
#define BUFSIZE 1025
#endif

#define AX25_MAX_ADDRS 10    /* Destination, Source, 8 digipeaters. */
#define AX25_MAX_INFO_LEN 2048    /* Maximum size for APRS. */
#define AX25_MAX_PACKET_LEN ( AX25_MAX_ADDRS * 7 + 2 + 3 + AX25_MAX_INFO_LEN)


static time_t s_lastSendTime = 0;
static time_t s_lastWindTime = 0;
static time_t s_lastGustTime = 0;
static time_t s_lastBaroTime = 0;

static int connectToDireWolf( void );
static int sendToRadio( const char* p );
static int send_to_kiss_tnc( int chan, int cmd, char *data, int dlen );


void buffer_input_flush()
{
    int c;
     // This will eat up all other characters
    while( (c = getchar()) != EOF && c != '\n' )
        ;
}

void printTime( int printNewline )
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    if( printNewline )
        printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    else
        printf("%d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
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


void updateStats( const Frame* data, Frame* min, Frame* max, Frame* ave )
{
    // we create a 10 minute window of instantaneous gust measurements
    if( timeGetTimeSec() > s_lastGustTime + kGustInterval )
    {
        max->windGustMs = 0;
        s_lastGustTime = timeGetTimeSec();
    }

    if( timeGetTimeSec() > s_lastWindTime + kWindInterval )
    {
        ave->windSpeedMs = 0;
        ave->windDirection = 0;
        s_lastWindTime = timeGetTimeSec();
    }

    if( timeGetTimeSec() > s_lastBaroTime + kBaroInterval )
    {
        min->pressure = 0;
        s_lastBaroTime = timeGetTimeSec();
    }
    
    
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
    
    // for wind we want the max instantaneous over the interval period
    if( data->flags & kDataFlag_wind )
    {
        // check for no data before calculating mean
        if( ave->windSpeedMs == 0.0 )
            ave->windSpeedMs = data->windSpeedMs;

        if( ave->windDirection == 0.0 )
            ave->windDirection = data->windDirection;

        ave->windSpeedMs = (data->windSpeedMs + ave->windSpeedMs) * 0.5f;
        ave->windDirection = (data->windDirection + ave->windDirection) * 0.5f;

        printf( "wind average[%0.2f°]: %0.2f mph, time: %ld, window: %d\n", ave->windDirection, ms2mph( ave->windSpeedMs ), timeGetTimeSec() - s_lastWindTime, kWindInterval );
    }

    if( data->flags & kDataFlag_gust )
    {
        max->windGustMs = fmax( data->windGustMs, max->windGustMs );
        printf( "gust max: %0.2f mph, time: %ld, window: %d\n", ms2mph( max->windGustMs ), timeGetTimeSec() - s_lastGustTime, kGustInterval );
    }


    if( data->flags & kDataFlag_pressure )
    {
        // check for no data before calculating min
        if( min->pressure == 0.0 )
            min->pressure = data->pressure;

        min->pressure = fmin( data->pressure, min->pressure );
        printf( "pressure min: %0.2f InHg, time: %ld, window: %d\n",(min->pressure * millibar2inchHg) + kLocalOffsetInHg, timeGetTimeSec() - s_lastBaroTime, kBaroInterval );
    }

//    if( data.flags & kDataFlag_rain )
//        printf( "rain:       %g\n", data.rain );

//    if( data.flags & kDataFlag_intTemp )
//        printf( "int temp:   %0.2f°F\n", c2f( data.intTempC ) );

}


#pragma mark -

int main(int argc, const char * argv[])
{
    char         packetToSend[BUFSIZE];
    char         packetFormat = UNCOMPRESSED_PACKET;

    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;
    memset( packetToSend, 0, sizeof( packetToSend ) );

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

    printf( "%s: reading from serial port: %s...\n\n", DEVICE_NAME_V, PORT_DEVICE );

    // this holds all the min/max/averages
    Frame minFrame = {};
    Frame maxFrame = {};
    Frame aveFrame = {};

    // master weather frame that is used to create APRS message
    Frame wxFrame = {};

    uint8_t receivedFlags = 0;
    ssize_t result = 0;
    while( 1 )
    {
        Frame frame = {};

        result = read( fd, &frame, sizeof( frame ) );
        if( result == sizeof( frame ) )
        {
//            printf( "\n" );
            printTime( false );
            printf( " station_id: 0x%x", frame.station_id );

            // read flags
            if( frame.flags & kDataFlag_temp )
            {
                printf( ", temp: %0.2f°F", c2f( frame.tempC ) );
                wxFrame.tempC = frame.tempC;
            }

            if( frame.flags & kDataFlag_humidity )
            {
                printf( ", humidity: %d%%", frame.humidity );
                wxFrame.humidity = frame.humidity;
            }

            if( frame.flags & kDataFlag_wind )
            {
                printf( ", wind: %0.2f mph, dir: %0.2f degrees", ms2mph( frame.windSpeedMs ), frame.windDirection );
                wxFrame.windSpeedMs = frame.windSpeedMs;
                wxFrame.windDirection = frame.windDirection;
            }

            if( frame.flags & kDataFlag_gust )
            {
                printf( ", gust: %0.2f mph", ms2mph( frame.windGustMs ) );
                wxFrame.windGustMs = frame.windGustMs;
            }

            if( frame.flags & kDataFlag_rain )
            {
                printf( ", rain: %g", frame.rain );
                wxFrame.rain = frame.rain;
            }

            if( frame.flags & kDataFlag_intTemp )
            {
                printf( ", int temp: %0.2f°F", c2f( frame.intTempC - kLocalTempErrorC ) );
                wxFrame.intTempC = frame.intTempC;
            }

            if( frame.flags & kDataFlag_pressure )
            {
                printf( ", pressure: %g InHg", (frame.pressure * millibar2inchHg) + kLocalOffsetInHg );
                wxFrame.pressure = frame.pressure;
            }

            printf( "\n" );
            
            updateStats( &frame, &minFrame, &maxFrame, &aveFrame );

            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to average data over the 5 minute period...
            receivedFlags |= frame.flags;
            if( (receivedFlags & 0x7F) == 0x7F )
            {
                if( timeGetTimeSec() > s_lastSendTime + kSendInterval )
                {
                    printf( "\n" );
                    printTime( false );
                    printf( " Sending weather info to APRS-IS...  next update @ " );
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

                    memset( packetToSend, 0, sizeof( packetToSend ) );
                    printAPRSPacket( &wx, packetToSend, packetFormat, 0, false );
                    // add some additional info
                    strcat( packetToSend, DEVICE_NAME_V );
                    strcat( packetToSend, "\n\0" );
                    printf( "%s\n", packetToSend );

                    // send packet to APRS-IS directly but also to Direwolf running locally to hit the radio path
                    if( sendPacket( "noam.aprs2.net", 10152, "K6LOT-13", "8347", packetToSend ) < 0 )
                        printf( "packet failed to send to APRS-IS...\n" );

#ifdef SEND_TO_RADIO
                    if( sendToRadio( packetToSend ) < 0 )
                        printf( "packet failed to send via Direwolf for radio path...\n" );
#endif
                    s_lastSendTime = timeGetTimeSec();
                }
            }
        }
        sleep( 1 );
    }

    return 0;
}


// test just sending to the radio...
//int main(int argc, const char * argv[])
//{
//    // form fake packet to test...
//    sendToRadio( "K6LOT-13>APRS,TCPIP*:@190340z3406.48N/11820.10W_113/000g000t073h63b10134folabs-wx-relay100" );
//
//}


int sendToRadio( const char* p )
{
    int result = 0;
    // Parse the "TNC2 monitor format" and convert to AX.25 frame.
    unsigned char frame_data[AX25_MAX_PACKET_LEN];
    packet_t pp = ax25_from_text( (char*)p, 1 );
    if( pp != NULL )
    {
        int frame_len = ax25_pack( pp, frame_data );
        result = send_to_kiss_tnc( 0, KISS_CMD_DATA_FRAME, (char*)frame_data, frame_len );
        ax25_delete (pp);
    }
    else
    {
        printf( "ERROR! Could not convert to AX.25 frame: %s\n", p );
        return -1;
    }
    return result;
}



/*-------------------------------------------------------------------
 *
 * Name:        send_to_kiss_tnc
 *
 * Purpose:     Encapsulate the data/command, into a KISS frame, and send to the TNC.
 *
 * Inputs:    chan    - channel number.
 *
 *        cmd    - KISS_CMD_DATA_FRAME, KISS_CMD_SET_HARDWARE, etc.
 *
 *        data    - Information for KISS frame.
 *
 *        dlen    - Number of bytes in data.
 *
 * Description:    Encapsulate as KISS frame and send to TNC.
 *
 *--------------------------------------------------------------------*/

int send_to_kiss_tnc( int chan, int cmd, char *data, int dlen )
{
    unsigned char temp[1000];
    unsigned char kissed[2000];
    int klen;
    int err = 0;

    if( chan < 0 || chan > 15 ) {
      printf( "ERROR - Invalid channel %d - must be in range 0 to 15.\n", chan );
      chan = 0;
    }
    if( cmd < 0 || cmd > 15 ) {
      printf( "ERROR - Invalid command %d - must be in range 0 to 15.\n", cmd );
      cmd = 0;
    }
    if( dlen < 0 || dlen > (int)(sizeof( temp ) - 1) ) {
      printf( "ERROR - Invalid data length %d - must be in range 0 to %d.\n", dlen, (int)(sizeof( temp ) - 1) );
      dlen = sizeof( temp ) - 1;
    }

    temp[0] = (chan << 4) | cmd;
    memcpy( temp + 1, data, dlen );

    klen = kiss_encapsulate( temp, dlen + 1, kissed );
    
    // connect to direwolf and send data
//    if( s_server_sock < 0 )
//        s_server_sock = connectToDireWolf();
    
    int server_sock = connectToDireWolf();
    if( server_sock < 0 )
    {
        printf("ERROR Can't connect to direwolf...\n");
        err = -1;
        goto exit_gracefully;
    }
    
    ssize_t rc = send( server_sock, (char*)kissed, klen, 0 );
    if( rc != klen )
    {
        printf("ERROR writing KISS frame to socket.\n");
        err = -1;
    }

exit_gracefully:
    shutdown( server_sock, 2 );
    return err;
}


// returns fd to use to communicate with
int connectToDireWolf( void )
{
//    const char*      server             = "localhost";
    const char*      server             = "10.0.1.208";
    uint16_t         port               = 8001;
    int              error              = 0;
    char             foundValidServerIP = 0;
    struct addrinfo* result             = NULL;
    struct addrinfo* results;
    int              socket_desc        = -1;

    error = getaddrinfo( server, NULL, NULL, &results );
    if( error != 0 )
    {
        if( error == EAI_SYSTEM )
        {
            perror( "getaddrinfo" );
        }
        else
        {
            fprintf( stderr, "error in getaddrinfo: %s\n", server );
        }
        return error;
    }

    for( result = results; result != NULL; result = result->ai_next )
    {
        /* For readability later: */
        struct sockaddr* const addressinfo = result->ai_addr;

        socket_desc = socket( addressinfo->sa_family, SOCK_STREAM, IPPROTO_TCP );
        if (socket_desc < 0)
        {
            perror("error in socket()");
            continue; /* for loop */
        }

        /* Assign the port number. */
        switch (addressinfo->sa_family)
        {
            case AF_INET:
                ((struct sockaddr_in*)addressinfo)->sin_port   = htons(port);
                break;
            case AF_INET6:
                ((struct sockaddr_in6*)addressinfo)->sin6_port = htons(port);
                break;
        }

        if( connect( socket_desc, addressinfo, result->ai_addrlen ) >= 0 )
        {
            foundValidServerIP = 1;
            break; /* for loop */
        }
        else
        {
            perror( "error in connect()" );
            shutdown( socket_desc, 2 );
        }
    }
    freeaddrinfo( results );
    if( foundValidServerIP == 0 )
    {
        fputs( "Could not connect to the server.\n", stderr );
        error = -1;
    }
    else
    {
        // do not close down the connection if we connected!
        return socket_desc;
    }
    return error;
}
