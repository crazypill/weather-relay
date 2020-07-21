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
#include <stdint.h>
#include <stdarg.h>

#include <strings.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/termios.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <netdb.h>
#include <getopt.h>

#include "main.h"

#include "wx_thread.h"
#include "TXDecoderFrame.h"
#include "aprs-wx.h"
#include "aprs-is.h"

#include "ax25_pad.h"
#include "kiss_frame.h"


static time_t s_lastSendTime    = 0;
static time_t s_lastWindTime    = 0;
static time_t s_lastGustTime    = 0;
static time_t s_lastBaroTime    = 0;
static time_t s_lastTempTime    = 0;
static time_t s_lastIntTempTime = 0;
static time_t s_lastHumiTime    = 0;

static float s_localOffsetInHg = 0.33f;
static float s_localTempErrorC = 2.033333333333333;

static bool s_debug = false;

static const char* s_logFilePath = NULL;
static FILE*       s_logFile     = NULL;

static wx_thread_return_t sendToRadio_thread_entry( void* args );
static wx_thread_return_t sendPacket_thread_entry( void* args );

static int connectToDireWolf( void );
static int sendToRadio( const char* p );
static int send_to_kiss_tnc( int chan, int cmd, char *data, int dlen );



#pragma mark -


void nullprint( const char* format, ... )
{
    
}


char* copy_string( const char* stringToCopy )
{
    if( !stringToCopy )
        return NULL;
    
    size_t bufSize = strlen( stringToCopy ) + 1;
    char* newString = (char*)malloc( bufSize );
    if( newString )
    {
        strcpy( newString, stringToCopy );
        newString[bufSize - 1] = '\0';
    }
    return newString;
}


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




#pragma mark -


void updateStats( Frame* data, Frame* min, Frame* max, Frame* ave )
{
    if( data->flags & kDataFlag_temp )
    {
        if( timeGetTimeSec() > s_lastTempTime + kTempInterval )
        {
            ave->tempC = 0;
            s_lastTempTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->tempC == 0.0 )
            ave->tempC = data->tempC;

        ave->tempC = (data->tempC + ave->tempC) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " temp average: %0.2f°F, time left: %ld\n", c2f( ave->tempC ), kTempInterval - (timeGetTimeSec() - s_lastTempTime) );
#endif
    }

    if( data->flags & kDataFlag_intTemp )
    {
        if( timeGetTimeSec() > s_lastIntTempTime + kIntTempInterval )
        {
            ave->intTempC = 0;
            s_lastIntTempTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->intTempC == 0.0 )
            ave->intTempC = data->intTempC;

        ave->intTempC = (data->intTempC + ave->intTempC) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " int temp average: %0.2f°F, time left: %ld\n", c2f( ave->intTempC ), kIntTempInterval - (timeGetTimeSec() - s_lastIntTempTime) );
#endif
    }

    
    if( data->flags & kDataFlag_humidity )
    {
        if( timeGetTimeSec() > s_lastHumiTime + kHumiInterval )
        {
            ave->humidity = 0;
            s_lastHumiTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->humidity == 0 )
            ave->humidity = data->humidity;
        ave->humidity = (data->humidity + ave->humidity) / 2;
#ifdef TRACE_STATS
        printTime( false );
        stats( " humidity average: %d%%, time left: %ld\n", ave->humidity, kHumiInterval - (timeGetTimeSec() - s_lastHumiTime) );
#endif
    }
    
    // for wind we want the ave over the interval period
    if( data->flags & kDataFlag_wind )
    {
        if( timeGetTimeSec() > s_lastWindTime + kWindInterval )
        {
            ave->windSpeedMs = 0;
            ave->windDirection = 0;
            s_lastWindTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->windSpeedMs == 0.0 )
            ave->windSpeedMs = data->windSpeedMs;

        if( ave->windDirection == 0.0 )
            ave->windDirection = data->windDirection;

        ave->windSpeedMs = (data->windSpeedMs + ave->windSpeedMs) * 0.5f;
        ave->windDirection = (data->windDirection + ave->windDirection) * 0.5f;
#ifdef TRACE_STATS
        printTime( false );
        stats( " wind average[%0.2f°]: %0.2f mph, time left: %ld\n", ave->windDirection, ms2mph( ave->windSpeedMs ), kWindInterval - (timeGetTimeSec() - s_lastWindTime) );
#endif
    }

    // for gusts we want the max instantaneous over the interval period
    if( data->flags & kDataFlag_gust )
    {
        // I saw a 700 MPH wind gust go by which seems nuts... so trap that error
        if( ms2mph( data->windGustMs ) > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong (except for baro and int temp)
            printTime( false );
            printf( " wind gust too high[%0.2f°]: %0.2f mph, time left: %ld\n", ave->windDirection, ms2mph( ave->windGustMs ), kGustInterval - (timeGetTimeSec() - s_lastGustTime) );
            data->flags &= ~kDataFlag_gust;
            return;
        }
        
        // we create a 10 minute window of instantaneous gust measurements
        if( timeGetTimeSec() > s_lastGustTime + kGustInterval )
        {
            max->windGustMs = 0;
            s_lastGustTime = timeGetTimeSec();
        }

        max->windGustMs = fmax( data->windGustMs, max->windGustMs );
#ifdef TRACE_STATS
        printTime( false );
        stats( " gust max: %0.2f mph, time left: %ld\n", ms2mph( max->windGustMs ), kGustInterval- (timeGetTimeSec() - s_lastGustTime) );
#endif
    }


    if( data->flags & kDataFlag_pressure )
    {
        if( timeGetTimeSec() > s_lastBaroTime + kBaroInterval )
        {
            min->pressure = 0;
            s_lastBaroTime = timeGetTimeSec();
        }

        // check for no data before calculating min
        if( min->pressure == 0.0 )
            min->pressure = data->pressure;

        min->pressure = fmin( data->pressure, min->pressure );
#ifdef TRACE_STATS
        printTime( false );
        stats( " pressure min: %0.2f InHg, time left: %ld\n",(min->pressure * millibar2inchHg) + kLocalOffsetInHg, kBaroInterval - (timeGetTimeSec() - s_lastBaroTime) );
#endif
    }

//    if( data.flags & kDataFlag_rain )
//        printf( "rain:       %g\n", data.rain );


}


void printFullWeather( const Frame* inst, Frame* min, Frame* max, Frame* ave )
{
    // only show this stuff if in debug mode
    if( !s_debug )
        return;
    
    log_error( "     wind[%06.2f°]: %0.2f mph,     gust: %0.2f mph --     temp: %0.2f°F,     humidity: %2d%%,     pressure: %0.3f InHg,     int temp: %0.2f°F, rain: %g\n", inst->windDirection, ms2mph( inst->windSpeedMs ), ms2mph( inst->windGustMs ), c2f( inst->tempC ), inst->humidity, (inst->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( inst->intTempC - s_localTempErrorC ), 0.0 );
    log_error( " avg wind[%06.2f°]: %0.2f mph, max gust: %0.2f mph -- ave temp: %0.2f°F, ave humidity: %2d%%, min pressure: %0.3f InHg  ave int temp: %0.2f°F\n",            ave->windDirection, ms2mph( ave->windSpeedMs ), ms2mph( max->windGustMs ), c2f( ave->tempC ), ave->humidity, (min->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( ave->intTempC - s_localTempErrorC ) );
}


void printCurrentWeather( Frame* min, Frame* max, Frame* ave )
{
    // only show this stuff if in debug mode
    if( !s_debug )
        return;

    printf( "Wind[%06.2f°]: %0.2f mph, gust: %0.2f mph, temp: %0.2f°F, humidity: %2d%%, pressure: %0.3f InHg, int temp: %0.2f°F, rain: %g\n", ave->windDirection, ms2mph( ave->windSpeedMs ), ms2mph( max->windGustMs ), c2f( ave->tempC ), ave->humidity, (min->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( ave->intTempC - s_localTempErrorC ), 0.0 );
}


void version( int argc, const char* argv[] )
{
    printf( "%s, version %s", PROGRAM_NAME, "1.0.0" );
#ifdef DEBUG
    fputs(", compiled with debugging output", stdout);
#endif
    puts(".\n\
        Copyright (c) 2020 Far Out Labs, LLC.\n\
        This program comes with ABSOLUTELY NO WARRANTY. This is free software, and you\n\
        are welcome to redistribute it under certain conditions.  See the GNU General\n\
        Public License (version 3.0) for more details.");
    return;
}

void usage( int argc, const char* argv[] )
{
    printf( "Typical usage: %s --baro [pressure offset in InHg] --temp [temp offset in C]\n", PROGRAM_NAME );
    return;
}



void help( int argc, const char* argv[] )
{
    version( argc, argv );
    puts("");
    usage( argc, argv );
    puts( "\n\
        Special parameters:\n\
            -H, --help                 Show this help and exit.\n\
            -v, --version              Show version and licensing information, and exit.\n\
            -d, --debug                Show the incoming radio data and packet sends.\n\
            -l, --log                  Log errors to this file (debug data does not go here).\n\
        Tuning parameters:\n\
            -b, --baro                 Set the barometric pressure offset in InHg.\n\
            -t, --temp                 Set the interior temperature offset in °C.\n\
        " );
}


void log_error( const char* format, ... )
{
    char buf[2048] = {0};
    va_list vaList;
    va_start( vaList, format );
    vsprintf( buf, format, vaList );
    va_end( vaList );
    
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    
    if( s_logFile )
    {
        fprintf( s_logFile, "%d-%02d-%02d %02d:%02d:%02d: %s", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, buf );
        fflush( s_logFile );
    }
    
    // print to debug as well...
    if( s_debug )
        printf( "%d-%02d-%02d %02d:%02d:%02d: %s", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, buf );
}


void log_unix_error( const char* prefix )
{
    char buffer[512] = {0};
    strerror_r( errno, buffer, sizeof( buffer ) );
    
    char finalBuffer[1024] = {0};
    strcat( finalBuffer, prefix );
    strcat( finalBuffer, buffer );
    log_error( finalBuffer );
}



#pragma mark -

int main( int argc, const char * argv[] )
{
    // do some command processing...
    if( argc >= 2 )
    {
        int c = '\0';          /* for getopt_long() */
        int option_index = 0;  /* for getopt_long() */

        const static struct option long_options[] = {
            {"help",                    no_argument,       0, 'H'},
            {"version",                 no_argument,       0, 'v'},
            {"debug",                   no_argument,       0, 'd'},
            {"temp",                    required_argument, 0, 't'},
            {"baro",                    required_argument, 0, 'b'},
            {"log",                     required_argument, 0, 'l'},

            {0, 0, 0, 0}
            };

        while( (c = getopt_long( argc, (char* const*)argv, "Hvdt:b:l:", long_options, &option_index)) != -1 )
        {
            switch( c )
            {
                /* Complete help (-H | --help) */
                case 'H':
                    help( argc, argv );
                    return EXIT_SUCCESS;

                /* Version information (-v | --version) */
                case 'v':
                    version( argc, argv );
                    return EXIT_SUCCESS;
                    
                case 'b':
                    s_localOffsetInHg = atof( optarg );
                    break;

                case 't':
                    s_localTempErrorC = atof( optarg );
                    break;

                case 'd':
                    s_debug = true;
                    break;

                case 'l':
                    s_logFilePath = copy_string( optarg );
                    break;
            }
        }
    }
    
    if( s_debug )
        printf( "%s, version %s -- pressure offset: %0.2f InHg, interior temp offset: %0.2f °C\n", PROGRAM_NAME, VERSION, s_localOffsetInHg, s_localTempErrorC );

    if( s_logFilePath && !s_logFile )
    {
        s_logFile = fopen( s_logFilePath, "w+" );
        if( !s_logFile )
            log_error( "  failed to open log file: %s\n", s_logFilePath );
        if( s_debug )
            printf( "logging errors to: %s\n", s_logFilePath );
    }
    
    char packetToSend[BUFSIZE];
    char packetFormat = UNCOMPRESSED_PACKET;

    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;
    memset( packetToSend, 0, sizeof( packetToSend ) );

    int fd = open( PORT_DEVICE, O_RDWR | O_NOCTTY | (blocking ? 0 : O_NDELAY) );
    if( fd < 0 )
    {
        log_error( "Failed to open serial port: %s\n", PORT_DEVICE );
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

    trace( "%s: reading from serial port: %s...\n\n", PROGRAM_NAME, PORT_DEVICE );

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
            // doing this first allows us to turn off flags for bad measurements so this code skips them too-
            updateStats( &frame, &minFrame, &maxFrame, &aveFrame );
            
#ifdef TRACE_INCOMING_WX
            printTime( false );
#endif
            trace( " station_id: 0x%x", frame.station_id );

            // read flags
            if( frame.flags & kDataFlag_temp )
            {
                trace( ", temp: %0.2f°F", c2f( frame.tempC ) );
                wxFrame.tempC = frame.tempC;
            }

            if( frame.flags & kDataFlag_humidity )
            {
                trace( ", humidity: %d%%", frame.humidity );
                wxFrame.humidity = frame.humidity;
            }

            if( frame.flags & kDataFlag_wind )
            {
                trace( ", wind: %0.2f mph, dir: %0.2f degrees", ms2mph( frame.windSpeedMs ), frame.windDirection );
                wxFrame.windSpeedMs = frame.windSpeedMs;
                wxFrame.windDirection = frame.windDirection;
            }

            if( frame.flags & kDataFlag_gust )
            {
                trace( ", gust: %0.2f mph", ms2mph( frame.windGustMs ) );
                wxFrame.windGustMs = frame.windGustMs;
            }

            if( frame.flags & kDataFlag_rain )
            {
                trace( ", rain: %g", frame.rain );
                wxFrame.rain = frame.rain;
            }

            if( frame.flags & kDataFlag_intTemp )
            {
                trace( ", int temp: %0.2f°F", c2f( frame.intTempC - kLocalTempErrorC ) );
                wxFrame.intTempC = frame.intTempC;
            }

            if( frame.flags & kDataFlag_pressure )
            {
                trace( ", pressure: %g InHg", (frame.pressure * millibar2inchHg) + s_localOffsetInHg );
                wxFrame.pressure = frame.pressure;
            }

            trace( "\n" );
            printFullWeather( &wxFrame, &minFrame, &maxFrame, &aveFrame );

            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to average data over the 5 minute period...
            receivedFlags |= frame.flags;
            if( (receivedFlags & 0x7F) == 0x7F )
            {
                if( timeGetTimeSec() > s_lastSendTime + kSendInterval )
                {
                    if( s_debug )
                    {
                        printf( "\n" );
                        printTime( false );
                        printf( " Sending weather info to APRS-IS...  next update @ " );
                        printTimePlus5();   // total hack and will display times such as 13:64 ?! (which is really 14:04)
                        printCurrentWeather( &minFrame, &maxFrame, &aveFrame );
                    }
    
                    APRSPacket wx;
                    packetConstructor( &wx );

                    uncompressedPosition( wx.latitude,    34.108,     IS_LATITUDE );
                    uncompressedPosition( wx.longitude, -118.3349371, IS_LONGITUDE );

                    int formatTruncationCheck = snprintf( wx.callsign, 10, "K6LOT-13" );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.windDirection, 4, "%03d", (int)(round(aveFrame.windDirection)) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.windSpeed, 4, "%03d", (int)(round(ms2mph(aveFrame.windSpeedMs))) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.gust, 4, "%03d", (int)(round(ms2mph(maxFrame.windGustMs))) );
                    assert( formatTruncationCheck >= 0 );

                    formatTruncationCheck = snprintf( wx.temperature, 4, "%03d", (int)(round(c2f(aveFrame.tempC))) );
                    assert( formatTruncationCheck >= 0 );

                    unsigned short int h = aveFrame.humidity;
                    // APRS only supports values 1-100. Round 0% up to 1%.
                    if( h == 0 )
                        h = 1;

                    // APRS requires us to encode 100% as "00".
                    else if( h >= 100 )
                        h = 0;

                    formatTruncationCheck = snprintf( wx.humidity, 3, "%.2d", h );
                    assert( formatTruncationCheck >= 0 );

                    // we are converting back from InHg because that's the offset we know based on airport data! (this means we go from millibars -> InHg + offset -> millibars)
                    formatTruncationCheck = snprintf( wx.pressure, 6, "%.5d", (int)(round(inHg2millibars((minFrame.pressure * millibar2inchHg) + s_localOffsetInHg) * 10)) );
                    assert( formatTruncationCheck >= 0 );

                    memset( packetToSend, 0, sizeof( packetToSend ) );
                    printAPRSPacket( &wx, packetToSend, packetFormat, 0, false );
                    // add some additional info
                    strcat( packetToSend, PROGRAM_NAME );
                    strcat( packetToSend, VERSION );
                    strcat( packetToSend, "\n\0" );
                    printf( "%s\n", packetToSend );
                    
                    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
                    wx_create_thread( sendPacket_thread_entry, copy_string( packetToSend ) );
                    wx_create_thread( sendToRadio_thread_entry, copy_string( packetToSend ) );

                    s_lastSendTime = timeGetTimeSec();
                }
            }
        }
        sleep( 1 );
    }
    
    if( s_logFile )
        fclose( s_logFile );
    return EXIT_SUCCESS;
}



#pragma mark -


wx_thread_return_t sendPacket_thread_entry( void* args )
{
    char* packetToSend = (char*)args;
    if( !packetToSend )
        wx_thread_return();

    // send packet to APRS-IS directly but also to Direwolf running locally to hit the radio path
    int err = sendPacket( "noam.aprs2.net", 10152, "K6LOT-13", "8347", packetToSend );
    if( err != 0 )
        log_error( "packet failed to send to APRS-IS, error: %d...\n", err );
    
    free( packetToSend );
    wx_thread_return();
}


wx_thread_return_t sendToRadio_thread_entry( void* args )
{
    char* packetToSend = (char*)args;
    if( !packetToSend )
        wx_thread_return();

    int err = sendToRadio( packetToSend );
    if( err != 0 )
        log_error( "packet failed to send via Direwolf for radio path, error: %d...\n", err );
    
    free( packetToSend );
    wx_thread_return();
}



#pragma mark -


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
        log_error( "ERROR! Could not convert to AX.25 frame: %s\n", p );
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

int send_to_kiss_tnc( int chan, int cmd, char* data, int dlen )
{
    unsigned char temp[1000];
    unsigned char kissed[2000];
    int klen;
    int err = 0;

    if( chan < 0 || chan > 15 ) {
      log_error( "ERROR - Invalid channel %d - must be in range 0 to 15.\n", chan );
      chan = 0;
    }
    if( cmd < 0 || cmd > 15 ) {
      log_error( "ERROR - Invalid command %d - must be in range 0 to 15.\n", cmd );
      cmd = 0;
    }
    if( dlen < 0 || dlen > (int)(sizeof( temp ) - 1) ) {
      log_error( "ERROR - Invalid data length %d - must be in range 0 to %d.\n", dlen, (int)(sizeof( temp ) - 1) );
      dlen = sizeof( temp ) - 1;
    }

    temp[0] = (chan << 4) | cmd;
    memcpy( temp + 1, data, dlen );

    klen = kiss_encapsulate( temp, dlen + 1, kissed );
    
    // connect to direwolf and send data
    int server_sock = connectToDireWolf();
    if( server_sock < 0 )
    {
        log_error( "ERROR Can't connect to direwolf...\n" );
        err = -1;
        goto exit_gracefully;
    }
    
    ssize_t rc = send( server_sock, (char*)kissed, klen, 0 );
    if( rc != klen )
    {
        log_error( "ERROR writing KISS frame to socket.\n" );
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
            log_unix_error( "connectToDireWolf:getaddrinfo: " );
        }
        else
        {
            log_error( "error in getaddrinfo: %s\n", server );
        }
        return error;
    }

    for( result = results; result != NULL; result = result->ai_next )
    {
        /* For readability later: */
        struct sockaddr* const addressinfo = result->ai_addr;

        socket_desc = socket( addressinfo->sa_family, SOCK_STREAM, IPPROTO_TCP );
        if( socket_desc < 0 )
        {
            log_unix_error( "connectToDireWolf:socket: " );
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
            log_unix_error( "connectToDireWolf:connect: " );
            shutdown( socket_desc, 2 );
        }
    }
    freeaddrinfo( results );
    if( foundValidServerIP == 0 )
    {
        log_error( "Could not connect to the server.\n" );
        error = -1;
    }
    else
    {
        // do not close down the connection if we connected!
        return socket_desc;
    }
    return error;
}
