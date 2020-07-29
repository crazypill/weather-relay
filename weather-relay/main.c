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
#include <signal.h>

#include "main.h"

#include "wx_thread.h"
#include "TXDecoderFrame.h"
#include "aprs-wx.h"
#include "aprs-is.h"

#include "ax25_pad.h"
#include "kiss_frame.h"


//#define TRACE_STATS
//#define TRACE_AIR_STATS
//#define TIME_OUT_OLD_DATA
//#define TRACE_INSERTS
//#define TRACE_AVERAGES

#define kCallSign "K6LOT-13"
#define kPasscode "8347"

#define kStartingTimeThreshold 30   // 30 seconds
#define kMaxNumberOfRecords    200  // 10 minutes (600 seconds) we need 600 / 5 sec = 120 wxrecords minimum.  Let's round up to 200.

typedef struct
{
    time_t timeStampSecs;
    Frame  frame;
} wxrecord;


static time_t s_lastSendTime      = 0;
static time_t s_lastWindTime      = 0;
static time_t s_lastGustTime      = 0;
static time_t s_lastBaroTime      = 0;
static time_t s_lastTempTime      = 0;
static time_t s_lastIntTempTime   = 0;
static time_t s_lastHumiTime      = 0;
static time_t s_lastAirTime       = 0;
static time_t s_lastParamsTime    = 0;
static time_t s_lastTelemetryTime = 0;
static time_t s_lastStatusTime    = 0;
static time_t s_startupTime       = 0;

static float s_localOffsetInHg = 0.33f;
static float s_localTempErrorC = 2.033333333333333;

static bool s_debug = false;

static const char* s_logFilePath = NULL;
static FILE*       s_logFile     = NULL;

static const char* s_seqFilePath = NULL;
static FILE*       s_seqFile     = NULL;

static const char* s_wxlogFilePath = NULL;
static FILE*       s_wxlogFile     = NULL;
static wxrecord*   s_wxlog         = NULL;

static const char* s_port_device  = PORT_DEVICE;
static const char* s_kiss_server  = "localhost";
static uint16_t    s_kiss_port    = 8001;
static uint8_t     s_num_retries  = 10;
static uint16_t    s_sequence_num = 0;
static size_t      s_wx_count     = 0;
static size_t      s_wx_size_secs = 0;
static bool        s_test_mode    = false;

static wx_thread_return_t sendToRadio_thread_entry( void* args );
static wx_thread_return_t sendPacket_thread_entry( void* args );

static int  connectToDireWolf( void );
static int  sendToRadio( const char* p );
static int  send_to_kiss_tnc( int chan, int cmd, char *data, int dlen );

static void transmit_wx_frame( const Frame* frame );
static void transmit_wx_data( const Frame* min, const Frame* max, const Frame* ave );
static void transmit_air_data( const Frame* frame );
static void transmit_status( const Frame* frame );

static bool wxlog_startup( void );
static bool wxlog_shutdown( void );
static bool wxlog_frame( const Frame* wxFrame );
static bool wxlog_get_wx_averages( Frame* wxFrame );

static void dump_frames( void );


#pragma mark -

int getErrno( int result )
{
    int err;
    
    err = 0;
    if (result < 0) {
        err = errno;
        assert(err != 0);
    }
    return err;
}


int ignoreSIGPIPE()
{
    int err;
    struct sigaction signalState;
    
    err = sigaction( SIGPIPE, NULL, &signalState );
    err = getErrno( err );
    if( err == 0 )
    {
        signalState.sa_handler = SIG_IGN;
        err = sigaction( SIGPIPE, &signalState, NULL );
        err = getErrno( err );
    }
    
    return err;
}


void signalHandler( int sig )
{
    switch( sig )
    {
        case SIGHUP:
            dump_frames();
            break;
            
        case SIGINT:
            wxlog_shutdown();
            exit( EXIT_SUCCESS );
            break;
            
        case SIGTERM:
            wxlog_shutdown();
            exit( EXIT_SUCCESS );
            break;

        default:
            assert( false );
            break;
    }
}



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
    time_t t = time( NULL );
    struct tm tm = *localtime(&t);
    if( printNewline )
        printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    else
        printf("%d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}


void printTimePlus5()
{
  time_t t = time( NULL );
  struct tm tm = *localtime(&t);
  printf("%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min + 5, tm.tm_sec);
}


void printFullWeather( const Frame* inst, const Frame* min, const Frame* max, const Frame* ave )
{
    // only show this stuff if in debug mode
    if( !s_debug )
        return;
    
    log_error( "     wind[%06.2f°]: %0.2f mph,     gust: %0.2f mph --     temp: %0.2f°F,     humidity: %2d%%,     pressure: %0.3f InHg,     int temp: %0.2f°F, rain: %g\n", inst->windDirection, ms2mph( inst->windSpeedMs ), ms2mph( inst->windGustMs ), c2f( inst->tempC ), inst->humidity, (inst->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( inst->intTempC - s_localTempErrorC ), 0.0 );
    log_error( " avg wind[%06.2f°]: %0.2f mph, max gust: %0.2f mph -- ave temp: %0.2f°F, ave humidity: %2d%%, min pressure: %0.3f InHg  ave int temp: %0.2f°F\n",            ave->windDirection, ms2mph( ave->windSpeedMs ), ms2mph( max->windGustMs ), c2f( ave->tempC ), ave->humidity, (min->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( ave->intTempC - s_localTempErrorC ) );
    log_error( " pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d),  3um: %03d,  5um: %03d,  10um: %03d,  25um: %03d,  50um: %03d,  100um: %03d\n", inst->pm10_standard, inst->pm10_env, inst->pm25_standard, inst->pm25_env, inst->pm100_standard, inst->pm100_env, inst->particles_03um, inst->particles_05um, inst->particles_10um, inst->particles_25um, inst->particles_50um, inst->particles_100um );
}


void printCurrentWeather( const Frame* frame, bool alwaysPrint )
{
    // only show this stuff if in debug mode
    if( !alwaysPrint && !s_debug && !s_test_mode )
        return;

    printf( "Wind[%06.2f°]: %0.2f mph, gust: %0.2f mph, temp: %0.2f°F, humidity: %2d%%, pressure: %0.3f InHg, int temp: %0.2f°F, rain: %g\n", frame->windDirection, ms2mph( frame->windSpeedMs ), ms2mph( frame->windGustMs ), c2f( frame->tempC ), frame->humidity, (frame->pressure * millibar2inchHg) + s_localOffsetInHg, c2f( frame->intTempC - s_localTempErrorC ), 0.0 );
}



void dump_frames( void )
{
    if( !s_wxlog )
        return;
    
    printf( "dumping %zu frames:\n", s_wx_count );
    
    for( int i = 0; i < s_wx_count; i++ )
    {
        struct tm tm = *localtime( &s_wxlog[i].timeStampSecs );
        printf( "%d-%02d-%02d %02d:%02d:%02d.%03d: ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, i );
        printCurrentWeather( &s_wxlog[i].frame, true );
    }
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


// https://weather.gladstonefamily.net/CWOP_Guide.pdf
void updateStats( Frame* data, Frame* min, Frame* max, Frame* ave )
{
    if( data->flags & kDataFlag_temp )
    {
        float tempF = c2f( data->tempC );
        if( tempF < -60.0f || tempF > 130.0f )
        {
            // blow off this entire frame of data- it's probably all wrong
            log_error( " temp out of range %0.2f°F, time left: %ld\n", tempF, kTempPeriod - (timeGetTimeSec() - s_lastTempTime) );
            data->flags &= ~kDataFlag_temp;
            return;
        }
        
        // do temporal check now
        if( ave->tempC )
        {
            // look at the average temp and see if the current temp is greater than 35°F/hr
            // !!@ we skip the per hour part and just look to see if any entry is that much different than the one before it...
            if( fabs( tempF - c2f( ave->tempC ) ) > 35.0f )
            {
                // blow off this entire frame of data- it's probably all wrong
                log_error( " temp temporal check failed: %0.2f°F, ave: %0.2f°F time left: %ld\n", tempF, c2f( ave->tempC ), kTempPeriod - (timeGetTimeSec() - s_lastTempTime) );
                data->flags &= ~kDataFlag_temp;
                return;
            }
        }

        if( timeGetTimeSec() > s_lastTempTime + kTempPeriod )
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
        stats( " temp average: %0.2f°F, time left: %ld\n", c2f( ave->tempC ), kTempPeriod - (timeGetTimeSec() - s_lastTempTime) );
#endif
    }

    if( data->flags & kDataFlag_intTemp )
    {
        if( timeGetTimeSec() > s_lastIntTempTime + kIntTempPeriod )
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
        stats( " int temp average: %0.2f°F, time left: %ld\n", c2f( ave->intTempC ), kIntTempPeriod - (timeGetTimeSec() - s_lastIntTempTime) );
#endif
    }

    
    if( data->flags & kDataFlag_humidity )
    {
        if( data->humidity < 0 || data->humidity > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong
            log_error( " humidity out of range %d%%, time left: %ld\n", data->humidity, kHumiPeriod - (timeGetTimeSec() - s_lastHumiTime) );
            data->flags &= ~kDataFlag_humidity;
            return;
        }

        if( timeGetTimeSec() > s_lastHumiTime + kHumiPeriod )
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
        stats( " humidity average: %d%%, time left: %ld\n", ave->humidity, kHumiPeriod - (timeGetTimeSec() - s_lastHumiTime) );
#endif
    }
    
    // for wind we want the ave over the interval period
    if( data->flags & kDataFlag_wind )
    {
        if( ms2mph( data->windSpeedMs ) > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong (except for baro and int temp)
            log_error( " wind speed too high[%0.2f°]: %0.2f mph, time left: %ld\n", data->windDirection, ms2mph( data->windSpeedMs ), kWindPeriod - (timeGetTimeSec() - s_lastWindTime) );
            data->flags &= ~kDataFlag_wind;
            return;
        }

        if( data->windDirection < 0 || data->windDirection > 360 )
        {
            // blow off this entire frame of data- it's probably all wrong
            log_error( " wind direction out of range [%0.2f°]: %0.2f mph, time left: %ld\n", data->windDirection, ms2mph( data->windSpeedMs ), kWindPeriod - (timeGetTimeSec() - s_lastWindTime) );
            data->flags &= ~kDataFlag_wind;
            return;
        }
        
        if( timeGetTimeSec() > s_lastWindTime + kWindPeriod )
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
        stats( " wind average[%0.2f°]: %0.2f mph, time left: %ld\n", ave->windDirection, ms2mph( ave->windSpeedMs ), kWindPeriod - (timeGetTimeSec() - s_lastWindTime) );
#endif
    }

    // for gusts we want the max instantaneous over the interval period
    if( data->flags & kDataFlag_gust )
    {
        // I saw a 700 MPH wind gust go by which seems nuts... so trap that error
        if( ms2mph( data->windGustMs ) > 100 )
        {
            // blow off this entire frame of data- it's probably all wrong (except for baro and int temp)
            log_error( " wind gust too high[%0.2f°]: %0.2f mph, time left: %ld\n", data->windDirection, ms2mph( data->windGustMs ), kGustPeriod - (timeGetTimeSec() - s_lastGustTime) );
            data->flags &= ~kDataFlag_gust;
            return;
        }
        
        // we create a 10 minute window of instantaneous gust measurements
        if( timeGetTimeSec() > s_lastGustTime + kGustPeriod )
        {
            max->windGustMs = 0;
            s_lastGustTime = timeGetTimeSec();
        }

        max->windGustMs = fmax( data->windGustMs, max->windGustMs );
#ifdef TRACE_STATS
        printTime( false );
        stats( " gust max: %0.2f mph, time left: %ld\n", ms2mph( max->windGustMs ), kGustPeriod - (timeGetTimeSec() - s_lastGustTime) );
#endif
    }


    if( data->flags & kDataFlag_pressure )
    {
        if( timeGetTimeSec() > s_lastBaroTime + kBaroPeriod )
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
        stats( " pressure min: %0.2f InHg, time left: %ld\n",(min->pressure * millibar2inchHg) + s_localOffsetInHg, kBaroPeriod - (timeGetTimeSec() - s_lastBaroTime) );
#endif
    }


    if( data->flags & kDataFlag_airQuality )
    {
        if( timeGetTimeSec() > s_lastAirTime + kAirPeriod )
        {
            ave->pm10_standard = 0;
            ave->pm25_standard = 0;
            ave->pm100_standard = 0;
            ave->pm10_env = 0;
            ave->pm25_env = 0;
            ave->pm100_env = 0;
            ave->particles_03um = 0;
            ave->particles_05um = 0;
            ave->particles_10um = 0;
            ave->particles_25um = 0;
            ave->particles_50um = 0;
            ave->particles_100um = 0;
            s_lastAirTime = timeGetTimeSec();
        }

        // check for no data before calculating mean
        if( ave->pm10_standard == 0 )
            ave->pm10_standard = data->pm10_standard;
        ave->pm10_standard = (data->pm10_standard + ave->pm10_standard) / 2;

        if( ave->pm25_standard == 0 )
            ave->pm25_standard = data->pm25_standard;
        ave->pm25_standard = (data->pm25_standard + ave->pm25_standard) / 2;

        if( ave->pm100_standard == 0 )
            ave->pm100_standard = data->pm100_standard;
        ave->pm100_standard = (data->pm100_standard + ave->pm100_standard) / 2;

        if( ave->pm10_env == 0 )
            ave->pm10_env = data->pm10_env;
        ave->pm10_env = (data->pm10_env + ave->pm10_env) / 2;

        if( ave->pm25_env == 0 )
            ave->pm25_env = data->pm25_env;
        ave->pm25_env = (data->pm25_env + ave->pm25_env) / 2;

        if( ave->pm100_env == 0 )
            ave->pm100_env = data->pm100_env;
        ave->pm100_env = (data->pm100_env + ave->pm100_env) / 2;

        if( ave->particles_03um == 0 )
            ave->particles_03um = data->particles_03um;
        ave->particles_03um = (data->particles_03um + ave->particles_03um) / 2;

        if( ave->particles_05um == 0 )
            ave->particles_05um = data->particles_05um;
        ave->particles_05um = (data->particles_05um + ave->particles_05um) / 2;

        if( ave->particles_10um == 0 )
            ave->particles_10um = data->particles_10um;
        ave->particles_10um = (data->particles_10um + ave->particles_10um) / 2;

        if( ave->particles_25um == 0 )
            ave->particles_25um = data->particles_25um;
        ave->particles_25um = (data->particles_25um + ave->particles_25um) / 2;

        if( ave->particles_50um == 0 )
            ave->particles_50um = data->particles_50um;
        ave->particles_50um = (data->particles_50um + ave->particles_50um) / 2;

        if( ave->particles_100um == 0 )
            ave->particles_100um = data->particles_100um;
        ave->particles_100um = (data->particles_100um + ave->particles_100um) / 2;

        
#ifdef TRACE_AIR_STATS
        log_error( "averages pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d), 3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d, 100um: %03d\n", ave->pm10_standard, ave->pm10_env, ave->pm25_standard, ave->pm25_env, ave->pm100_standard, ave->pm100_env, ave->particles_03um, ave->particles_05um, ave->particles_10um, ave->particles_25um, ave->particles_50um, ave->particles_100um );
#endif
    }

    
//    if( data.flags & kDataFlag_rain )
//        printf( "rain:       %g\n", data.rain );


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
    printf( "Typical usage: %s --kiss \"10.0.1.208\"\n", PROGRAM_NAME );
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
            -x, --test                 Do everything except send the actual packets, for testing...\n\
            -l, --log                  Log errors and debug info to this file.\n\
            -w, --wxlog                Set the weather log file to use for up to the minute stats on restart.\n\
        Tuning parameters:\n\
            -b, --baro                 Set the barometric pressure offset in InHg.\n\
            -t, --temp                 Set the interior temperature offset in °C.\n\
        Override parameters:\n\
            -k, --kiss                 Set the server we want to use, defaults to localhost.\n\
            -p, --port                 Set the port we want to use, defaults to 8001.\n\
            -s, --seq                  Set the starting sequence number.\n\
            -e, --device               Set the serial device to use (defaults to /dev/serial0).\n\
         Required parameters:\n\
            -f, --file                 Set the sequence file to use.\n\
        " );
}


void handle_command( int argc, const char * argv[] )
{
    int c = '\0';          /* for getopt_long() */
    int option_index = 0;  /* for getopt_long() */

    const static struct option long_options[] = {
        {"help",                    no_argument,       0, 'H'},
        {"version",                 no_argument,       0, 'v'},
        {"debug",                   no_argument,       0, 'd'},
        {"test",                    no_argument,       0, 'x'},
        {"temp",                    required_argument, 0, 't'},
        {"baro",                    required_argument, 0, 'b'},
        {"log",                     required_argument, 0, 'l'},
        {"kiss",                    required_argument, 0, 'k'},
        {"port",                    required_argument, 0, 'p'},
        {"seq",                     required_argument, 0, 's'},
        {"file",                    required_argument, 0, 'f'},
        {"wxlog",                   required_argument, 0, 'w'},
        {"device",                  required_argument, 0, 'e'},

        {0, 0, 0, 0}
        };

    while( (c = getopt_long( argc, (char* const*)argv, "Hvdxt:b:l:k:p:s:f:w:e:", long_options, &option_index)) != -1 )
    {
        switch( c )
        {
            /* Complete help (-H | --help) */
            case 'H':
                help( argc, argv );
                break;

            /* Version information (-v | --version) */
            case 'v':
                version( argc, argv );
                break;

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
                s_logFilePath = optarg;
                break;

            case 'k':
                s_kiss_server = optarg;
                break;

            case 'p':
                s_kiss_port = atoi( optarg );
                break;

            case 's':
                s_sequence_num = atoi( optarg );
                break;

            case 'f':
                s_seqFilePath = optarg;
                break;
                
            case 'w':
                s_wxlogFilePath = optarg;
                break;

            case 'e':
                s_port_device = optarg;
                break;
                
            case 'x':
                s_test_mode = true;
                break;
        }
    }
}


#pragma mark -
         
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
    strcat( finalBuffer, "\n" );
    log_error( finalBuffer );
}



#pragma mark -

int main( int argc, const char * argv[] )
{
    int err = ignoreSIGPIPE();
    if( err == 0 )
    {
        signal( SIGINT,  signalHandler );
        signal( SIGTERM, signalHandler );
        signal( SIGHUP,  signalHandler );
    }

    // do some command processing...
    if( argc >= 2 )
        handle_command( argc, argv );
    
    if( s_debug )
        printf( "%s, version %s -- pressure offset: %0.2f InHg, interior temp offset: %0.2f °C, kiss: %s:%d\n", PROGRAM_NAME, VERSION, s_localOffsetInHg, s_localTempErrorC, s_kiss_server, s_kiss_port );

    if( s_logFilePath && !s_logFile )
    {
        s_logFile = fopen( s_logFilePath, "a" );
        if( !s_logFile )
            log_error( "  failed to open log file: %s\n", s_logFilePath );
        if( s_debug )
        {
            printf( "logging errors to: %s\n", s_logFilePath );
            
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            
            if( s_logFile )
                fprintf( s_logFile, "%d-%02d-%02d %02d:%02d:%02d: %s, version %s -- pressure offset: %0.2f InHg, interior temp offset: %0.2f °C, kiss: %s:%d\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, PROGRAM_NAME, VERSION, s_localOffsetInHg, s_localTempErrorC, s_kiss_server, s_kiss_port );
        }
    }

    // look for sequence file if we have a path and do not have a sequence number override
    if( !s_sequence_num && s_seqFilePath && !s_seqFile )
    {
        s_seqFile = fopen( s_seqFilePath, "rb" );
        if( !s_seqFile )
            log_error( "  failed to open sequence file: %s\n", s_seqFilePath );
        if( s_seqFile )
        {
            fread( &s_sequence_num, sizeof( uint16_t ), 1, s_seqFile );
            fclose( s_seqFile );
        }
    }
    
    wxlog_startup();

    // open the serial port
    bool blocking = false;

    // Settings structure old and new
    struct termios newtio;

    int fd = open( s_port_device, O_RDWR | O_NOCTTY | (blocking ? 0 : O_NDELAY) );
    if( fd < 0 )
    {
        log_error( " failed to open serial port: %s\n", s_port_device );
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

    trace( "%s: reading from serial port: %s...\n\n", PROGRAM_NAME, s_port_device );
    
    // this holds all the min/max/averages
    Frame minFrame = {};
    Frame maxFrame = {};
    Frame aveFrame = {};

    // primary weather frame that is used to create APRS message
    Frame wxFrame;
    memset( &wxFrame, 0, sizeof( Frame ) );

    uint8_t receivedFlags = 0;
    ssize_t result = 0;
    while( 1 )
    {
        Frame frame;
       
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
                trace( ", int temp: %0.2f°F", c2f( frame.intTempC - s_localTempErrorC ) );
                wxFrame.intTempC = frame.intTempC;
            }

            if( frame.flags & kDataFlag_pressure )
            {
                trace( ", pressure: %g InHg", (frame.pressure * millibar2inchHg) + s_localOffsetInHg );
                wxFrame.pressure = frame.pressure;
            }
            
            if( frame.flags & kDataFlag_airQuality )
            {
                trace( "          pm10: %03d (%03d), pm25: %03d (%03d), pm100: %03d (%03d), 3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d, 100um: %03d\n", frame.pm10_standard, frame.pm10_env, frame.pm25_standard, frame.pm25_env, frame.pm100_standard, frame.pm100_env, frame.particles_03um, frame.particles_05um, frame.particles_10um, frame.particles_25um, frame.particles_50um, frame.particles_100um );
                wxFrame.pm10_standard = frame.pm10_standard;
                wxFrame.pm10_env = frame.pm10_env;
                wxFrame.pm25_standard = frame.pm25_standard;
                wxFrame.pm25_env = frame.pm25_env;
                wxFrame.pm100_standard = frame.pm100_standard;
                wxFrame.pm100_env = frame.pm100_env;
                wxFrame.particles_03um = frame.particles_03um;
                wxFrame.particles_05um = frame.particles_05um;
                wxFrame.particles_10um = frame.particles_10um;
                wxFrame.particles_25um = frame.particles_25um;
                wxFrame.particles_50um = frame.particles_50um;
                wxFrame.particles_100um = frame.particles_100um;
            }

            trace( "\n" );
            printFullWeather( &wxFrame, &minFrame, &maxFrame, &aveFrame );

            // ok keep track of all the weather data we received, lets only send a packet once we have all the weather data
            // and at least 5 minutes has passed...  !!@ also need to change averaging code to use average up to the instant
            // using saved data...  right now have good code to deal with not having enough data for averages.
            receivedFlags |= frame.flags;
            
            uint8_t dataMask = kDataFlag_allMask;
            if( s_test_mode )
                dataMask = 0x1F; // this is just the data from the radio with no additional sensor data
                
            if( (receivedFlags & dataMask) == dataMask )
            {
                // set the startup time to right now so that when we startup we don't send all the messages at the same time - do this only once, first time we get full weather message
                if( !s_startupTime )
                    s_startupTime = timeGetTimeSec();

                // this is where we record the data to disk FILO up to our longest window (10 minutes).
                wxlog_frame( &wxFrame );
                
                time_t current = timeGetTimeSec();
                
                if( current > s_lastSendTime + kSendInterval )
                {
                    // get real-time averages from disk-data...for tx
                    if( wxlog_get_wx_averages( &wxFrame ) )
                        transmit_wx_frame( &wxFrame );
                    else
                        transmit_wx_data( &minFrame, &maxFrame, &aveFrame );
                    
                    s_lastSendTime = timeGetTimeSec();
                }

                if( (current > s_lastTelemetryTime + kSendInterval) && (current - s_startupTime > kTelemDelaySecs ) )
                {
                    // get real-time averages from disk-data...for tx
                    if( wxlog_get_wx_averages( &wxFrame ) )
                        transmit_air_data( &wxFrame );
                    else
                        transmit_air_data( &aveFrame );
                    
                    s_lastTelemetryTime = timeGetTimeSec();
                }
                
                if( (current > s_lastStatusTime + kStatusInterval) && (current - s_startupTime > kStatusDelaySecs) )
                {
                    // get real-time averages from disk-data...for tx
                    if( wxlog_get_wx_averages( &wxFrame ) )
                        transmit_status( &wxFrame );
                    else
                        transmit_status( &aveFrame );

                    s_lastStatusTime = timeGetTimeSec();
                }
            }
        }
        sleep( 1 );
    }
    
    // loop never finishes so this code never executes...
    if( s_logFile )
        fclose( s_logFile );
    return EXIT_SUCCESS;
}



#pragma mark -


wx_thread_return_t sendPacket_thread_entry( void* args )
{
    const char* packetToSend = (const char*)args;
    if( !packetToSend )
        wx_thread_return();

    if( s_test_mode )
    {
        log_error( "packet that would be sent: %s\n", packetToSend );
    }
    else
    {
        bool success = false;
        for( int i = 0; i < s_num_retries; i++ )
        {
            // send packet to APRS-IS directly...  oh btw, if you use this code, please get your own callsign and passcode!  PLEASE
            int err = sendPacket( "noam.aprs2.net", 10152, kCallSign, kPasscode, packetToSend );
            if( err == 0 )
            {
                log_error( "packet sent: %s\n", packetToSend );
                success = true;
                break;
            }
            else
                log_error( "packet failed to send to APRS-IS, error: %d.  %d of %d retries: %s\n", err, i + 1, s_num_retries, packetToSend );
        }
        
        if( !success )
            log_error( "packet NOT sent to APRS-IS, error: %d...%s\n", errno, packetToSend );
    }
    
    free( (void*)packetToSend );
    wx_thread_return();
}


wx_thread_return_t sendToRadio_thread_entry( void* args )
{
    char* packetToSend = (char*)args;
    if( !packetToSend )
        wx_thread_return();
    
    // also send a packet to Direwolf running locally to hit the radio path...
    int err = sendToRadio( packetToSend );
    if( err != 0 )
        log_error( "packet failed to send via Direwolf for radio path, error: %d...\n", err );
    
    free( packetToSend );
    wx_thread_return();
}


void transmit_wx_data( const Frame* minFrame, const Frame* maxFrame, const Frame* aveFrame )
{
    Frame wx;
    wx.windDirection = aveFrame->windDirection;
    wx.windSpeedMs   = aveFrame->windSpeedMs;
    wx.windGustMs    = maxFrame->windGustMs;
    wx.tempC         = aveFrame->tempC;
    wx.humidity      = aveFrame->humidity;
    wx.pressure      = minFrame->pressure;
    transmit_wx_frame( &wx );
}


void transmit_wx_frame( const Frame* frame )
{
    char packetToSend[BUFSIZE];
    char packetFormat = UNCOMPRESSED_PACKET;

    if( s_debug )
    {
        printf( "\n" );
        printTime( false );
        printf( " Sending weather info to APRS-IS...  next update @ " );
        printTimePlus5();   // total hack and will display times such as 13:64 ?! (which is really 14:04)
        printCurrentWeather( frame, false );
    }

    APRSPacket wx;
    packetConstructor( &wx );

    uncompressedPosition( wx.latitude,    34.108,     IS_LATITUDE );
    uncompressedPosition( wx.longitude, -118.3349371, IS_LONGITUDE );

    int formatTruncationCheck = snprintf( wx.callsign, 10, "K6LOT-13" );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.windDirection, 4, "%03d", (int)(round(frame->windDirection)) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.windSpeed, 4, "%03d", (int)(round(ms2mph(frame->windSpeedMs))) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.gust, 4, "%03d", (int)(round(ms2mph(frame->windGustMs))) );
    assert( formatTruncationCheck >= 0 );

    formatTruncationCheck = snprintf( wx.temperature, 4, "%03d", (int)(round(c2f(frame->tempC))) );
    assert( formatTruncationCheck >= 0 );

    unsigned short int h = frame->humidity;
    // APRS only supports values 1-100. Round 0% up to 1%.
    if( h == 0 )
        h = 1;

    // APRS requires us to encode 100% as "00".
    else if( h >= 100 )
        h = 0;

    formatTruncationCheck = snprintf( wx.humidity, 3, "%.2d", h );
    assert( formatTruncationCheck >= 0 );

    // we are converting back from InHg because that's the offset we know based on airport data! (this means we go from millibars -> InHg + offset -> millibars)
    formatTruncationCheck = snprintf( wx.pressure, 6, "%.5d", (int)(round(inHg2millibars((frame->pressure * millibar2inchHg) + s_localOffsetInHg) * 10)) );
    assert( formatTruncationCheck >= 0 );

    memset( packetToSend, 0, sizeof( packetToSend ) );
    printAPRSPacket( &wx, packetToSend, packetFormat, 0, false );
    
    // add a comment with our software version info
    strcat( packetToSend, PROGRAM_NAME );
    strcat( packetToSend, VERSION );
    if( s_debug )
        printf( "%s\n\n", packetToSend );

    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
    wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
    wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
}



void transmit_air_data( const Frame* frame )
{
    char packetToSend[BUFSIZE];

    if( s_debug )
    {
        printTime( false );
        printf( " Sending air quality info to APRS-IS...\n" );
        printf( "3um: %03d, 5um: %03d, 10um: %03d, 25um: %03d, 50um: %03d\n", frame->particles_03um, frame->particles_05um, frame->particles_10um, frame->particles_25um, frame->particles_50um );
    }
    
    if( s_sequence_num >= 999 )
        s_sequence_num = 0;
    
    // we need to see if we ever sent the parameters, units and equations...
    if( timeGetTimeSec() > s_lastParamsTime + kParamsInterval )
    {
        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :PARM.0.3um,0.5um,1.0um,2.5um,5.0um", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
        sleep( 1 ); // avoid rate limiting
        
        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :UNIT.pm/0.1L,pm/0.1L,pm/0.1L,pm/0.1L,pm/0.1L", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
        sleep( 1 ); // avoid rate limiting

        sprintf( packetToSend, "%s>APRS,TCPIP*::%s :EQNS.0,256,0,0,256,0,0,256,0,0,256,0,0,256,0", kCallSign, kCallSign );
        if( s_debug )
            printf( "%s\n", packetToSend );
        wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
        wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
        sleep( 1 ); // avoid rate limiting

        s_lastParamsTime = timeGetTimeSec();
    }
    
    sprintf( packetToSend, "%s>APRS,TCPIP*:T#%03d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%d%d%d%d%d%d%d%d", kCallSign, s_sequence_num++,
              frame->particles_03um / 256.0,
              frame->particles_05um / 256.0,
              frame->particles_10um / 256.0,
              frame->particles_25um / 256.0,
              frame->particles_50um / 256.0,
              1,0,1,0,1,0,1,0 );

    if( s_debug )
        printf( "%s\n\n", packetToSend );

    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
    wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
    wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
    
    if( s_seqFilePath )
    {
        s_seqFile = fopen( s_seqFilePath, "wb" );
        if( !s_seqFile )
            log_error( "  failed to open sequence file: %s\n", s_seqFilePath );
        if( s_seqFile )
        {
            fwrite( &s_sequence_num, sizeof( uint16_t ), 1, s_seqFile );
            fclose( s_seqFile );
        }
    }
}


void transmit_status( const Frame* frame )
{
    char packetToSend[BUFSIZE];

    if( s_debug )
    {
        printTime( false );
        printf( " Sending status to APRS-IS...\n" );
        printf( "wx-relay case temp: %0.2f°F\n", c2f( frame->intTempC - s_localTempErrorC ) );
    }
    
    time_t     t   = time( NULL );
    struct tm* now = gmtime(&t);  // APRS uses GMT
    sprintf( packetToSend, "%s>APRS,TCPIP*:>%.2d%.2d%.2dzwx-relay temp %0.2fF", kCallSign, now->tm_mday, now->tm_hour, now->tm_min, c2f( frame->intTempC - s_localTempErrorC ) );
    if( s_debug )
        printf( "%s\n\n", packetToSend );

    // we need to create copies of the packet buffer and send that instead as we don't know the life of those other threads we light off...
    wx_create_thread_detached( sendPacket_thread_entry, copy_string( packetToSend ) );
    wx_create_thread_detached( sendToRadio_thread_entry, copy_string( packetToSend ) );
}


#pragma mark -


int sendToRadio( const char* p )
{
    int result = 0;
    
    if( s_test_mode )
        return result;
    
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
      log_error( "invalid channel %d - must be in range 0 to 15.\n", chan );
      chan = 0;
    }
    if( cmd < 0 || cmd > 15 ) {
      log_error( "invalid command %d - must be in range 0 to 15.\n", cmd );
      cmd = 0;
    }
    if( dlen < 0 || dlen > (int)(sizeof( temp ) - 1) ) {
      log_error( "invalid data length %d - must be in range 0 to %d.\n", dlen, (int)(sizeof( temp ) - 1) );
      dlen = sizeof( temp ) - 1;
    }

    temp[0] = (chan << 4) | cmd;
    memcpy( temp + 1, data, dlen );

    klen = kiss_encapsulate( temp, dlen + 1, kissed );
    
    // connect to direwolf and send data
    int server_sock = connectToDireWolf();
    if( server_sock < 0 )
    {
        log_error( "can't connect to direwolf...\n" );
        err = -1;
        goto exit_gracefully;
    }
    
    ssize_t rc = send( server_sock, (char*)kissed, klen, 0 );
    if( rc != klen )
    {
        log_error( "error writing KISS frame to socket.\n" );
        err = -1;
    }

exit_gracefully:
    shutdown( server_sock, 2 );
    close( server_sock );
    return err;
}


// returns fd to use to communicate with
int connectToDireWolf( void )
{
    int              error              = 0;
    char             foundValidServerIP = 0;
    struct addrinfo* result             = NULL;
    struct addrinfo* results;
    int              socket_desc        = -1;

    error = getaddrinfo( s_kiss_server, NULL, NULL, &results );
    if( error != 0 )
    {
        if( error == EAI_SYSTEM )
        {
            log_unix_error( "connectToDireWolf:getaddrinfo: " );
        }
        else
        {
            log_error( "error in getaddrinfo: %s\n", s_kiss_server );
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
                ((struct sockaddr_in*)addressinfo)->sin_port   = htons( s_kiss_port );
                break;
            case AF_INET6:
                ((struct sockaddr_in6*)addressinfo)->sin6_port = htons( s_kiss_port );
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
            close( socket_desc );
        }
    }
    freeaddrinfo( results );
    if( foundValidServerIP == 0 )
    {
        log_error( "connectToDireWolf: could not connect to the server.\n" );
        if( error )
            return error;
        else
            return -1;
    }

    return socket_desc;
}

#pragma mark -

bool wxlog_startup( void )
{
    size_t wxlog_size = sizeof( wxrecord ) * kMaxNumberOfRecords;

    if( !s_wxlogFilePath )
    {
        log_error( " wxlog_startup: don't have a wx history file to save our statistics to.\n" );
        if( !s_wxlog )
            s_wxlog = malloc( wxlog_size );
        
        if( !s_wxlog )
        {
            log_error( " failed to allocate memory for wx log\n" );
            return false;
        }
        s_wx_size_secs = 0;
        s_wx_count = 0;
        return true;
    }
    
    s_wxlogFile = fopen( s_wxlogFilePath, "rb" );
    if( s_wxlogFile )
    {
        // count number of items (look at file size / record size)
        wxrecord first;
        wxrecord last;

        size_t recs_read = fread( &first, sizeof( wxrecord ), 1, s_wxlogFile );
        if( recs_read == 1 )
        {
            fseek( s_wxlogFile, 0, SEEK_END );
            long file_size = ftell( s_wxlogFile );
            s_wx_count = (file_size / sizeof( wxrecord ));
            
            fseek( s_wxlogFile, file_size - sizeof( wxrecord ), SEEK_SET );
            recs_read = fread( &last, sizeof( wxrecord ), 1, s_wxlogFile );

            // look at first and last entry to determine window size and start time
            if( recs_read == 1 )
            {
#ifdef TIME_OUT_OLD_DATA
                // if the start time is too far away from right NOW, dump file and start fresh...
                if( (timeGetTimeSec() - first.timeStampSecs) > kStartingTimeThreshold )
                    s_wx_size_secs = 0;
                else
                    s_wx_size_secs = first.timeStampSecs - last.timeStampSecs;
#else
                s_wx_size_secs = first.timeStampSecs - last.timeStampSecs;
#endif
                
                // now read in all the data
                if( s_wx_size_secs )
                {
                    // alloc memory for entire thing -- data comes in at about 5 second intervals if we are lucky.
                    // so for 10 minutes (600 seconds) we need 600 / 5 sec = 120 wxrecords minimum.  Let's round up to 200.
                    s_wxlog = malloc( wxlog_size );

                    if( s_wx_count < kMaxNumberOfRecords )
                    {
                        if( s_wxlog )
                        {
                            rewind( s_wxlogFile );
                            recs_read = fread( s_wxlog, sizeof( wxrecord ), s_wx_count, s_wxlogFile );
                            if( recs_read != s_wx_count )
                            {
                                log_error( " failed to read wx log. needed: %ld, got: %ld\n", s_wx_count, recs_read );
                                s_wx_size_secs = 0;
                                s_wx_count = 0;
                            }
                        }
                        else
                        {
                            log_error( " failed to allocate memory for wx log\n" );
                            s_wx_size_secs = 0;
                            s_wx_count = 0;
                        }
                    }
                    else
                    {
                        log_error( " wx log is bigger than our buffer! truncation not implemented...\n" );
                        s_wx_size_secs = 0;
                        s_wx_count = 0;
                    }
                }
            }
        }
        else
        {
            // there isn't even one record in this file...
            log_error( " wx log has no records!\n" );
            s_wx_count = 0;
            s_wx_size_secs = 0;
        }
        
        // close file
        fclose( s_wxlogFile );
        s_wxlogFile = NULL;
    }
    
    // make sure that we have our buffer allocated
    if( !s_wxlog )
        s_wxlog = malloc( wxlog_size );
    
    if( !s_wxlog )
    {
        log_error( " failed to allocate memory for wx log\n" );
        return false;
    }

    return true;
}


bool wxlog_shutdown( void )
{
    if( !s_wxlogFilePath || !s_wxlog || !s_wx_count )
        return false;

    size_t wxlog_size = sizeof( wxrecord ) * s_wx_count;
    
    s_wxlogFile = fopen( s_wxlogFilePath, "wb" );
    if( !s_wxlogFile )
        return false;

    size_t recs_out = fwrite( s_wxlog, sizeof( wxrecord ), s_wx_count, s_wxlogFile );
    if( recs_out != s_wx_count )
        log_error( " failed to write wx log. wrote: %ld, total: %ld\n", recs_out, wxlog_size / sizeof( wxrecord ) );
    
    fclose( s_wxlogFile );
    s_wxlogFile = NULL;
    
    free( s_wxlog );
    s_wxlog = NULL;
    return true;
}


bool wxlog_frame( const Frame* wxFrame )
{
    if( !wxFrame || !s_wxlog )
        return false;

    // add entry
    wxrecord wx = { .timeStampSecs = timeGetTimeSec(), .frame = *wxFrame };
    
    // see if there's room to move stuff without truncating
    if( s_wx_count )
    {
        if( s_wx_count < kMaxNumberOfRecords - 1 )
        {
            // copy only what we have, which will be faster than copying the entire buffer every add
            memmove( &s_wxlog[1], &s_wxlog[0], s_wx_count * sizeof( wxrecord ) );
        }
        else
        {
            // the buffer is full so we need to move a shortened version of the buffer (which effectively truncates it)
            memmove( &s_wxlog[1], &s_wxlog[0], (s_wx_count - 1) * sizeof( wxrecord ) );
        }
    }
    
    // stop buffering if we have a big enough window, this can grow but never shrink
    if( (s_wx_count < kMaxNumberOfRecords - 1) && (s_wx_size_secs < kLongestInterval) )
        ++s_wx_count;

    // put the actual data in there now
    memcpy( s_wxlog, &wx, sizeof( wxrecord ) );

    s_wx_size_secs = timeGetTimeSec() - s_wxlog[s_wx_count - 1].timeStampSecs;
    
#ifdef TRACE_INSERTS
    printTime( false );
    printf( " -> record[%03zu]: window: %3ld secs, ", s_wx_count, s_wx_size_secs );
    printCurrentWeather( &s_wxlog->frame, false );
#endif
    
    return true;
}


bool wxlog_get_wx_averages( Frame* wxFrame )
{
    if( !wxFrame )
        return false;
    
    if( s_wx_size_secs < kLongestInterval )
        return false;
    
    time_t current = timeGetTimeSec();
    memset( wxFrame, 0, sizeof( Frame ) );
    
    // these counts are used to derive the averages
    size_t tempCount     = 0;
    size_t intTempCount  = 0;
    size_t windCount     = 0;
    size_t humidityCount = 0;
    size_t airCount      = 0;
    
    // accumulator for humidity
    uint32_t humidity = 0;

    // accumulators for air counts
    uint32_t pm10_standard   = 0;
    uint32_t pm25_standard   = 0;
    uint32_t pm100_standard  = 0;
    uint32_t pm10_env        = 0;
    uint32_t pm25_env        = 0;
    uint32_t pm100_env       = 0;
    uint32_t particles_03um  = 0;
    uint32_t particles_05um  = 0;
    uint32_t particles_10um  = 0;
    uint32_t particles_25um  = 0;
    uint32_t particles_50um  = 0;
    uint32_t particles_100um = 0;
    
    // start with really high pressure as we want the low of the period
    wxFrame->pressure = 2000; // that'll crush you
    
    // go thru all averages, min and max- records are in order of newest to oldest
    for( int i = 0; i < s_wx_count; i++ )
    {
        time_t timeIndexSecs = current - s_wxlog[i].timeStampSecs;  // timestamps go in decreasing order
        
        // different data has different averaging windows or min/max windows - accumulate directly inside the wxFrame (except for air stuff which might not fit in 16 bits while accumulating)
        if( timeIndexSecs < kTempPeriod )
        {
            wxFrame->tempC += s_wxlog[i].frame.tempC;
            ++tempCount;
        }

        if( timeIndexSecs < kIntTempPeriod )
        {
            wxFrame->intTempC += s_wxlog[i].frame.intTempC;
            ++intTempCount;
        }

        if( timeIndexSecs < kHumiPeriod )
        {
            humidity += s_wxlog[i].frame.humidity;
            ++humidityCount;
        }

        if( timeIndexSecs < kWindPeriod )
        {
            wxFrame->windDirection += s_wxlog[i].frame.windDirection;
            wxFrame->windSpeedMs   += s_wxlog[i].frame.windSpeedMs;
            ++windCount;
        }

        if( timeIndexSecs < kAirPeriod )
        {
            pm10_standard  += s_wxlog[i].frame.pm10_standard;
            pm25_standard  += s_wxlog[i].frame.pm25_standard;
            pm100_standard += s_wxlog[i].frame.pm100_standard;

            pm10_env  += s_wxlog[i].frame.pm10_env;
            pm25_env  += s_wxlog[i].frame.pm25_env;
            pm100_env += s_wxlog[i].frame.pm100_env;

            particles_03um  += s_wxlog[i].frame.particles_03um;
            particles_05um  += s_wxlog[i].frame.particles_05um;
            particles_10um  += s_wxlog[i].frame.particles_10um;
            particles_25um  += s_wxlog[i].frame.particles_25um;
            particles_50um  += s_wxlog[i].frame.particles_50um;
            particles_100um += s_wxlog[i].frame.particles_100um;
            ++airCount;
        }
        
        // these two don't have counts because they only do min/max
        if( timeIndexSecs < kGustPeriod )
            wxFrame->windGustMs = fmax( s_wxlog[i].frame.windGustMs, wxFrame->windGustMs );

        if( timeIndexSecs < kBaroPeriod )
            wxFrame->pressure = fmin( s_wxlog[i].frame.pressure, wxFrame->pressure );
    }

#ifdef TRACE_AVERAGES
    struct tm tm = *localtime( &current );
    printf( "%d-%02d-%02d %02d:%02d:%02d: counts: t:%zu, i:%zu, h:%zu, w:%zu, a:%zu -> ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, tempCount, intTempCount, humidityCount, windCount, airCount );
#endif
    
    // guard against divide by zero and also ensure we have an instantaneous value rather than zero if no average
    if( !airCount )
        airCount = 1;
    if( !tempCount )
        tempCount = 1;
    if( !intTempCount )
        intTempCount = 1;
    if( !humidityCount )
        humidityCount = 1;
    if( !windCount )
        windCount = 1;
    
    // take means
    wxFrame->tempC         /= tempCount;
    wxFrame->intTempC      /= intTempCount;
    wxFrame->windDirection /= windCount;
    wxFrame->windSpeedMs   /= windCount;
    wxFrame->humidity       = humidity / humidityCount;

    wxFrame->pm10_standard   = pm10_standard  / airCount;
    wxFrame->pm25_standard   = pm25_standard  / airCount;
    wxFrame->pm100_standard  = pm100_standard / airCount;
    wxFrame->pm10_env        = pm10_env       / airCount;
    wxFrame->pm25_env        = pm25_env       / airCount;
    wxFrame->pm100_env       = pm100_env      / airCount;
    wxFrame->particles_03um  = particles_03um / airCount;
    wxFrame->particles_05um  = particles_05um / airCount;
    wxFrame->particles_10um  = particles_10um / airCount;
    wxFrame->particles_25um  = particles_25um / airCount;
    wxFrame->particles_50um  = particles_50um / airCount;
    wxFrame->particles_100um = particles_100um / airCount;

#ifdef TRACE_AVERAGES
    printCurrentWeather( wxFrame, false );
#endif
    return true;
}

