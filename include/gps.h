#ifndef _gps_h_
#define _gps_h_


#define MAX_LOC_SIZE 20
#define MAX_GPS_QUEUE_SIZE 20
#define MAX_GPS_STRING_SIZE 256
#define MAX_GPS_PACKET_SIZE 256
#define ASCII_0 0x30
#define ASCII_1 0x31
#define ASCII_2 0x32
#define ASCII_9 0x39
#define UTC_KOREA_TIME 9
#define GPS_THREAD_IDLE_DELAY 100000
#define GPS_START_CONDITION '$'
#define GPS_END_CONDITION 0x0A

#define GPS_PORT "/dev/ttyAMA0"


#define MAX_GPS_NOT_RECEIVED_COUNT 50

#define error_message printf

struct sDate
{
	int dd;
	int mm;
	int yy;
};

struct sTime
{
	int hh;
	int mm;
	int ss;
};

struct sLocation
{
	char lat[MAX_LOC_SIZE];
	char lon[MAX_LOC_SIZE];
};

void init_gps();
struct sLocation getLocation();
struct sTime getTime();
struct sDate getDate();
int getGpsFix();
void finish_gps();
int getGpsStatus();
#endif
