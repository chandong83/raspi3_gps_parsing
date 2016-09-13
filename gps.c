#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <termio.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

#include "gps.h"

pthread_mutex_t gpsloc_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gpstime_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gpsdate_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gpsfix_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gpsstatus_mutex = PTHREAD_MUTEX_INITIALIZER;

int bFirstFix=0;
int bGPSStat=0;
//
// gps thread finish
//
int gps_finish = 0;

//
// gps data
//
struct sTime gpstime;
struct sDate gpsdate;
struct sLocation gpsloc;
int gpsfix=0;

//
// queue
//
char gps_queue[MAX_GPS_QUEUE_SIZE][MAX_GPS_PACKET_SIZE];
int gps_front = -1;
int gps_rear  = -1;

//
// gps tok
//
char tokdata[256];
char str_ret[256];
char *tok_cur;

//
//
// gps queue
//
//
void init_gps_queue()
{
  gps_front = gps_rear = 0;
}

void clear_gps_queue()
{
  gps_front = gps_rear;
}

int gps_put(char* buf)
{
  if((gps_rear+1) % MAX_GPS_QUEUE_SIZE == gps_front)
  {
    printf("queue overflow!\n");
    return -1;
  }
  memcpy(gps_queue[gps_rear],buf,MAX_GPS_PACKET_SIZE);

  gps_rear++;
  gps_rear %= MAX_GPS_QUEUE_SIZE;
  return 1;
}

int gps_get(char* buf)
{
  if(gps_front == gps_rear)
  {
    //printf("queue unflow\n");
    return -1;
  }
  memcpy(buf, gps_queue[gps_front],MAX_GPS_PACKET_SIZE);
  gps_front++;
  gps_front %= MAX_GPS_QUEUE_SIZE;
  return 1;
}

//
//
// gps port
//
//
int set_interface_attribs (int *fd, int speed, int parity)
{
  struct termios tty;

  bzero(&tty, sizeof(tty));

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;     // 8-bit chars
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_cflag |= parity;

  if (tcsetattr (*fd, TCSANOW, &tty) != 0)
  {
    error_message ("error %d from tcsetattr", errno);
    return -1;
  }

  tcflush(*fd,TCIOFLUSH);
  return 0;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    error_message ("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    error_message ("error %d setting term attributes", errno);
}




//
//
// gps tok
//
//
char *gpstok(char *str)
{
	int i=0;
	int len=0;
	if(str == NULL)
	{
		len = strlen(tok_cur);
		for(i=0;i<len;i++)
		{
			if(*tok_cur==',')
			{
				tok_cur++;
				break;
			}
			str_ret[i] = *tok_cur;
			tok_cur++;
		}
	}
	else
	{
		memcpy(tokdata, str, strlen(str));
		tok_cur = tokdata;
		for(i=0;i<strlen(tokdata);i++)
		{
			if(*tok_cur==',')
			{
				tok_cur++;
				break;
			}
			str_ret[i] = *tok_cur;
			tok_cur++;
		}
	}
	str_ret[i]=0;
	return str_ret;
}


// char to number
int Char2Number(char ch)
{
	int ret=-1;
	if((ch>=ASCII_0)&&(ch<=ASCII_9))
	{
		ret = (int)ch-ASCII_0;
	}
	return ret;
}


//string to time
int str2time(char *str, int *hour, int *min, int *sec)
{
	int len = strlen(str);
	int tmp;
	int tmp2;

	if(len>0)
	{
		tmp = Char2Number(str[0]);
		tmp2 = Char2Number(str[1]);
		if((tmp<0)&&(tmp2<0))
			return -1;
		*hour	= tmp*10+tmp2;
		tmp = Char2Number(str[2]);
		tmp2 = Char2Number(str[3]);
		if((tmp<0)&&(tmp2<0))
			return -1;
		*min	= tmp*10+tmp2;

		tmp = Char2Number(str[4]);
		tmp2 = Char2Number(str[5]);
		if((tmp<0)&&(tmp2<0))
			return -1;
		*sec	= tmp*10+tmp2;

		return 1;
	}
	else
		return -1;
}

//
// string to date
//
int str2date(char *str, int *year, int *month, int *day)
{
    int len = strlen(str);
    int tmp;
    int tmp2;

    if(len>0)
    {
        tmp = Char2Number(str[0]);
        tmp2 = Char2Number(str[1]);
        if((tmp<0)&&(tmp2<0))
            return -1;
        *day	= tmp*10+tmp2;

        tmp = Char2Number(str[2]);
        tmp2 = Char2Number(str[3]);
        if((tmp<0)&&(tmp2<0))
            return -1;
        *month	= tmp*10+tmp2;

        tmp = Char2Number(str[4]);
        tmp2 = Char2Number(str[5]);
        if((tmp<0)&&(tmp2<0))
            return -1;
        *year	= tmp*10+tmp2+2000;

        return 1;
    }
    else
        return -1;
}


// string to double
double stof(char *str)
{
  int i=0;
  int isPt=0;
  int index=0;
  double point=0;
  double inti=0;

  for(;i<strlen(str);i++){
      if((str[i] >= '0')&&(str[i] <= '9')){ //number
        if(isPt==0){
          inti*=10;
          inti+=(str[i]-0x30);
        }else{
          point += ((str[i]-0x30)/(pow(10,index)));
          index++;
        }
      }else if(str[i] == '.'){ //dot
        isPt = 1;
        index = 1;
      }else{

      }
  }
//  printf("%f %f %f\n",inti, point, inti+point);
  return (inti+point);
}

//change the longitude unit
void unit_change_lon(char* lon)
{
  int temp2=0;
  double temp=0;
  double temp3=0;
  temp = stof(lon);;
  temp2 = temp;
  temp2 = (temp2/100)*100;
  temp3 = (temp-temp2);
  temp3 = temp3/60;
  sprintf(lon, "%f",((temp2/100)+temp3));
}


//change the latitude unit
void unit_change_lat(char* lat)
{
  int temp2=0;
  double temp=0;
  double temp3=0;
  temp = stof(lat);
  temp2 = temp;
  temp2 = (temp2/100)*100;
  temp3 = (temp-temp2);
  temp3 = temp3/60;
  sprintf(lat, "%f",((temp2/100)+temp3));
}

//
//
// gps data
//
//
void setGpsFix(int fix)
{
	pthread_mutex_lock(&gpsfix_mutex);
	gpsfix = fix;
	pthread_mutex_unlock(&gpsfix_mutex);
}

int getGpsFix()
{
	int ret=0;
	pthread_mutex_lock(&gpsfix_mutex);
	ret = gpsfix;
	pthread_mutex_unlock(&gpsfix_mutex);
	return ret;
}

int setDate(struct sDate sD)
{
    pthread_mutex_lock(&gpsdate_mutex);
    gpsdate.yy = sD.yy;
    gpsdate.mm = sD.mm;
    gpsdate.dd = sD.dd;
    pthread_mutex_unlock(&gpsdate_mutex);
    return getGpsFix();
}

struct sDate getDate()
{
    struct sDate sD;
    pthread_mutex_lock(&gpsdate_mutex);
    sD.yy = gpsdate.yy;
    sD.mm = gpsdate.mm;
    sD.dd = gpsdate.dd;
    pthread_mutex_unlock(&gpsdate_mutex);
    return sD;
}

int setTime(struct sTime sT)
{
	pthread_mutex_lock(&gpstime_mutex);
	gpstime.hh = sT.hh;
	gpstime.mm = sT.mm;
	gpstime.ss = sT.ss;
	pthread_mutex_unlock(&gpstime_mutex);
  return getGpsFix();
}

struct sTime getTime()
{
	struct sTime sT;
	pthread_mutex_lock(&gpstime_mutex);
	sT.hh = gpstime.hh;
	sT.mm = gpstime.mm;
	sT.ss = gpstime.ss;
	pthread_mutex_unlock(&gpstime_mutex);
	return sT;
}

int setLocation(struct sLocation loc)
{
	pthread_mutex_lock(&gpsloc_mutex);
	strcpy(gpsloc.lat,loc.lat);
	strcpy(gpsloc.lon,loc.lon);
	pthread_mutex_unlock(&gpsloc_mutex);
  return getGpsFix();
}

struct sLocation getLocation()
{
	struct sLocation sL;
	pthread_mutex_lock(&gpsloc_mutex);
	strcpy(sL.lat,gpsloc.lat);
	strcpy(sL.lon,gpsloc.lon);
	pthread_mutex_unlock(&gpsloc_mutex);
	return sL;
}




//
//
// gps parsing
//
//
void GPRMC_Parsing(char* packet)
{
  char *tok;
  int bValid_data=0;
//  struct sLocation sLoc;
  struct sTime sT;
  struct sDate sD;
  tok = gpstok(packet);
  tok = gpstok(NULL);//utc hhmmss.ss
  //printf("time : %s\n",tok);
  if(str2time(tok,&sT.hh,&sT.mm,&sT.ss)>0)
  {
    //printf("K Time : %d:%d:%d\n",sT.hh+UTC_KOREA_TIME,sT.mm,sT.ss);
    //sT.hh+=UTC_KOREA_TIME;
    //setTime(sT);
  }
  tok = gpstok(NULL);//state
  //printf("Status : %s\n",tok);
  if((tok[0] == 'A') || (tok[0] == 'V'))
  {
    bValid_data=1;
  }
  tok = gpstok(NULL);//latitude
  //printf("lat : %s\n",tok);
  tok = gpstok(NULL);
  tok = gpstok(NULL);//longitude
  //printf("long : %s\n",tok);
  tok = gpstok(NULL);
  tok = gpstok(NULL);//speed
  tok = gpstok(NULL);//trck
  tok = gpstok(NULL);//date, ddmmyy
  if(str2date(tok,&sD.yy,&sD.mm,&sD.dd)>0){
    if(bValid_data){
      setDate(sD);
      setTime(sT);
      if(bFirstFix==0){
          bFirstFix=1;

          printf("%d-%d-%d %d:%d:%d\n",sD.yy, sD.mm, sD.dd, sT.hh, sT.mm, sT.ss);
          printf("#####Gps is Fixed\n");
          //Send_RTC(sD.yy, sD.mm, sD.dd, sT.hh, sT.mm, sT.ss);
          //set_system_time(sD.yy, sD.mm, sD.dd, sT.hh, sT.mm, sT.ss);
      }
    }
  }
}

void GPGGA_Parsing(char *packet)
{
  char *tok;
  struct sLocation sLoc;
  struct sTime sT;

  tok = gpstok(packet);
  tok = gpstok(NULL);//utc hhmmss.ss
  if(str2time(tok,&sT.hh,&sT.mm,&sT.ss)<0)
  {
  }
  tok = gpstok(NULL);//latitude
  strcpy(sLoc.lat,tok);

  tok = gpstok(NULL);
  tok = gpstok(NULL);//longitude
  strcpy(sLoc.lon,tok);

  tok = gpstok(NULL);
  tok = gpstok(NULL);//status
  if((tok[0] == ASCII_1)||(tok[0] == ASCII_2)) //1 or 2 means fixed
  {
    setGpsFix(1);
  }
  else
  {
    setGpsFix(0);
  }

  if(getGpsFix())
  {
    unit_change_lon(sLoc.lon);
    unit_change_lat(sLoc.lat);
    setLocation(sLoc);
  }
}


void GPGSA_Parsing(char *packet)
{
/*
  char *tok;
  tok = gpstok(packet);
  tok = gpstok(NULL);//Auto Selection of 2D or 3D fix (M = manual)
  tok = gpstok(NULL);//1 = no fix, 2 = 2D fix, 3 = 3D fix
*/
}


void run_gpsparsing()
{
	char packet[MAX_GPS_PACKET_SIZE];
	while(!gps_finish)
	{
		if(gps_get(packet)>0)
		{
            //printf("%s\n",packet);
			if(!strncmp(packet,"$GPRMC",6))
			{
                //printf("%s\n",packet);
                GPRMC_Parsing(packet);
			}
			else if(!strncmp(packet,"$GPGGA",6))
			{
		        GPGGA_Parsing(packet);
			}
	/*		else if(!strncmp(packet,"$GPGSA",6))
			{
		    GPGSA_Parsing(packet);
			}*/
		}
		usleep(GPS_THREAD_IDLE_DELAY);
	}
}

void setGpsStatus(int stat)
{
  pthread_mutex_lock(&gpsstatus_mutex);
  bGPSStat = stat;
  pthread_mutex_unlock(&gpsstatus_mutex);
}

int getGpsStatus()
{
  int ret;
  pthread_mutex_lock(&gpsstatus_mutex);
  ret = bGPSStat;
  pthread_mutex_unlock(&gpsstatus_mutex);
  return ret;
}

void gps_handle()
{
    pthread_t tGpsParsingId=0;
    char *portname = GPS_PORT;
    char *rx;
    int iores, iocount;
    int i=0;
    int nGpsNotReceivedCnt=0;
    int bGpsNotReceived=0xFF;
    //rcount = 0;
    char gpsbuf[MAX_GPS_PACKET_SIZE];
    int gpsindex=0;
    int fd = open (portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
        return;
    }
    printf("portname : %s\n", GPS_PORT);
    init_gps_queue();
    pthread_create(&tGpsParsingId, NULL,(void *)&run_gpsparsing, NULL);


    struct termios tty;

    bzero(&tty, sizeof(tty));

    cfsetospeed (&tty, B9600);
    cfsetispeed (&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout



    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
      error_message ("error %d from tcsetattr", errno);
      return ;
    }

    tcflush(fd,TCIOFLUSH);

//    set_blocking (fd, 1);                // set no blocking
    															 // receive 25:  approx 100 uS per char transmit
    printf("start gps \n");
    while(!gps_finish) {
        iocount = 0;
        iores = ioctl(fd, FIONREAD, &iocount);
        if(!iocount)
        {
            nGpsNotReceivedCnt++;
            //printf("Gps not receive count %d\n", nGpsNotReceivedCnt);
            if(nGpsNotReceivedCnt > MAX_GPS_NOT_RECEIVED_COUNT){
                if(bGpsNotReceived!=1){
                    printf("gps status is Disconnected\n");
                    setGpsStatus(0);
                    setGpsFix(0);
                    bGpsNotReceived=1;
                }
                nGpsNotReceivedCnt = MAX_GPS_NOT_RECEIVED_COUNT;
            }
            usleep(GPS_THREAD_IDLE_DELAY);
            continue;
        }
        nGpsNotReceivedCnt=0;
        if(bGpsNotReceived!=0){
            printf("gps status is connected\n");
            setGpsStatus(1);
            bGpsNotReceived=0;
        }

        rx = malloc(iocount);
        iores = read(fd, rx, iocount);

        for(i=0;i<iores;i++)
        {
            if(rx[i] == GPS_START_CONDITION)
            {
                memset(gpsbuf,0,MAX_GPS_PACKET_SIZE);
                gpsindex=0;
                gpsbuf[gpsindex++]=rx[i];
            }
            else if(rx[i] == GPS_END_CONDITION) //end of packet
            {
                if(gpsindex >= MAX_GPS_PACKET_SIZE)
                {
                    gpsindex=0;
                }
                else if(gpsindex != 0)
                {
                    gpsbuf[gpsindex++]=rx[i];
                    gps_put(gpsbuf);
                    gpsindex=0;
                }
            }
            else
            {
                if(gpsindex >= MAX_GPS_PACKET_SIZE)
                {
                    gpsindex=0;
                }
            gpsbuf[gpsindex++]=rx[i];
            }
        }
        free(rx);
        usleep(GPS_THREAD_IDLE_DELAY);
    }

}

void init_gps()
{
  pthread_t tGpsId;
  pthread_create(&tGpsId, NULL,(void *)&gps_handle, NULL);
}


void finish_gps()
{
  gps_finish=1;
}
