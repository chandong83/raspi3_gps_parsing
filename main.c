#include <stdio.h>
#include <unistd.h>
#include "gps.h"

int main(int argc, char *argv[])
{
    struct sTime tm;
    init_gps();
    while(1)
    {
     tm = getTime();
     printf("hh : %d, mm : %d, ss : %d, fixed : %d\n", tm.hh, tm.mm, tm.ss, getGpsFix());
     sleep(1);
    }
    
    finish_gps();
    return 1;
}
