/* **************************************************************************************
 * All Rights Reserved
 * Solen Liu
 * **************************************************************************************
 * Name : uart_test
 *
 * Description : HAL Level Exe Application, Operate /dev/tty* Device to Write/Read 
 *               Already Verified on Qcom SDM670/845 Platform 
 *               In default, Serial Port Transmit the Local timestamp and Set the GPIO to High 
 *               Per Seconds.
 *               It's Designed to Test synchronization performance 
 *
 * Author : solen.liu <solen.pico@gmail.com>
 *
 * Date : 2019/02/27
 *
 * ***************************************************************************************/ 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>
#include <linux/time.h>
#include <signal.h>

//********EXPORT GPIO Port********//
#define SYSFS_GPIO_EXPORT       "/sys/class/gpio/export"  
#define SYSFS_GPIO_PIN_VAL      "134"   
#define SYSFS_GPIO_DIR          "/sys/class/gpio/gpio134/direction"
#define SYSFS_GPIO_DIR_VAL      "out"  
#define SYSFS_GPIO_VAL          "/sys/class/gpio/gpio134/value"
#define SYSFS_GPIO_VAL_H        "1"
#define SYSFS_GPIO_VAL_L        "0"

#define TEST_GPIO_EXPORT       "/sys/class/gpio/export"  
#define TEST_GPIO_PIN_VAL      "125"   
#define TEST_GPIO_DIR          "/sys/class/gpio/gpio125/direction"
#define TEST_GPIO_DIR_VAL      "out"  
#define TEST_GPIO_VAL          "/sys/class/gpio/gpio125/value"
#define TEST_GPIO_VAL_H        "1"
#define TEST_GPIO_VAL_L        "0"


#define BAUDRATE        B115200
#define UART_DEVICE     "/dev/ttyGS0"

#define FALSE  -1
#define TRUE   0

//#define TRANSDATALEN   6
int fd;
int fd_gpio;
int fd_gpiotest;
int TRANSDATALEN = 64;
long tv_sec_0,tv_usec_0;  //Reference Time
char  writebuf[256] = {0};
int SYNC_PERIOD = 10;  //10 * 100 ms
/*----GPIO----*/
int init_gpio(void) //GPIO134
{
	/*-Init GPIO-*/
	fd_gpio = open(SYSFS_GPIO_EXPORT,O_WRONLY);
	if(fd_gpio == -1)
	{
		printf("cannot open SYSFS_GPIO_EXPORT!\n");
		return -1;
	}
	write(fd_gpio,SYSFS_GPIO_PIN_VAL,sizeof(SYSFS_GPIO_PIN_VAL));  //Export GPIO
	close(fd_gpio);
	
	fd_gpio = open(SYSFS_GPIO_DIR,O_WRONLY);
	if(fd_gpio == -1)
	{
		printf("cannot open SYSFS_GPIO_DIR! \n");
		return -1;
	}
	write(fd_gpio,SYSFS_GPIO_DIR_VAL,sizeof(SYSFS_GPIO_DIR_VAL));
	close(fd_gpio);
	
	fd_gpio = open(SYSFS_GPIO_VAL,O_WRONLY);
	if(fd_gpio == -1)
	{
		printf("cannot open SYSFS_GPIO_VAL \n");
		return -1;
	}
	return 0;
}
int init_gpiotest(void) //GPIO125
{	/*-Init GPIO-*/
	fd_gpiotest = open(TEST_GPIO_EXPORT,O_WRONLY);
	if(fd_gpiotest == -1)
	{
		printf("cannot open TEST_GPIO_EXPORT!\n");
		return -1;
	}
	write(fd_gpiotest,TEST_GPIO_PIN_VAL,sizeof(TEST_GPIO_PIN_VAL));  //Export GPIO
	close(fd_gpiotest);
	
	fd_gpiotest = open(TEST_GPIO_DIR,O_WRONLY);
	if(fd_gpiotest == -1)
	{
		printf("cannot open TEST_GPIO_DIR! \n");
		return -1;
	}
	write(fd_gpiotest,TEST_GPIO_DIR_VAL,sizeof(TEST_GPIO_DIR_VAL));
	close(fd_gpiotest);
	
	fd_gpiotest = open(TEST_GPIO_VAL,O_WRONLY);
	if(fd_gpiotest == -1)
	{
		printf("cannot open TEST_GPIO_VAL \n");
		return -1;
	}
	return 0;
}
void set_gpio_high(void)
{
	write(fd_gpio,SYSFS_GPIO_VAL_H,sizeof(SYSFS_GPIO_VAL_H)); //Low Level
}
void set_gpio_low(void)
{
	write(fd_gpio,SYSFS_GPIO_VAL_L,sizeof(SYSFS_GPIO_VAL_L)); //Low Level
}
void set_gpiotest_high(void)
{
	write(fd_gpiotest,TEST_GPIO_VAL_H,sizeof(TEST_GPIO_VAL_H)); //Low Level
}
void set_gpiotest_low(void)
{
	write(fd_gpiotest,TEST_GPIO_VAL_L,sizeof(TEST_GPIO_VAL_L)); //Low Level
}



///////////////////////////////////////////////////////////////////
/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*/
int speed_arr[] = {B1000000, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                   B1000000,B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {1000000,115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, 
                  1000000,115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed){
  int   i; 
  int   status; 
  struct termios   Opt;
  tcgetattr(fd, &Opt); 
  printf("Baudrate set to %d\n",speed);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) { 
    if  (speed == name_arr[i]) {     
      tcflush(fd, TCIOFLUSH);     
      cfsetispeed(&Opt, speed_arr[i]);  
      cfsetospeed(&Opt, speed_arr[i]);   
      status = tcsetattr(fd, TCSANOW, &Opt);  
      if  (status != 0) {        
        perror("tcsetattr fd1");  
        return;     
      }    
      tcflush(fd,TCIOFLUSH);   
    }  
  }
}
///////////////////////////////////////////////////////////////////
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{ 
    struct termios options; 
    if  ( tcgetattr( fd,&options)  !=  0) { 
        perror("SetupSerial 1");     
        return(FALSE);  
    }
    options.c_cflag &= ~CSIZE; 
    switch (databits) /*设置数据位数*/
    {   
    case 7:     
        options.c_cflag |= CS7; 
        break;
    case 8:     
        options.c_cflag |= CS8;
        break;   
    default:    
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);  
    }
    switch (parity) 
    {   
        case 'n':
        case 'N':    
            options.c_cflag &= ~PARENB;   /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
            break;  
        case 'o':   
        case 'O':     
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/  
            options.c_iflag |= INPCK;             /* Disnable parity checking */ 
            break;  
        case 'e':  
        case 'E':   
            options.c_cflag |= PARENB;     /* Enable parity */    
            options.c_cflag &= ~PARODD;   /* 转换为偶效验*/     
            options.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S': 
        case 's':  /*as no parity*/   
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;  
        default:   
            fprintf(stderr,"Unsupported parity\n");    
            return (FALSE);  
        }  
    /* 设置停止位*/  
    switch (stopbits)
    {   
        case 1:    
            options.c_cflag &= ~CSTOPB;  
            break;  
        case 2:    
            options.c_cflag |= CSTOPB;  
           break;
        default:    
             fprintf(stderr,"Unsupported stop bits\n");  
             return (FALSE); 
    } 
    /* Set input parity option */ 
    if (parity != 'n')   
        options.c_iflag |= INPCK; 
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 0; /* 设置超时15 seconds*/   
    options.c_cc[VMIN] = TRANSDATALEN; /* Update the options and do it NOW */
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    if (tcsetattr(fd,TCSANOW,&options) != 0)   
    { 
        perror("SetupSerial 3");   
        return (FALSE);  
    } 

    return (TRUE);  
}

/////
void delay_us(int second)
{
	struct timeval tvusc;
	tvusc.tv_usec = second;
	select(0,NULL,NULL,NULL,&tvusc);
}

////
void timer_handle(void)
{
	static int cnt = 0;
	struct timeval tv_tm;
	long ap_time;
	int res;
	if(cnt == 0)
	{
		gettimeofday(&tv_tm,NULL);
		set_gpio_high();
    	ap_time = (tv_tm.tv_sec - tv_sec_0)*1000000 + (tv_tm.tv_usec - tv_usec_0);
		writebuf[0] = 0xAA;
		writebuf[1] = ap_time;
		writebuf[2] = ap_time >> 8;
		writebuf[3] = ap_time >> 16;
		writebuf[4] = ap_time >> 24;
		writebuf[5] = ap_time >> 32;
		writebuf[6] = ap_time >> 40;
		writebuf[7] = ap_time >> 48;
		writebuf[8] = ap_time >> 56;
		res = write(fd, writebuf, TRANSDATALEN); //START PACKET
		if(res < 0)
		{
			printf("uart_test>>>solen>>> fatal error cannot write!\n");
		}
		printf("uart_test>>>solen>>> sync time = %ld\n",ap_time);
	}
	if(cnt == 1)
	{
		set_gpio_low();
	}
	
	cnt++;
	if(cnt == SYNC_PERIOD)
		cnt = 0;
}
void init_sigaction(void)
{
	struct sigaction act;
	act.sa_handler = timer_handle;
	act.sa_flags  = 0;
	sigemptyset(&act.sa_mask);
	sigaction(SIGALRM, &act, NULL);
	
	//signal(SIGALRM, signalHandler);
}
void init_time()
{
    struct itimerval val;          
    val.it_value.tv_sec = 0; 
    val.it_value.tv_usec = 100000; //10ms
    val.it_interval = val.it_value; 
    setitimer(ITIMER_REAL, &val, NULL);
}

///////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    int  res;
	int  cnt = 0,i;
	
	char  readbuf[256] = {0};
	long mcu_usec,ap_usec;
	struct timeval tv;
	
	printf("uart_test>>>solen>>> Start CDC timestamp test1!\n");
	/*Init GPIO to Low*/
	init_gpio();
	init_gpiotest();
	set_gpio_low();
	set_gpiotest_low();
	usleep(100000);
	TRANSDATALEN = 64;
	printf("uart_test>>>solen>>> TRANSDATALEN set to %d in default!\n",TRANSDATALEN);
	
	if(argc == 3)
	{
		SYNC_PERIOD = (*argv[2] - 0x30)*10 + *(argv[2]+1) - 0x30;
		printf("uart_test>>>solen>>> SYNC_PERIOD = %d ms\n",SYNC_PERIOD * 100);
	}
    /*Init UART*/
	fd = open(argv[1], O_RDWR);
    if (fd < 0) 
    {
		printf("uart_test>>>solen>>> CDC Open Fail, exit...\n");
        return -1;
    }
    printf("uart_test>>>solen>>> CDC Open Succefully\n");
    set_speed(fd,1000000);
    if (set_Parity(fd,8,1,'N') == FALSE)  
    {
        printf("uart_test>>>solen>>> set_Parity Fail, exit...\n");
        return -1;
    }
	
	/*Get Reference Time*/
	gettimeofday(&tv,NULL);
	set_gpio_high();
    tv_sec_0 = tv.tv_sec;
	tv_usec_0 = tv.tv_usec;


    /*Begin to Transfer*/
	writebuf[0] = 0xAA;
	writebuf[1] = 0; //LSB  uint64
	writebuf[2] = 0;
	writebuf[3] = 0;
	writebuf[4] = 0;
	writebuf[5] = 0;
	writebuf[6] = 0;
	writebuf[7] = 0;
	writebuf[8] = 0; //MSB
	res = write(fd, writebuf, TRANSDATALEN); //START PACKET
	if(res < 0)
	{
		printf("uart_test>>>solen>>> fatal error cannot write!\n");
	}
	usleep(100000);//high for 100ms
	set_gpio_low();
	init_sigaction();
	init_time(); 
	
    while(1)
    {

#if 0
		cnt++;
		res = read(fd,readbuf,TRANSDATALEN);//Recv Data 
		tcflush(fd,TCIFLUSH); //清空缓存
		printf("uart_test>>>solen>>>res = %d\n",res);
		set_gpio_low();
		gettimeofday(&tv,NULL);
		if(res < 0)
		{
			printf("uart_test>>>solen>>> fatal error cannot read!\n");
		}
		ap_usec = (tv.tv_sec - tv_sec_0)*1000000 + (tv.tv_usec - tv_usec_0);
		printf("uart_test>>>solen>>> recv mcu time:\n");
		for(i = 0; i < TRANSDATALEN-1; i++)
		{
			printf("%2x ",readbuf[i]);
		}
		printf("\n");
		
		if(readbuf[0] == 0xAA)
		{
			mcu_usec = readbuf[1]|(readbuf[2]<<8)|(readbuf[3]<<16)|((long)readbuf[4]<<24)|((long)readbuf[5]<<32)|((long)readbuf[6]<<40)|((long)readbuf[7]<<48)|((long)readbuf[8]<<56);
			printf("uart_test>>>solen>>> recv count = %d,ap_usec = %ld, mcu_usec = %ld  ----",cnt,ap_usec,mcu_usec);
			
			if(ap_usec > mcu_usec)
				printf("time jitter = %ld\n",ap_usec - mcu_usec);
			else
				printf("time jitter = %ld\n",mcu_usec - ap_usec);
		}
		
		sleep(5);//Wait 5 second
		gettimeofday(&tv,NULL);
		ap_usec = (tv.tv_sec - tv_sec_0)*1000000 + (tv.tv_usec - tv_usec_0);
		printf("uart_test>>>solen>>> ap send time %ld\n",ap_usec);
		writebuf[1] = ap_usec;
		writebuf[2] = ap_usec >> 8;
		writebuf[3] = ap_usec >> 16;
		writebuf[4] = ap_usec >> 24;
		writebuf[5] = ap_usec >> 32;
		writebuf[6] = ap_usec >> 40;
		writebuf[7] = ap_usec >> 48;
		writebuf[8] = ap_usec >> 56;
		set_gpio_high();
		tcflush(fd,TCOFLUSH);//清空缓存
		res = write(fd, writebuf, TRANSDATALEN);
		if(res < 0)
		{
			printf("uart_test>>>solen>>> fatal error cannot write!\n");
		}
#endif
    }

    printf("uart_test>>>solen>>> Close...\n");
    close(fd);
    return 0;
}
