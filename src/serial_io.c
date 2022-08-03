#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#define iSTAR_DEV "/dev/ttymxc5"
/**************************************************
name: Uart device initialize and setup
function: 
return: >0, fd
		0/-1: abnormal
****************************************************/
int Uart_Init()
{
  int fd;
  struct termios newtio;

  fd = open(iSTAR_DEV, O_RDWR|O_NOCTTY);
  if (fd < 0) {
    printf("[%s:%d] open %s failed: %s\n", __FILE__, __LINE__, iSTAR_DEV, strerror(errno));
    return -1;
  }

  fcntl(fd, F_SETFL, 0);
  if(isatty(STDIN_FILENO) == 0){
        fprintf(stderr, "standard input is not a terminal device\n");
		close(fd);
		return -1;
  }
  

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL; 
  newtio.c_cflag |= CREAD;
  newtio.c_cflag &= ~CSIZE;
  newtio.c_cflag &= ~CSTOPB;
  newtio.c_cflag &= ~PARENB;
  newtio.c_cflag |= CS8;
  cfsetispeed(&newtio, B115200);
  cfsetospeed(&newtio, B115200);
  newtio.c_cc[VTIME] = 10; //time out interver per 0.1s for read return
  newtio.c_cc[VMIN] = 1; //the least byte received before read return
  tcflush(fd, TCIFLUSH);
  if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
    printf("[%s:%d] tcsetattr error: %s\n", __FILE__, __LINE__, strerror(errno));
    close(fd);
    return -1;
  }
  //if (set_nonblock(fd)<0)
  //{close(fd);  return -1;}
  printf("get serial fd %d\n",fd);
  return fd;
}
int recvdata(int fd, char * backmsg,int buflen)
{
	int ret;
	
	fd_set recv_fds;
	struct timeval t_out;
	int nbytes=0;
	char buf[64];
	char *p;
	while(1)
	{
		t_out.tv_sec=5;
		t_out.tv_usec=0;
	
		FD_ZERO(&recv_fds);
    		FD_SET(fd, &recv_fds);
    		ret=select(fd+1,&recv_fds,NULL,NULL,&t_out);
		if (ret<=0)
		{
			//printf("IO select finally time out or err:%d, we total get %d bytes: %s\n",ret,nbytes,backmsg);
			if (ret<0)
				return -1;
			
			return nbytes;
		}
		if(FD_ISSET(fd, &recv_fds)){
			memset(buf,0,sizeof(buf));
			ret=read(fd,buf,sizeof(buf));
			if (ret<=0)
				return nbytes;
			//printf("[Info:%s] %d time try recv %d bytes msg:%s\n",msg,ind++,ret, buf);
			if (strlen(backmsg)+strlen(buf)>buflen){
				printf("sorry!instruction overflow\n");
				return -1;
			}
			strcat(backmsg,buf);
			nbytes+=ret;
			p=strstr(backmsg,"\r\n");
			if (p!=NULL)
			{
				printf("we found 2-bytes end %x\n",*p);
				*p='\0';
				return nbytes;
			}
			p=strstr(backmsg,"\n");
			if (p != NULL)
			{				
				printf("we found 1-bytes n-end %x\n",*p);
				*p='\0';
				return nbytes;			
			}
			p=strstr(backmsg,"\r");
			if (p != NULL)
			{				
				printf("we found 1-bytes r-end %x\n",*p);
				*p='\0';
				return nbytes;			
			}
		}
		
	}
}


#if 0
int main(int argc, char **argv)
{
	int myfd=Uart_Init();
	int nbytes;
	char recvmsg[64],sendmsg[128];
	if (myfd<=0)
		return -1;
printf("string(\"123456\"),len=%d\n",strlen("123456"));	
	//sprintf(sendbuf,"%s\n","please enter an instruction");
	while(1){
		//nbytes=write(fd,sendbuf,strlen(sendbuf));
		memset(recvmsg,0,sizeof(recvmsg));
		nbytes=recvdata(myfd,recvmsg,sizeof(recvmsg));
		if (nbytes >0)
		{
			printf("get %d bytes:%s, len %d,\n",nbytes,recvmsg,strlen(recvmsg));
			memset(sendmsg,0,sizeof(sendmsg));
			sprintf(sendmsg,"echo data:%s\n",recvmsg);
			nbytes=write(myfd,sendmsg,strlen(sendmsg));
		}
	}
	close(myfd);

}
#endif
