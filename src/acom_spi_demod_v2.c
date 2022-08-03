#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/mman.h>
#include <stdlib.h>

#include "acom_phy.h"

#include "acom_fec.h"
#include "crc16.h"

#include "serial_io.c"

//--------------------------------------------------------------------------------------------------------------------------------

//#define ACOM_SAVE_ADC_DATA		
//#define ACOM_SAVE_COP_DATA		
//#define ACOM_SAVE_COD_DATA		

#define DBG_PRT


#define PEAKVALUE_PRT_CNT		0			//打印8组数据每组2个。分别实际的相关峰值和iir参考门限
											//(0.05+0.04+0.512+0.05)*4*2/4096*8000 = 10.1875
//--------------------------------------------------------------------------------------------------------------------------------
/** 设置消息长度 */
#define MAX_MESSAGE_LENGTH      512         // spi数据传输大小
#define MESSAGE_LENGTH          4           // spi控制传输大小
#define MYSPI_IOCTL_TRANSFER    0           // ioctl控制标志
#define MAX_PAGE_SIZE           4096        // 分页大小

#define PAGES_PER_8KFILE		(int)((ACOM_CHIRP_LEN +50 + ACOM_FRAME_LEN + 128)*8*4*2/4096)			// 8k采样数据保存文件，每个文件分页个数. 帧长ms*8k*4byte*2通道/MAX_PAGE_SIZE
//--------------------------------------------------------------------------------------------------------------------------------
//解调信息存储
//以40ms+50ms来计算，chirp+mute为1.4个分页,所以用于解调的cop存储只存前面部分
#define PAGES_COP_8KFILE		2					// 8k采样数据保存文件，每个文件分页个数. 帧长*4byte*2通道/MAX_PAGE_SIZE
#define PAGES_COD_8KFILE		PAGES_PER_8KFILE	// 8k采样数据保存文件，每个文件分页个数. 帧长*4byte*2通道/MAX_PAGE_SIZE
//--------------------------------------------------------------------------------------------------------------------------------

//最长支持1024个符号,占用时长为2048ms
unsigned char ref_nfsk[1024] ;


/** spi数据传输结构体 */
struct spi_buffer{
    char* buffer_write;   // 传输buffer
    char* buffer_read ;   // 接收buffer
    int cnt;              // 需要发送或接收的字节个数
};

int cod_store_cnt = 0;	//相关延迟输出数据存储数据块个数
int cod_store_ena = 0;	//相关延迟输出数据存储使能

int cop_store_cnt = 0;	//相关峰存储数据块个数
int cop_store_ena = 0;	//相关峰存储使能

int	cop_cnt = 0		;

int peak_index		;	//记录相关峰值的实际位置

int flag_acom_snd_clear;

char uart_recvmsg[64],uart_sendmsg[128];
int uart_nbytes = 0;
int uart_data_ready = 0;

/** 用于线程同步的锁定义 */
#ifdef ACOM_SAVE_ADC_DATA
sem_t mutex_adc;                      // ADC输入
#endif

sem_t mutex_cod;                      // 相关器数据

sem_t mutex_cop;                      // 相关器峰值
sem_t mutex_dem;                      // 相关器峰值
sem_t mutex_uart;                      // 相关器峰值

//sem_t mutex_dummy;                          // dummy填充数据

/** 线程函数 */
#ifdef ACOM_SAVE_ADC_DATA
void* write_adc(void* arg);                 // 向ADC log写线程
#endif
void* write_cod(void* arg);           // 向相关器数据log写线程
void* write_cop(void* arg);           // 向相关器峰值log写线程

//void* demod_fsk(void* arg);           // 实时解调线程
//void* modem_enc(void* arg);           // 数据发送线程
//void* uart_comm(void* arg);         // 串口通信线程

/**
 * buffer定义
 * 只有当buffer存满了才会触发文件读写（buffer为一个页框大小）
 */

#ifdef ACOM_SAVE_ADC_DATA
	char adc_buf[MAX_PAGE_SIZE];
	char adc_buf_tmp[MAX_PAGE_SIZE];
#endif

	char cod_buf[MAX_PAGE_SIZE];
	char cod_buf_tmp[MAX_PAGE_SIZE];

	char cop_buf[MAX_PAGE_SIZE];
	char cop_buf_tmp[MAX_PAGE_SIZE];

	char cod_frm[MAX_PAGE_SIZE*PAGES_COD_8KFILE];	//用于存下整个数据帧的buffer，作为解调的数据输入
	char cop_frm[MAX_PAGE_SIZE*PAGES_COP_8KFILE];	//用于存下整个数据帧的buffer，作为解调的数据输入

	short cod_i[MAX_PAGE_SIZE/8*PAGES_COD_8KFILE];	//用于存下整个数据帧的buffer，作为解调的数据输入
	short cod_q[MAX_PAGE_SIZE/8*PAGES_COD_8KFILE];	//用于存下整个数据帧的buffer，作为解调的数据输入

//char dummy_buf[MAX_PAGE_SIZE];



//char dummy_buf_tmp[MAX_PAGE_SIZE];

/** 当前buffer存储偏移 */
#ifdef ACOM_SAVE_ADC_DATA
int  adc_offset   = 0;
#endif
int cod_offset = 0;
int cop_offset = 0;
//int dummy_offset = 0;

/** 当前文件大小（以页框为单位） */
#ifdef ACOM_SAVE_ADC_DATA
int  adc_page_num = 0;
#endif
int cod_page_num = 0;
int cop_page_num = 0;
//int dummy_page_num = 0;

/** 描述符定义 */
int fd;                       // 调用/dev/myspi设备
int fd_spi3;                  // 调用/dev/myspi3设备
int	fd_uart;
#ifdef ACOM_SAVE_ADC_DATA
int fd_adc;                   // adc.dat文件
#endif

//int fd_cod;                   // cod.dat文件
int fd_cod_frm;               // cod.dat文件

//int fd_cop;                   // cop.dat文件
int fd_cop_frm;               // cod.dat文件

//int fd_dummy;               // 
//int fd_buffer;              // 

//相关检测门限所在寄存器:0x1f
static char copd_thresh[MESSAGE_LENGTH] = {0x5f, 0x00, 0x00, 0x00};
static char send_buf_cont1[MAX_MESSAGE_LENGTH] = {0x00, 0xaa, 0xa5, 0x20};
static char send_buf_contn[MAX_MESSAGE_LENGTH] = {0x00, 0xaa, 0xaa, 0x20};

int	  n_len;

#ifdef ACOM_SAVE_ADC_DATA
int      n_adc;
#endif

int      n_cop;
int      n_cod;
int		 n_dem;

#ifdef DBG_PRT
FILE* fp_log;
#endif

#ifdef PRINT_SOFT_INFO
FILE* fp_fec;
#endif

int		cop_val;
float	cop_tim;
int		cop_max_val = 0;
int		cop_max_val_m1 = 0;
int		cop_max_val_p1 = 0;

int		cop_max_slt = 0;
int		cop_max_idx = 0;
float	cop_max_tim = 0;

int		cop_max_val_intp = 0;
float	cop_max_tim_intp = 0;

char	flag_peak_fnd = 0;

int storage_release();      // 分开存储主函数，release版本


unsigned char dev_id  = 255;	//接收命令行参数，device ID
unsigned char auto_snd = 0;		//接收命令行参数，置1，自动发送（包括自己产生测试数据或者读取串口）
unsigned char beat_snd = 0;		//接收命令行参数，置1，内部产生静态心跳数据自动发送,置2，内部产生动态心跳数据自动发送

unsigned char beat_start=48;

#include "thread_demod_fsk.c"
#include "thread_modem_enc.c"
#include "thread_uart_comm.c"

extern acom_fec_version[];

int main(int argc, char *argv[])
{
	FILE *fp_ref;
    char *devname;

    if (argc < 5) {
        printf("Error! Missing parameter.\nUsage:\n\t%s rec_time[s] snd_filename Sym_Cnt Dev_Id\r\n",argv[0]);
        return -1;
    }

	//开串口设备
	fd_uart=Uart_Init();
	if (fd_uart<=0){
        printf("Serial port open error!");
        return -1;
	}

    devname = "/dev/myspi";
    fd = open(devname, O_RDWR);
    if(fd < 0) {
        printf("can't open spi  device %s\r\n", devname);
        return -1;
    }

#ifdef USE_SPI3
    devname = "/dev/myspi3";
    fd_spi3 = open(devname, O_RDWR);
    if(fd_spi3 < 0) {
        printf("can't open spi3 device %s\r\n", devname);
        return -1;
    }
#endif

	n_len =atoi(argv[1]);
#ifdef ACOM_SAVE_ADC_DATA
    n_adc = n_len* 512 / 4;
#endif



	printf("\nacom-fec version: %s\n\n",acom_fec_version);


	//用文件内容填充参考信号
	int i;
	int sym;

	if((fp_ref=fopen(argv[2],"r"))==NULL)   exit(0);
	for(i = 0;i<atoi(argv[3]);i++){
		fscanf(fp_ref,"%02d\n",&sym);
		ref_nfsk[i*2] = sym-((int)(sym/10))*10;
		ref_nfsk[i*2+1] = (int)(sym/10);
		//printf("%02d %d %d\n",sym,ref_nfsk[i*2+1],ref_nfsk[i*2]);
	}

	dev_id  = (unsigned char)atoi(argv[4]);
	auto_snd = (unsigned char)atoi(argv[5]);
	beat_snd = (unsigned char)atoi(argv[6]);

//每秒钟存12个page,共计12×MAX_PAGE_SIZE
#ifdef ACOM_SAVE_COP_DATA
    n_cop = n_len*PAGES_PER_8KFILE;
#endif
#ifdef ACOM_SAVE_COD_DATA
    n_cod = n_len*PAGES_PER_8KFILE;
#endif

	n_dem = n_len;

    pthread_t tid_uart;
    pthread_t tid_enc;

   		if(pthread_create(&tid_uart, NULL, uart_comm, NULL) == -1){ fprintf(stderr, "pthread create error:uart_comm"); }

	//auto_snd关闭的时候，例如转发节点和信宿，暂时不要创建发送线程。
	if(auto_snd){	
	   	if(pthread_create(&tid_enc,  NULL, modem_enc, NULL) == -1){ fprintf(stderr, "pthread create error:modem_enc"); }
	}



//    sem_post(&mutex_uart);	

	storage_release();

    return 0;
}


int storage_release(){

    time_t t;
    struct tm *cur_time;
	struct timeb t2;

    char path_log[64];
    char path_fec[64];
    char path_log1[64];
    char path_log2[64];
    time(&t);
    cur_time = localtime(&t);

#ifdef ACOM_SAVE_ADC_DATA
    // ADC输入
    strftime(path_log, 64, "/fifo/adc_%Y%m%d_%H%M%S.dat", cur_time);
    fd_adc = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
    if(fd_adc < 0) {
        fprintf(stderr, "can't open log file \r\n");
        return -1;
    }
#endif


    // 相关器数据
//    strftime(path_log, 64, "/mnt/mmc/cod_%Y%m%d_%H%M%S--------", cur_time);
//    fd_cod = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
//    if(fd_cod < 0) {
//        printf("can't open log file \r\n");
//        return -1;
//    }

    // 相关器峰值
//    strftime(path_log, 64, "/mnt/mmc/cop_%Y%m%d_%H%M%S--------", cur_time);
//    fd_cop = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
//    if(fd_cop < 0) {
//        printf("can't open log file \r\n");
//        return -1;
//    }

#ifdef DBG_PRT
    strftime(path_log, 64, "/mnt/mmc/log_%Y%m%d_%H%M%S.log", cur_time);
	if((fp_log = fopen(path_log,"wt"))==NULL){ 
		printf("can't open log file.");
		return -1;
	}
#endif

    char cod_buf_I[MESSAGE_LENGTH];
    char cod_buf_Q[MESSAGE_LENGTH];

    char cop_buf_I[MESSAGE_LENGTH];
    char cop_buf_Q[MESSAGE_LENGTH];

	//存储空间清零
    memset(cod_buf_I, 0, MESSAGE_LENGTH);
    memset(cod_buf_Q, 0, MESSAGE_LENGTH);

    memset(cop_buf_I, 0, MESSAGE_LENGTH);
    memset(cop_buf_Q, 0, MESSAGE_LENGTH);
//adc 路数据
#ifdef ACOM_SAVE_ADC_DATA
    sem_init(&mutex_adc, 0, 0);
#endif
    sem_init(&mutex_cod, 0, 0);
    sem_init(&mutex_cop, 0, 0);
    sem_init(&mutex_dem, 0, 0);

    sem_init(&mutex_uart, 0, 0);
//    sem_init(&mutex_dummy, 0, 0);

    //pthread_t tid_adc, tid_rcv, tid_cod, tid_cop;//, tid_dummy;

#ifdef ACOM_SAVE_ADC_DATA
    pthread_t tid_adc ;
#endif
    pthread_t tid_cod;
    pthread_t tid_cop;

    pthread_t tid_dem;

//创建线程,带有wirte的函数都是往文件中写数据？
#ifdef ACOM_SAVE_ADC_DATA
    if(pthread_create(&tid_adc, NULL, write_adc, NULL) == -1){ fprintf(stderr, "pthread create error:write_adc"); }
#endif
    if(pthread_create(&tid_cod, NULL, write_cod, NULL) == -1){ fprintf(stderr, "pthread create error:write_cod"); }
    if(pthread_create(&tid_cop, NULL, write_cop, NULL) == -1){ fprintf(stderr, "pthread create error:write_cop"); }

    if(pthread_create(&tid_dem, NULL, demod_fsk, NULL) == -1){ fprintf(stderr, "pthread create error:demod_fsk"); }

	//读取门限配置
    char 	recv_buf[MAX_MESSAGE_LENGTH];
    struct 	spi_buffer spi_buf;
    spi_buf.cnt = 4;
    spi_buf.buffer_read 	= recv_buf;
    spi_buf.buffer_write 	= copd_thresh;

    ioctl(fd, 0, &spi_buf);
	int cfg_copd_thresh = (recv_buf[2]&0x0f);
	//int cfg_copd_thresh =1; 

//	printf("%02x%02x%02x%02x %d\n",recv_buf[0],recv_buf[1],recv_buf[2],recv_buf[3],cfg_copd_thresh);


    spi_buf.buffer_write = send_buf_cont1;
	//首次读取丢弃,长度仍然是4，所以，只是用来复位一下ram
#ifdef USE_SPI3
    ioctl(fd_spi3, 0, &spi_buf);
#else
    ioctl(fd, 0, &spi_buf);
#endif


	int filecnt = 0;
    spi_buf.cnt = MAX_MESSAGE_LENGTH; //MAX_MESSAGE_LENGTH=512
    spi_buf.buffer_write = send_buf_contn;	//后续连续读取指令
	char frame_head;
	char frame_tail;
	int cop_print_cnt = 0;
	int baseline ;
    int cnt = 0;

	uint8_t head_data_I ;
	uint8_t head_data_Q ;

	uint8_t head_peak_I ;
	uint8_t head_peak_Q ;
	//每次分析4个字节，对应spi读上来的32bit。第一个字节称为头，最后1个字节称为尾。
//    char frame_head ;
//    char frame_tail ;

	cop_max_idx = 0;
	cop_cnt = 0;
	cop_max_tim = 0;

    while (1) {
        cnt = 0;

#ifdef USE_SPI3
        ioctl(fd_spi3, 0, &spi_buf);
#else
        ioctl(fd, 0, &spi_buf);
#endif
        //cnt += MESSAGE_LENGTH;
        while(cnt < MAX_MESSAGE_LENGTH-1){
            frame_head = recv_buf[cnt];
            frame_tail = recv_buf[cnt+MESSAGE_LENGTH-1];

            switch(frame_head & 0xf0){//根据头区分数据类型


                case 0b00000000:    // 填充，直接break
					break;//跳出switch

                case 0b10000000:    // 1pps 10MHz计数
					//printf("1pps cnt: %02x%02x%02x\n",recv_buf[cnt+1],recv_buf[cnt+2],recv_buf[cnt+3]);
					break;


                case 0b00010000:    // ADC输入
#ifdef ACOM_SAVE_ADC_DATA
                    memcpy(adc_buf+adc_offset, recv_buf+cnt, MESSAGE_LENGTH);

					#ifdef DBG_PRT
					//fprintf(fp_log,"%03x ",adc_offset);
					#endif

                    adc_offset += MESSAGE_LENGTH;

                    if(adc_offset == MAX_PAGE_SIZE){
						#ifdef DBG_PRT
						//fprintf(fp_log,"\n",adc_offset);
						#endif
                        memcpy(adc_buf_tmp, adc_buf, MAX_PAGE_SIZE);
                        //memset(adc_buf, 0, MAX_PAGE_SIZE);
                        adc_offset = 0;
                        sem_post(&mutex_adc);
                    }
#endif
                    break;



				//------------------------------------------------------------
				//0x40
                case 0b01000000:    // 相关器数据I
					if((frame_tail !=0 ) && cod_store_ena == 0){		//LSB不为0代表有相关峰值出现, cod_store_ena == 0说明到来的是新的相关峰
						cod_store_cnt = PAGES_PER_8KFILE;
						cod_store_ena = 1;
						cod_page_num  = 0;
						cop_cnt = 0;
						cop_max_idx = 0;

						//创建文件
						ftime(&t2);
						t = t2.time;
    					cur_time = localtime(&t);

//#ifdef ACOM_SAVE_COD_DATA		
//						strftime(path_log, 64, "/mnt/mmc/cod_%Y%m%d_%H%M%S", cur_time);
//						snprintf(path_log1,64,"_%03d.dat",t2.millitm);//获取毫秒
//						strcat(path_log,path_log1);
//						fd_cod_frm = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
//						if(fd_cod_frm < 0) {
//						    printf("can't open log file \r\n");
//						    return -1;
//						}
//#endif

						filecnt++;
						strftime(path_log, 64, "%Y%m%d_%H%M%S", cur_time);
						snprintf(path_log1,64,"_%03d",t2.millitm);//获取毫秒
						strcat(path_log,path_log1);
#ifdef DBG_PRT
						//fprintf(fp_log,"%4d : %s\n",filecnt,path_log);
#endif
						printf(        "FRM:%-4d CPU_Time: %s  ",filecnt,path_log);
						fprintf(fp_log,"FRM:%-4d CPU_Time: %s  ",filecnt,path_log);
						//printf("\n\n %4d : %s\n----+----+----+----+----+----+----+\n",filecnt,path_log);

						//write(fd_cod_frm,path_log,strlen(path_log));
						//close(fd_cod_frm);
					}
					
					if(cod_store_cnt > 0)
                    	memcpy(cod_buf_I, recv_buf+cnt, MESSAGE_LENGTH);
                    break;
				//------------------------------------------------------------
				//0x50
                case 0b01010000:    // 相关器数据Q
					if(cod_store_cnt > 0) {
                    	memcpy(cod_buf_Q, recv_buf+cnt, MESSAGE_LENGTH);

                    	head_data_I = cod_buf_I[0] & 0x0f;
                    	head_data_Q = cod_buf_Q[0] & 0x0f;

                    	if(head_data_I == head_data_Q){
                    	    memcpy(cod_buf+cod_offset, cod_buf_I, MESSAGE_LENGTH); cod_offset += MESSAGE_LENGTH;
                    	    memcpy(cod_buf+cod_offset, cod_buf_Q, MESSAGE_LENGTH); cod_offset += MESSAGE_LENGTH;
                    	}

						#ifdef DBG_PRT
						//fprintf(fp_log,"d: %4d  ",cod_offset);
						#endif
						//printf("cod_offset: %4d / cod_store_cnt: %d\n ",cod_offset,cod_store_cnt);
                    	if(cod_offset == MAX_PAGE_SIZE){
                    	    memcpy(cod_buf_tmp, cod_buf, MAX_PAGE_SIZE);
							//数据存储到cod_frm
                    	    memcpy(cod_frm+MAX_PAGE_SIZE*(PAGES_PER_8KFILE-cod_store_cnt), cod_buf, MAX_PAGE_SIZE);

                    	    memset(cod_buf, 0, MAX_PAGE_SIZE);
                    	    cod_offset = 0;
							if((flag_peak_fnd == 0) && (cod_store_cnt >1)){
								//printf(" Peak_Info: %d @ %d TOA: %f\n",cop_max_val,cop_max_idx,cop_max_tim);
								flag_peak_fnd = 1;
							}

//						printf(">"); fflush(stdout);
						#ifdef DBG_PRT
						//fprintf(fp_log,">"); 
						#endif


						cod_store_cnt--;
                   	    sem_post(&mutex_cod);

						//当cod_store_cnt减到0，数据收集完毕，发送消息给解调.
						if(cod_store_cnt == 0){
							//printf("trigger demodulator...\n");

#ifdef PRINT_SOFT_INFO
    						strftime(path_fec, 64, "/fifo/fec_%Y%m%d_%H%M%S.dat", cur_time);
							if((fp_fec = fopen(path_fec,"wt"))==NULL){ 
								printf("can't open log file.");
								return -1;
							}
#endif

                   	    	sem_post(&mutex_dem);
							}
                    	}
					}
                    break;



				//------------------------------------------------------------
				//0x60
                case 0b01100000:    // 相关器峰值I
					if((frame_tail!=0) && cop_store_ena == 0){	
						cop_store_cnt = PAGES_PER_8KFILE;
						cop_store_ena = 1;
						cop_page_num  = 0;

						cop_max_tim = 0;
						cop_max_val = 0;

						//创建文件
						ftime(&t2);
						t = t2.time;
    					cur_time = localtime(&t);

#ifdef ACOM_SAVE_COP_DATA
						strftime(path_log, 64, "/fifo/cop_%Y%m%d_%H%M%S", cur_time);
						snprintf(path_log1,64,"_%03d.dat",t2.millitm);
						strcat(path_log,path_log1);
						fd_cop_frm = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
						if(fd_cop_frm < 0) {
						    printf("can't open cop file \r\n");
						    return -1;
						}
#endif

#ifdef ACOM_SAVE_COD_DATA		
						strftime(path_log, 64, "/fifo/cod_%Y%m%d_%H%M%S", cur_time);
						snprintf(path_log1,64,"_%03d.dat",t2.millitm);//获取毫秒
						strcat(path_log,path_log1);
						fd_cod_frm = open(path_log, O_CREAT | O_RDWR,S_IRWXU);
						if(fd_cod_frm < 0) {
						    printf("can't open cod file \r\n");
						    return -1;
						}
#endif

						cop_print_cnt = PEAKVALUE_PRT_CNT;
                    }

					//相关峰值搜索
					cop_val = (unsigned int)recv_buf[cnt+1]*256+(unsigned int)recv_buf[cnt+2];
					if( cop_max_val < cop_val && (cop_cnt/8 < 128)){
						cop_max_val = cop_val;
						cop_max_slt = (recv_buf[cnt+3]&0x7f)/2;
						cop_max_tim =  cop_max_slt + (float)(((unsigned int)recv_buf[cnt+5]*256+(unsigned int)recv_buf[cnt+6])*256+(unsigned int)recv_buf[cnt+7])/10000000;
						cop_max_idx = cop_cnt/8;
					}

					if(cop_print_cnt){
						printf("%6d ",recv_buf[cnt+1]*256+recv_buf[cnt+2]);
						cop_print_cnt --;
					}

					if(cop_store_cnt > 0)
                    	memcpy(cop_buf_I, recv_buf+cnt, MESSAGE_LENGTH);

					cop_cnt = cop_cnt +4;
                    break;


				//------------------------------------------------------------
				//0x70
                case 0b01110000:    // 相关器峰值Q
					if(cop_store_cnt > 0) {
                    	memcpy(cop_buf_Q, recv_buf+cnt, MESSAGE_LENGTH);
                    	head_peak_I = cop_buf_I[0] & 0x0f;
                    	head_peak_Q = cop_buf_Q[0] & 0x0f;
                    	if(head_peak_I == head_peak_Q){
                    	    memcpy(cop_buf+cop_offset, cop_buf_I, MESSAGE_LENGTH);
                    	    cop_offset += MESSAGE_LENGTH;
                    	    memcpy(cop_buf+cop_offset, cop_buf_Q, MESSAGE_LENGTH);
                    	    cop_offset += MESSAGE_LENGTH;
                    	}

#ifdef DBG_PRT
						//fprintf(fp_log,"p: %4d\n",cop_offset);
#endif

                    	if(cop_offset == MAX_PAGE_SIZE){
                    	    memcpy(cop_buf_tmp, cop_buf, MAX_PAGE_SIZE);
							if(PAGES_PER_8KFILE - cop_store_cnt <PAGES_COP_8KFILE)
                    	    memcpy(cop_frm+MAX_PAGE_SIZE*(PAGES_PER_8KFILE-cop_store_cnt), cop_buf, MAX_PAGE_SIZE);

                    	    memset(cop_buf, 0, MAX_PAGE_SIZE);
                    	    cop_offset = 0;
							cop_store_cnt--;
                    	    sem_post(&mutex_cop);
                    	}

						//if(recv_buf[cnt+3]&0x01)
//						if(recv_buf[cnt+3])
//							//cop_tim =  (recv_buf[cnt+3]&0x7f)/2+((recv_buf[cnt+1]*256*256+recv_buf[cnt+2]*256+recv_buf[cnt+3])/10e6);
//							cop_tim =  cop_max_slt + ((recv_buf[cnt+1]*256*256+recv_buf[cnt+2]*256+recv_buf[cnt+3])/10e6);

//						if(cop_max_tim < cop_tim)
//							cop_max_tim = cop_tim;
							//cop_max_tim = cop_max_slt + cop_tim;
							//cop_max_tim =  cop_max_slt + ((recv_buf[cnt+1]*256*256+recv_buf[cnt+2]*256+recv_buf[cnt+3])/10e6);

						if(cop_print_cnt){	//打印输出相关峰数据
							baseline = recv_buf[cnt+1]*256+recv_buf[cnt+2];
							//printf("%6d %6d %s\n",baseline,(baseline+1)*32-1,recv_buf[cnt+3]?"peak":" ");
							if(recv_buf[cnt+3])
							printf("%9.6fs %s\n",
									cop_max_slt+((recv_buf[cnt+1]*256*256+recv_buf[cnt+2]*256+recv_buf[cnt+3])/10e6), 
									//((recv_buf[cnt+1]*256*256+recv_buf[cnt+2]*256+recv_buf[cnt+3])/10e6), 
									"peak");
							else
							printf("%6d %5d \n",
									baseline,
									((baseline+1)*(1<<(4+cfg_copd_thresh))-1));

//							printf("------ %d ------\n",cop_print_cnt);
							cop_print_cnt --;
						}
					}
					cop_cnt = cop_cnt +4;
                    break;



                default:
                    //memcpy(dummy_buf+dummy_offset, recv_buf+cnt, MESSAGE_LENGTH);
                    //dummy_offset+=MESSAGE_LENGTH;
                    //if(dummy_offset == MAX_PAGE_SIZE){
                    //    memcpy(dummy_buf_tmp, dummy_buf, MAX_PAGE_SIZE);
                    //    memset(dummy_buf, 0, MAX_PAGE_SIZE);
                    //    dummy_offset = 0;
                    //    sem_post(&mutex_dummy);
                    //}
                    break;
            }
            cnt += MESSAGE_LENGTH;
        }
    }

#ifdef ACOM_SAVE_ADC_DATA
    sem_destroy(&mutex_adc);
#endif
#ifdef ACOM_SAVE_COD_DATA
    sem_destroy(&mutex_cod);
#endif
#ifdef ACOM_SAVE_COP_DATA
    sem_destroy(&mutex_cop);
#endif
    sem_destroy(&mutex_dem);
//    sem_destroy(&mutex_dummy);

}

#ifdef ACOM_SAVE_ADC_DATA
void *write_adc(void* arg){
	char *page_adc ; 

    while(1){
        ftruncate(fd_adc, (adc_page_num+1)*MAX_PAGE_SIZE);
        page_adc = (char *)mmap(NULL, MAX_PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd_adc, adc_page_num * MAX_PAGE_SIZE);
#ifdef DBG_PRT
//fprintf(fp_log,"page_adc:%d\n",page_adc);
#endif
        sem_wait(&mutex_adc);
#ifdef DBG_PRT
//fprintf(fp_log,"---write_adc----\n");
#endif
        memcpy(page_adc, adc_buf_tmp, MAX_PAGE_SIZE);
        //断开 page_adc与adc_buf_tmp的连接，让adc_buf_tmp可以接收新数据？
        munmap(page_adc, MAX_PAGE_SIZE);
        adc_page_num++;
        memset(adc_buf_tmp, 0, MAX_PAGE_SIZE);

//ADC应该一直保持记录，不要退出
//		n_adc--; if(n_adc == 0) { printf("\n\nADC file store Terminated."); pthread_exit(0); }

    }
}
#endif


void* write_cod(void* arg){
	char *page_cod ;
    while(1){
		if(cod_store_ena ==0) continue;

#ifdef ACOM_SAVE_COD_DATA		
        ftruncate(fd_cod_frm, PAGES_PER_8KFILE*MAX_PAGE_SIZE);
        page_cod = (char *)mmap(NULL, MAX_PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd_cod_frm, cod_page_num * MAX_PAGE_SIZE);
#endif

#ifdef DBG_PRT
//fprintf(fp_log,"page_cod:%d\n",page_cod);
#endif
        sem_wait(&mutex_cod);
#ifdef DBG_PRT
//fprintf(fp_log,"---write_cod----\n");
#endif

#ifdef ACOM_SAVE_COD_DATA		
        memcpy(page_cod, cod_buf_tmp, MAX_PAGE_SIZE);
        munmap(page_cod, MAX_PAGE_SIZE);
#endif

        cod_page_num++;
        memset(cod_buf_tmp, 0, MAX_PAGE_SIZE);


		if (cod_store_cnt == 0){ 
#ifdef ACOM_SAVE_COD_DATA
			close(fd_cod_frm);
#endif
			cod_store_ena = 0;
		}

//		n_cod--; if(n_cod == 0) { printf("\n\ncod file store Terminated."); pthread_exit(0); } 

    }
}

void* write_cop(void* arg){
	char *page_cop ;
    while(1){
		if(cop_store_ena ==0) continue;

#ifdef ACOM_SAVE_COP_DATA		
        ftruncate(fd_cop_frm, PAGES_PER_8KFILE*MAX_PAGE_SIZE);
        page_cop = (char *)mmap(NULL, MAX_PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd_cop_frm, cop_page_num * MAX_PAGE_SIZE);
#endif

#ifdef DBG_PRT
//fprintf(fp_log,"page_cop:%d\n",page_cop);
#endif
        sem_wait(&mutex_cop);
#ifdef DBG_PRT
//fprintf(fp_log,"---write_cop----\n");
#endif

#ifdef ACOM_SAVE_COP_DATA		
        memcpy(page_cop, cop_buf_tmp, MAX_PAGE_SIZE);
        munmap(page_cop, MAX_PAGE_SIZE);
#endif
        cop_page_num++;
        memset(cop_buf_tmp, 0, MAX_PAGE_SIZE);

		if (cop_store_cnt == 0){ 
#ifdef ACOM_SAVE_COP_DATA
			close(fd_cop_frm);
#endif
			cop_store_ena = 0;
			cop_max_tim = 0;
			cop_max_val = 0;
			flag_peak_fnd = 0;
		}

		n_cop--; if(n_cop == 0) { 
			//printf("\n\nTerminated by write_cop"); 
//			exit(0); 
		} 
    }
}

/*

void* write_cop(void* arg){
    while(1){
        ftruncate(fd_cop, (cop_page_num+1)*MAX_PAGE_SIZE);
        char *page_cop = (char *)mmap(NULL, MAX_PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd_cop, cop_page_num * MAX_PAGE_SIZE);
        sem_wait(&mutex_cop);
        memcpy(page_cop, cop_buf_tmp, MAX_PAGE_SIZE);
        munmap(page_cop, MAX_PAGE_SIZE);
        cop_page_num++;
        memset(cop_buf_tmp, 0, MAX_PAGE_SIZE);
//		n_cop--; if(n_cop == 0) { printf("\n\nTerminated by write_cop"); exit(0); } 
    }
}

*/

