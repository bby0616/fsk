#ifndef __ACOM_PHY_H__
#define __ACOM_PHY_H__
//--------------------------------------------------------------------------------------------------------------------------------

//#define ACOM_SYM_CNT			640

#define ACOM_CHIRP_LEN			40	//ms
#define ACOM_FRAME_LEN			(ACOM_SYM_CNT*2)	//ms

#define COD_STORE_LEN			((ACOM_CHIRP_LEN+50+ACOM_FRAME_LEN + 100)*8)	//采样长度

//--------------------------------------------------------------------------------------------------------------------------------


//spi数据传输结构体 
typedef struct __spibuff{
    char* 	buffer_write;// 传输buffer
    char* 	buffer_read ;// 接收buffer
    int 	cnt			;// 需要发送或接收的字节个数
} str_SpiBuff;

//解调输出信息结构体
typedef struct __fskinfo{
	float	pow_chirp	;
	float	pow_nep1	;//normalized echo power1
	float	pow_nep2	;//normalized echo power1
	float	pow_nep3	;//normalized echo power1
	float	pow_nep4	;//normalized echo power1
	float	pow_nep5	;//normalized echo power1
} str_FskInfo;


//--------------------------------------------------------------------------------------------------------------------------------
void acom_phy_reg_wr(int spi_fd,unsigned char addr,unsigned int *data);
void acom_phy_reg_rd(int spi_fd,unsigned char addr,unsigned int *data);
void acom_phy_reg_rw(int spi_fd,unsigned char addr,unsigned char R_nW,unsigned char *send_buf,unsigned char *recv_buf);

void acom_phy_snd_one_frame(int spi_fd,int sym_cnt,unsigned char *data);
void acom_phy_rcv_one_frame(int spi_fd,int sym_cnt,unsigned char *data);

void acom_phy_8fsk_demod(short* codi,short* codq,unsigned char car_bgn,unsigned char car_end,str_FskInfo *fskinfo,int *soft_info,unsigned char *hard_deci);
/*
参数说明
codi/codq
	指向FPGA原始IQ信息存放位置的指针

car_bgn/car_end
	为起止载波序号，对于8psk分别置0和7，对于4fsk置2和5

fskinfo
	指向结构体的指针，结构存放chirp功率和回声功率等信息

soft_info	
	指向二维数组的指针,soft_info存放输出的软信息

hard_deci
	指向应判决结果存放位置的指针

*/


//--------------------------------------------------------------------------------------------------------------------------------
#endif
