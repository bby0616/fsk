#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "acom_phy.h"

/*
center				    
8fsk	m4	m3	m2	m1	dc	p1	p2	p3
4fsk	      	m2	m1	dc	p1	      	
*/

//旋转矩阵，适用于4fsk、8fsk

//从上到下分别为m4/m3/m2/m1/dc/p1/p2/p3
//real(exp(j*2*pi*[4:-1:-3]'*t/8))
float	refi_8fft[8][8] = {
{-1      ,  1      , -1      ,  1      , -1      ,  1      , -1      ,  1     },\
{-0.7071 , -0.0000 ,  0.7071 , -1.0000 ,  0.7071 ,  0.0000 , -0.7071 ,  1.0000},\
{ 0.0000 , -1.0000 , -0.0000 ,  1.0000 ,  0.0000 , -1.0000 , -0.0000 ,  1.0000},\
{ 0.7071 ,  0.0000 , -0.7071 , -1.0000 , -0.7071 , -0.0000 ,  0.7071 ,  1.0000},\
{ 1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1      ,  1     },\
{ 0.7071 ,  0.0000 , -0.7071 , -1.0000 , -0.7071 , -0.0000 ,  0.7071 ,  1.0000},\
{ 0.0000 , -1.0000 , -0.0000 ,  1.0000 ,  0.0000 , -1.0000 , -0.0000 ,  1.0000},\
{-0.7071 , -0.0000 ,  0.7071 , -1.0000 ,  0.7071 ,  0.0000 , -0.7071 ,  1.0000}};

//imag(exp(j*2*pi*[4:-1:-3]'*t/8))
float	refq_8fft[8][8] = {
{ 0      ,  0      ,  0      ,  0      ,  0      ,  0      ,  0      ,  0     },\
{ 0.7071 , -1.0000 ,  0.7071 ,  0.0000 , -0.7071 ,  1.0000 , -0.7071 , -0.0000},\
{ 1.0000 ,  0.0000 , -1.0000 , -0.0000 ,  1.0000 ,  0.0000 , -1.0000 , -0.0000},\
{ 0.7071 ,  1.0000 ,  0.7071 ,  0.0000 , -0.7071 , -1.0000 , -0.7071 , -0.0000},\
{ 0      ,  0      ,  0      ,  0      ,  0      ,  0      ,  0      ,  0     },\
{-0.7071 , -1.0000 , -0.7071 , -0.0000 ,  0.7071 ,  1.0000 ,  0.7071 ,  0.0000},\
{-1.0000 , -0.0000 ,  1.0000 ,  0.0000 , -1.0000 , -0.0000 ,  1.0000 ,  0.0000},\
{-0.7071 ,  1.0000 , -0.7071 , -0.0000 ,  0.7071 , -1.0000 ,  0.7071 ,  0.0000}};


//--------------------------------------------------------------------------------------------------------------------------------
void acom_phy_reg_wr(int spi_fd,unsigned char addr,unsigned int *data)
{
    	char send_buf[4] = {0x00,0x00,0x00,0x00};
    	char recv_buf[4] = {0x00,0x00,0x00,0x00};

    	send_buf[0] =  addr;
    	send_buf[1] =  (unsigned char)(((*data)>>16)&0x000000ff);
    	send_buf[2] =  (unsigned char)(((*data)>> 8)&0x000000ff);
    	send_buf[3] =  (unsigned char)(((*data)    )&0x000000ff);

    	str_SpiBuff spi_buf;
    	spi_buf.buffer_write = send_buf;
    	spi_buf.buffer_read  = recv_buf;

    	spi_buf.cnt = 4;

    	ioctl(spi_fd, 0, &spi_buf); 
}

void acom_phy_reg_rd(int spi_fd,unsigned char addr,unsigned int *data)
{
    	char send_buf[4] = {0x00,0x00,0x00,0x00};
    	char recv_buf[4] = {0x00,0x00,0x00,0x00};

    	send_buf[0] =  addr + 0x40;

    	str_SpiBuff spi_buf;
    	spi_buf.buffer_write = send_buf;
    	spi_buf.buffer_read  = recv_buf;
    	spi_buf.cnt = 4;

    	ioctl(spi_fd, 0, &spi_buf); 
		*data =   ((unsigned int)recv_buf[1])*65536   
				+ ((unsigned int)recv_buf[2])*256
				+ ((unsigned int)recv_buf[3]);
}

void acom_phy_reg_rw(int spi_fd,unsigned char addr,unsigned char R_nW,unsigned char *send_buf,unsigned char *recv_buf)
{
    	send_buf[0] =  addr + (R_nW ? 0x40 : 0x00);
		bzero(recv_buf,4);

    	str_SpiBuff spi_buf;
    	spi_buf.buffer_write = send_buf;
    	spi_buf.buffer_read  = recv_buf;
    	spi_buf.cnt = 4;

    	ioctl(spi_fd, 0, &spi_buf); 
}
//--------------------------------------------------------------------------------------------------------------------------------

void	acom_phy_snd_one_frame(int spi_fd,int sym_cnt,unsigned char *data)
{
		
		//printf("acom_snd_one_frame called. \n");
		int i;
	    //设置发送接收buffer
	    char  send_buf_ref[324];
	    char  recv_buf_ref[324];
	    //struct spi_buffer spi_ref;                                                                                                                                                                                     
		str_SpiBuff spi_ref;
	    spi_ref.buffer_read  = recv_buf_ref;
	    spi_ref.buffer_write = send_buf_ref;

		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//acom_reg_wr 3b 000011: cmd_fsk_wr_bgn
	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x11;
	    ioctl(spi_fd, 0, &spi_ref);

		//写payload
	    //spi_ref.cnt          = 320+4;
	    spi_ref.cnt          = sym_cnt/2 + 4;
	
	    send_buf_ref[0] =  0x00;
	    send_buf_ref[1] =  0xaa;
	    send_buf_ref[2] =  0xaa;
	    send_buf_ref[3] =  0x08;
	
	    for(i = 4;i<4+sym_cnt/2;i++){
	        send_buf_ref[i] = *(data+i-4);
	    }

	    //for(i = 0;i<384;i=i+2){
	    //    send_buf_ref[4+i/2] = ref_nfsk[i] + ref_nfsk[i+1]*16;
	    //}
	
		//数据写入FPGA
	    ioctl(spi_fd, 0, &spi_ref);

		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//发送
		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//发送写指令,等待自己的时隙到来后发送
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x00;
	    send_buf_ref[1] =  0xff;
	    send_buf_ref[2] =  0xff;
	    send_buf_ref[3] =  0xef;
	    ioctl(spi_fd, 0, &spi_ref);

//		printf("Received frame repeated ! \n");
}

void	acom_phy_rcv_one_frame(int spi_fd,int sym_cnt,unsigned char *data)
{
		
		//printf("acom_snd_one_frame called. \n");
		int i;
	    //设置发送接收buffer
	    char  send_buf_ref[324];
	    char  recv_buf_ref[324];
	    //struct spi_buffer spi_ref;                                                                                                                                                                                     
		str_SpiBuff spi_ref;
	    spi_ref.buffer_read  = recv_buf_ref;
	    spi_ref.buffer_write = send_buf_ref;

		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//acom_reg_wr 3b 000012: cmd_fsk_rd_bgn
	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x12;
	    ioctl(spi_fd, 0, &spi_ref);

		//写payload
	    //spi_ref.cnt          = 320+4;
	    spi_ref.cnt          = sym_cnt/2 + 4;
	
	    send_buf_ref[0] =  0x00;
	    send_buf_ref[1] =  0xaa;
	    send_buf_ref[2] =  0xaa;
	    send_buf_ref[3] =  0x08;
	

	    //for(i = 0;i<384;i=i+2){
	    //    send_buf_ref[4+i/2] = ref_nfsk[i] + ref_nfsk[i+1]*16;
	    //}
	
		//数据交换
	    ioctl(spi_fd, 0, &spi_ref);

	    for(i = 4;i<4+sym_cnt/2;i++){
	         *(data+i-4) = recv_buf_ref[i] ;
	    }

		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//发送
		//acom_reg_wr 3b 000000:cmd_fsk_op_end
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x3b;
	    send_buf_ref[1] =  0x00;
	    send_buf_ref[2] =  0x00;
	    send_buf_ref[3] =  0x00;
	    ioctl(spi_fd, 0, &spi_ref);

		//发送写指令,等待自己的时隙到来后发送
	    spi_ref.cnt          = 4;

	    send_buf_ref[0] =  0x00;
	    send_buf_ref[1] =  0xff;
	    send_buf_ref[2] =  0xff;
	    send_buf_ref[3] =  0xef;
	    ioctl(spi_fd, 0, &spi_ref);

//		printf("Received frame repeated ! \n");
}


//void acom_4fsk_demod(short* codi,short* codq,str_FskInfo *fskinfo,int (*soft_info)[4])
void acom_phy_8fsk_demod(short* codi,short* codq,unsigned char car_bgn,unsigned char car_end,str_FskInfo *fskinfo,int *soft_info,unsigned char *hard_deci)
{

	float codi_flt;
	float codq_flt;

	int max_soft_1[8] = {0,0,0,0,0,0,0,0};
	int max_soft_2[8] = {0,0,0,0,0,0,0,0};
	int mean_soft[8]  = {0,0,0,0,0,0,0,0};

//	int max_p1=0,max_dc=0,max_m1=0,max_m2=0;
//	int max_p1_2=0,max_dc_2=0,max_m1_2=0,max_m2_2=0;
//	int mean_p1,mean_dc,mean_m1,mean_m2;

	int   offset = 4;
	int   i,j,k;
	float pow_chirp;
	float pow_echo;
	float pow_echo1 = 0;
	float pow_echo2 = 0;
	float pow_echo3 = 0;
	float pow_echo4 = 0;
	float pow_echo5 = 0;

	pow_chirp = 0;
	pow_echo  = 0;

	int pow_offset = 4;

	for(i = 20;i< 320+20;i++){
		pow_chirp += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
	}
		pow_chirp = pow_chirp/320/65536*100;

	//40ms*8 = 320
	//(40ms+50ms)*8 = 720
	for(i = 320+pow_offset;i< 720+pow_offset;i++){
		if     ((i> 320+pow_offset      ) && (i<=320+pow_offset+80*1))
			pow_echo1  += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
		else if((i> 320+pow_offset+ 80*1) && (i<=320+pow_offset+80*2))
			pow_echo2  += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
		else if((i> 320+pow_offset+ 80*2) && (i<=320+pow_offset+80*3))
			pow_echo3  += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
		else if((i> 320+pow_offset+ 80*3) && (i<=320+pow_offset+80*4))
			pow_echo4  += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
		else if((i> 320+pow_offset+ 80*4) && (i<=320+pow_offset+80*5))
			pow_echo5  += (float)codi[i]*(float)codi[i] + (float)codq[i]*(float)codq[i];
	}

			pow_echo1  = pow_echo1/80/65536*100;
			pow_echo2  = pow_echo2/80/65536*100;
			pow_echo3  = pow_echo3/80/65536*100;
			pow_echo4  = pow_echo4/80/65536*100;
			pow_echo5  = pow_echo5/80/65536*100;


	int	cod_soft[8][COD_STORE_LEN/8];	
	//for(i = 0:i<8;i++) {
	for(i = car_bgn;i<=car_end;i++) {
		for(j = offset;j< COD_STORE_LEN;j=j+8){
			codi_flt 	= *(codi+j-4)*refi_8fft[i][0] - *(codq+j-4)*refq_8fft[i][0] 
						+ *(codi+j-3)*refi_8fft[i][1] - *(codq+j-3)*refq_8fft[i][1] 
						+ *(codi+j-2)*refi_8fft[i][2] - *(codq+j-2)*refq_8fft[i][2] 
						+ *(codi+j-1)*refi_8fft[i][3] - *(codq+j-1)*refq_8fft[i][3] 
						+ *(codi+j  )*refi_8fft[i][4] - *(codq+j  )*refq_8fft[i][4] 
						+ *(codi+j+1)*refi_8fft[i][5] - *(codq+j+1)*refq_8fft[i][5] 
						+ *(codi+j+2)*refi_8fft[i][6] - *(codq+j+2)*refq_8fft[i][6] 
						+ *(codi+j+3)*refi_8fft[i][7] - *(codq+j+3)*refq_8fft[i][7]; 

			codq_flt 	= *(codi+j-4)*refq_8fft[i][0] + *(codq+j-4)*refi_8fft[i][0] 
						+ *(codi+j-3)*refq_8fft[i][1] + *(codq+j-3)*refi_8fft[i][1] 
						+ *(codi+j-2)*refq_8fft[i][2] + *(codq+j-2)*refi_8fft[i][2] 
						+ *(codi+j-1)*refq_8fft[i][3] + *(codq+j-1)*refi_8fft[i][3] 
						+ *(codi+j  )*refq_8fft[i][4] + *(codq+j  )*refi_8fft[i][4] 
						+ *(codi+j+1)*refq_8fft[i][5] + *(codq+j+1)*refi_8fft[i][5] 
						+ *(codi+j+2)*refq_8fft[i][6] + *(codq+j+2)*refi_8fft[i][6] 
						+ *(codi+j+3)*refq_8fft[i][7] + *(codq+j+3)*refi_8fft[i][7]; 

			cod_soft[i][(j-4)/8] = codi_flt*codi_flt+ codq_flt*codq_flt;

			//printf("%4d  %5d\n",(j-4)/8,cod_soft[i][(j-4)/8]);

			//求最大值和次大值
			if( max_soft_1[i]   < cod_soft[i][(j-4)/8]){
				max_soft_2[i]   = max_soft_1[i];
				max_soft_1[i]   = cod_soft[i][(j-4)/8];
			}
		}
	}
	
	//最大值和次大值取平均
	//for (i = 0:i<8;i++){
	for (i = car_bgn;i<=car_end;i++){
		mean_soft[i] = (max_soft_1[i] + max_soft_2[i])/200;
	}

	//频域均衡
	//for (i = 0:i<8;i++){
	for (i = car_bgn;i<=car_end;i++){
		for(j = offset;j< COD_STORE_LEN/8;j++){
			cod_soft[i][j] = cod_soft[i][j]/mean_soft[i];
		}
	}

	//fsk判决输出
	int  deci_max;
	//char deci[ACOM_SYM_CNT];

	for(i = 92;i<92+(ACOM_SYM_CNT*2);i=i+2){
		deci_max = 0;
		//for(j = 0;i<8;j++){
		for(j = car_bgn;j<=car_end;j++){
			if(cod_soft[j][i] > deci_max){
				deci_max = cod_soft[j][i] ;
				*(hard_deci+(i-92)/2) = j-car_bgn;
				//deci[(i-92)/2] = j-car_bgn;
			}
		}
		//printf("%6d : %12d %12d %12d %12d :%10d %10d\n",i,cod_soft[2][i],cod_soft[3][i],cod_soft[4][i],cod_soft[5][i],deci[i],*(hard_deci+(i-92)/2));
	}

		
	//for (i = 0:i<8;i++){
	for (i = car_bgn;i<=car_end;i++){
		for(j = 92;j<92+(ACOM_SYM_CNT*2);j=j+2){
			*(soft_info + (j-92)/2*8 + i) = cod_soft[i][j];
		}
	}

//解调完成，结构体信息写入。误码统计、打印、编码等放到外面做。
	fskinfo->pow_chirp 	= pow_chirp;
	fskinfo->pow_nep1	= pow_echo1*100/pow_chirp; 
	fskinfo->pow_nep2	= pow_echo2*100/pow_chirp; 
	fskinfo->pow_nep3	= pow_echo3*100/pow_chirp; 
	fskinfo->pow_nep4	= pow_echo4*100/pow_chirp; 
	fskinfo->pow_nep5	= pow_echo5*100/pow_chirp; 
}


//=====================================================================================================================

//	int sym_err_cnt;
//	sym_err_cnt = 0;
//	for (i = 0;i<(ACOM_SYM_CNT);i++){
//		sym_err_cnt += (deci[i] != ref_nfsk[i]);	
//	}


//	unsigned char fec_out[ACOM_SYM_CNT];
//	int  sym_err_cnt_fec;
//	sym_err_cnt_fec = -1;
//  	//译码
//  	acom_fec_4fsk_v2(soft_info,640,macinfo);	//frm_len = 128sym + 512sym
//  	//sym_err_cnt_fec = acom_fec_4fsk(soft_info,ACOM_SYM_CNT,ref_nfsk,fec_out);
//  	//*err_cnt = sym_err_cnt_fec;
//  
//  	printf(        "sym_err: %3d  pow: %9.2f echo:[ %5.2f %5.2f %5.2f %5.2f %5.2f ] %3d\n",sym_err_cnt,pow_chirp,pow_echo1*100/pow_chirp,pow_echo2*100/pow_chirp,pow_echo3*100/pow_chirp,pow_echo4*100/pow_chirp,pow_echo5*100/pow_chirp,sym_err_cnt_fec);
//  	fprintf(fp_log,"sym_err: %3d  pow: %9.2f echo:[ %5.2f %5.2f %5.2f %5.2f %5.2f ] %3d\n",sym_err_cnt,pow_chirp,pow_echo1*100/pow_chirp,pow_echo2*100/pow_chirp,pow_echo3*100/pow_chirp,pow_echo4*100/pow_chirp,pow_echo5*100/pow_chirp,sym_err_cnt_fec); fflush(fp_log);
//  
//  #ifdef PRINT_SOFT_INFO
//  	for(i = 92;i<92+(ACOM_SYM_CNT*2);i=i+2){
//  		fprintf(fp_fec,"%6d %6d %6d %6d    %d  %d\n",cod_m2[i],cod_m1[i],cod_dc[i],cod_p1[i],(ref_nfsk[(i-92)/2]>>1),fec_out[(i-92)/2]); fflush(fp_fec);
//  	}
//  
//  	fclose(fp_fec);
//  #endif



