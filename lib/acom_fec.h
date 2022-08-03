#ifndef __ACOM_FEC__
#define __ACOM_FEC__

typedef struct mac_hdr
{
	unsigned short dst_addr;
	unsigned short src_addr;
	unsigned short via_addr;
	unsigned short crc16;
	unsigned short ttl;
}__attribute__((__packed__)) str_Header;

typedef struct mac_packet
{
	str_Header hdr_mac;
	unsigned char payload_data[70];
}__attribute__((__packed__)) str_Packet;

typedef struct mac_info
{
	unsigned char dst;
    unsigned char src;
	unsigned char via;
	unsigned char ttl;
	unsigned char op_code;
	unsigned char data[65];
	int hdrlength;
	int dtlength;
}str_Macinfo;

//外部接口函数

/* 8fsk 1/3 rate */

void acom_enc_8fsk(str_Macinfo *macinfo,int *sym_cnt,unsigned char *enc_out);

void acom_fec_8fsk(int (*soft_in)[8],int sym_cnt,str_Macinfo *macinfo);

/* 4fsk */

void acom_enc_4fsk(str_Macinfo *macinfo,int *sym_cnt,unsigned char *enc_out);

void acom_fec_4fsk(int (*soft_in)[4],int sym_cnt,str_Macinfo *macinfo);

/* 8fsk 1/2 rate */

void acom_enc_8fsk_half(str_Macinfo *macinfo,int *sym_cnt,unsigned char *enc_out);

void acom_fec_8fsk_half(int (*soft_in)[8],int sym_cnt,str_Macinfo *macinfo);



//内部函数
void logmap(int length,double* Le,double *Lout,double *x_n,double *p_n);

void logmap_4fsk(int length,double* Le,double *Lout,int (*y_n)[4]);

void descramble(unsigned char *infobits,unsigned char *scrambler,unsigned char *originalbits,int bit_num);

void RSC(int bit_cnt,unsigned char* codein,unsigned char* codeout);

void permutation(int* a,int length);

void scramble(str_Packet *pkt,unsigned char *scrambler, unsigned char *infobits,int bit_num);

void readcfg(void);

#endif
