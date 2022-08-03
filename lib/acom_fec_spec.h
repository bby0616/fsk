#ifndef __ACOM_FEC__
#define __ACOM_FEC__

typedef struct mac_info
{
	unsigned char dst		;	//目标地址
    unsigned char src		;	//源地址
	unsigned char via		;	//转发节点地址
	unsigned char ttl		;	//ttl
	unsigned char op_code	;	//反馈给调用者的operation code
	unsigned char data[64]	;	//payload存放空间
	unsigned char valid_len	;	//有效数据长度
								//其他结构成员视需要进一步扩展
}__attribute__((__packed__)) str_Macinfo;


//可以合并修改成如下接口:
void acom_fec_dec(int **soft_in,int sym_cnt,int nfsk,str_Macinfo *macinfo);
/*
soft_in : 为指向2维数组的指针，数据组织方式为sym_cnt行，nfsk列
sym_cnt : 符号个数,目前信道帧本身目前并不携带该信息，需要应用程序传参获取,或者define为固定值。
nfsk    : fsk子载波个数

macinfo.valid_len 解码后的数据可能只有部分有效，解码程序需要在这里给出有效数据长度

*/

void acom_fec_enc(str_Macinfo *macinfo,int *sym_cnt,unsigned char *enc_out);
/*
sym_cnt : 反馈给调用程序的编码后待发送数据长度
enc_out : 反馈给调用程序的编码后待发送数据

macinfo.valid_len 如果待编码数据来自解码结果，在变长码的情况下，该参数将影响最终编码输出的长度，编码后待发送数据写给sym_cnt
*/

#endif
