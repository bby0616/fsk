

//fsk解调线程函数

/*
根据收集到的cod数据，先后调用fsk解调、解码

*/
void* demod_fsk(void* arg){
	int i,j;
	float cop_max_tim_p1 ;
	float cop_max_tim_m1 ;

    while(1){
        sem_wait(&mutex_dem);
		cop_max_tim =  cop_max_slt + (float)(((unsigned int)cop_frm[cop_max_idx*8+5]*256+(unsigned int)cop_frm[cop_max_idx*8+6])*256+(unsigned int)cop_frm[cop_max_idx*8+7])/10000000;


		printf(        "Peak: %6d @ %-3d  TOA: %9.6f  ",cop_max_val,cop_max_idx,cop_max_tim);
		fprintf(fp_log,"Peak: %6d @ %-3d  TOA: %9.6f  ",cop_max_val,cop_max_idx,cop_max_tim);

		//截取cod数据
		int cod_start;
		int cod_tmp;
		//相关检测模块故意引入128拍延迟
		cod_start = cop_max_idx*4*2 + (128-8)*2*4;
		int end_addr;
		int max_addr;
		end_addr = cod_start+COD_STORE_LEN*8;

		//地址越界保护
		max_addr = (end_addr  > MAX_PAGE_SIZE*PAGES_COD_8KFILE ) ? MAX_PAGE_SIZE*PAGES_COD_8KFILE : end_addr;

		//for (i = cod_start;i<MAX_PAGE_SIZE*PAGES_COD_8KFILE;i=i+4){
		//for (i = cod_start;i<cod_start+COD_STORE_LEN*8;i=i+8){

			//printf("\nmax_addr = %d cod_start = %d\n====================================\n",max_addr,cod_start);
		for (i = cod_start;i<max_addr;i=i+8){
			//赋值给short型变量(2byte)，自动转有符号数
			//这里除8是为了后面平滑的时候不会溢出
			cod_i[(i-cod_start)/8] = ((short)(cod_frm[i+1]*256+cod_frm[i+2])/8);
			cod_q[(i-cod_start)/8] = ((short)(cod_frm[i+5]*256+cod_frm[i+6])/8);

			//if((i-cod_start)/8 < 320) {
			//	printf("%04x ",cod_i[(i-cod_start)/8] & 0x0000ffff);
			//	if((i-cod_start)/8 % 32 == 31) printf("\n");
			//}
		}
			//printf("\n====================================\n");

		str_Macinfo macinfo;
		int	err_cnt;
		int fec_len;

  		int soft_info_8fsk[ACOM_SYM_CNT][8];
  		//int soft_info_4fsk[ACOM_SYM_CNT][4];
  		int soft_info_4fsk[ACOM_SYM_CNT][8];
  		unsigned char 	hard_deci[ACOM_SYM_CNT];
		str_FskInfo	fskinfo;
		//fsk解调
		//acom_phy_8fsk_demod(cod_i,cod_q,2,5,&fskinfo,&soft_info_8fsk[0][0],&hard_deci[0]);
		acom_phy_8fsk_demod(cod_i,cod_q,0,7,&fskinfo,&soft_info_8fsk[0][0],&hard_deci[0]);

		//应判决错误统计
  		int sym_err_cnt;
  		sym_err_cnt = 0;
  		for (i = 0;i<(ACOM_SYM_CNT);i++){
  			sym_err_cnt += (hard_deci[i] != ref_nfsk[i]);	
  		}
		//-------------------------------------------------------
		//fec译码
		//拷贝到4fsk软信息ram
//		for(j=0;j<ACOM_SYM_CNT;j++){
//			printf("%5d %12d\n",j,hard_deci[j]);
//		}

		//for(i=2;i<=5;i++){
		for(i=0;i<=7;i++){
			for(j=0;j<ACOM_SYM_CNT;j++){
			soft_info_4fsk[j][i-2] = soft_info_8fsk[j][i];
			//printf("%d ",soft_info_8fsk[j][i]);
			}
			//printf("\n");
		}
  		//acom_fec_4fsk_v2_256(soft_info_4fsk,ACOM_SYM_CNT,&macinfo);	//frm_len = 128sym + 512sym
#ifdef USE_8FSK_1_2
  		acom_fec_8fsk_half(soft_info_8fsk,ACOM_SYM_CNT,&macinfo);	//frm_len = 256sym 
#else
  		acom_fec_8fsk(soft_info_8fsk,ACOM_SYM_CNT,&macinfo);	//frm_len = 384sym 
#endif

		 //printf("\nfec-dout:"); for(i = 0;i<22;i++) { printf("%02x ",macinfo.data[i]); } printf("\n\n");


		int sym_err_cnt_fec = -1;

  		printf(        "sym_err: %3d  pow: %9.2f echo:[ %5.2f %5.2f %5.2f %5.2f %5.2f ] %3d\n",sym_err_cnt,fskinfo.pow_chirp,fskinfo.pow_nep1*100/fskinfo.pow_chirp,fskinfo.pow_nep2*100/fskinfo.pow_chirp,fskinfo.pow_nep3*100/fskinfo.pow_chirp,fskinfo.pow_nep4*100/fskinfo.pow_chirp,fskinfo.pow_nep5*100/fskinfo.pow_chirp,sym_err_cnt_fec);
  		fprintf(fp_log,"sym_err: %3d  pow: %9.2f echo:[ %5.2f %5.2f %5.2f %5.2f %5.2f ] %3d\n",sym_err_cnt,fskinfo.pow_chirp,fskinfo.pow_nep1*100/fskinfo.pow_chirp,fskinfo.pow_nep2*100/fskinfo.pow_chirp,fskinfo.pow_nep3*100/fskinfo.pow_chirp,fskinfo.pow_nep4*100/fskinfo.pow_chirp,fskinfo.pow_nep5*100/fskinfo.pow_chirp,sym_err_cnt_fec); fflush(fp_log);

		//写软信息文件
  		#ifdef PRINT_SOFT_INFO
  			for(i = 0;i<(ACOM_SYM_CNT);i++){
  				fprintf(fp_fec,"%6d %6d %6d %6d %6d %6d %6d %6d    %d\n",\
								soft_info_8fsk[i][0],soft_info_8fsk[i][1],soft_info_8fsk[i][2],soft_info_8fsk[i][3],\
								soft_info_8fsk[i][4],soft_info_8fsk[i][5],soft_info_8fsk[i][6],soft_info_8fsk[i][7],\
								(ref_nfsk[i]>>1)); fflush(fp_fec);
  				//fprintf(fp_fec,"%6d %6d %6d %6d  : %6d %6d %6d %6d : %d\n",	soft_info_8fsk[i][2],soft_info_8fsk[i][3],soft_info_8fsk[i][4],soft_info_8fsk[i][5],\
				//															soft_info_4fsk[i][0],soft_info_4fsk[i][1],soft_info_4fsk[i][2],soft_info_4fsk[i][3],\
				//															(ref_nfsk[i]>>1)); fflush(fp_fec);
  			}
  			fclose(fp_fec);
  		#endif


		//最后一个字符设为
		char  mac_data[65];
		char  uart_msg[65];
		bzero(mac_data,65);//全部填充串尾符
		//memcpy(mac_data,macinfo.data,64);
		strcpy(mac_data,macinfo.data);
		//macinfo.data[63]=0;

		if((macinfo.src == dev_id)&&(macinfo.via != 0)){
			bzero(uart_msg,65);
			sprintf(uart_msg,"\nMessage repeated by %x. ",macinfo.via);
    		write(fd_uart,uart_msg,strlen(uart_msg));
    		//write(fd_uart,macinfo.data,strlen(macinfo.data));
		}

	if(macinfo.op_code== 1){
		if (macinfo.dst == dev_id){
	 	 printf(       "DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ======: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		fprintf(fp_log,"DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ======: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);

		 printf(       "\n");
		fprintf(fp_log,"\n");fflush(fp_log);
		//信息到达信宿，不再转发，进入下一个循环

		//strcpy(uart_msg,"\nMessage (correct): ");
		//uart_msg = "\nMessage (correct): ";
		//uart_msg = "\nMessage ( error ): ";
    	//write(fd_uart,"\nErr-free message   : ",20);
		bzero(uart_msg,65);
		sprintf(uart_msg,"\nMessage [%x -> %x -> %x]: ",macinfo.src,macinfo.via,macinfo.dst);
    	write(fd_uart,uart_msg,strlen(uart_msg));
    	write(fd_uart,macinfo.data,strlen(macinfo.data));

		continue;
		}                                                                                                                                            
		else if(macinfo.src == dev_id){
	 	 printf(       "DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ======: %s",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		fprintf(fp_log,"DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ======: %s",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		 printf(       " | Repeat of my transmit received! Ignore ... \n");
		fprintf(fp_log," | Repeat of my transmit received! Ignore ... \n");

		 printf(       "\n");
		fprintf(fp_log,"\n");fflush(fp_log);



		//收到别人repeat的数据,丢弃数据进入下一个循环
		continue;
		}
		else if(macinfo.dst == 0x0f){
	 	 printf(       "DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ......: %s",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		fprintf(fp_log,"DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | ......: %s",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		 printf(       " | Broadcast Infomation ! Ignore ... \n");
		fprintf(fp_log," | Broadcast Infomation ! Ignore ... \n");

		 printf(       "\n");
		fprintf(fp_log,"\n");fflush(fp_log);

		bzero(uart_msg,65);
		sprintf(uart_msg,"\nMessage [%x Broadcast]: ",macinfo.src);
    	write(fd_uart,uart_msg,strlen(uart_msg));
    	write(fd_uart,macinfo.data,strlen(macinfo.data));

		 //printf(       "cor-data:"); for(i = 0;i<22;i++) { printf(       "%02x ",macinfo.data[i]); } printf(       "\n\n");

		//收到广播的数据,丢弃数据进入下一个循环
		continue;
		}
		else{                                                                                                                                        
	 	 printf(       "DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | >>>>>>: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		fprintf(fp_log,"DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | >>>>>>: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);

		 printf(       "\n");
		fprintf(fp_log,"\n");fflush(fp_log);

		}

		//fprintf(fp_log,"cor-data:"); for(i = 0;i<22;i++) { fprintf(fp_log,"%02x ",macinfo.data[i]); } fprintf(fp_log,"\n\n");

	}
	else{
		 printf(       "DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | xxxxxx: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);
		fprintf(fp_log,"DevId: %x |OpCode: %x | %x -> %x -> %x |ttl: %x | xxxxxx: %s\n",dev_id,macinfo.op_code,macinfo.src,macinfo.via,macinfo.dst,macinfo.ttl,mac_data);


		////打印错误的数据帧内容
		//fprintf(fp_log,"err-data:");
		//for(i = 0;i<23;i++) {
		//	fprintf(fp_log,"%02x",macinfo.data[i]);
		//}

		 printf(       "\n");
		fprintf(fp_log,"\n");fflush(fp_log);


		
		if(macinfo.dst == dev_id){
			bzero(uart_msg,65);
			sprintf(uart_msg,"\nMessage (%x -> %x -> %x): ",macinfo.src,macinfo.via,macinfo.dst);
    		write(fd_uart,uart_msg,strlen(uart_msg));
    		write(fd_uart,macinfo.data,strlen(macinfo.data));
			//strcpy(uart_msg,"\nMessage ( error ): ");
    		//write(fd_uart,"\nMessage with error: ",21);
		}

		//数据包丢弃后,放弃后续操作进入下一个循环
		continue;
	}
	
		//重新编码
		unsigned char enc_out[ACOM_SYM_CNT/2];
		//bzero(enc_out,sizeof(enc_out));
		macinfo.via = dev_id;			//中继地址替换
		macinfo.ttl = macinfo.ttl - 1;	//ttl -1


		if(macinfo.ttl >=  0 ){
			fec_len = ACOM_SYM_CNT;
			//acom_enc_4fsk_v2_256(&macinfo,&fec_len,enc_out);
#ifdef USE_8FSK_1_2
			acom_enc_8fsk_half(&macinfo,&fec_len,enc_out);
#else
			acom_enc_8fsk(&macinfo,&fec_len,enc_out);
#endif
			acom_phy_snd_one_frame(fd,ACOM_SYM_CNT,enc_out);
			//for(i = 0;i<ACOM_SYM_CNT/2;i++) printf("%02x ",enc_out[i]); printf("\n");
		} else{

			printf("TTL == 0. Drop!\n");
			continue;
		}


		//处理完后，为下次接收做准备
//		cop_max_idx = 0;
//		cop_cnt = 0;
		n_dem--; if(n_dem == 0) { exit(0);}
	}
}

