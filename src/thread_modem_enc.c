
void* modem_enc(void* arg){
//每秒读取一次fpga的发送等待状态，如果没有待发送数据，生成本地测试数据（读取串口数据）编码后发送。
//需要注意和转发互斥，避免同时访问spi。

		//编码
		int i;
		int	fec_len;
//		unsigned char info[64];		//编码前数据
		unsigned char enc_out[ACOM_SYM_CNT/2];	//编码后数据
// recv
		unsigned char enc_chk[ACOM_SYM_CNT/2];	//编码后数据

		str_Macinfo macinfo;

		macinfo.src  = dev_id;	
		macinfo.dst  = 0x0f;		//广播地址. 如果冲串口读数据，数据中应该包含目的地址。
		macinfo.via  = 0;			//中继做信源的时候，中继地址填0
		macinfo.ttl  = 2;	
//		macinfo.data = info;	

		while(1){
			//检查FPGA 发送等待状态
	    	unsigned char  send_buf_ref[4];
	    	unsigned char  recv_buf_ref[4];
	    	unsigned char  uart_snd_msg[65];
	    	struct spi_buffer spi_ref;                                                                                                                                                                                     
	    	spi_ref.buffer_read  = recv_buf_ref;
	    	spi_ref.buffer_write = send_buf_ref;
	    	spi_ref.cnt          = 4;

	    	send_buf_ref[0] =  0x7e;
	    	send_buf_ref[1] =  0x00;
	    	send_buf_ref[2] =  0x00;
	    	send_buf_ref[3] =  0x00;
	    	ioctl(fd, 0, &spi_ref);
//			printf("0x3e: %02x_%02x_%02x_%02x\n",recv_buf_ref[0],recv_buf_ref[1],recv_buf_ref[2],recv_buf_ref[3]);
	    	if(recv_buf_ref[1] == 0 ){//FPGA空闲，允许发送
					//printf("FPGA clear to send.\n");
				//先检查串口 是否有接收到数据
					bzero(macinfo.data,64);

				if (uart_data_ready){
					macinfo.src  = dev_id			 ;//ttl暂时设2
					macinfo.via  = 0				 ;//中继地址暂时设0
					macinfo.dst  = uart_recvmsg[0]-48;//目的地址. 
					macinfo.ttl  = 2				 ;//ttl暂时设2

					memcpy(&macinfo.data[0],uart_recvmsg,64*sizeof(char));
					uart_data_ready   = 0;	//旗标清零
                	sem_post(&mutex_uart);	//解除阻塞
					printf("UART data sent to acom phy.\n");

//					acom_enc_4fsk(&macinfo,&fec_len,enc_out);
#ifdef USE_8FSK_1_2
					acom_enc_8fsk_half(&macinfo,&fec_len,enc_out);
#else
					acom_enc_8fsk(&macinfo,&fec_len,enc_out);
#endif
					acom_phy_snd_one_frame(fd, fec_len,enc_out);
				}
				else if(beat_snd){

					//printf("FPGA clear to send ...\n");
					//如果空闲，生成数据，并发送


					//获取系统时间
					time_t t;
    				struct tm *cur_time;
    				time(&t);
    				cur_time = localtime(&t);


					macinfo.ttl  = 2	;//心跳全网广播，ttl设2

					if(beat_snd > 10)
						macinfo.dst  = (beat_snd - 10) & 0x0f;//定向心跳。
					else
						macinfo.dst  = 0x0f	;//广播地址. 如果冲串口读数据，数据中应该包含目的地址。

					unsigned char cur_time_string[22];

					//填充0的话，如果数据不加扰码，会对性能有影响，这里还是填充可见字符。
					for(i=0;i<23;i++) 
						macinfo.data[i] = 0;
						//macinfo.data[i] = 48+i;
					//macinfo.data[63] = 0;

					bzero(cur_time_string,22);	

					if(beat_snd == 1){
						strcpy(&macinfo.data[0],"Heartbeat @ 2021-08-24 08:00:00");
					}
					else if(beat_snd >10 ){
					  	strcpy(&macinfo.data[0],"> ");
    				  	strftime(cur_time_string, 23, "%Y-%m-%d %H:%M:%S", cur_time);
					  	strcpy(&macinfo.data[2],cur_time_string);
						macinfo.data[21] = 0;

						//发送回显
						bzero(uart_snd_msg,65);
					  	sprintf(uart_snd_msg,"Auto-send to %x : ",macinfo.dst);
    	        		write(fd_uart,uart_snd_msg,strlen(uart_snd_msg));
    	        		write(fd_uart,macinfo.data,strlen(macinfo.data));
	
					}
					else {
					//	for(i=0;i<22;i++) macinfo.data[i] = (beat_start + i)%64 + 48;
					//	beat_start = (beat_start +1)%64;

					  //memcpy(&macinfo.data[0],"Heartbeat @ ",12*sizeof(char));
					  strcpy(&macinfo.data[0],"@ ");
    				  strftime(cur_time_string, 23, "%Y-%m-%d %H:%M:%S", cur_time);
					  //memcpy(&macinfo.data[12],cur_time_string,22*sizeof(char));
					  	strcpy(&macinfo.data[2],cur_time_string);
						macinfo.data[21] = 0;


					}
					//strcpy(&macinfo.data[0],"Heartbeat @ 2021-08-24 02:37:07");
                    macinfo.data[21] = 0; 
					printf("Heartbeat sent to acom phy: %s\n",macinfo.data);
		 			printf("send-data:"); for(i = 0;i<22;i++) { printf("%02x ",macinfo.data[i]); } printf("\n\n");


//					acom_enc_4fsk(&macinfo,&fec_len,enc_out);
#ifdef USE_8FSK_1_2
					acom_enc_8fsk_half(&macinfo,&fec_len,enc_out);
#else
					acom_enc_8fsk(&macinfo,&fec_len,enc_out);
#endif
					//printf("fec_len: %d\n",fec_len);
						//printf("===============================================\n");
					//for(i=0;i<fec_len/2;i++) {
					//	printf("%02x ",enc_out[i]);
					//	if(i%32==31) printf("\n");
					//}
					acom_phy_snd_one_frame(fd, fec_len,enc_out);
					//recv
					acom_phy_rcv_one_frame(fd, fec_len,enc_chk);
						//printf("-----------------------------------------------\n");
					//for(i=0;i<fec_len/2;i++) {
					//	printf("%02x ",enc_chk[i]);
					//	if(i%32==31) printf("\n");
					//}
					int check_sum;
					check_sum = 0;
						//printf("===============================================\n");
					//for(i=0;i<fec_len/2;i++) { check_sum+= (enc_chk[i] != enc_out[i]);}
					//	printf("FPGA_download check mismatch count: %4d \n",check_sum);
				}


			}
//			else
//				printf("send command pending...\n");
			
			sleep(1.5);
		}


}



