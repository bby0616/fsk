

void* uart_comm(void* arg){

	while(1){

    	memset(uart_sendmsg,0,sizeof(uart_sendmsg));
    	sprintf(uart_sendmsg,"\nacom_modem ready. Input message:\n");
    	write(fd_uart,uart_sendmsg,strlen(uart_sendmsg));

		uart_nbytes = 0;

    	while(1){
    	    memset(uart_recvmsg,0,sizeof(uart_recvmsg));
    	    uart_nbytes=recvdata(fd_uart,uart_recvmsg,sizeof(uart_recvmsg));
    	    if (uart_nbytes >0) 
    	    {   

				//应用程序打印信息
    	        printf("\nUART: get %d bytes:%s, len %d,\n",uart_nbytes,uart_recvmsg,strlen(uart_recvmsg));

				//uart数据准备好标识置1
				uart_data_ready = 1;
				
				//发送反馈给人机接口:
    	        memset(uart_sendmsg,0,sizeof(uart_sendmsg));
    	        sprintf(uart_sendmsg,"acom_modem sending :%s\n",uart_recvmsg);
    	        write(fd_uart,uart_sendmsg,strlen(uart_sendmsg));

				break;	//接到数据退出读取uart循环,阻塞线程
    	    }   
    	} //读取uart循环  

        sem_wait(&mutex_uart);

	}//线程阻塞循环

}


