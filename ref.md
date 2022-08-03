# **一、文件结构**
(a)、主程序
acom_spi_demod_v2.c  
包含的函数结构：uart_init、开启spi的函数、modem_enc函数、storage_release()  
* 其中storage_releas()包含功能最为丰富
---
* 其中storage_releas函数流程  
 1、创建信号量  
 2、创建4个线程：adc,cod,cop,demod_fsk  
 
---
* 信号量用法  
    1、声明信号量 sem_t a;  
    2、利用函数sem_init()生成一个信号量;  
    3、将某一线程在某一阶段用函数sem_wait()阻塞,然后在外部，当完成某一操作
    后，利用sem_post()解除sem_wait()的阻塞，继续执行线程中的功能。
---
**storage_releas()中具体函数:**  
1、write_adc():  
(1)使用了mmap函数，作用是
Linux通过内存映像机制来提供用户程序对内存直接访问的能力。  
使用mmap方式获取磁盘上的文件信息，只需要将磁盘上的数据拷贝至那块共享内存中去，用户进程可以直接获取到信息，而相对于传统的write/read　IO系统调用, 
必须先把数据从磁盘拷贝至到内核缓冲区中(页缓冲)，然后再把数据拷贝至用户进程中。
两者相比，mmap会少一次拷贝数据，这样带来的性能提升是巨大的。  
thread_modem_enc中具体函数  
1、acom_phy_snd_one_frame(int spi_fd,int sym_cnt,unsigned char *data)  

* `data中包含要发送的数据`
* 1.1 定义传输数组为324字节
* 1.2 发送4字节指令为3b 000000:cmd_fsk_op_end  
* 1.3 发送4字节指令为3b 000011: cmd_fsk_wr_bgn  
* 1.4 发送4字节指令以及数据  
   
2、acom_enc_8fsk(str_Macinfo *macinfo,int *sym_cnt,unsigned char *enc_out)  
* 此函数会将sym_cnt数值改变=384,然后
     

---  

# 杂项
* `1、adc的数据是存入到文件中？`
* `2、带有write的函数都是往文件中写数据？`
* `3、spi一次读入512个char`  
* `4、spi给fpga发送IQ数据的时候，是先发4字节I路数据，再发4字节Q路数据，IQ路数据交替发送`  


# **二、接收端处理流程**

1、用spi传一个copd_thresh给fpga，数据长度4字节  
spi传输流程为  

    spi_buf.cnt = 4;  //可以通过这个变量控制spi发送的字节数
    spi_buf.buffer_read 	= recv_buf;  
    spi_buf.buffer_write 	= copd_thresh;  
    ioctl(fd, 0, &spi_buf);  //发送与接收同时完成
2、用spi传一个send_buf_cont1给fpga，数据长度4字节，用来复位ram？  
3、用spi传一个send_buf_contn给fpga，数据长度512字节，通知fpga，接下来连续读取数据   
4、进入while循环  

    1、执行ioctl，进行一次spi数据传输，传输的数据量为512字节  
    2、进入while循环，遍历512字节数据，取出帧头数据与帧尾数据
        -2.1 判断帧头数据，不同帧头数据处理方式不同
        -2.2 帧头为0b00000000，则跳出switch，然后cnt+=MESSAGE_LENGTH(4)  
           表明一帧数据为4字节，且第一字节为帧头表示数据的类型  
        -2.3 帧头为0b10000000，则跳出switch，然后cnt+=MESSAGE_LENGTH(4)  
        -2.4 帧头为0b00010000，表示数据为adc输入，然后每4字节写入adc_buf,当  
            写满4096字节，也就是一页数据，将数据写入adc_buf_tmp,然后用信号mutex_adc  
            通知adc_write线程，写入到文件中
        -2.5 帧头为0b01000000，则得到相关器（-数据I-），对条件(frame_tail !=0 ) && cod_store_ena == 0  
            进行判断(LSB不为0代表有相关峰值出现, cod_store_ena == 0说明到来的是新的相关峰)  
            如果满足上述条件，则说明新的相关峰到来，然后需要重新设置一些东西，然后存储数据，  
            如果不满足，则直接将接收到4字节数据存入到cod_buf_I，然后跳出switch
        -2.6 帧头为0b01010000，则得到相关器数据Q，接收到4字节数据存入到cod_buf_Q,  
            然后将cod_buf_I，和cod_buf_Q中的数据依次存入到cod_buf中，如果存满一页，  
            则存入到文件cod_frm中，然后会通过信号mutex_dem通知解调进程demod_fsk开始执行  
            其中demod_fsk的输入数据为cod_frm中的数据，且cod_frm中的数据为降采样到8k的数据
         -2.7 帧头为0b01100000，则得到相关器（-峰值I-），然后需要进行峰值搜索，此时的一个数据帧  
            的结构为想：|帧头（数据类型）|高位相关峰值|低位相关峰值|峰值时隙(slot)| 
            
      
 5、解调进程demod_fsk
        
# **三、总流程**
1、打开串口线程  
2、打开调制线程  
--- 
* 调制线程（thread_modem_enc函数）  
1、定义str_Macinfo macinfo;  
2、进入while(1)循环  
     ``` 
     2.1 声明一个4字节的spi数据帧，然后ioctl发送给fpga，进行检查FPGA 发送等待状态  
     2.2 检查fpga通过spi给cpu的数据，如果满足recv_buf_ref[1] == 0，  
        则表示FPGA空闲，可以进行数据的发送  
     2.3  
  ```
3、  
                                                     