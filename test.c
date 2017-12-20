void FpgaGetMsgPktFromAU( void ) 
{
	UINT32 i;
	UINT32 tmp;
	static UCHAR8 des_fp, des_re,des_ree;
	static UCHAR8 src_fp, src_re,src_ree; 
	INT16  frame_len;
	UCHAR8 frame_no,frame_no1;
	BOOL   msg_end_flag = b_FALSE;
	UINT32 msg_len;
	UCHAR8 *p_msg_dat = 0;
	
	//TRACE_INFO(" the topo addr of ree(%02X).\r\n", 0x001f&FpgaReadRegister(FPGA_REG_R_REENODE_ST));
	//TRACE_INFO(" the topo addr of re(%02X). \r\n", 0x00ff&FpgaReadRegister(FPGA_REG_R_RENODE_ST));
	//TRACE_INFO(" the topo addr of rec(%02X).\r\n", (0xff00&FpgaReadRegister(FPGA_REG_R_RECNODE_ST))>>8);
	if ( FPGA_LDST_OK != fpga_load_status )
	{
		return;  // FPGA故障，返回 
	}    
	//升级的时候不检查，避免影响到升级速度
	/*if (   sys_work_info&SYSTEM_FLAG_MCU_UPDATE
	    || sys_work_info &SYSTEM_FLAG_FPGA_UPDATE )
	{
	      
		 return;
	}  */

	// 数据包错误计数
	bit_err_cnt += FpgaReadRegister(FPGA_REG_PKT_ERR_COUNT);
	FpgaReadRegister(FPGA_REG_CLEAR_PKT_ERR);
	//TRACE_INFO_WP("fpga rx1\r\n");
	//TRACE_INFO("ree fpga rx re(%02x)\r\n",FpgaReadRegister( FPGA_REG_MSG_RX_FIFO_ST ));
	// 读取数据包FIFO状态,判断光口是否有收到数据包
	if ( 0 != (FpgaReadRegister( FPGA_REG_MSG_RX_FIFO_ST ) & MSG_RX_DAT_FLAG) )
	{
		TRACE_INFO_WP("fpga rx2\r\n");

		// 读取一帧数据
		p_msg_dat = sys_temp_buff;
		
		for ( i=0; i<FPGA_FRAME_FIFO_SIZE; i++ )
		{
			*p_msg_dat++ = (UCHAR8)( 0x00FF & FpgaReadRegister( FPGA_REG_R_MSG_DAT ) );
		}

   		// 读取数据长度
   		frame_len = sys_temp_buff[4];	// FpgaReadRegister( FPGA_REG_R_MSG_DAT );
   		
   		// 读取数据帧编号  
   		frame_no = sys_temp_buff[5];	// FpgaReadRegister( FPGA_REG_R_MSG_DAT );
   		frame_no1 = sys_temp_buff[5];	// FpgaReadRegister( FPGA_REG_R_MSG_DAT );
		
	//	if ( 0 ==(frame_no & MSG_FRAME_INDEX_MASK) )
		{
			
			des_fp  = sys_temp_buff[6]; 
			des_re  = sys_temp_buff[7];
           		 des_ree  = sys_temp_buff[8];
			
			src_fp  = sys_temp_buff[9];
			src_re  = sys_temp_buff[10];
           		src_ree  = sys_temp_buff[11];
		}
		
		
		if ((( 0x0f&src_fp)>FP_MAX )||( src_re>RE_MAX )||(0==frame_len)||(frame_len>FPGA_MSG_BUFF_SIZE)||
				((frame_no&MSG_FRAME_INDEX_MASK)>=MSG_MAX_FRAME_INDEX) )
		{
			TRACE_INFO_WP("FpgaGetMsgPkt Err(%02X:%02X:%02X->%02X:%02X:%02X-%d).", src_fp, src_re,src_ree, des_fp, des_re,des_ree, frame_len);
			// 切换FIFO页
			FpgaReadRegister( FPGA_REG_R_NEXT_MSG );
			return;
		}
		
		TRACE_INFO_WP("FpgaGetMsgPkt(%02X:%02X:%02X->%02X:%02X:%02X,len=%d,frame_num=0x%x).", src_fp, src_re,src_ree, des_fp, des_re,des_ree, frame_len,frame_no);

		frame_len--;	// 长度减1,即为有效数据长度

		//TRACE_INFO_WP("frame_no \r\n",frame_no);
		if ( 0 != (frame_no & MSG_FRAME_END_FLAG) )	// 最末帧
		{
			TRACE_INFO_WP("END_ Pkt \r\n");
			msg_end_flag = b_TRUE;
			
			frame_no &= MSG_FRAME_INDEX_MASK;	// 取得帧编号
			
			// 判断是否是大数据包
			if ( frame_no > 0 )
			{
				// 大数据包，判断RE是否有大缓冲使用权
				tmp = GetReBigMsgBuffIndex( src_fp, src_re,src_ree );
				
				if ( 0 == tmp ) 	// 该RE没有大缓冲的使用权
				{
					
					p_msg_dat = 0;
					// 返回接收端未就绪的应答
					MsgReceiverBusy( src_fp, src_re, src_ree );
					return;
				}
				else		// 存入大数据缓冲
				{
					tmp--;
					p_msg_dat = msg_big_buff[tmp].buff;
					p_msg_dat += (FPGA_MSG_FRAME_LEN+ (frame_no-1) * (FPGA_MSG_FRAME_LEN-FPGA_MSG_ADDR_LEN) );
					msg_len = FPGA_MSG_FRAME_LEN+((frame_no-1) * (FPGA_MSG_FRAME_LEN-FPGA_MSG_ADDR_LEN)) + (frame_len-FPGA_MSG_ADDR_LEN);
					TRACE_INFO_WP("Pkt End\r\n");
				}
			}
			else
			{ 
				// 普通数据包，直接保存并处理
				p_msg_dat = msg_buff;
				msg_len = frame_len;
				TRACE_INFO_WP("small Pkt \r\n");
			}
		}
		else
		{
			TRACE_INFO_WP("Big Pkt \r\n");
			// 不是最后的数据帧
			msg_end_flag = b_FALSE; 
			
			// 数据需要保存到大缓冲中，先判断RE有没有大缓冲的使用权
			tmp = GetReBigMsgBuffIndex( src_fp, src_re, src_ree);
			
			if ( 0 == tmp )  
			{
				// RE没有大数据缓冲的使用权
				tmp = GetFreeBigBuffIndex();

				if ( 0 != tmp )
				{
					TRACE_INFO_WP("new buff[%d] ", tmp);
					tmp--;
					// 大数据缓冲空闲,将其分配给当前RE
					msg_big_buff[tmp].owner = ((src_fp<<16)|(src_re<<8)|src_ree);
					msg_big_buff[tmp].time_out = MSG_BIG_PKT_TIME_OUT;  //5秒钟释放

					// 数据指针指向大数据缓冲
					p_msg_dat = msg_big_buff[tmp].buff;
					p_msg_dat += ( frame_no * FPGA_MSG_FRAME_LEN );
				}
				else
				{
					//TRACE_INFO_WP("no buff ");
					// 大数据缓冲被占用
					p_msg_dat = 0;
					// 返回接收端未就绪的应答
					MsgReceiverBusy( src_fp, src_re,src_ree );
					return;
				}
			}
			else
			{
				TRACE_INFO_WP("buff[%d] ", tmp);
				tmp--;
				msg_big_buff[tmp].time_out = MSG_BIG_PKT_TIME_OUT;
				p_msg_dat = msg_big_buff[tmp].buff;
				if(frame_no>0)
					p_msg_dat += (FPGA_MSG_FRAME_LEN+ (frame_no-1) * (FPGA_MSG_FRAME_LEN-FPGA_MSG_ADDR_LEN) );
			}  
		}
		WTD_CLR;
		
		// 读取数据
		TRACE_INFO_WP("save dat..");
		if ( (frame_no1 & MSG_FRAME_END_FLAG) == 0 )
		{
			if ( (frame_no1 & MSG_FRAME_INDEX_MASK) == 0 )//第一个包
			{
				for ( i=0; i<frame_len; i++ )
				{
					//TRACE_INFO_WP("%d ",*p_msg_dat);
					*p_msg_dat++ = sys_temp_buff[i+6];//去掉包头
				}
			}
			else
			{//中间包
				for ( i=0; i<(frame_len-FPGA_MSG_ADDR_LEN); i++ )
				{
					//TRACE_INFO_WP("%d ",*p_msg_dat);
					*p_msg_dat++ = sys_temp_buff[i+6+FPGA_MSG_ADDR_LEN];//去掉包头
				}
			}
		}
		else
		{//最后一个包
			if ( (frame_no1 & MSG_FRAME_INDEX_MASK) == 0 )//只有一个包
			{
				for ( i=0; i<frame_len; i++ )
				{
					//TRACE_INFO_WP("%d ",*p_msg_dat);
					*p_msg_dat++ = sys_temp_buff[i+6];//去掉包头
				}
			}
			else
			{
				for ( i=0; i<(frame_len-FPGA_MSG_ADDR_LEN); i++ )
				{
					//TRACE_INFO_WP("%d ",*p_msg_dat);
					*p_msg_dat++ = sys_temp_buff[i+6+FPGA_MSG_ADDR_LEN];//去掉包头
				}

			}
		}
		 
		// 切换FIFO页
		FpgaReadRegister( FPGA_REG_R_NEXT_MSG );
		TRACE_INFO_WP("end\r\n");

		// 处理消息包
		if ( b_TRUE == msg_end_flag )
		{
			TRACE_INFO_WP("handle.");
			if ( frame_no>0 )
			{
				tmp = GetReBigMsgBuffIndex( src_fp, src_re, src_ree);
				
				if ( tmp>0 )
				{
					tmp--;
					MsgHandle( msg_len, msg_big_buff[tmp].buff );
					msg_big_buff[tmp].owner = 0;		// 释放大数据缓冲资源
				}
			}
			else
			{
				MsgHandle( msg_len, msg_buff );
#if 0  	
				/****test*********///20121206
			    TRACE_INFO_WP("msg_buff\r\n");
				
				for(i=0;i<msg_len;i++)
				{
              	TRACE_INFO("msg_buff[%d]=%d\n",i,msg_buff[i]); 
				}
				/****test*********///20121206	
#endif
				
			}  
			
			fpga_rx_pkt_cnt++;
		}
	}
}
#else
#endif

#ifdef FPGA_MSG_ADD_ADDRESS
