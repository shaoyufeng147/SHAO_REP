/*
 * UART.c
 *
 *  Created on: 2019-4-18
 *      Author: SHYF
 */
#include "UART.h"
#include <string.h>

REV_DATA rev_data;

void Init_revdata(REV_DATA *rev)
{
	rev->rev_Flag = NO_REV_FRAM;
	rev->rev_p = 0;
	memset(rev->rev_buff,0,sizeof(rev->rev_buff));
}

void uart_rdataproc(REV_DATA* dst,char *src)
{
	int check_sum = 0;
	switch(dst->rev_Flag)
	{
		case NO_REV_FRAM :
		{
			if(dst->rev_p == 0)
			{
				if(*src != 0xEB) return;
				dst->rev_buff[dst->rev_p++] = *src;
			}
			else if(dst->rev_p == 1)
			{
				if(*src != 0x90)
				{
					Init_revdata(dst);
					return;
				}
				dst->rev_buff[dst->rev_p++] = *src;
			}
			else
			{
				dst->rev_buff[dst->rev_p++] = *src;
				if(dst->rev_p ==7)  //帧头接收完成
				{
					int i;
					for(i=0;i<7;i++)
						check_sum += dst->rev_buff[i];
					if(check_sum&0x000000FF != dst->rev_buff[6])  //验证校验和
					{
						Init_revdata(dst);
						return;
					}
					dst->rev_Flag = REV_FRAM_HEAD;
				}

			}
			return;
		}
		case REV_FRAM_HEAD:
		{
			if(dst->rev_p < dst->rev_buff[2]+1)	//数据接收未完成
				dst->rev_buff[dst->rev_p++] = *src;
			if(dst->rev_p == dst->rev_buff[2]+1)//接收完成
				dst->rev_Flag = REV_FRAM_ALL;
			return;
		}
	}
}

int Fram_Make (char *send_buff,char *data_buff,char dstID,char data_len)
{
	*(send_buff+0) = 0xEB;
	*(send_buff+1) = 0x90;
	*(send_buff+2) = data_len+7-1;//帧长
	*(send_buff+3) = 0x00;		//帧ID根据data_len修改，先暂定为0x00
	*(send_buff+4) = 0x03;		//源ID
	*(send_buff+5) = dstID;		//目的ID
	int temp=0,i;
	for(i=0;i<6;i++)
		temp += *(send_buff+i);
	*(send_buff+6) = temp&0x000000FF;
	memcpy(send_buff+7,data_buff,data_len);
	return 7+data_len;
}












