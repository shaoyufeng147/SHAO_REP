/*
 * UART.h
 *
 *  Created on: 2019-4-18
 *      Author: SHYF
 */

#ifndef UART_H_
#define UART_H_


typedef enum {
	NO_REV_FRAM =0x00,
	REV_FRAM_HEAD = 0x01,
	REV_FRAM_ALL =0x02
}REV_FLAG;

typedef struct {
	char rev_buff[512];
	char rev_Flag;//���ձ�־λ
	char rev_p;//����ָ�루��ǰ����λ�ã�
}REV_DATA;


extern REV_DATA rev_data;


void uart_rdataproc(REV_DATA* dst,char *src);
void Init_revdata(REV_DATA *rev);
int  Fram_Make (char *send_buff,char *data_buff,char dstID,char data_len);

#endif /* UART_H_ */
