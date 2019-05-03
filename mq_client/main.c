/*
 * main.c
 *
 *  Created on: May 1, 2019
 *      Author: root
 */

/*
 * main.c
 *
 *  Created on: May 1, 2019
 *      Author: root
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <msg_queue.h>

//typedef struct _buff_content
//{
//	char routing_key[50];
//	char buff[500];
//}BUFF_CONTENT;
/*Define Routing*/
#define UART_COMMAND  	"/uart.command"
#define UART_INFOR	  	"/uart.infor"
#define UART_CHANGE		"/uart.change"
#define UART_GATEWAY	"/uart.gateway"
#define SERVER_COMMAND	"/server.command"
#define SERVER_INFOR	"/server.infor"
#define SERVER_CHANGE	"/server.change"
#define SERVER_GATEWAY	"/server.gateway"
/*No Routing*/
#define ROUTING_MAX		8

QUEUE_PACK mq_client[ROUTING_MAX];
const char *queue_name[ROUTING_MAX]= 	{ 	UART_COMMAND,
											UART_INFOR,
											UART_CHANGE,
											UART_GATEWAY,
											SERVER_COMMAND,
											SERVER_INFOR,
											SERVER_CHANGE,
											SERVER_GATEWAY	};//this is routing key

void delay(uint64_t delay_time);


int main(int argc, char **argv)
{
	char buff[1000];
	int  len_buff;
	int ret_val;
	int index;

	for(index = 0; index < ROUTING_MAX; index++)
	{
		ret_val = open_queue_server(&mq_client[index], queue_name[index] \
									, O_CREAT | O_RDONLY | O_NONBLOCK, 0777, 1024, 100);
		if(ret_val < 0)
		{
			printf("Fail to open queue msg for %s routing key\n", queue_name[index]);
			return -1;
		}
	}



	while(1)
	{
		for(index = 0; index < ROUTING_MAX; index++)
		{
			sprintf(buff, "{type: DEVICE_INFO,payload: {platform: RF|BLE|LORA|...,version: \
							 version of the device,mac_address: mac_address of the device,local_id: This is node_id of the of the device,device_type: \
							 STREETLIGHT|HUMIDITY|H2|CO2|CO|MACHINE|...,location: { lat: 0.000,long: 0.000,},\
							 settings: {min_power: 0,max_power: 50,time_dimer: 2,groups: [ group_id_1, group_id_2, ...],...},is_online: true|false, this is status of the device,\
							 state_update_duration: Number of seconds the devices should auto send state update to API,\
							 meta_data: {is_driver_error: false,temperature: 25.5,humidity: 40.5,dimmer: 0-255, it is depending sensor,}},\
							 time: timestamp when the message is created}");
			len_buff = (int)strlen((const char *)buff);
			ret_val = write_queue(&mq_client[ROUTING_MAX], buff, len_buff);
			if(ret_val == 0)
			{
				printf("Send Package \n");
			}
		}
		delay(1000);

	}

	for(index = 0; index < ROUTING_MAX; index++)
	{
		ret_val = release_queue(&mq_client[index], queue_name[index]);
		if(ret_val < 0)
		{
			return -1;
		}
	}

	printf("Close queue\n");
	return 0;
}


void delay(uint64_t delay_time)
{
	int milli_seconds = 1000 * delay_time;

	// Stroing start time
	clock_t start_time = clock();

	// looping till required time is not acheived
	while (clock() < start_time + milli_seconds);
}




