/*
 * msg_queue.h
 *
 *  Created on: May 2, 2019
 *      Author: root
 */

#ifndef _MSG_QUEUE_H_
#define _MSG_QUEUE_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>

typedef struct _queue_pack
{
	mqd_t mq_id;
	struct mq_attr attr;
	bool queue_is_full;
	bool queue_is_empty;
}QUEUE_PACK;

int open_queue_server(QUEUE_PACK *qpack, const char *queue_name, int flag, mode_t mode, int max_size_msg, int max_no_msg);
int open_queue_client(QUEUE_PACK *qpack, const char *queue_name, int flag);
int release_queue(QUEUE_PACK *qpack, const char *queue_name);//Using only at server side
int write_queue(QUEUE_PACK *qpack, char *buff, int len_buff);
int read_queue(QUEUE_PACK *qpack, char *buff);
int queue_was_full(QUEUE_PACK *qpack);
#endif /* MQ_LIB_MSG_QUEUE_H_ */
