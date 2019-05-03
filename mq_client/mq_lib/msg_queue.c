/*
 * msg_queue.c
 *
 *  Created on: May 2, 2019
 *      Author: root
 */
#include <unistd.h>
#include <errno.h>
#include "./msg_queue.h"

extern int errno;

int open_queue_server(QUEUE_PACK *qpack, const char *queue_name, int flag, mode_t mode, int max_size_msg, int max_no_msg)
{
	mqd_t mq_id;
	struct mq_attr attr;
	int ret_val;

	attr.mq_maxmsg  = max_no_msg;
	attr.mq_msgsize = max_size_msg;
	attr.mq_flags = O_NONBLOCK ;
	attr.mq_curmsgs = 0;
	mq_id = mq_open(queue_name, flag, 0777, &attr);
	if(mq_id < 0)
	{
		printf("Opening queue for server side is fail\n");
		return -1;
	}

	qpack->mq_id = mq_id;
	qpack->attr  = attr;
	return 0;
}

int open_queue_client(QUEUE_PACK *qpack, const char *queue_name, int flag)
{
	mqd_t mq_id;

	mq_id = mq_open(queue_name, flag);
	if(mq_id < 0)
	{
		printf("Opening queue for client side is fail\n");
		return  -1;
	}
	return 0;
}

int release_queue(QUEUE_PACK *qpack, const char *queue_name)
{
	int ret_val;

	ret_val = mq_close(qpack->mq_id);
	if(ret_val < 0)
	{
		printf("Closing for msg queue is fail\n");
		return -1;
	}

	ret_val = mq_unlink(queue_name);
	if(ret_val < 0)
	{
		printf("Destroy msg queue is fail\n");
		return -1;
	}
	return 0;
}

int write_queue(QUEUE_PACK *qpack, char *buff, int len_buff)
{
	int ret_val;
	int file_d;
	off_t size_file;
	char *file_buff = NULL;
	char *new_file_buff = NULL;

	if(qpack->queue_is_full == true)
	{
		file_d = open("data.txt", O_RDWR, 0777);
		size_file = lseek(file_d, 0, SEEK_END);
		read(file_d, file_buff, size_file);
		sprintf(new_file_buff, "%s\n%s", file_buff, buff);
		qpack->queue_is_full = true;
		close(file_d);
		return 1;
	}
	else
	{
		ret_val = mq_getattr(qpack->mq_id, &(qpack->attr));
		if(ret_val < 0)
		{
			ret_val = errno;
			fprintf(stderr, "Error send msg: %s\n", strerror( ret_val ));
			printf("Fail to get queue msg status\n");
			return -1;
		}
		else
		{
			if(qpack->attr.mq_curmsgs == qpack->attr.mq_maxmsg - 1)
			{
				file_d = open("data.txt", O_RDWR, 0777);
				size_file = lseek(file_d, 0, SEEK_END);
				read(file_d, file_buff, size_file);
				close(file_d);
				sprintf(new_file_buff, "%s\n%s", file_buff, buff);
				qpack->queue_is_full = true;
				return 1;
			}
		}
		ret_val = mq_send(qpack->mq_id, (const char *)buff, (size_t)len_buff, 0);
		if(ret_val < 0)
		{
			printf("Sending msg to queue is fail\n");
			return -1;
		}
		return 0;
	}

}

int read_queue(QUEUE_PACK *qpack, char *buff)
{
	char *tmp_buff;
	int ret_val;

	ret_val = mq_getattr(qpack->mq_id, &(qpack->attr));
	if(ret_val < 0)
	{
		printf("Fail to get queue msg status\n");
		return -1;
	}
	else
	{
		if(qpack->attr.mq_curmsgs == 0)
		{
			printf("Queue is empty\n");
			return 1;
		}
	}

	tmp_buff = malloc(500*sizeof(char));
	memset(tmp_buff, NULL, 500 * sizeof(char));
	ret_val = (int)mq_receive(qpack->mq_id, tmp_buff, qpack->attr.mq_msgsize, 0);
	if(ret_val < 0)
	{
		free(tmp_buff);
		return -1;
	}
	else
	{
		memcpy(buff, (const char *)tmp_buff, (ssize_t)ret_val);
		free(tmp_buff);
		return 0;
	}
}

int queue_was_full(QUEUE_PACK *qpack)
{
	int ret_val;
	int file_d;
	off_t size_file;
	char *file_buff = NULL;
	char *buff = NULL;
	int index;
	int buff_index;
	int current_index;
	ret_val = mq_getattr(qpack->mq_id, &(qpack->attr));
	if(ret_val < 0)
	{
		printf("Fail to get queue msg status\n");
		return -1;
	}
	else
	{
		if(qpack->attr.mq_curmsgs == qpack->attr.mq_maxmsg - 1)
		{
			file_d = open("data.txt", O_RDWR, 0777);
			size_file = lseek(file_d, 0, SEEK_END);
			read(file_d, file_buff, size_file);

			index = 0;
			current_index = 0;
			buff_index = 0;
			while(index <= (int)size_file)
			{
				if(strcmp((const char *)file_buff[index], "\n") == 0)
				{
					current_index = index + 1;
					ret_val = mq_send(qpack->mq_id, (const char *)buff, (size_t)buff_index, 0);
					if(ret_val < 0)
					{
						printf("Sending msg to queue is fail\n");
						return -1;
					}
					buff_index = 0;
					index = current_index;
				}
				buff[buff_index] = file_buff[index];
				index++;
				buff_index++;
			}
			ftruncate(file_d, size_file);
			close(file_d);
			qpack->queue_is_full = true;
			return 0;
		}
	}
	return 1;
}
