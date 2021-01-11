#ifndef _NRF24_NET_H_
#define _NRF24_NET_H_

#include "nRF24L01.h"

#define FIFO_SIZE			65536
#define SKB_BUFFER_SIZE     1024

typedef struct nrf24_pipe_cfg {
	uint8_t			address[5];
	uint8_t			ack;
	ssize_t			plw;
};

typedef struct nrf24_pipe {
	int			            id;
	struct nrf24_pipe_cfg	cfg;

	uint32_t			    sent;
	bool			        write_done;
};

struct nrf24_device_cfg {
	uint16_t    data_rate;
	uint8_t		crc;
	uint8_t		retr_count;
	uint16_t	retr_delay;
	uint8_t		rf_power;
	uint8_t		address_width;
};

struct nrf24_tx_data {
	struct nrf24_pipe	pipe[6];
	uint8_t			    size;
	uint8_t			    pload[PLOAD_MAX];
};

struct nrf24_device {
    struct net_device *net_dev;
    struct skb_buff   *rx_buff;
    struct skb_buff   *tx_buff;
    
	struct device		dev;
	struct spi_device	*spi;

	struct gpio_desc	*ce; // control ce pin

	struct nrf24_device_cfg	cfg; // config device

	/* for irqsave */
	spinlock_t		    lock;

	struct work_struct	isr_work;//work struct for interrupt
	struct work_struct	rx_work;//work struct when receive packet from module
    struct work_struct  tx_work;// work struct when receive new packet when to transmit

	/* tx */
	struct mutex		tx_skb_mutex;//because, we will create tx thread 
	wait_queue_head_t	tx_wait_queue;//help to make start change module to TX
	wait_queue_head_t	tx_done_wait_queue;// help to know module transmit packet finish
	atomic_t			tx_done; 
	atomic_t			tx_failed;

	/* rx */
	struct timer_list	rx_active_timer;// time to run module in RX mode
	atomic_t			rx_active; //the flag help to know system is active

};

#define to_nrf24_device(device)	container_of(device, struct nrf24_device, dev)



#endif