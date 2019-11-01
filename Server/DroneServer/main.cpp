#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <termios.h>
#include <thread>
#include <string>
#include <iostream>
#include <signal.h>
#include "SerialCon.h"
#include <queue>
#include <semaphore.h>

#define MAX_SIZE_BUF 1000

char* ring_buffer[MAX_SIZE_BUF];
char buf[4096];
char* trimmed_buffer[4096];
int front = 0;
int rear = 0;

pthread_mutex_t a_mutex = PTHREAD_MUTEX_INITIALIZER;
sem_t producer_sem;
sem_t consumer_sem;

int fd;

#define TRUE 1
#define FALSE 0

void producer(std::queue<std::string>& queue);
void consumer(std::queue<std::string>& queue);
void trim_data(char* string, int size);



int main(int argc, char** argv)
{
	// parameter setting
	pthread_t p_producer, p_consumer;
	char cc;
	int thr_id;
	int status;
	int baud = 115200;
	std::string dev_name; //  = "/dev/ttyUSB0"


	std::cout << "enter serial port directory" << std::endl;
	std::getline(std::cin, dev_name);
	char* p_dev_name = (char*)dev_name.c_str();
	// open serial

	SerialCon serial;
	fd = serial.open_serial(p_dev_name, baud, 10, 32);
	if (fd < 0) return -2;

		printf("Serial start (%s)\n", __DATE__);

	// 1. ESP32
	char input[1];
	scanf("%s", input);
	write(fd, input, sizeof(input));

	std::queue<std::string> que;

	sem_init(&producer_sem, 0, 4);
	sem_init(&consumer_sem, 0, 0);

	std::thread th_p (producer, ref(que));
	std::thread th_c (consumer, ref(que));

	th_p.join();
	th_c.join();

	sem_destroy(&producer_sem);
	sem_destroy(&consumer_sem);
	status = pthread_mutex_destroy(&a_mutex);

	serial.close_serial(fd);
	printf("Serial end\n");


	return 0;
}



void producer(std::queue<std::string>& queue)
{
	int rdcnt;

	while (1)
	{
    	sem_wait(&producer_sem);

		memset(buf, 0, sizeof(buf));
		rdcnt = read(fd, buf, sizeof(buf));

		if (rdcnt > 0)
		{
			std::string s(buf);
			queue.push(s);
		}

		sem_post(&consumer_sem);
	}

}

void consumer(std::queue<std::string>& queue)
{
	while (1)
	{
    	sem_wait(&consumer_sem);
		std::cout << queue.front() << std::endl;
		queue.pop();

		sem_post(&producer_sem);
	}
}

void trim_data(char* string, int size)
{

}
