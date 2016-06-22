#include "beng.h"
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static int fd_led_ph15 = 0;
static int fd_led_ph16 = 0;
static char value_led[] ={'0','1'};

int open_port()
{
	fd_led_ph15 = open(THE_DEVICE15, O_RDWR);
	if(fd_led_ph15 < 0){
		printf("open fd_f15 failed !\n ");
		return -1;
	}

	fd_led_ph16 = open(THE_DEVICE16, O_RDWR);
	if(fd_led_ph16 < 0){
		printf("open fd_f16 failed !\n ");
		return -1;
	}

	return 1;
}

//打开泵
int open_beng()
{
	int ret;

	//打开气泵
	ret = write(fd_led_ph15, &value_led[0], sizeof(value_led[0]));
	if(ret < 0)
	{
		printf("send fd_ph15 fail\n");
		return -1;
	}

	//关闭电磁阀
	ret = write(fd_led_ph16, &value_led[1], sizeof(value_led[1]));
	if(ret < 0)
	{
		printf("send fd_ph16 fail\n");
		return -1;
	}

	return 1;
}

//关闭泵
int close_beng()
{
	int ret;

	//关闭气泵
	ret = write(fd_led_ph15, &value_led[1], sizeof(value_led[1]));
	if(ret < 0)
	{
		printf("send fd_ph15 fail\n");
		return -1;
	}

	//打开电磁阀
	ret = write(fd_led_ph16, &value_led[0], sizeof(value_led[0]));
	if(ret < 0)
	{
		printf("send fd_ph16 fail\n");
		return -1;
	}

	return 1;
}

int close_port()
{
	close(fd_led_ph15);
	close(fd_led_ph16);

	return 0;
}
