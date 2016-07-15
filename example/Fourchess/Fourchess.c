#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dynamixel.h"
#include "beng.h"
#include "serialCommuni.h"

// Control table address
#define P_TORQUE_ENABLE	     (24) //是否激活扭矩
#define P_CW_Margin	         (26) //柔性边距
#define P_Margin             (27) //柔性边距
#define P_CW_Slope	         (28) //柔性斜率
#define P_Slope		         (29) //柔性斜率
#define P_GOAL_POSITION_L	 (30)
#define P_GOAL_POSITION_H	 (31)
#define P_MOVE_SPEED	  	 (32) //运动速度
#define P_TORQUE_VALUE	     (34) //设置扭矩
#define P_PRESENT_POSITION_L (36)
#define P_PRESENT_POSITION_H (37)
#define P_MOVING			 (46)
#define P_Punch		  		 (48) //Punch

//全局变量
static int objK[10][3]; //位置刻度

//全局函数
int set_servo_byte(int id, int address, int value);
int set_servo_word(int id, int address, int value);
int set_three_servo_byte(int address, int value1, int value2, int value3);
int set_three_servo_word(int address, int value1, int value2, int value3);
int get_servo_byte(int id, int address);
int get_servo_word(int id, int address);
int wait_move_stop(int id);
int wait_three_move_stop();
int exact_regulate(int id, int diff); //精确调节
void send_data(unsigned char *data); //给主机发送数据

int initial_sys(int device, int baudNum); //系统初始化
int back_default_pos(); 				  //运动到初始位置
int back_center_pos();					  //回到中心位置等待
int move_to_channel(int channelNum);      //运动到指定的管道
int get_ball();							  //取球
int analyze_packet(unsigned char packet[], int *status); //解析串口信息

void delay_us(unsigned long usec);
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

int main()
{
	unsigned char packet[20] = {0, }, return_packet[4] = "okk";
	int flag_stop = 0, obj_num, flag;
	
	//初始化
	if(1 != initial_sys(5, 1200)) {
		printf("error-->Fourchess::main:initial_sys() failed\n");
		return -1;
	}
	else{
		printf("sys init success...\n");
		delay_us(1000000);//延迟1s开始
	}

	do{
		if((2 == flag_stop) || (1 == flag_stop)) {
			back_default_pos();    //运动到初始位置
			wait_three_move_stop();//等待运动结束
			get_ball();            //取球
		}
		
		back_center_pos();//回到中间位置
		
		while(1) {
        	//1.获取字符
        	flag = receiveMessage(packet, 20);
			if(-1 == flag) {
				printf("receiving data failed!\r"); fflush(stdout);
			}
			else if(0 == flag) {
				printf("continue receiving data ...\r"); fflush(stdout);
			}
			else if(flag > 0) {
				printf("print data ..."); fflush(stdout);
				if(flag >= 19)
					packet[19] = '\0';
				else
					packet[flag] = '\0';
				printf("the receiving data (%s)\n", packet); fflush(stdout);
				break;
			}
			else {
			}
		} 
		flag_stop = 2; //运动状态变量：0:扔掉球回到初始位置；1：吸球回到初始位置；2：正在进行中;3:重新取串口值
		
		//解析字符信息
		obj_num = analyze_packet(packet, &flag_stop);
		
		if((0 == flag_stop) || (2 == flag_stop)) { //结束或者是丢球到指定槽道
			if('p' != packet[2])			
				move_to_channel(obj_num); //运动指定槽道
			
			//发送动作完成信号
			while(1) {
				flag = sendMessage(return_packet, 3);
				if(flag != 3){
					printf("failed send data\r"); fflush(stdout);
				}
				else{
					printf("success send data (%s)\n", return_packet); fflush(stdout);
					break;
				}
			}
		}
	}while(1);
	
	serial_close();
	close_port();
	
	return 1;
}


int set_servo_byte(int id, int address, int value)
{
	int commStatus, write_num = 20;
	
	while(write_num-- >= 0){
		dxl_write_byte(id, address, value);
		commStatus = dxl_get_result();
		if(commStatus == COMM_RXSUCCESS) {
			PrintErrorCode();
            return 1;
		}
		else
			PrintCommStatus(commStatus);
	}
	
	return -1;
}

int set_servo_word(int id, int address, int value)
{
	int commStatus, write_num = 20;

    while(write_num-- >= 0) {
        dxl_write_word(id, address, value);
        commStatus = dxl_get_result();
        if( commStatus == COMM_RXSUCCESS ) {
            PrintErrorCode();
            return 1;
        }
        else {
            PrintCommStatus(commStatus);
            //return -1;
        }
    }

    return -1;
}

int set_three_servo_byte(int address, int value1, int value2, int value3)
{
	set_servo_byte(7, address, value1);
	set_servo_byte(8, address, value2);
	set_servo_byte(9, address, value3);
	
	return 1;
}

int set_three_servo_word(int address, int value1, int value2, int value3)
{
	set_servo_word(7, address, value1);
	set_servo_word(8, address, value2);
	set_servo_word(9, address, value3);
	
	return 1;
}

int get_servo_byte(int id, int address)
{
	int value, commStatus, num = 0;
	
	while(1) {
		value = dxl_read_byte(id, address);
		commStatus = dxl_get_result();
		if((commStatus == COMM_RXSUCCESS) && (value < 255) && (value >= 0)) {
			PrintErrorCode();
            break;
		}
		else {
			PrintCommStatus(commStatus);
            printf("-->get_servo_byte:servo(%d) errorNum(%d)\n", id, num++);fflush(stdout);
//            dxl_hal_clear();//清空数据包
		}
	}
	
	return value;
}

int get_servo_word(int id, int address)
{
	int value, commStatus, num = 0;
	
    while(1) {
        value = dxl_read_word(id, address);
        commStatus = dxl_get_result();
        if((COMM_RXSUCCESS == commStatus) && (value < 1023) && (value >= 0)) {
            PrintErrorCode();
            break;
        }
        else {
            PrintCommStatus(commStatus);
            printf("-->get_servo_word:servo(%d) errorNum(%d)\n", id, num++);fflush(stdout);
//            dxl_hal_clear();//清空数据包
        }
    }

    return value;
}

int wait_move_stop(int id)
{
	int moving, read_num = 0;
//	int value;
	
//	value = get_servo_word(id, P_GOAL_POSITION_L);
//	printf("Fourchess::wait_move_stop:id(%d) objK(%d)\n", id, value);
	
	do{
		moving = get_servo_byte(id, P_MOVING);
		if(0 == moving) {
			read_num++;
//			printf("Fourchess::wait_move_stop:read success:id(%d) readNum(%d)\n",id, read_num);
		}
		else
			read_num = 0;
	}while(read_num <= 4);
	
//	value = get_servo_word(id, P_PRESENT_POSITION_L);
//	printf("Fourchess::wait_move_stop:id(%d) preK(%d)\n",id, value);
	
	return 1;
}


int wait_three_move_stop()
{
	wait_move_stop(7);
	wait_move_stop(8);
	wait_move_stop(9);
	
	return 1;
}

//精确调节
//如果一次性到达，返回1；其余的返回2。
int exact_regulate(int id, int diff)
{
	int goal_k, pre_k, k_diff;
	int goal_temp;
	
	//读取servo的目标刻度和当前刻度
	goal_k = get_servo_word(id, P_GOAL_POSITION_L);
	pre_k = get_servo_word(id, P_PRESENT_POSITION_L);
	printf("first-->exact_reculate:id(%d) goal_k(%d) pre_k(%d)\n", id, goal_k, pre_k);
	
	k_diff = goal_k - pre_k;
	if(abs(k_diff) <= diff) {
		printf("last-->exact_regulate:id(%d) goal_k(%d) pre_k(%d)\n", id, goal_k, pre_k);
		return 1;
	}
	else{
		goal_temp = goal_k;
		
		do{
			if(k_diff > 0) {
				//当前刻度远小于目标刻度
				goal_temp += 1;
			}
			else {
				//当前刻度远大于目标刻度
				goal_temp -= 1;
			}
			set_servo_word(id, P_GOAL_POSITION_L, goal_temp); //设置目标刻度
			delay_us(200000);//等待运动停止
			wait_move_stop(id);
		
			pre_k = get_servo_word(id, P_GOAL_POSITION_L);//读取目标刻度
			printf("exact_regulate:id(%d) goal_temp(%d) pre_k(%d)\n", id, goal_temp, pre_k);
		
			k_diff = goal_k - pre_k;
			
		}while(abs(k_diff) > diff);
	}
	
	printf("last-->exact_regulate:id(%d) goal_k(%d) pre_k(%d)\n", id, goal_k, pre_k);
	
	return 2;
}

//系统初始化
int initial_sys(int device, int baudNum)
{
	//打开串口通信
	if(1 != serial_open(device, baudNum)) {
        printf("double_4-axis::initail_sys:serial_open failed\n");
        return -1;
    }
	
	//打开气泵
	if(1 != open_port()) {
        printf("double_4-axis::initail_sys:open_port failed\n");
        return -1;
    }
    else {
    	//打开成功，则关闭气泵
    	close_beng();
    }
    
	//打开机械臂
	if( dxl_initialize(0, 207) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		//getchar();
		return 0;
	}
	else
		printf( "Succeed to open USB2Dynamixel!\n" );
		
	//机械臂位置初始化
	//设置Punch值
	set_three_servo_word(P_Punch, 50, 70, 100);
	//设置Slope
	set_three_servo_byte(P_Slope, 128, 64, 128);
	set_three_servo_byte(P_CW_Slope, 128, 64, 128);
	//设置温度
	set_three_servo_byte(11, 85, 85, 85);
	//设置力矩
	set_three_servo_word(P_TORQUE_VALUE, 258, 1023, 1023);
	//设置初始速度
	set_three_servo_word(P_MOVE_SPEED, 60, 100, 100);
	//打开扭矩
	set_three_servo_byte(P_TORQUE_ENABLE, 1, 1, 1);
	//设置机械臂初始位置
	set_three_servo_word(P_GOAL_POSITION_L, 680, 716, 392);
	
	//初始化棋子列表区
	objK[0][0] = 566; objK[0][1] = 551; objK[0][2] = 407;
	objK[1][0] = 606; objK[1][1] = 551; objK[1][2] = 407;
	objK[2][0] = 642; objK[2][1] = 551; objK[2][2] = 407;
	objK[3][0] = 680; objK[3][1] = 551; objK[3][2] = 407;
	objK[4][0] = 716; objK[4][1] = 551; objK[4][2] = 407;
	objK[5][0] = 754; objK[5][1] = 551; objK[5][2] = 407;
	objK[6][0] = 794; objK[6][1] = 551; objK[6][2] = 407;
	objK[7][0] = 829; objK[7][1] = 351; objK[7][2] = 116; //取球处
	objK[8][0] = 829; objK[8][1] = 324; objK[8][2] = 201; //向下取球
	objK[9][0] = 829; objK[9][1] = 394; objK[9][2] = 91;  //向上抬球
	
	//等待运动结束
	wait_three_move_stop();
	
	return 1;
}

//运动到初始位置
int back_default_pos()
{
	set_three_servo_word(P_MOVE_SPEED, 120, 173, 193);//设置运动速度
	set_three_servo_word(P_GOAL_POSITION_L, 827, 716, 472);//设置目标位置
	
	return 1;
}

//回到中心位置等待
int back_center_pos()
{
	set_three_servo_word(P_GOAL_POSITION_L, 680, 712, 402);
	
	return 1;
}		  

//运动到指定的管道 				  
int move_to_channel(int channelNum)
{
	int num = channelNum;	
	
	if(-1 != num) {
	
		//运动到指定管道
		set_three_servo_word(P_MOVE_SPEED, 131, 93, 164);
		set_three_servo_word(P_GOAL_POSITION_L, objK[num][0], objK[num][1], objK[num][2]);
		wait_three_move_stop();
	
		//放球
		close_beng();
		
		return 1;
	}
	
	return -1;
}
      
//取球
int get_ball()
{
	//运动到吸管上方
//	printf("move to ball up...\n");
	set_servo_word(7, P_GOAL_POSITION_L, 827);
	set_servo_word(9, P_GOAL_POSITION_L, 116);
	set_servo_word(8, P_GOAL_POSITION_L, 351);
	wait_move_stop(9);
	wait_move_stop(8);
	wait_move_stop(7);
	//delay_us(500000);
	//exact_regulate(7, 0);
	
	//打开气泵
//	printf("open beng...\n");
	open_beng();
	
	//设置7 servo的属性
//	printf("set 7 servo property ...\n");
	set_servo_word(7, P_Punch, 50);//Punch
	set_servo_byte(7, P_Slope, 32);//slop
	set_servo_byte(7, P_CW_Slope, 32);
	set_servo_word(7, P_TORQUE_VALUE, 1023);//Torque
	wait_move_stop(7);
	
	//发送high信号
	send_data((unsigned char *)"high");
	
	//向下取球
//	printf("down to get ball...\n");
	set_three_servo_word(P_MOVE_SPEED, 1, 30, 95);//原30 75
	set_servo_word(7, P_GOAL_POSITION_L, 825);
	set_servo_word(9, P_GOAL_POSITION_L, objK[8][2] + 10);
	set_servo_word(8, P_GOAL_POSITION_L, objK[8][1]);
	wait_move_stop(7);
	//wait_move_stop(9);
	//wait_move_stop(8);

//	printf("exact_regulate 7 servo ...\n");
	//exact_regulate(8, 2);
	//exact_regulate(9, 2);
	//exact_regulate(7, 0);
	
	printf("\n-->move to 827\n\n");
	set_servo_word(7, P_GOAL_POSITION_L, 827);
	wait_move_stop(7);
	exact_regulate(7, 0);
	
	printf("\nthe tempreture id(7) tempreture(%d):\n\n", get_servo_byte(7, 43));
//	getchar();
	
	//左右摆动
	//set_servo_word(7, P_GOAL_POSITION_L, 826);
	//wait_move_stop(7);exact_regulate(7, 0);
	//delay_us(1000000);
//	getchar();
	//set_servo_word(7, P_GOAL_POSITION_L, 825);
	//wait_move_stop(7);exact_regulate(7, 0);
	//delay_us(1000000);
//	getchar();
	
	//回复7 servo的属性
	set_servo_word(7, P_Punch, 50);//Punch
	set_servo_byte(7, P_Slope, 128);//slop
	set_servo_byte(7, P_CW_Slope, 128);
	set_servo_word(7, P_TORQUE_VALUE, 258);//Torque
	
	//回到安全位置
	set_three_servo_word(P_MOVE_SPEED, 60, 90, 120);
	set_servo_word(9, P_GOAL_POSITION_L, objK[9][2]);
	set_servo_word(8, P_GOAL_POSITION_L, objK[9][1]);
	wait_move_stop(9);
	wait_move_stop(8);
	
	return 1;
}
					  
//解析串口信息
int analyze_packet(unsigned char packet[], int *status)
{
	char numChar[3] = "3";
	int num;

	if('0' == packet[0]) //运动到槽道0 
		return 0; 
	else if('1' == packet[0]) //运动到槽道1
		return 1;
	else if('2' == packet[0]) //运动到槽道2
		return 2;
	else if('3' == packet[0]) //运动到槽道3
		return 3;
	else if('4' == packet[0]) //运动到槽道4
		return 4;
	else if('5' == packet[0]) //运动到槽道5
		return 5;
	else if('6' == packet[0]) //运动到槽道6
		return 6;
	else if('8' == packet[0]) //棋局开始：吸球准备
		*status = 1;
	else if(('7' == packet[0]) && ('8' == packet[5])) { //在结束的同时，马上开始
		if(('0' <= packet[9])&& (packet[9] <= '6')) {   //在结束的同时，马上开始，又同时接受槽道信息
			*status = 2; //正在进心中
			numChar[0] = (char)packet[9];
			num = atoi(numChar);
			return num;
		}
		else { //在结束的同时，马上开始，但是没有受到槽道信息
			*status = 3;
			return -1;
		}
	}
	else { //回到初始位置
		*status = 0;//回到初始位置
		return 3;
	}
	
	return 3;
}

//给主机发送数据
void send_data(unsigned char *data)
{
	int datalen = sizeof(data);
	int flag;
	
	//发送动作完成信号
	while(1) {
		flag = sendMessage(data, datalen);
		if(flag != datalen){
			printf("failed send data\r"); fflush(stdout);
		}
		else{
			printf("success send data %s ...\n", data); fflush(stdout);
			break;
		}
	}
}


void delay_us(unsigned long usec)
{
	struct timeval start, end;
	unsigned long diff;
	gettimeofday(&start, NULL);
	do{
		gettimeofday(&end, NULL);
		diff = 1000000*(end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
	}while(diff <= usec);
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}
