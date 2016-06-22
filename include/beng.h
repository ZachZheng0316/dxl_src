#ifndef _BENG_HEADER
#define _BENG_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#define THE_DEVICE15 "/sys/class/gpio_sw/PH15/data" //
#define THE_DEVICE16 "/sys/class/gpio_sw/PH16/data" //

int open_port(); //打开端口
int open_beng(); //打开甭
int close_beng();//
int close_port(); //关闭端口


#ifdef __cplusplus
}

#endif

#endif
