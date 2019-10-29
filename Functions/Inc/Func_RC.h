#ifndef  __RC_H
#define  __RC_H

#include "System_Config.h"

#ifdef  __FUNC_RC_GLOBALS
#define __FUNC_RC_EXT
#else
#define __FUNC_RC_EXT extern
#endif

 


//遥控器通道相对最大值
#define RC_CH_MAX_RELATIVE 660.0f

#define RC_FRAME_LEN        18U         //遥控器数据帧长
#define RC_FRAME_LEN_BACK   7U          //增加两个字节保持稳定


/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH0    ((uint8_t)0)
#define RC_CH1    ((uint8_t)1)
#define RC_CH2    ((uint8_t)2)
#define RC_CH3    ((uint8_t)3)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP    ((uint16_t)1)
#define RC_SW_MID   ((uint16_t)3)
#define RC_SW_DOWN  ((uint16_t)2)
#define RC_SW_Right ((uint8_t)0)
#define RC_SW_Left  ((uint8_t)1)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)


#define KEY_W         ((uint8_t)0) //改成了uint16_t,原来是uint8_t
#define KEY_S         ((uint8_t)1)
#define KEY_A         ((uint8_t)2)
#define KEY_D         ((uint8_t)3)
#define KEY_SHIFT     ((uint8_t)4)
#define KEY_CTRL      ((uint8_t)5)
#define KEY_Q         ((uint8_t)6)
#define KEY_E         ((uint8_t)7)
#define KEY_R         ((uint8_t)8)
#define KEY_F         ((uint8_t)9)
#define KEY_G         ((uint8_t)10)
#define KEY_Z         ((uint8_t)11)
#define KEY_X         ((uint8_t)12)
#define KEY_C         ((uint8_t)13)
#define KEY_V         ((uint8_t)14)
#define KEY_B         ((uint8_t)15)
#define KEY_OFFSET    ((uint8_t)0)
/* ----------------------- PC Mouse Definition-------------------------------- */
#define MOUSE_X                 ((uint8_t)0)
#define MOUSE_Y                 ((uint8_t)1)
#define MOUSE_Z                 ((uint8_t)2)
#define MOUSE_SPEED_OFFSET      ((uint16_t)0)
#define MOUSE_LEFT              ((uint8_t)3)
#define MOUSE_RIGHT             ((uint8_t)4)
#define MOUSE_PRESSED_OFFSET    ((uint8_t)0)
/* ----------------------- Data Struct ------------------------------------- */

typedef struct
{ 
  uint16_t RCFrameCounter;       //遥控器帧率计数器

  uint16_t ch0;
  uint16_t ch1;
  uint16_t ch2;
  uint16_t ch3;
  uint8_t  Switch_Left;
  uint8_t  Switch_Right;

  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_left;
    uint8_t press_right;
  }mouse;

  struct
  {
/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
***********************************************************************************/
    uint16_t key_code;
  }key_board;
}RCDecoding_Type;


__FUNC_RC_EXT RCDecoding_Type RC_ReceiveData,LastRC_ReceiveData;
__FUNC_RC_EXT uint8_t RCBuffer[2][RC_FRAME_LEN+RC_FRAME_LEN_BACK];
__FUNC_RC_EXT uint8_t RC_Rx_Mem;            //指示当前遥控器接收使用的缓冲区编号 
void RC_Receive_Enable(UART_HandleTypeDef *huart);
void RC_InitConfig(void);
void RC_Data_Update(void);
int16_t Get_Channel_Val(RCDecoding_Type *RC_ReceiveData,uint8_t channel_num);
uint8_t Get_Switch_Val(RCDecoding_Type *RC_ReceiveData,uint8_t switch_num);
int16_t Get_Mouse_Speed(RCDecoding_Type *RC_ReceiveData,uint8_t xyz);
uint8_t Get_Mouse_Pressed(RCDecoding_Type *RC_ReceiveData,uint8_t button);
uint8_t Get_Keyboard_Val(RCDecoding_Type *RC_ReceiveData,uint8_t key);

#endif
