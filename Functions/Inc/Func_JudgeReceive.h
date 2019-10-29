#ifndef __TASKJUDGERECEIVE_H
#define __TASKJUDGERECEIVE_H

#include "System_Config.h"


/*--------------CmdID(2-Byte)----------------*/
        #define GAMESTATE           0X0001 //比赛状态数据，1Hz 周期发送
        #define GAMERESULT          0X0002 //比赛结果数据，比赛结束后发送
        #define ROBOTSURVIVORS      0X0003 //比赛机器人存活数据，1Hz 周期发送
        #define EVENTDATA           0X0101 //场地事件数据，事件改变后发送
        #define SUPPLYACTION        0X0102 //场地补给站动作标识数据，动作改变后发送
        #define SUPPLYBOOKING       0X0103 //请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
        #define GAMEROBOTSTATE      0X0201 //机器人状态数据，10Hz 周期发送
        #define POWERHEATDATA       0X0202 //实时功率热量数据，50Hz 周期发送
        #define GAMEROBOTPOS        0X0203 //机器人位置数据，10Hz 发送
        #define BUFFMUSK            0X0204 //机器人增益数据，增益状态改变后发送
        #define AERIALROBOTENERGY   0X0205 //空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
        #define ROBOTHURT           0X0206 //伤害状态数据，伤害发生后发送
        #define SHOOTDATA           0X0207 //实时射击数据，子弹发射后发送
        #define INTERACTIVEHEADER   0X0301 //交互数据接收信息，发送频率：上限 10Hz
/*--------------CmdID(2-Byte)----------------*/

/*--------------DataSize----------------*/
    #define GAMESTATE_DATA_SIZE         3
    #define GAMERESULT_DATA_SIZE        1
    #define ROBOTSURVIVORS_DATA_SIZE    2
    #define EVENTDATA_DATA_SIZE         4
    #define SUPPLYACTION_DATA_SIZE      3
    #define SUPPLYBOOKING_DATA_SIZE     2
    #define GAMEROBOTSTATE_DATA_SIZE    15
    #define POWERHEATDATA_DATA_SIZE     14
    #define GAMEROBOTPOS_DATA_SIZE      16
    #define BUFFMUSK_DATA_SIZE          1
    #define AERIALROBOTENERGY_DATA_SIZE 3
    #define ROBOTHURT_DATA_SIZE         1
    #define SHOOTDATA_DATA_SIZE         6
    #define INTERACTIVEHEADER_DATA_SIZE(n)      (n+9)
		#define JUDGE_DATA_LENGTH(n)                (n+9)
/*--------------DataSize----------------*/

/*--------------偏移位置----------------*/
    #define JUDGE_SOF_OFFSET            0
    #define JUDGE_DATALENGTH_OFFSET     1
    #define JUDGE_SEQ_OFFSET            3
    #define JUDGE_CRC8_OFFSET           4
    #define JUDGE_CMDID_OFFSET          5
    #define JUDGE_DATA_OFFSET           7
    #define JUDGE_CRC16_OFFSET(n)       (n+JUDGE_DATA_OFFSET)
/*--------------偏移位置----------------*/

/*-------------------------------------------CRC校验---------------------------------------------------*/
/**
  * @brief  裁判系统数据校验
  * @param  __RECEIVEBUFFER__：  接收到的裁判系统数据头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval 1：                  校验正确
  * @retval 0：                  校验错误
  * @note	None
  */
#define Verify_CRC_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__)      (Verify_CRC8_Check_Sum(__RECEIVEBUFFER__, JUDGE_CRC8_OFFSET+1)\
                                                    &&Verify_CRC16_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__+JUDGE_DATA_LENGTH(0)))

/**
  * @brief  裁判系统添加校验
  * @param  __TRANSMITBUFFER__： 发送到裁判系统的数据中头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval None
  * @note	None
  */
#define Append_CRC_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__)                    \
do                                                                                  \
{                                                                                   \
    Append_CRC8_Check_Sum(__TRANSMITBUFFER__, JUDGE_CRC8_OFFSET+1);                 \
    Append_CRC16_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__+JUDGE_DATA_LENGTH(0));\
}while(0U)


//1.	比赛机器人状态(0x0001)
typedef __packed struct
{
    /*
    0-3 bit：比赛类型
    ? 1：RoboMaster 机甲大师赛；
    ? 2：RoboMaster 机甲大师单项赛；
    ? 3：ICRA RoboMaster 人工智能挑战赛
    */
    uint8_t game_type : 4;

    /*
    4-7 bit：当前比赛阶段
    ? 0：未开始比赛；
    ? 1：准备阶段；
    ? 2：自检阶段；
    ? 3：5s 倒计时；
    ? 4：对战中；
    ? 5：比赛结算中
    */
    uint8_t game_progress : 4;
    
    /*
    当前阶段剩余时间，单位 s
    */
    uint16_t stage_remain_time;
}ext_game_state_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef __packed struct
{
    /*0 平局 1 红方胜利 2 蓝方胜利*/
    uint8_t winner;
} ext_game_result_t;

//3. 机器人存活数据：0x0003。发送频率：1Hz
typedef __packed struct
{
    /*
    bit 0：红方英雄机器人；
    bit 1：红方工程机器人；
    bit 2：红方步兵机器人 1；
    bit 3：红方步兵机器人 2；
    bit 4：红方步兵机器人 3；
    bit 5：红方空中机器人；
    bit 6：红方哨兵机器人；
    bit 7：保留
    bit 8：蓝方英雄机器人；
    bit 9：蓝方工程机器人；
    bit 10：蓝方步兵机器人 1；
    bit 11：蓝方步兵机器人 2；
    bit 12：蓝方步兵机器人 3；
    bit 13：蓝方空中机器人；
    bit 14：蓝方哨兵机器人；
    bit 15：保留
    对应的 bit 数值置 1 代表机器人存活，数值置 0 代表机器人死亡或者未上场。
    */
    uint16_t robot_legion;
} ext_game_robot_survivors_t;

//4.场地事件数据：0x0101。发送频率：事件改变后发送
typedef __packed struct
{
    /*
    bit 0-1：己方停机坪占领状态
    ? 0 为无机器人占领；
    ? 1 为空中机器人已占领但未停桨；
    ? 2 为空中机器人已占领并停桨
    bit 2：己方补给站 1 号补血点占领状态 1 为已占领；
    bit 3：己方补给站 2 号补血点占领状态 1 为已占领；
    bit 4：己方补给站 3 号补血点占领状态 1 为已占领；
    bit 5-6：己方大能量机关状态：
    ? 0 为打击点未占领且大能量机关未激活；
    ? 1 为打击点占领且大能量机关未激活；
    ? 2 为大能量机关已激活；
    ? 3 为大能量机关已激活且打击点被占领；
    bit 7：己方关口占领状态 1 为已占领；
    bit 8：己方碉堡占领状态 1 为已占领；
    bit 9：己方资源岛占领状态 1 为已占领；
    bit 10-11：己方基地防御状态：
    ? 2 为基地 100%防御；
    ? 1 为基地有哨兵防御；
    ? 0 为基地无防御；
    bit 12-13：ICRA 红方防御加成
    ? 0：防御加成未激活；
    ? 1：防御加成 5s 触发激活中；
    ? 2：防御加成已激活
    bit 14-15：ICRA 蓝方防御加成
    ? 0：防御加成未激活；
    ? 1：防御加成 5s 触发激活中；
    ? 2：防御加成已激活
    其余保留
    */
    uint32_t event_type;
} ext_event_data_t;

//5. 补给站动作标识：0x0102。发送频率：动作改变后发送
typedef __packed struct
{
    /*
    补给站口 ID：
    1：1 号补给口；
    2：2 号补给口
    */
    uint8_t supply_projectile_id;

    /*
    补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程
    机器人补弹，3/4/5 为红方步兵机器人补弹，11 为蓝方英雄机器人补弹，12 为蓝方
    工程机器人补弹，13/14/15 为蓝方步兵机器人补弹
    */
    uint8_t supply_robot_id;

    /*
    出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
    */
    uint8_t supply_projectile_step;

    /*
    补弹数量：
    50：50 颗子弹；
    100：100 颗子弹；
    150：150 颗子弹；
    200：200 颗子弹。
    */
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//6.请求补给站补弹子弹：cmd_id (0x0103)。发送频率：上限 10Hz。RM 对抗赛尚未开放
typedef __packed struct
{
    /*
    补给站补弹口 ID：
    1：1 号补给口
    */
    uint8_t supply_projectile_id;

    /*
    补弹机器人 ID：1 为红方英雄机器人补弹，2 为红方工程机器人补弹，
    3/4/5 为红方步兵机器人补弹，11 为蓝方英雄机器人补弹，12 为蓝方
    工程机器人补弹，13/14/15 为蓝方步兵机器人补弹
    */
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

//7.比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct
{
    /*
    机器人 ID：
    1：红方英雄机器人；
    2：红方工程机器人；
    3/4/5：红方步兵机器人；
    6：红方空中机器人；
    7：红方哨兵机器人；
    11：蓝方英雄机器人；
    12：蓝方工程机器人；
    13/14/15：蓝方步兵机器人；
    16：蓝方空中机器人；
    17：蓝方哨兵机器人。
    */
    uint8_t robot_id;

    /*
    机器人等级：
    1：一级；2：二级；3：三级。
    */
    uint8_t robot_level;

    /*
    机器人剩余血量
    */
    uint16_t remain_HP;

    /*
    机器人上限血量
    */
    uint16_t max_HP;

    /*
    机器人 17mm 枪口每秒冷却值
    */
    uint16_t shooter_heat0_cooling_rate;

    /*
    机器人 17mm 枪口热量上限
    */
    uint16_t shooter_heat0_cooling_limit;

    /*
    机器人 42mm 枪口每秒冷却值
    */
    uint16_t shooter_heat1_cooling_rate;

    /*
    机器人 42mm 枪口热量上限
     */
    uint16_t shooter_heat1_cooling_limit;

    /*
    主控电源输出情况：
    0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
    1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
    2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
    */
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//8.实时功率热量数据：0x0202。发送频率：50Hz
typedef __packed struct
{
    //底盘输出电压 单位 毫伏
    uint16_t chassis_volt;
    //底盘输出电流 单位 毫安
    uint16_t chassis_current;
    //底盘输出功率 单位 W 瓦
    float chassis_power;
    //底盘功率缓冲 单位 J 焦耳
    uint16_t chassis_power_buffer;
    //17mm 枪口热量
    uint16_t shooter_heat0;
    //42mm 枪口热量
    uint16_t shooter_heat1;
}ext_power_heat_data_t;

//9.机器人位置：0x0203。发送频率：10Hz
typedef __packed struct
{
   
    float x;//位置 x 坐标，单位 m
    float y;//位置 y 坐标，单位 m
    float z;//位置 z 坐标，单位 m
    float yaw;//位置枪口，单位度
} ext_game_robot_pos_t;

//10. 机器人增益：0x0204。发送频率：状态改变后发送
typedef __packed struct
{
    /*
    bit 0：机器人血量补血状态
    bit 1：枪口热量冷却加速
    bit 2：机器人防御加成
    bit 3：机器人攻击加成
    其他 bit 保留
    */
    uint8_t power_rune_buff;
}ext_buff_musk_t;

//11. 空中机器人能量状态：0x0205。发送频率：10Hz
typedef __packed struct
{

    uint8_t energy_point;//积累的能量点
    uint8_t attack_time; //可攻击时间 单位 s。50s 递减至 0
} aerial_robot_energy_t;

//12. 伤害状态：0x0206。发送频率：伤害发生后发送
typedef __packed struct
{
    /*
    bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人
    的五个装甲片，其他血量变化类型，该变量数值为 0。
    */
    uint8_t armor_id : 4;
    /*bit 4-7：血量变化类型
    0x0 装甲伤害扣血；
    0x1 模块掉线扣血；
    0x2 超枪口热量扣血；
    0x3 超底盘功率扣血。
    */
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;
//13. 实时射击信息：0x0207。发送频率：射击后发送
typedef __packed struct
{
    uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
    uint8_t bullet_freq;//子弹射频 单位 Hz
    float bullet_speed; //子弹射速 单位 m/s
} ext_shoot_data_t;

//交互数据接收信息：0x0301。发送频率：上限 10Hz
typedef __packed struct
{

    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


extern ext_game_state_t                ext_game_state;
extern ext_game_result_t               ext_game_result;
extern ext_game_robot_survivors_t      ext_game_robot_survivors;
extern ext_event_data_t                ext_event_data;
extern ext_supply_projectile_action_t  ext_supply_projectile_action;
extern ext_supply_projectile_booking_t ext_supply_projectile_booking;
extern ext_game_robot_state_t          ext_game_robot_state;
extern ext_power_heat_data_t           ext_power_heat_data;
extern ext_game_robot_pos_t            ext_game_robot_pos;
extern ext_buff_musk_t                 ext_buff_musk;
extern aerial_robot_energy_t           aerial_robot_energy;
extern ext_robot_hurt_t                ext_robot_hurt;
extern ext_shoot_data_t                ext_shoot_data;
extern ext_student_interactive_header_data_t			      ext_student_interactive_header_data;



void JudgeConnection_Init(UART_HandleTypeDef *huart);
void Judge_IDLECallback(UART_HandleTypeDef *huart);
void JudgeTransmit(void);
void JudgeReceive(void);
void Judge_Receive_Data_Processing(uint8_t SOF, uint16_t CmdID);

/*--------------------------------------------------校验函数--------------------------------------------------*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
/*--------------------------------------------------校验函数--------------------------------------------------*/

#endif
