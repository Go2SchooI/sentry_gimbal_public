#ifndef _JUDGEMENT_INFO_1__
#define _JUDGEMENT_INFO_1__

#include "usart.h"
#include "stdint.h"

#define JUDGE_SOF (uint8_t)0xA5
#define GAME_DATA 0x0201
#define ROBOT_STATE_DATA 0x0001
#define POWER_HEAT_DATA 0x0202
#define ROBOT_POS_DATA 0x0203
#define ROBOT_HURT_DATA 0x0206
#define USER_DATA 0x0301

typedef enum
{
    GAME_STATUS_CMD_ID = 0x0001,
    GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    DART_STATUS_CMD_ID = 0x0004,
    ICRA_BUFF_DEBUFF_ZONE_STATUS_CMD_ID = 0x0005,
    EVEN_DATA_CMD_ID = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,
    REFEREE_WARNING_CMD_ID = 0x0104,
    DART_REMAINING_TIME_CMD_ID = 0X0105,
    GAME_ROBOT_STATUS_CMD_ID = 0x0201,
    POWER_HEAT_DATA_CMD_ID = 0x0202,
    GAME_ROBOT_POS_CMD_ID = 0x0203,
    BUFF_MUSK_CMD_ID = 0x0204,
    ROBOT_ENERGY_CMD_ID = 0x0205,
    ROBOT_HURT_CMD_ID = 0x0206,
    SHOOT_DATA_CMD_ID = 0x0207,
    BULLET_REMAINING_CMD_ID = 0x0208,
    RFID_STATUS_CMD_ID = 0x0209,
    DART_CLIENT_CMD_ID = 0x020A,
    STUDENT_INTERACTIVE_HEADER_DATA_CMD_ID = 0x0301,

    INTERACTIVE_DATA_CMD_ID = 0x0302,
    MAP_INTERACTIVITY_CMD_ID = 0x0303,
    IDCustomData,
} referee_cmd_id_t;

typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 11,
    BLUE_ENGINEER = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL = 16,
    BLUE_SENTRY = 17,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;

typedef struct
{
    uint8_t buf[70];
    uint8_t header[5];
    uint16_t cmd;
    uint8_t data[30];
    uint8_t tail[2];
} judge_receive_t;

// ����״̬���ݣ�0x0001
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

// ����������ݣ�0x0002
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

// ������Ѫ�����ݣ�0x0003
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// ���ڷ���״̬��0x0004
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

// �˹�������ս���ӳ���ͷ���״̬��0x0005
typedef __packed struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

// �����¼����ݣ�0x0101
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

// ����վ������ʶ��0x0102
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// ���°��ֲ�û��д
//  typedef __packed struct //0x0103
//  {
//      uint8_t supply_projectile_id;
//      uint8_t supply_robot_id;
//      uint8_t supply_num;
//  } ext_supply_projectile_booking_t;

// ���о�����Ϣ��0x0104
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

// ���ڷ���ڵ���ʱ��0x0105
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// ����������״̬��0X201
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;

    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

// ʵʱ�����������ݣ�0x0202
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

// ������λ�ã�0x0203
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

// ���������棺0x0204
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

// ���л���������״̬��0x0205
typedef __packed struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;

// �˺�״̬:0x0206
typedef __packed struct // 0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

// ʵʱ�����Ϣ��0x0207
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

// �ӵ�ʣ�෢������0x0208
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

// ������RFID״̬��0x0209
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

// ���ڻ����˿ͻ���ָ�����ݣ�0x020A
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

// �������ݽ�����Ϣ��0x0301
typedef __packed struct
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

// ѧ�������˼�ͨ�� cmd_id 0x0301������ID:0x0200~0x02FF

// ��������
typedef __packed struct
{
    uint8_t data[118];
} robot_interactive_data_t;

// �ͻ���ɾ��ͼ��
typedef __packed struct
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

// ͼ������
typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} graphic_data_struct_t;

// �ͻ��˻���һ��ͼ��
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

// �ͻ��˻��ƶ���ͼ��
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
// �ͻ��˻������ͼ��
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

// �ͻ��˻����ַ�
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

// �ͻ��˻����߸�ͼ��
typedef __packed struct
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

// �������ݽ�����Ϣ��0x0302��
typedef __packed struct
{
    uint8_t data[30];
} ext_robot_interactive_data_t;

// С��ͼ������Ϣ��ʶ��0x0303
typedef __packed struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
} ext_map_interactivity_t;

extern ext_game_status_t game_status;                                         // ����״̬����
extern ext_game_result_t game_result;                                         // �����������
extern ext_game_robot_HP_t game_robot_HP;                                     // ������Ѫ������
extern ext_dart_status_t dart_status;                                         // ���ڷ���״̬
extern ext_ICRA_buff_debuff_zone_status_t ICRA_buff_debuff_zone_status;       // �˹�������ս���ӳ���ͷ���״̬
extern ext_event_data_t event_data;                                           // �����¼�����
extern ext_supply_projectile_action_t supply_projectile_action;               // ����վ������ʶ
extern ext_referee_warning_t referee_warning;                                 // ���о�����Ϣ
extern ext_dart_remaining_time_t dart_remaining_time;                         // ���ڷ���ڵ���ʱ
extern ext_game_robot_status_t robot_state;                                   // ����������״̬
extern ext_power_heat_data_t power_heat_data_t;                               // ʵʱ������������
extern ext_game_robot_pos_t game_robot_pos;                                   // ������λ��
extern ext_buff_musk_t buff_musk;                                             // ����������
extern aerial_robot_energy_t robot_energy;                                    // ���л���������״̬
extern ext_robot_hurt_t robot_hurt;                                           // �˺�״̬
extern ext_shoot_data_t shoot_data;                                           // ʵʱ�����Ϣ
extern ext_rfid_status_t rfid_status;                                         // �ӵ�ʣ�෢����
extern ext_bullet_remaining_t bullet_remaining;                               // ������RFID״̬
extern ext_dart_client_cmd_t dart_client_cmd;                                 // ���ڻ����˿ͻ���ָ������
extern ext_student_interactive_header_data_t student_interactive_header_data; // �������ݽ�����Ϣ

extern ext_robot_interactive_data_t robot_interactive_data; // �������ݽ�����Ϣ
extern ext_map_interactivity_t map_interactivity;           // С��ͼ������Ϣ��ʶ

extern UART_HandleTypeDef *JudgeUSART;
extern uint8_t JudgeRxData[8];

extern HAL_StatusTypeDef IT_DMA_Begain(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size); // �������ж�
extern judge_receive_t judgement_receive;
void Judge_Control_Init(UART_HandleTypeDef *huart);
void judgement_info_handle(void);
void unpack_fifo_handle(uint8_t *prxbuf);
void judgement_info_updata(void);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void judgement_data_decode(void);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif

/*
1.������ȡ��DMA��������
2.�ڻص�������UART_IdleRxCallback���е��ô���������unpack_fifo_handle��
3.��������������judgement_race_data�ṹ���У�
*/
