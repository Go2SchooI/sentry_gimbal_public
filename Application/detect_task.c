#include "detect_task.h"

uint32_t resetCount = 0;

Detect_t Detect_List[DETECT_LIST_LENGHT + 1];
BoardState_t BoardState;
uint32_t delta_time;

static void Detect_Init(uint32_t time);

void Detect_Task_Init(void)
{
    static uint32_t system_time;
    system_time = USER_GetTick();

    Detect_Init(system_time);

    USER_Delay_ms(60);
}

void Detect_Task(void)
{
    static uint32_t system_time, last_system_time;
    static uint8_t error_num_display = 0;
    delta_time = USER_GetTick() - system_time;

    error_num_display = DETECT_LIST_LENGHT;
    Detect_List[DETECT_LIST_LENGHT].is_Lost = 0;
    Detect_List[DETECT_LIST_LENGHT].Error_Exist = 0;

    // BoardState.Voltage = get_battery_voltage();
    // BoardState.Temperature = get_temprate();

    for (int i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        if (Detect_List[i].Enable == 0)
        {
            continue;
        }
        system_time = USER_GetTick();

        // �����ж�
        if (system_time - Detect_List[i].New_Time > Detect_List[i].Offline_Time)
        {
            if (Detect_List[i].Error_Exist == 0)
            {
                // ��¼�����Լ�����ʱ��
                Detect_List[i].is_Lost = 1;
                Detect_List[i].Error_Exist = 1;
                Detect_List[i].Lost_Time = system_time;
            }

            // �������ȼ���ߵĴ�����
            if (Detect_List[i].Priority > Detect_List[error_num_display].Priority)
            {
                error_num_display = i;
            }

            Detect_List[DETECT_LIST_LENGHT].is_Lost = 1;
            Detect_List[DETECT_LIST_LENGHT].Error_Exist = 1;

            // ����ṩ������������н������
            if (Detect_List[i].f_Solve_Lost != NULL)
            {
                Detect_List[i].f_Solve_Lost();
            }
        }
        else if (system_time - Detect_List[i].Work_Time < Detect_List[i].Online_Time)
        {
            // �ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
            Detect_List[i].is_Lost = 0;
            Detect_List[i].Error_Exist = 1;
        }
        else
        {
            Detect_List[i].is_Lost = 0;
            // �ж��Ƿ�������ݴ���
            if (Detect_List[i].is_Data_Error != NULL)
            {
                Detect_List[i].Error_Exist = 1;
            }
            else
            {
                Detect_List[i].Error_Exist = 0;
            }

            // ����Ƶ��
            if (Detect_List[i].New_Time > Detect_List[i].Last_Time)
            {
                Detect_List[i].frequency = 1000.0f / (float)(Detect_List[i].New_Time - Detect_List[i].Last_Time);
            }
        }
    }
    last_system_time = system_time;
}

uint8_t is_TOE_Error(uint8_t toe)
{
    return (Detect_List[toe].Error_Exist == 1);
}

void Detect_Hook(uint8_t toe)
{
    Detect_List[toe].Last_Time = Detect_List[toe].New_Time;
    Detect_List[toe].New_Time = USER_GetTick();

    if (Detect_List[toe].is_Lost)
    {
        Detect_List[toe].is_Lost = 0;
        Detect_List[toe].Work_Time = Detect_List[toe].New_Time;
    }

    if (Detect_List[toe].f_is_Data_Error != NULL)
    {
        if (Detect_List[toe].f_is_Data_Error())
        {
            Detect_List[toe].Error_Exist = 1;
            Detect_List[toe].is_Data_Error = 1;

            if (Detect_List[toe].f_Solve_Data_Error != NULL)
            {
                Detect_List[toe].f_Solve_Data_Error();
            }
        }
        else
        {
            Detect_List[toe].is_Data_Error = 0;
        }
    }
    else
    {
        Detect_List[toe].is_Data_Error = 0;
    }
}

static void Detect_Init(uint32_t time)
{
    // ��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime Priority
    uint16_t set_item[DETECT_LIST_LENGHT][3] =
        {
            {30, 3, 14},     // yaw
            {30, 3, 13},     // pitch
            {5, 10, 11},     // trigger 1
            {5, 10, 11},     // trigger 2
            {50, 3, 12},     // RC device
            {70, 4, 10},     // autoaim device
            {1000, 1000, 9}, // refuree device
            {500, 500, 12},  // VTM device
        };

    for (uint8_t i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        Detect_List[i].Offline_Time = set_item[i][0];
        Detect_List[i].Online_Time = set_item[i][1];
        Detect_List[i].Priority = set_item[i][2];
        Detect_List[i].f_is_Data_Error = NULL;
        Detect_List[i].f_Solve_Lost = NULL;
        Detect_List[i].f_Solve_Data_Error = NULL;

        Detect_List[i].Enable = 1;
        Detect_List[i].Error_Exist = 1;
        Detect_List[i].is_Lost = 1;
        Detect_List[i].is_Data_Error = 1;
        Detect_List[i].frequency = 0.0f;
        Detect_List[i].New_Time = time;
        Detect_List[i].Last_Time = time;
        Detect_List[i].Lost_Time = time;
        Detect_List[i].Work_Time = time;
    }
}
