#include "SerialDebug.h"

UART_HandleTypeDef *pDebugUSART;
uint8_t Initialized = 0;

static uint8_t DebugBuffer[DEBUG_BUFFER_LEN] = {0};
static uint8_t position = 0;
static uint8_t Debug_Count = 0;
static uint8_t Debug_Period = 0;

static void Debug_Buf_Send(UART_HandleTypeDef *huart, uint8_t *DebugBuffer, uint8_t length);
static void float2char(float floatdata, uint8_t *buffer, uint8_t n, uint8_t *position);
static void Debug_Buf_Generate(UART_HandleTypeDef *huart, float a, float b, float c, float d, float e, float f);
static void Debug_Buf_Generate_2(uint8_t num, float *buffer);

void Serial_Debug_Init(UART_HandleTypeDef *huart)
{
    pDebugUSART = huart;
    Initialized = 1;
}

void Serial_Debug(UART_HandleTypeDef *huart, uint16_t debug_period, float a, float b, float c, float d, float e, float f)
{
    if (debug_period == 0)
        return;
    if (Debug_Count >= debug_period)
    {
        Debug_Count = 0;
        Debug_Buf_Generate(huart, a, b, c, d, e, f);
    }
    Debug_Count++;
}

void Serial_Debug_Indeterminate_Length(uint8_t num, ...)
{
    va_list valist;
    va_start(valist, num);
    float buffer[num];
    for (uint8_t i = 0; i < num; i++)
    {
        buffer[i] = va_arg(valist, double);
    }
    va_end(valist);

    Debug_Buf_Generate_2(num, buffer);
}

static void Debug_Buf_Send(UART_HandleTypeDef *huart, uint8_t *DebugBuffer, uint8_t length)
{
    if (HAL_UART_GetState(huart) != HAL_UART_STATE_BUSY_TX && (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == SET))
        HAL_UART_Transmit_DMA(huart, DebugBuffer, length);
}

static int _float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal >= 0.5f)
        integer++;
    return integer;
}

static void Debug_Buf_Generate(UART_HandleTypeDef *huart, float a, float b, float c, float d, float e, float f)
{
    uint8_t position = 0;
    float buffer[6] = {a, b, c, d, e, f};
    for (uint8_t cnt = 0; cnt < 6; cnt++)
    {
        if (position > DEBUG_BUFFER_LEN - 20)
            break;
        if (buffer[cnt] > -0.01f && buffer[cnt] < 0.01f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        if (buffer[cnt] > 0.0f && buffer[cnt] < 0.1f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = _float_rounding(buffer[cnt] * 100) % 10 + '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        if (buffer[cnt] > -0.1f && buffer[cnt] < 0.0f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '-';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = _float_rounding(-buffer[cnt] * 100) % 10 + '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        float2char(buffer[cnt], DebugBuffer, cnt, &position);
    }
    DebugBuffer[position - 1] = '\n';
    Debug_Buf_Send(huart, (uint8_t *)DebugBuffer, position);
}

static void Debug_Buf_Generate_2(uint8_t num, float *buffer)
{
    uint8_t position = 0;

    for (uint8_t cnt = 0; cnt < num; cnt++)
    {
        if (position > DEBUG_BUFFER_LEN - 20)
            break;
        if (buffer[cnt] > -0.01f && buffer[cnt] < 0.01f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        if (buffer[cnt] > 0.0f && buffer[cnt] < 0.1f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = _float_rounding(buffer[cnt] * 100) % 10 + '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        if (buffer[cnt] > -0.1f && buffer[cnt] < 0.0f)
        {
            DebugBuffer[position++] = cnt + 'a';
            DebugBuffer[position++] = '=';
            DebugBuffer[position++] = '-';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = '.';
            DebugBuffer[position++] = '0';
            DebugBuffer[position++] = _float_rounding(-buffer[cnt] * 100) % 10 + '0';
            DebugBuffer[position++] = ',';
            continue;
        }
        float2char(buffer[cnt], DebugBuffer, cnt, &position);
    }
    DebugBuffer[position - 1] = '\n';
    if (Initialized)
        Debug_Buf_Send(pDebugUSART, (uint8_t *)DebugBuffer, position);
}

static void float2char(float floatdata, uint8_t *buffer, uint8_t n, uint8_t *position) // �����������洢���ַ����飬�ַ�����ĳ���
{
    int32_t slope;
    int32_t temp;
    int8_t i, j;
    slope = _float_rounding(floatdata * 100);
    buffer[*position] = n + 'a';
    *position += 1;
    buffer[*position] = '=';
    *position += 1;
    if (slope < 0) // �ж��Ƿ����0
    {
        buffer[*position] = '-';
        slope = -slope;
        *position += 1;
    }
    temp = slope;               // ȡ��������
    for (i = 0; temp != 0; i++) // �����������ֵ�λ��
        temp /= 10;
    temp = slope;
    for (j = i; j >= 0; j--) // ����������ת�����ַ�����
    {
        buffer[*position + j] = temp % 10 + '0'; // �Ӻ���ǰ
        temp /= 10;
        if ((i - j) == 1)
        {
            buffer[*position + j - 1] = '.'; // ����С����
            j -= 1;
        }
    }
    *position += i + 1;
    buffer[*position] = ',';
    *position += 1;
}
