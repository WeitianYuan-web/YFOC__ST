/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_ID_1_4           0x101
#define FEEDBACK_ID_BASE         0x111
#define INFOMATION_ID_BASE       0x121
#define AS5600_ADDR       0x36    // AS5600 I2C地址
#define AS5600_ANGLE_REG  0x0C    // 角度寄存器地址
#define _2PI              6.28318530718f

//FOC.C
#define deg2rad(a) (_PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define  voltage_power_supply 12.0

#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#define _sqrt(a) (_sqrtApprox(a))

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

#define POLE_PAIRS 7 // 极对数

// 电路参数：
#define R_SHUNT 0.02           // 电流采样电阻，欧姆
#define OP_GAIN 50             // 运放放大倍数
#define MAX_CURRENT 3          // 最大q轴电流，安培A
#define ADC_REFERENCE_VOLT 3.3 // 电流采样adc参考电压，伏
#define ADC_BITS 12            // ADC精度，bit
#define VOLTAGE_POWER_SUPPLY 12.0f  // 直流母线电压

// 单片机配置参数：
#define motor_pwm_freq 20000      // 驱动桥pwm频率，Hz
#define motor_speed_calc_freq 1000 // 电机速度计算频率，Hz

// 软件参数：
#define position_cycle (6 * PI) // 电机多圈周期，等于正半周期+负半周期

#define rad2deg(a) (180 * (a) / PI)
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//标识符
volatile uint8_t uart_data_ready = 0;


uint8_t uart_rx_buffer[4];  // 修改为4字节（32位浮点数大小）
uint8_t uart_rx_length = 0; // 新增变量，记录UART接收到的有效数据长度
char uart_buffer[100];

uint32_t  time_start = 0;
uint32_t  time_end = 0;
uint32_t  time_diff = 0;

//开环控制用
uint32_t lastTime_open = 0;
float angle_open = 0.0f;
float targetVoltage_open = 0.8f;
float Uq_open = 0.0f;
float elec_angle_open = 0.0f;


const int sine_array[200] = {0, 79, 158, 237, 316, 395, 473, 552, 631, 710, 789, 867, 946, 1024, 1103, 1181,
                             1260, 1338, 1416, 1494, 1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192,
                             2269, 2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030, 3105, 3180,
                             3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772, 3845, 3918, 3990, 4063, 4135,
                             4206, 4278, 4349, 4420, 4491, 4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046,
                             5113, 5181, 5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839, 5903,
                             5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461, 6521, 6581, 6640, 6699,
                             6758, 6815, 6873, 6930, 6987, 7043, 7099, 7154, 7209, 7264, 7318, 7371, 7424,
                             7477, 7529, 7581, 7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072,
                             8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512, 8553, 8594, 8634,
                             8673, 8712, 8751, 8789, 8826, 8863, 8899, 8935, 8970, 9005, 9039, 9072, 9105,
                             9138, 9169, 9201, 9231, 9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481,
                             9506, 9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721, 9739, 9757,
                             9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876, 9888, 9899, 9910, 9920, 9930,
                             9939, 9947, 9955, 9962, 9969, 9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999,
                             10000, 10000};



// AS5600 DMA相关变量
uint8_t as5600_dma_buffer[2]; // 用于接收编码器数据的DMA缓冲区
uint8_t as5600_reg_addr = AS5600_ANGLE_REG; // 角度寄存器地址
volatile uint8_t as5600_dma_complete = 0; // DMA完成标志

//foc_runing


// 用于存储角度和速度的变量
float position_rad = 0.0f; // 当前位置（弧度）
float velocity_rad_per_sec = 0.0f; // 当前速度（弧度/秒）
float velocity_rpm = 0.0f; // 当前速度（RPM）


// AS5600结构体 
typedef struct {
    I2C_HandleTypeDef *hi2c;
    int32_t full_rotations;        // 完整旋转圈数
    int32_t vel_full_rotations;    // 速度计算用的圈数
    uint16_t raw_angle;            // 当前原始角度值(0-4095)
    int32_t total_angle_raw;       // 总共旋转的角度-一圈是4096
    float rotor_zero_elec_angle;        // 零电角度
    uint16_t angle_prev;           // 上一次角度值(0-4095)
    uint32_t angle_prev_ts;        // 上一次角度时间戳
    float position_rad;            // 当前位置（弧度）
    float velocity_rad_per_sec;    // 当前速度（弧度/秒）
    float velocity_rpm;            // 当前速度（RPM）
    float velocity_rad_per_sec_filtered; // 滤波后的速度（弧度/秒）
    float velocity_rpm_filtered;   // 滤波后的速度（RPM）
} AS5600_TypeDef;

AS5600_TypeDef as5600;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float _sin(float a)
{
    if (a < _PI_2)
    {
        //return sine_array[(int)(199.0f*( a / (_PI/2.0)))];
        //return sine_array[(int)(126.6873f* a)];           // float array optimized
        return 0.0001f * sine_array[_round(126.6873f * a)];      // int array optimized
    } else if (a < _PI)
    {
        // return sine_array[(int)(199.0f*(1.0f - (a-_PI/2.0) / (_PI/2.0)))];
        //return sine_array[398 - (int)(126.6873f*a)];          // float array optimized
        return 0.0001f * sine_array[398 - _round(126.6873f * a)];     // int array optimized
    } else if (a < _3PI_2)
    {
        // return -sine_array[(int)(199.0f*((a - _PI) / (_PI/2.0)))];
        //return -sine_array[-398 + (int)(126.6873f*a)];           // float array optimized
        return -0.0001f * sine_array[-398 + _round(126.6873f * a)];      // int array optimized
    } else
    {
        // return -sine_array[(int)(199.0f*(1.0f - (a - 3*_PI/2) / (_PI/2.0)))];
        //return -sine_array[796 - (int)(126.6873f*a)];           // float array optimized
        return -0.0001f * sine_array[796 - _round(126.6873f * a)];      // int array optimized
    }
}


// function approximating cosine calculation by using fixed size array
float _cos(float a)
{
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}


// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle)
{
    float a = fmod(angle, _2PI);
    return a >= 0 ? a : (a + _2PI);
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs)
{
    return (shaft_angle * (float)pole_pairs) - as5600.rotor_zero_elec_angle;
}

// square root approximation function using
float _sqrtApprox(float number)
{//low in fat
    long i;
    float y;
    // float x;
    // const float f = 1.5F; // better precision

    // x = number * 0.5F;
    y = number;
    i = *(long*) &y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*) &i;
    // y = y * ( f - ( x * y * y ) ); // better precision
    return number * y;
}

float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_output;
}

void set_pwm_duty(float d_u, float d_v, float d_w)
{
  d_u = min(d_u, 0.9);
  d_v = min(d_v, 0.9);
  d_w = min(d_w, 0.9);
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d_u * period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d_v * period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, d_w * period);
  __enable_irq();
}

void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w) {
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    // 计算 alpha-beta 坐标系中的电压分量
    float alpha = -q * _sin(phi) + d * _cos(phi);
    float beta = q * _cos(phi) + d * _sin(phi);
    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = _sin(sector * rad60) * alpha - _cos(sector * rad60) * beta;
    float t_n = beta * _cos(sector * rad60 - rad60) - alpha * _sin(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;
    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

float velocityOpenloop(float target_velocity) {
  uint32_t now_ms = HAL_GetTick();
  float Ts = (now_ms - lastTime_open) * 1e-3f; // 转换为秒

  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // 处理方向
  int direction = (target_velocity >= 0) ? 1 : -1;
  float abs_speed = fabsf(target_velocity);

  // 更新角度
  angle_open = _normalizeAngle(angle_open - target_velocity * Ts);

  float Uq = voltage_power_supply/3;


    foc_forward(0, 0.5f, angle_open);
  lastTime_open = now_ms;
  return Uq;
}

void position_control(float rad)
{
}

uint16_t AS5600_GetRawAngle(AS5600_TypeDef *as5600)
{
    uint8_t data[2];
    uint16_t raw_angle = 0;

    // 读取角度寄存器 (0x0C和0x0D)
    HAL_I2C_Mem_Read(as5600->hi2c, (AS5600_ADDR << 1), 0x0C, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

    // 组合两个字节，高字节在前，低字节在后
    raw_angle = ((uint16_t)data[0] << 8) | data[1];

    // AS5600是12位分辨率，所以我们只保留低12位
    raw_angle &= 0x0FFF;

    return raw_angle;
}
void AS5600_Init(AS5600_TypeDef *as5600, I2C_HandleTypeDef *hi2c) {
    as5600->hi2c = hi2c;
    as5600->full_rotations = 0;
    as5600->vel_full_rotations = 0;
    as5600->raw_angle = 0;
    as5600->position_rad = 0.0f;
    as5600->velocity_rad_per_sec = 0.0f;
    as5600->velocity_rpm = 0.0f;

    // 读取初始角度
    uint8_t data[2];
    HAL_I2C_Mem_Read(as5600->hi2c, (AS5600_ADDR << 1), AS5600_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    
    // 组合两个字节，高字节在前，低字节在后
    as5600->raw_angle = ((uint16_t)data[0] << 8) | data[1];
    as5600->angle_prev = as5600->raw_angle & 0x0FFF; // 12位分辨率
    as5600->angle_prev_ts = HAL_GetTick();
}
// 启动连续DMA读取
void AS5600_StartDMAReading(void) {
    // 启动定时器
    HAL_TIM_Base_Start_IT(&htim2);
    
    // 发起首次读取
    as5600_dma_complete = 0;
    HAL_I2C_Mem_Read_DMA(&hi2c1, (AS5600_ADDR << 1), AS5600_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, as5600_dma_buffer, 2);
}

// 处理DMA完成回调
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // 设置DMA完成标志
        as5600_dma_complete = 1;
    }
}

// 计算角度和速度
void AS5600_ProcessData(AS5600_TypeDef *as5600) {
    // 获取当前时间戳
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - as5600->angle_prev_ts) / 1000.0f; // 转换为秒
    
    if (dt <= 0.0f) dt = 0.001f; // 防止除零
    
    // 解析收到的角度数据
    uint16_t raw_angle = ((uint16_t)as5600_dma_buffer[0] << 8) | as5600_dma_buffer[1];
    raw_angle &= 0x0FFF; // 保留12位数据
    
    as5600->raw_angle = raw_angle ;
    
    // 检测圈数变化
    int16_t angle_diff = (int16_t)raw_angle - (int16_t)as5600->angle_prev;
    
    // 检测跨越零点
    if (angle_diff < -2048) { // 过零点，顺时针旋转
        as5600->full_rotations++;
    } else if (angle_diff > 2048) { // 过零点，逆时针旋转
        as5600->full_rotations--;
    }
    
    // 计算总角度（原始值）
    as5600->total_angle_raw = as5600->full_rotations * 4096 + raw_angle;
    
    // 转换为弧度
    as5600->position_rad = (float)as5600->total_angle_raw * _2PI / 4096.0f;
    
    // 计算角速度
    float angle_diff_rad = (float)angle_diff * _2PI / 4096.0f;
    if (angle_diff < -2048) angle_diff_rad += _2PI;
    else if (angle_diff > 2048) angle_diff_rad -= _2PI;
    
    // 原始速度计算
    float velocity_rad_per_sec_raw = angle_diff_rad / dt;
    as5600->velocity_rad_per_sec = velocity_rad_per_sec_raw;
    as5600->velocity_rpm = velocity_rad_per_sec_raw * 60.0f / _2PI;
    
    // 应用低通滤波 - 选择合适的滤波系数
    float alpha = 0.2f; // 滤波系数，值越小滤波效果越强，但响应越慢

    // 首次运行时初始化滤波值
    if (as5600->angle_prev_ts == 0) {
        as5600->velocity_rad_per_sec_filtered = velocity_rad_per_sec_raw;
        as5600->velocity_rpm_filtered = as5600->velocity_rpm;
    } else {
        // 对弧度/秒速度应用滤波
        as5600->velocity_rad_per_sec_filtered = low_pass_filter(
            velocity_rad_per_sec_raw, 
            as5600->velocity_rad_per_sec_filtered, 
            alpha
        );
        
        // 对RPM速度应用滤波
        as5600->velocity_rpm_filtered = low_pass_filter(
            as5600->velocity_rpm, 
            as5600->velocity_rpm_filtered, 
            alpha
        );
    }
    
    // 更新上一次的角度和时间戳
    as5600->angle_prev = raw_angle;
    as5600->angle_prev_ts = now;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    AS5600_Init(&as5600, &hi2c1);
    AS5600_StartDMAReading();
    //初始化PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    /* 使能输出 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    set_pwm_duty(0.5, 0, 0);              // 生成SVPWM模型中的基础矢量1，即对应转子零度位置
    HAL_Delay(400);                       // 保持一会，转子吸引过来需要时间
    as5600.rotor_zero_elec_angle = _electricalAngle(as5600.position_rad, POLE_PAIRS) ;
    set_pwm_duty(0, 0, 0);                // 松开电机
    HAL_Delay(10);
    // 启动定时器
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   

      /*// 可以在此处添加一些非实时处理代码，比如UART发送数据
      sprintf(uart_buffer, "角度encode:%d total_encode:%ld 总rad:%.2f rad 速度:%.2f rad/s %.2f RPM\r\n",
              as5600.raw_angle,as5600.total_angle_raw, as5600.position_rad, as5600.velocity_rad_per_sec, as5600.velocity_rpm);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

      HAL_Delay(100); // 每100ms输出一次数据*/

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 定时器中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (as5600_dma_complete) {
            // 处理上一次读取的数据
            AS5600_ProcessData(&as5600);
            // 启动新的读取
            as5600_dma_complete = 0;
            HAL_I2C_Mem_Read_DMA(&hi2c1, (AS5600_ADDR << 1), AS5600_ANGLE_REG, I2C_MEMADD_SIZE_8BIT, as5600_dma_buffer, 2);
        }
    }
    if (htim->Instance == TIM3)
    {
        velocityOpenloop(5.0f);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3) {
        // 设置标志，表示数据已准备好，在主循环中处理
        uart_data_ready = 1;

        // 立即重新启动UART接收
        HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 4);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
