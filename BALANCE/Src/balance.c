#include "balance.h"

int Time_count = 0; //计时变量

//机器人模式是否出错检测标志位
int robot_mode_check_flag = 0;

short test_num;

Encoder OriginalEncoder;   //编码器原始数据

u8 command_lost_count = 0; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
/**************************************************************************
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx, float Vy, float Vz) {
    float amplitude = 3.5;      //车轮目标速度限幅

    //阿克曼小车专用相关变量
    float R, Ratio = 636.56, AngleR, Angle_Servo;

    //对于阿克曼小车Vz代表右前轮转向角度
    AngleR = Vz;
    R = Axle_spacing / tan(AngleR) - 0.5f * Wheel_spacing;

    //前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
    AngleR = target_limit_float(AngleR, -0.49f, 0.32f);

    //运动学逆解
    if (AngleR != 0) {
        MOTOR_A.Target = Vx * (R - 0.5f * Wheel_spacing) / R;
        MOTOR_B.Target = Vx * (R + 0.5f * Wheel_spacing) / R;
    } else {
        MOTOR_A.Target = Vx;
        MOTOR_B.Target = Vx;
    }
    //舵机PWM值，舵机控制前轮转向角度
    Angle_Servo = -0.628f * pow(AngleR, 3) + 1.269f * pow(AngleR, 2) - 1.772f * AngleR + 1.573f;
    Servo = SERVO_INIT + (Angle_Servo - 1.572f) * Ratio;

    //车轮(电机)目标速度限幅
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
    MOTOR_C.Target = 0;         //没有使用到
    MOTOR_D.Target = 0;         //没有使用到
    Servo = target_limit_int(Servo, 800, 2200);         //舵机PWM值限幅

}

/**************************************************************************
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters) {
    u32 lastWakeTime = getSysTickCnt();

    while (1) {
        //此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        //时间计数，30秒后不再需要
        if (Time_count < 3000)Time_count++;

        //获取编码器数据，即车轮实时速度，并转换位国际单位
        Get_Velocity_Form_Encoder();

        //如果没有启动自检模式
        if (Check == 0) {
//				command_lost_count++; //串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
//				if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0) //不是APP、PS2、航模遥控模式，就是CAN、串口1、串口3控制模式
//					Move_X=0, Move_Y=0, Move_Z=0;

            if (APP_ON_Flag) {
                //处理APP遥控命令
                Get_RC();
            } else if (Remote_ON_Flag) {
                //处理航模遥控命令
                Remote_Control();
            } else if (PS2_ON_Flag) {
                //处理PS2手柄控制命令
                PS2_control();
            } else {
                //CAN、串口1、串口3(ROS)、串口5控制直接得到三轴目标速度，无须额外处理
                Drive_Motor(Move_X, Move_Y, Move_Z);
            }

            //单击用户按键更新陀螺仪零点
            Key();

            //如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
            if (Turn_Off(Voltage) == 0) {
                //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
                MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
                MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
                MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
                MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

                Limit_Pwm(16700);

                //根据不同小车型号设置不同的PWM控制极性 //阿克曼小车
                Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo);
            } else {
                //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
                Set_Pwm(0, 0, 0, 0, 0);
            }
        }
    }
}

/**************************************************************************
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo) {
    //电机正反转控制
    if (motor_a < 0) PWMA1 = 16799, PWMA2 = 16799 + motor_a;
    else
        PWMA2 = 16799, PWMA1 = 16799 - motor_a;

    //电机正反转控制
    if (motor_b < 0) PWMB1 = 16799, PWMB2 = 16799 + motor_b;
    else
        PWMB2 = 16799, PWMB1 = 16799 - motor_b;
    //  PWMB1=10000,PWMB2=5000;

    //电机正反转控制
    if (motor_c < 0) PWMC1 = 16799, PWMC2 = 16799 + motor_c;
    else
        PWMC2 = 16799, PWMC1 = 16799 - motor_c;

    //电机正反转控制
    if (motor_d < 0) PWMD1 = 16799, PWMD2 = 16799 + motor_d;
    else
        PWMD2 = 16799, PWMD1 = 16799 - motor_d;

    //舵机控制
    Servo_PWM = servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude) {
    MOTOR_A.Motor_Pwm = target_limit_float(MOTOR_A.Motor_Pwm, -amplitude, amplitude);
    MOTOR_B.Motor_Pwm = target_limit_float(MOTOR_B.Motor_Pwm, -amplitude, amplitude);
    MOTOR_C.Motor_Pwm = target_limit_float(MOTOR_C.Motor_Pwm, -amplitude, amplitude);
    MOTOR_D.Motor_Pwm = target_limit_float(MOTOR_D.Motor_Pwm, -amplitude, amplitude);
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert, float low, float high) {
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

int target_limit_int(int insert, int low, int high) {
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}

/**************************************************************************
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off(int voltage) {
    u8 temp;
    if (voltage < 10 || EN == 0 || Flag_Stop == 1) {
        temp = 1;
        PWMA1 = 0;
        PWMA2 = 0;
        PWMB1 = 0;
        PWMB2 = 0;
        PWMC1 = 0;
        PWMC1 = 0;
        PWMD1 = 0;
        PWMD2 = 0;
    } else
        temp = 0;
    return temp;
}

/**************************************************************************
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a) {
    u32 temp;
    if (a < 0) temp = -a;
    else temp = a;
    return temp;
}

/**************************************************************************
函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //保存上一次偏差
    return Pwm;
}

int Incremental_PI_B(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //保存上一次偏差
    return Pwm;
}

int Incremental_PI_C(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //保存上一次偏差
    return Pwm;
}

int Incremental_PI_D(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //保存上一次偏差
    return Pwm;
}

/**************************************************************************
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void) {
    //处理方向控制命令
    switch (Flag_Direction) {
        case 1:
            Move_X = +RC_Velocity;
            Move_Z = 0;
            break;
        case 2:
            Move_X = +RC_Velocity;
            Move_Z = -PI / 2;
            break;
        case 3:
            Move_X = 0;
            Move_Z = -PI / 2;
            break;
        case 4:
            Move_X = -RC_Velocity;
            Move_Z = -PI / 2;
            break;
        case 5:
            Move_X = -RC_Velocity;
            Move_Z = 0;
            break;
        case 6:
            Move_X = -RC_Velocity;
            Move_Z = +PI / 2;
            break;
        case 7:
            Move_X = 0;
            Move_Z = +PI / 2;
            break;
        case 8:
            Move_X = +RC_Velocity;
            Move_Z = +PI / 2;
            break;
        default:
            Move_X = 0;
            Move_Z = 0;
            break;
    }
    if (Flag_Left == 1) Move_Z = PI / 2;            //左自转
    else if (Flag_Right == 1) Move_Z = -PI / 2;     //右自转

    //Z轴数据转化
    //阿克曼结构小车转换为前轮转向角度
    Move_Z = Move_Z * 2 / 9;

    //单位转换，mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void) {
    int LX, LY, RY;
    int Threshold = 20; //阈值，忽略摇杆小幅度动作

    //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
    LY = -(PS2_LX - 128);
    LX = -(PS2_LY - 128);
    RY = -(PS2_RX - 128);

    //忽略摇杆小幅度动作
    if (LX > -Threshold && LX < Threshold)LX = 0;
    if (LY > -Threshold && LY < Threshold)LY = 0;
    if (RY > -Threshold && RY < Threshold)RY = 0;

    if (PS2_KEY == 11) RC_Velocity += 5;        //加速
    else if (PS2_KEY == 9) RC_Velocity -= 5;    //减速

    if (RC_Velocity < 0) RC_Velocity = 0;

    //对PS2手柄控制命令进行处理
    Move_X = LX * RC_Velocity / 128;
    Move_Y = LY * RC_Velocity / 128;
    Move_Z = RY * (PI / 2) / 128;

    //Z轴数据转化
    //阿克曼结构小车转换为前轮转向角度
    Move_Z = Move_Z * 2 / 9;


    //单位转换，mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void) {
    //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice = 100;
    //阈值，忽略摇杆小幅度动作
    int Threshold = 100;

    //限幅
    int LX, LY, RY, RX, Remote_RCvelocity;
    Remoter_Ch1 = target_limit_int(Remoter_Ch1, 1000, 2000);
    Remoter_Ch2 = target_limit_int(Remoter_Ch2, 1000, 2000);
    Remoter_Ch3 = target_limit_int(Remoter_Ch3, 1000, 2000);
    Remoter_Ch4 = target_limit_int(Remoter_Ch4, 1000, 2000);

    //左摇杆前后方向。控制前进后退。
    LX = Remoter_Ch2 - 1500;

    //左摇杆左右方向。控制左右移动。麦轮全向轮才会使用到改通道。阿克曼小车使用该通道作为PWM输出控制舵机
    LY = Remoter_Ch4 - 1500;

    //右摇杆前后方向。油门/加减速。
    RX = Remoter_Ch3 - 1500;

    //右摇杆左右方向。控制自转。
    RY = Remoter_Ch1 - 1500;

    if (LX > -Threshold && LX < Threshold)LX = 0;
    if (LY > -Threshold && LY < Threshold)LY = 0;
    if (RX > -Threshold && RX < Threshold)RX = 0;
    if (RY > -Threshold && RY < Threshold)RY = 0;

    //油门相关
    Remote_RCvelocity = RC_Velocity + RX;
    if (Remote_RCvelocity < 0)Remote_RCvelocity = 0;

    //对航模遥控控制命令进行处理
    Move_X = LX * Remote_RCvelocity / 500;
    Move_Y = -LY * Remote_RCvelocity / 500;
    Move_Z = -RY * (PI / 2) / 500;

    //Z轴数据转化
    //阿克曼结构小车转换为前轮转向角度
    Move_Z = Move_Z * 2 / 9;

    //单位转换，mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //对进入航模控制模式后1秒内的数据不处理
    if (thrice > 0) Move_X = 0, Move_Z = 0, thrice--;

    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void) {
    u8 tmp;
    tmp = click_N_Double_MPU6050(50);
    if (tmp == 2)
        memcpy(Deviation_gyro, Original_gyro, sizeof(gyro)), memcpy(Deviation_accel, Original_accel, sizeof(accel));
}

/**************************************************************************
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void) {
    //获取编码器的原始数据
    float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr, Encoder_D_pr;
    OriginalEncoder.A = Read_Encoder(2);
    OriginalEncoder.B = Read_Encoder(3);
    OriginalEncoder.C = Read_Encoder(4);
    OriginalEncoder.D = Read_Encoder(5);

    //根据不同小车型号决定编码器数值极性
    Encoder_A_pr = OriginalEncoder.A;
    Encoder_B_pr = -OriginalEncoder.B;
    Encoder_C_pr = OriginalEncoder.C;
    Encoder_D_pr = OriginalEncoder.D;

    //编码器原始数据转换为车轮速度，单位m/s
    MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_C.Encoder = Encoder_C_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_D.Encoder = Encoder_D_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
}

/**************************************************************************
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx, float vy, float vz) {
    float step = 0.01;

    if (vx > 0) smooth_control.VX += step;
    else if (vx < 0) smooth_control.VX -= step;
    else if (vx == 0) smooth_control.VX = smooth_control.VX * 0.9f;

    if (vy > 0) smooth_control.VY += step;
    else if (vy < 0) smooth_control.VY -= step;
    else if (vy == 0) smooth_control.VY = smooth_control.VY * 0.9f;

    if (vz > 0) smooth_control.VZ += step;
    else if (vz < 0) smooth_control.VZ -= step;
    else if (vz == 0) smooth_control.VZ = smooth_control.VZ * 0.9f;

    smooth_control.VX = target_limit_float(smooth_control.VX, -float_abs(vx), float_abs(vx));
    smooth_control.VY = target_limit_float(smooth_control.VY, -float_abs(vy), float_abs(vy));
    smooth_control.VZ = target_limit_float(smooth_control.VZ, -float_abs(vz), float_abs(vz));
}

/**************************************************************************
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert) {
    if (insert >= 0) return insert;
    else return -insert;
}

/**************************************************************************
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void) {
    static u8 error = 0;

    if (abs(MOTOR_A.Motor_Pwm) > 2500 || abs(MOTOR_B.Motor_Pwm) > 2500 || abs(MOTOR_C.Motor_Pwm) > 2500 ||
        abs(MOTOR_D.Motor_Pwm) > 2500)
        error++;
    //If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
    //如果连续6次接近满幅输出，判断为电机乱转，让电机失能
    if (error > 6) EN = 0, Flag_Stop = 1, robot_mode_check_flag = 1;
}
