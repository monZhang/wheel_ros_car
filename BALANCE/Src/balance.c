#include "balance.h"

int Time_count = 0; //��ʱ����

//������ģʽ�Ƿ�������־λ
int robot_mode_check_flag = 0;

short test_num;

Encoder OriginalEncoder;   //������ԭʼ����

u8 command_lost_count = 0; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
/**************************************************************************
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx, float Vy, float Vz) {
    float amplitude = 3.5;      //����Ŀ���ٶ��޷�

    //������С��ר����ر���
    float R, Ratio = 636.56, AngleR, Angle_Servo;

    //���ڰ�����С��Vz������ǰ��ת��Ƕ�
    AngleR = Vz;
    R = Axle_spacing / tan(AngleR) - 0.5f * Wheel_spacing;

    //ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
    AngleR = target_limit_float(AngleR, -0.49f, 0.32f);

    //�˶�ѧ���
    if (AngleR != 0) {
        MOTOR_A.Target = Vx * (R - 0.5f * Wheel_spacing) / R;
        MOTOR_B.Target = Vx * (R + 0.5f * Wheel_spacing) / R;
    } else {
        MOTOR_A.Target = Vx;
        MOTOR_B.Target = Vx;
    }
    //���PWMֵ���������ǰ��ת��Ƕ�
    Angle_Servo = -0.628f * pow(AngleR, 3) + 1.269f * pow(AngleR, 2) - 1.772f * AngleR + 1.573f;
    Servo = SERVO_INIT + (Angle_Servo - 1.572f) * Ratio;

    //����(���)Ŀ���ٶ��޷�
    MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
    MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
    MOTOR_C.Target = 0;         //û��ʹ�õ�
    MOTOR_D.Target = 0;         //û��ʹ�õ�
    Servo = target_limit_int(Servo, 800, 2200);         //���PWMֵ�޷�

}

/**************************************************************************
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters) {
    u32 lastWakeTime = getSysTickCnt();

    while (1) {
        //��������100Hz��Ƶ�����У�10ms����һ�Σ�
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        //ʱ�������30�������Ҫ
        if (Time_count < 3000)Time_count++;

        //��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
        Get_Velocity_Form_Encoder();

        //���û�������Լ�ģʽ
        if (Check == 0) {
//				command_lost_count++; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
//				if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0) //����APP��PS2����ģң��ģʽ������CAN������1������3����ģʽ
//					Move_X=0, Move_Y=0, Move_Z=0;

            if (APP_ON_Flag) {
                //����APPң������
                Get_RC();
            } else if (Remote_ON_Flag) {
                //����ģң������
                Remote_Control();
            } else if (PS2_ON_Flag) {
                //����PS2�ֱ���������
                PS2_control();
            } else {
                //CAN������1������3(ROS)������5����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
                Drive_Motor(Move_X, Move_Y, Move_Z);
            }

            //�����û������������������
            Key();

            //�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0
            if (Turn_Off(Voltage) == 0) {
                //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��
                MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
                MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
                MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
                MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

                Limit_Pwm(16700);

                //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ��� //������С��
                Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo);
            } else {
                //���Turn_Off(Voltage)����ֵΪ1�����������С�������˶���PWMֵ����Ϊ0
                Set_Pwm(0, 0, 0, 0, 0);
            }
        }
    }
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo) {
    //�������ת����
    if (motor_a < 0) PWMA1 = 16799, PWMA2 = 16799 + motor_a;
    else
        PWMA2 = 16799, PWMA1 = 16799 - motor_a;

    //�������ת����
    if (motor_b < 0) PWMB1 = 16799, PWMB2 = 16799 + motor_b;
    else
        PWMB2 = 16799, PWMB1 = 16799 - motor_b;
    //  PWMB1=10000,PWMB2=5000;

    //�������ת����
    if (motor_c < 0) PWMC1 = 16799, PWMC2 = 16799 + motor_c;
    else
        PWMC2 = 16799, PWMC1 = 16799 - motor_c;

    //�������ת����
    if (motor_d < 0) PWMD1 = 16799, PWMD2 = 16799 + motor_d;
    else
        PWMD2 = 16799, PWMD1 = 16799 - motor_d;

    //�������
    Servo_PWM = servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
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
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
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
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a) {
    u32 temp;
    if (a < 0) temp = -a;
    else temp = a;
    return temp;
}

/**************************************************************************
�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //������һ��ƫ��
    return Pwm;
}

int Incremental_PI_B(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //������һ��ƫ��
    return Pwm;
}

int Incremental_PI_C(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //������һ��ƫ��
    return Pwm;
}

int Incremental_PI_D(float Encoder, float Target) {
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; //Calculate the deviation //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if (Pwm > 16700)Pwm = 16700;
    if (Pwm < -16700)Pwm = -16700;
    Last_bias = Bias; //Save the last deviation //������һ��ƫ��
    return Pwm;
}

/**************************************************************************
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void) {
    //�������������
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
    if (Flag_Left == 1) Move_Z = PI / 2;            //����ת
    else if (Flag_Right == 1) Move_Z = -PI / 2;     //����ת

    //Z������ת��
    //�������ṹС��ת��Ϊǰ��ת��Ƕ�
    Move_Z = Move_Z * 2 / 9;

    //��λת����mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void) {
    int LX, LY, RY;
    int Threshold = 20; //��ֵ������ҡ��С���ȶ���

    //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
    LY = -(PS2_LX - 128);
    LX = -(PS2_LY - 128);
    RY = -(PS2_RX - 128);

    //����ҡ��С���ȶ���
    if (LX > -Threshold && LX < Threshold)LX = 0;
    if (LY > -Threshold && LY < Threshold)LY = 0;
    if (RY > -Threshold && RY < Threshold)RY = 0;

    if (PS2_KEY == 11) RC_Velocity += 5;        //����
    else if (PS2_KEY == 9) RC_Velocity -= 5;    //����

    if (RC_Velocity < 0) RC_Velocity = 0;

    //��PS2�ֱ�����������д���
    Move_X = LX * RC_Velocity / 128;
    Move_Y = LY * RC_Velocity / 128;
    Move_Z = RY * (PI / 2) / 128;

    //Z������ת��
    //�������ṹС��ת��Ϊǰ��ת��Ƕ�
    Move_Z = Move_Z * 2 / 9;


    //��λת����mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void) {
    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice = 100;
    //��ֵ������ҡ��С���ȶ���
    int Threshold = 100;

    //�޷�
    int LX, LY, RY, RX, Remote_RCvelocity;
    Remoter_Ch1 = target_limit_int(Remoter_Ch1, 1000, 2000);
    Remoter_Ch2 = target_limit_int(Remoter_Ch2, 1000, 2000);
    Remoter_Ch3 = target_limit_int(Remoter_Ch3, 1000, 2000);
    Remoter_Ch4 = target_limit_int(Remoter_Ch4, 1000, 2000);

    //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX = Remoter_Ch2 - 1500;

    //��ҡ�����ҷ��򡣿��������ƶ�������ȫ���ֲŻ�ʹ�õ���ͨ����������С��ʹ�ø�ͨ����ΪPWM������ƶ��
    LY = Remoter_Ch4 - 1500;

    //��ҡ��ǰ��������/�Ӽ��١�
    RX = Remoter_Ch3 - 1500;

    //��ҡ�����ҷ��򡣿�����ת��
    RY = Remoter_Ch1 - 1500;

    if (LX > -Threshold && LX < Threshold)LX = 0;
    if (LY > -Threshold && LY < Threshold)LY = 0;
    if (RX > -Threshold && RX < Threshold)RX = 0;
    if (RY > -Threshold && RY < Threshold)RY = 0;

    //�������
    Remote_RCvelocity = RC_Velocity + RX;
    if (Remote_RCvelocity < 0)Remote_RCvelocity = 0;

    //�Ժ�ģң�ؿ���������д���
    Move_X = LX * Remote_RCvelocity / 500;
    Move_Y = -LY * Remote_RCvelocity / 500;
    Move_Z = -RY * (PI / 2) / 500;

    //Z������ת��
    //�������ṹС��ת��Ϊǰ��ת��Ƕ�
    Move_Z = Move_Z * 2 / 9;

    //��λת����mm/s -> m/s
    Move_X = Move_X / 1000;
    Move_Y = Move_Y / 1000;
    Move_Z = Move_Z;

    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if (thrice > 0) Move_X = 0, Move_Z = 0, thrice--;

    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void) {
    u8 tmp;
    tmp = click_N_Double_MPU6050(50);
    if (tmp == 2)
        memcpy(Deviation_gyro, Original_gyro, sizeof(gyro)), memcpy(Deviation_accel, Original_accel, sizeof(accel));
}

/**************************************************************************
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void) {
    //��ȡ��������ԭʼ����
    float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr, Encoder_D_pr;
    OriginalEncoder.A = Read_Encoder(2);
    OriginalEncoder.B = Read_Encoder(3);
    OriginalEncoder.C = Read_Encoder(4);
    OriginalEncoder.D = Read_Encoder(5);

    //���ݲ�ͬС���ͺž�����������ֵ����
    Encoder_A_pr = OriginalEncoder.A;
    Encoder_B_pr = -OriginalEncoder.B;
    Encoder_C_pr = OriginalEncoder.C;
    Encoder_D_pr = OriginalEncoder.D;

    //������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
    MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_C.Encoder = Encoder_C_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_D.Encoder = Encoder_D_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
}

/**************************************************************************
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
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
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert) {
    if (insert >= 0) return insert;
    else return -insert;
}

/**************************************************************************
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void) {
    static u8 error = 0;

    if (abs(MOTOR_A.Motor_Pwm) > 2500 || abs(MOTOR_B.Motor_Pwm) > 2500 || abs(MOTOR_C.Motor_Pwm) > 2500 ||
        abs(MOTOR_D.Motor_Pwm) > 2500)
        error++;
    //If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
    //�������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��
    if (error > 6) EN = 0, Flag_Stop = 1, robot_mode_check_flag = 1;
}
