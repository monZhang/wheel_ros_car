#include "show.h"

int Voltage_Show;
unsigned char i;
unsigned char Send_Count;
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count;
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
/**************************************************************************
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count = 25;

void show_task(void *pvParameters) {
    u32 lastWakeTime = getSysTickCnt();
    while (1) {
        int i = 0;
        static int LowVoltage_1 = 0, LowVoltage_2 = 0;
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������

        //����ʱ���������ݷ�������������
        if (Time_count < 50)Buzzer = 1;
        else if (Time_count >= 51 && Time_count < 100)Buzzer = 0;

        if (LowVoltage_1 == 1 || LowVoltage_2 == 1)Buzzer_count = 0;
        if (Buzzer_count < 5)Buzzer_count++;
        if (Buzzer_count < 5)Buzzer = 1;  //����������
        else if (Buzzer_count == 5)Buzzer = 0;

        //��ȡ��ص�ѹ
        for (i = 0; i < 10; i++) {
            Voltage_All += Get_battery_volt();
        }
        Voltage = Voltage_All / 10;
        Voltage_All = 0;

        if (LowVoltage_1 == 1)LowVoltage_1++; //ȷ��������ֻ��0.5��
        if (LowVoltage_2 == 1)LowVoltage_2++; //ȷ��������ֻ��0.5��
        if (Voltage >= 12.6f)Voltage = 12.6f;
        else if (10 <= Voltage && Voltage < 10.5f && LowVoltage_1 < 2)
            LowVoltage_1++;   //10.5V���͵���ʱ��������һ�α���
        else if (Voltage < 10 && LowVoltage_1 < 2)
            LowVoltage_2++;   //10V��С����ֹ����ʱ�������ڶ��α���

        APP_Show();      //��APP��������
        oled_show();     //��ʾ����ʾ����
    }
}

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void) {

//    if (Car_Mode_Show > 5)Car_Mode_Show = 5;
//    Car_Mode_Show = (int) ((Get_adc_Average(Potentiometer, 10)) / Divisor_Mode);
//    int Car_Mode_Show;
//    //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
//    Divisor_Mode = 2048 / CAR_NUMBER + 5;

    static int count = 0;
    Voltage_Show = Voltage * 100;
    count++;

    //û�п����Լ�ģʽʱС��������ʾ
    if (Check == 0) {

        //��ʾ����1����ʾ����
        OLED_ShowString(0, 16, "Akm ");
        //�����������١��������Ĵ�����ʾ���������
        OLED_ShowString(55, 16, "BIAS");
        if (Deviation_gyro[2] < 0) {
            OLED_ShowString(90, 16, "-");
            OLED_ShowNumber(100, 16, -Deviation_gyro[2], 3);  //Zero-drift data of gyroscope Z axis
        } else {
            OLED_ShowString(90, 16, "+");
            OLED_ShowNumber(100, 16, Deviation_gyro[2], 3);    //������z�����Ư������
        }
        //��ʾ����1����ʾ����

        //��ʾ����2����ʾ����
        //�����������١�̹��С����ʾZ����ٶ�
        OLED_ShowString(00, 17, "GYRO_Z:");
        if (gyro[2] < 0) {
            OLED_ShowString(60, 17, "-");
            OLED_ShowNumber(75, 17, -gyro[2], 5);
        } else {
            OLED_ShowString(60, 17, "+");
            OLED_ShowNumber(75, 17, gyro[2], 5);
        }
        //��ʾ����2����ʾ����//

        //��ʾ����3��4����ʾ����//
        //�����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
        OLED_ShowString(0, 18, "L:");
        if (MOTOR_A.Target < 0) {
            OLED_ShowString(15, 18, "-");
            OLED_ShowNumber(20, 18, -MOTOR_A.Target * 1000, 5);
        } else {
            OLED_ShowString(15, 18, "+");
            OLED_ShowNumber(20, 18, MOTOR_A.Target * 1000, 5);
        }
        if (MOTOR_A.Encoder < 0) {
            OLED_ShowString(60, 18, "-");
            OLED_ShowNumber(75, 18, -MOTOR_A.Encoder * 1000, 5);
        } else {
            OLED_ShowString(60, 18, "+");
            OLED_ShowNumber(75, 18, MOTOR_A.Encoder * 1000, 5);
        }
        //�����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
        OLED_ShowString(0, 19, "R:");
        if (MOTOR_B.Target < 0) {
            OLED_ShowString(15, 19, "-");
            OLED_ShowNumber(20, 19, -MOTOR_B.Target * 1000, 5);
        } else {
            OLED_ShowString(15, 19, "+");
            OLED_ShowNumber(20, 19, MOTOR_B.Target * 1000, 5);
        }
        if (MOTOR_B.Encoder < 0) {
            OLED_ShowString(60, 19, "-");
            OLED_ShowNumber(75, 19, -MOTOR_B.Encoder * 1000, 5);
        } else {
            OLED_ShowString(60, 19, "+");
            OLED_ShowNumber(75, 19, MOTOR_B.Encoder * 1000, 5);
        }
//			 if( Remoter_Ch1<0)	    OLED_ShowString(15,20,"-"),
//															OLED_ShowNumber(20,20,-Remoter_Ch1,5,12);
//			 else                 	OLED_ShowString(15,20,"+"),
//															OLED_ShowNumber(20,20, Remoter_Ch1,5,12);
//			 if( Remoter_Ch2<0)	    OLED_ShowString(60,20,"-"),
//															OLED_ShowNumber(75,20,-Remoter_Ch2,5,12);
//			 else                 	OLED_ShowString(60,20,"+"),
//															OLED_ShowNumber(75,20, Remoter_Ch2,5,12);
//			 if( Remoter_Ch3<0)	    OLED_ShowString(15,30,"-"),
//															OLED_ShowNumber(20,30,-Remoter_Ch3,5,12);
//			 else                 	OLED_ShowString(15,30,"+"),
//															OLED_ShowNumber(20,30, Remoter_Ch3,5,12);
//			 if( Remoter_Ch4<0)	    OLED_ShowString(60,30,"-"),
//															OLED_ShowNumber(75,30,-Remoter_Ch4,5,12);
//			 else                 	OLED_ShowString(60,30,"+"),
//															OLED_ShowNumber(75,30, Remoter_Ch4,5,12);
        //��ʾ����3��4����ʾ����//


        //��ʾ����5����ʾ����//
        //������С����ʾ�����PWM����ֵ
        OLED_ShowString(00, 20, "SERVO:");
        if (Servo < 0) {
            OLED_ShowString(60, 20, "-");
            OLED_ShowNumber(80, 20, -Servo, 4);
        } else {
            OLED_ShowString(60, 20, "+");
            OLED_ShowNumber(80, 20, Servo, 4);
        }
        //��ʾ����5����ʾ����//


        if (PS2_ON_Flag == 1) OLED_ShowString(0, 21, "PS2  ");
        else if (APP_ON_Flag == 1) OLED_ShowString(0, 21, "APP  ");
        else if (Remote_ON_Flag == 1)OLED_ShowString(0, 21, "R-C  ");
        else if (CAN_ON_Flag == 1) OLED_ShowString(0, 21, "CAN  ");
        else if ((Usart1_ON_Flag || Usart5_ON_Flag) == 1) OLED_ShowString(0, 21, "USART");
        else OLED_ShowString(0, 21, "ROS  ");

        //��ʾ��ǰС���Ƿ��������
        if (EN == 1 && Flag_Stop == 0) OLED_ShowString(45, 21, "O N");
        else OLED_ShowString(45, 21, "OFF");

        OLED_ShowNumber(75, 21, Voltage_Show / 100, 2);
        OLED_ShowString(88, 21, ".");
        OLED_ShowNumber(98, 21, Voltage_Show % 100, 2);
        OLED_ShowString(110, 21, "V");
        if (Voltage_Show % 100 < 10) OLED_ShowNumber(92, 21, 0, 2);
    }

    //OLED_Clear();
}

/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void) {
    static u8 flag_show;
    int Left_Figure, Right_Figure, Voltage_Show;

    //�Ե�ص�ѹ����ɰٷֱ���ʽ
    Voltage_Show = (Voltage * 1000 - 10000) / 27;
    if (Voltage_Show > 100)Voltage_Show = 100;

    //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
    Left_Figure = MOTOR_A.Encoder * 100;
    if (Left_Figure < 0)Left_Figure = -Left_Figure;
    Right_Figure = MOTOR_B.Encoder * 100;
    if (Right_Figure < 0)Right_Figure = -Right_Figure;

    //���ڽ����ӡAPP���ݺ���ʾ����
    flag_show = !flag_show;

    if (PID_Send == 1) {
        //���Ͳ�����APP��APP�ڵ��Խ�����ʾ
        printf("{C%d:%d:%d}$", (int) RC_Velocity, (int) Velocity_KP, (int) Velocity_KI);
        PID_Send = 0;
    } else if (flag_show == 0) {
        //���Ͳ�����APP��APP����ҳ��ʾ
        printf("{A%d:%d:%d:%d}$", (u8) Left_Figure, (u8) Right_Figure, Voltage_Show, (int) gyro[2]);
    } else {
        //���Ͳ�����APP��APP�ڲ��ν�����ʾ
        printf("{B%d:%d:%d}$", (int) gyro[0], (int) gyro[1], (int) gyro[2]);
    }
}


