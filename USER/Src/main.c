#include "system.h"

//�������ȼ�
#define START_TASK_PRIO    1

//�����ջ��С
#define START_STK_SIZE    256

//������
TaskHandle_t StartTask_Handler;

//������
void start_task(void *pvParameters);

//������
int main(void) {

    //Ӳ����ʼ��
    systemInit();

    //������ʼ����
    xTaskCreate((TaskFunction_t) start_task,            //������
                (const char *) "start_task",       //��������
                (uint16_t) START_STK_SIZE,     //�����ջ��С
                (void *) NULL,                 //���ݸ��������Ĳ���
                (UBaseType_t) START_TASK_PRIO,    //�������ȼ�
                (TaskHandle_t *) &StartTask_Handler);   //������
    vTaskStartScheduler();  //�����������

}

//��ʼ����������
void start_task(void *pvParameters) {
    //�����ٽ���
    taskENTER_CRITICAL();

    //��������
    xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO,
                NULL);    //Vehicle motion control task //С���˶���������
    xTaskCreate(MPU6050_task, "MPU6050_task", MPU6050_STK_SIZE, NULL, MPU6050_TASK_PRIO,
                NULL);    //IMU data read task //IMU���ݶ�ȡ����
    xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO,
               NULL); //The OLED display displays tasks //OLED��ʾ����ʾ����
    xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);    //LED light flashing task //LED����˸����
    xTaskCreate(pstwo_task, "PSTWO_task", PS2_STK_SIZE, NULL, PS2_TASK_PRIO,
                NULL);    //Read the PS2 controller task //��ȡPS2�ֱ�����
    xTaskCreate(data_task, "DATA_task", DATA_STK_SIZE, NULL, DATA_TASK_PRIO,
                NULL);    //Usartx3, Usartx1 and CAN send data task //����3������1��CAN������������

    //ɾ�����������������
    vTaskDelete(StartTask_Handler);
    //�˳��ٽ���
    taskEXIT_CRITICAL();
}
