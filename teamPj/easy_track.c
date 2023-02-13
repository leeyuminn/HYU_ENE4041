#include "msp.h"
#include "Clock.h"
#include <stdio.h>

uint16_t first_left;
uint16_t first_right;

uint16_t period_left;
uint16_t period_right;

uint32_t right_count;

//right
void TA3_0_IRQHandler(void){
    TIMER_A3->CCTL[0] &= ~0x0001;
    period_right = TIMER_A3->CCR[0] - first_right;
    first_right = TIMER_A3->CCR[0];
    right_count++;
}

//left
void TA3_N_IRQHandler(void){
    TIMER_A3->CCTL[1] &= ~0x0001;
    period_left = TIMER_A3->CCR[1] - first_left;
    first_left = TIMER_A3->CCR[1];

}

uint32_t get_left_rpm(){
    return 2000000 / period_left;
}


void led_init(){

    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;

    P2->DIR |= 0x07;

    P2->OUT &= ~0x07;
}

void turn_on_led(){
    P2->OUT |= 0x07;
}

void turn_off_led(){
    P2->OUT &= ~0x07;
}

void switch_init(){

    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;

    P1->DIR &= ~0x12;

    P1->REN |= 0x12;

    P1->OUT |= 0x12;
}

void systick_init(void){
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void systick_wait1ms(){
    SysTick->LOAD = 48000;
    SysTick->VAL = 0; //SysTick count 초기화
    while((SysTick->CTRL & 0x00010000) == 0){};
}

void systick_wait1s(){
    int i;
    int count = 1000;

    for(i = 0; i < count; i++){
        systick_wait1ms();
    }
}


void pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4){

    TIMER_A0->CCR[0] = period;

    TIMER_A0->EX0 = 0x0000;

    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    TIMER_A0->CTL = 0x02F0;

    //set alternative
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
}

void motor_init(void){
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;

    pwm_init34(7500, 0, 0);
}

void sensor_init(void){
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
}


void move(uint16_t leftDuty, uint16_t rightDuty){
    P3->OUT |= 0xC0;
    TIMER_A0->CCR[3] = leftDuty;
    TIMER_A0->CCR[4] = rightDuty;
}

void left_forward(){
    P5->OUT &= ~0x10;
}

void left_backward(){
    P5->OUT |= 0x10;
}

void right_forward(){
    P5->OUT &= ~0x20;
}

void right_backward(){
    P5->OUT |= 0x20;
}

void (*TimerA2Task)(void);

void TimerA2_Init(void(*task)(void), uint16_t period){
    TimerA2Task = task;
    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (period - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFFFF00)|0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;
}

void TA2_0_IRQHandler(void){
    TIMER_A2->CCTL[0] &= ~0x0001;
    (*TimerA2Task)();
}

void task(){
    printf("interrupt occurs!\n");
}

void timer_A3_capture_init(){
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;

    TIMER_A3->CTL &= ~0x0030;
    TIMER_A3->CTL = 0x0200;

    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;

    NVIC->IP[3] = (NVIC->IP[3]&0x0000FFFF) | 0x404000000;
    NVIC->ISER[0] = 0x0000C000;
    TIMER_A3->CTL |= 0x0024;
}

/**
 * main.c
 */
void main(void)
{
    timer_A3_capture_init();
    Clock_Init48MHz();
    //led_init();
    systick_init();
    motor_init();
    //switch_init()
    sensor_init();
    //TimerA2_Init(&task, 50000);




    while(1){
    //turn on IR LEDs
    P5->OUT |=0x08;
    P9->OUT |=0x04;

    //make P7.0-P7.7 as output
    P7->DIR = 0XFF;

    //charges a capacitor
    P7->OUT = 0xFF;

    //wait for fully charged
    Clock_Delay1us(1500); // 검정색 charge 대기 시간

    //make P7.0-P7.7 as input
    P7->DIR = 0x00;

    Clock_Delay1us(1000);

    int sensor45 = P7->IN & 0x18; // 4, 5번째 센서
    int sensor1 = P7->IN & 0x80;
    int sensor2 = P7->IN & 0x40;
    int sensor3 = P7->IN & 0x20;
    int sensor6 = P7->IN & 0x04;
    int sensor7 = P7->IN & 0x02;
    int sensor8 = P7->IN & 0x01;


    if(sensor45){
        left_forward();
        right_forward();
        move(1000, 1000);

        if(sensor3){
            move(1000, 500);
        }
        if(sensor6){
            move(500, 1000);
        }

        if(sensor2 && sensor3 && sensor6 && sensor7){
            move(0,0);
            Clock_Delay1ms(100);
        }

    }
    else if(sensor6 || sensor7 || sensor8){
        left_forward();
        right_backward();
        move(500, 500);
        printf("678\n");
    }

    else if(sensor1 || sensor2 || sensor3){
        right_forward();
        left_backward();
        move(500, 500);
        printf("123\n");
    }

    else{
        move(0,0);
        Clock_Delay1ms(100);
    }
    }
}