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

    int stop_count = 0;

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

    int sensor1 = P7->IN & 0x80;
    int sensor2 = P7->IN & 0x40;
    int sensor3 = P7->IN & 0x20;
    int sensor4 = P7->IN & 0x10;
    int sensor5 = P7->IN & 0x08;
    int sensor6 = P7->IN & 0x04;
    int sensor7 = P7->IN & 0x02;
    int sensor8 = P7->IN & 0x01;
    int sensorright1 = P7->IN & 0x07; //00000111
    int sensorright2 = P7->IN & 0x0F; //00001111
    int sensorright3 = P7->IN & 0x1F; //00011111
    int sensorright4 = P7->IN & 0x1A; //00011010
    int sensorright5 = P7->IN & 0x1B; //00011011
    int sensorright6 = P7->IN & 0x19; //00011001

    int sensorleft1 = P7->IN & 0xE0; //11100000
    int sensorleft2 = P7->IN & 0xF0; //11110000
    int sensorleft3 = P7->IN & 0xF8; //11111000
    int sensorleft4 = P7->IN & 0x58; //01011000
    int sensorleft5 = P7->IN & 0xD8; //11011000
    int sensorleft6 = P7->IN & 0x98; //10011000

    int allsensor = P7->IN & 0xFF;  //11111111
    int allsensorl = P7->IN & 0xFE; //11111110
    int allsensorr = P7->IN & 0x7F; //01111111

    //left 경우의수
    int leftturn1 = P7->IN & 0b00100000; //3
    int leftturn2 = P7->IN & 0b00110000; //3 ?
    int leftturn3 = P7->IN & 0b00111000; //3
    int leftturn4 = P7->IN & 0b01000000; //2
    int leftturn5 = P7->IN & 0b01010000; //2
    //01011000 2 x
    int leftturn6 = P7->IN & 0b10000000; //1
    int leftturn7 = P7->IN & 0b10010000; //1
    //10011000 1 x
    int leftturn8 = P7->IN & 0b01100000; //23
    int leftturn9 = P7->IN & 0b01110000; //23
    int leftturn10 = P7->IN & 0b01111000; //23
    int leftturn11 = P7->IN & 0b11000000; //12
    int leftturn12 = P7->IN & 0b11010000; //12
    //11011000 12 x

    //right 경우의수
    int rightturn1 = P7->IN & 0b00000100; //6
    int rightturn2 = P7->IN & 0b00001100; //6 ?
    int rightturn3 = P7->IN & 0b00011100; //6
    int rightturn4 = P7->IN & 0b00000010; // 7
    int rightturn5 = P7->IN & 0b00001010; // 7
    //00011010 7 x
    int rightturn6 = P7->IN & 0b00000001; // 8
    int rightturn7 = P7->IN & 0b00001001; // 8
    //00011001 8 x
    int rightturn8 = P7->IN & 0b00000110; // 67
    int rightturn9 = P7->IN & 0b00001110; // 67
    int rightturn10 = P7->IN & 0b00011110; // 67
    int rightturn11 = P7->IN & 0b00000011; // 78
    int rightturn12 = P7->IN & 0b00001011; // 78
    //00011011 78 x

   //멈추는 경우의 수
    int stoppoint1 = P7->IN & 0b01111110;
    int stoppoint2 = P7->IN & 0b00111110;
    int stoppoint3 = P7->IN & 0b01111100;
    int stoppoint4 = P7->IN & 0b00111100;

    if(sensor4 || sensor5){
        printf("if45\n");
        move(0,0);
        left_forward();
        right_forward();
        move(1500, 1500);

        if(sensorleft1 || sensorleft3 || sensorleft4 || sensorleft5 || sensorleft6){
            printf("if45-left\n");
            move(0,0);
            right_count = 0;
                while(1){
                    left_forward();
                    right_forward();
                    move(1500, 1500);
                    if(right_count > 33){ //2cm ==33

                        if(sensor4 || sensor5){
                            printf("if45-left-front\n");
                            left_forward();
                            right_forward();
                            move(1500, 1500);
                            break;
                        }
                        else{
                            printf("if45-left90\n");
                            right_count = 0;
                            right_forward();
                            left_backward();
                            while(1){
                                move(1500, 1500);
                                if(right_count > 160){
                                    move(0,0);
                                    break;
                                }
                            };
                        }
                    }

                };
                continue;
        }

        else if(sensorright1 || sensorright3 || sensorright4 || sensorright5 || sensorright6){
            printf("if45-right\n");
                move(0,0);
                right_count = 0;
                while(1){
                    left_forward();
                    right_forward();
                    move(1500, 1500);
                    if(right_count > 33){ // 2CM

                        if(sensor4 || sensor5){
                            printf("if45-right-front\n");
                            left_forward();
                            right_forward();
                            move(1500, 1500);
                            break;
                        }
                        else{
                            printf("if45-right90\n");
                            right_count = 0;
                            left_forward();
                            right_backward();
                            while(1){
                                move(1500, 1500);
                                if(right_count > 160){
                                    move(0,0);
                                    break;
                                }
                            };
                        }
                    }

                };
                continue;
        }
        else if(allsensor || allsensorl || allsensorr){
            printf("ifall->left90\n");
            right_count = 0;
            right_forward();
            left_backward();
            while(1){
                move(1500, 1500);
                if(right_count > 160){
                    move(0,0);
                    break;
                }
            };
        }

        else if(sensor6 || sensor7 || sensor8){
            printf("if678조정\n");
            move(1500, 1350);
        }

        else if(sensor1 || sensor2 || sensor3){
            printf("if123조정\n");
            move(1350, 1500);
        }
    }
    else if(leftturn1 || leftturn2 || leftturn3 || leftturn4 || leftturn5 || leftturn6 || leftturn7 || leftturn8 || leftturn9 || leftturn10 || leftturn11 || leftturn12){
        printf("if123돌자\n");
        move(0,0);
        right_forward();
        left_backward();
        move(1000, 1000);
    }
    else if(rightturn1 || rightturn2 || rightturn3 || rightturn4 || rightturn5 ||rightturn6 || rightturn7 || rightturn8 || rightturn9 || rightturn10 || rightturn11 || rightturn12){
        printf("if678돌자\n");
        move(0,0);
        right_backward();
        left_forward();
        move(1000, 1000);
     }

    //stop point
    else if(stoppoint1 || stoppoint2 || stoppoint3 || stoppoint4){
        stop_count++;
        if(stop_count >= 2){
            move(0,0);
            Clock_Delay1ms(100);
        }
    }
    else{
        printf("stop\n");
        move(0,0);
        Clock_Delay1ms(100);
    }

   };
}
