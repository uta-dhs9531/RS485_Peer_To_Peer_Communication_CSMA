// Serial C/ASM Mix Example
// Darshil Shah

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define MAX_CHARS               80
#define MAX_FIELDS              50
#define N
#define A
#define SIZEP                   20
#define ALL_ADD                 255
#define MY_NULL                 0x00
#define CMD_ACK                 0x70
#define DATA_SIZE_ACK           1
#define CMD_SET                 0
#define MAX_MSGS                20
#define MAXIMUM_RETRIES         3
#define CMD_POLL                0x78
#define DATA_SIZE_POLL          0
#define CMD_SA                  0x7A
#define DATA_SIZE_SA            1
#define CMD_RESET               0x7F
#define DATA_SIZE_RESET         0
#define DATA_SIZE_RGB           3
#define CMD_RGB                 0x48
#define red_led_onBoard_mask    32
#define green_led_onBoard_mask  16
#define CMD_GET_REQ             0x48
#define DATA_SIZE_POLL_REQ      0
#define CMD_GET_RES             0x31
#define DATA_SIZE_GET_RES       1
#define DATA_SIZE_GET_REQ       1

#define DE             (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define green_led_onBoard    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define red_led_onBoard      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0X40000000)*32 + 4*4)))

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

typedef struct _TX_485
{
    uint8_t DST_ADDRESS;
    uint8_t SRC_ADDRESS;
    uint8_t SEQ_ID;
    uint8_t COMMAND;
    uint8_t CHANNEL;
    uint8_t SIZE;
    uint8_t DATA[SIZEP];
    uint8_t CHECKSUM;
    bool ack;
    uint8_t VALID;

} TX_485;

TX_485 TX485MESSAGE[255];

bool CST1, CST2;
bool ackEnabled, csEnabled, randomEnabled, alertEnabled;
bool randre;
bool flagGreen = false;
uint8_t checksum;
int16_t message_in_progress = -1;
uint8_t message_phase = 0;
uint8_t sumdata = 0;
uint8_t rxphase = 0;
uint8_t rxdata[MAX_CHARS] = { 0 };
uint8_t SRC_ADD = 0x01;
uint8_t check_temp = 0;
uint8_t retranscount[MAX_MSGS];
uint16_t retranstimeout[MAX_MSGS] = { 0 };
uint8_t number = 0;
uint8_t sequence[10] = { 2, 6, 9, 7, 3, 1, 7, 10, 1, 6 }; //Sequence used to generate random numbers for Retranstimeout
uint16_t txtimeout = 0;
uint16_t timerToNullify = 0;
uint8_t d[MAX_MSGS];
uint8_t address = 0; //Used to store the address of the given command
uint8_t my_na[0] = { 0 };
uint8_t value[MAX_CHARS] = { 0 }; //Used to store the data/value of the given command
uint8_t WriteIndex = 0;
uint8_t str[12] = { 'G', 'O', 'O', 'D', ' ', 'M', 'O', 'R', 'N', 'I', 'N', 'G' };
uint8_t ReadIndex;
uint8_t WriteIndex;
static uint8_t x;
uint8_t v;
static uint8_t u;

//Temporary variables for sprintf
char temp1[MAX_CHARS];
char temp3[MAX_CHARS];
char temp7[MAX_CHARS];

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void send_rs_485_byte()
{
    DE = 1;
//uint8_t c=0;
    static uint8_t i;

    if (message_in_progress == -1)
    {
        for (i = 0; i < 255; i++)
        {
            if (TX485MESSAGE[i].VALID == 1)
            {
                message_phase = 0;
                message_in_progress = 0;
                break;
            }
        }
    }
    if (message_in_progress == 0)
    {
        switch (message_phase)
        {
        case 0:
            UART1_LCRH_R &= ~UART_LCRH_EPS;
            UART1_DR_R = TX485MESSAGE[i].DST_ADDRESS;
            message_phase = message_phase + 1;
            break;
        case 1:
            UART1_LCRH_R |= UART_LCRH_EPS;
            UART1_DR_R = TX485MESSAGE[i].SRC_ADDRESS;
            message_phase = message_phase + 1;

            break;

        case 2:
            UART1_DR_R = TX485MESSAGE[i].SEQ_ID;
            message_phase = message_phase + 1;
            break;
        case 3:
            UART1_DR_R = TX485MESSAGE[i].COMMAND;
            message_phase = message_phase + 1;
            break;

        case 4:
            UART1_DR_R = TX485MESSAGE[i].CHANNEL;
            message_phase = message_phase + 1;
            break;
        case 5:
            UART1_DR_R = TX485MESSAGE[i].SIZE;
            message_phase = message_phase + 1;
            break;
        default:
            if (message_phase == 5 + (TX485MESSAGE[i].SIZE))
            {
                for (x = 0; x <= TX485MESSAGE[i].SIZE - 1; x++)
                {
                    UART1_DR_R = TX485MESSAGE[i].DATA[x];
                }

                message_phase = message_phase + 1;
                break;
            }
            else if (message_phase == 6 + (TX485MESSAGE[i].SIZE))
            {
                {
                    UART1_DR_R = TX485MESSAGE[i].CHECKSUM;
                    message_phase = message_phase + 1;
                    break;
                }
            }
            else
            {
                if (message_phase == 7 + (TX485MESSAGE[i].SIZE))
                {
                    message_phase = 0;
                    message_in_progress = -1;
                    TX485MESSAGE[i].VALID = 0;
                    DE = 0;
                }

//                                                    else if(message_phase == (TX485MESSAGE[i].SIZE+7))
//                                                    {
//                                                        {
//                                                            txtimeout=0;
//                                                            DE=0;
//                                                            message_in_progress =-1;
//                                                            message_phase = 0;
//                                                         if(ackEnabled == 0)
//                                                             TX485MESSAGE[i].VALID=0;
//                                                         else if(ack == 1)
//                                                             {if(TX485MESSAGE[i].COMMAND == 0x70)
//                                                                 TX485MESSAGE[i].VALID=0;
//                                                             else
//                                                             {
//                                                             retranscount[i]++;
                //                                                                if(randre==0)
//                                                                 {
//                                                                     retranstimeout[i] = 600 + (2^retranscount[i])*100;  //Can be changed according to the requirement
//                                                                 }
//                                                                 else
//                                                                 {
////                                                                 retranstimeout[i] = 600 + (2^sequence[number])*100;
////                                                                 number=(number+1)%10;
//                                                                 }
//                                                                 if(retranscount[i] > 1)
//                                                                     {
//                                                                   sprintf(temp3,"Transmitting message %u, retransmission attempt %u\r\n",TX485MESSAGE[i].SEQ_ID[i],(retranscount[i]-1));
//                                                                     putsUart0(temp3);
//                                                                     if(retranscount[i] > MAXIMUM_RETRIES)
//                                                                         {
//                                                                         TX485MESSAGE[i].VALID=0;
////                                                                             RED_BOARD = 1;
////                                                                             u = 2000;                                                                             red_flag = true;
//                                                                            sprintf(temp1,"Message %u was retransmitted %u times, but not succeeded\r\n",TX485MESSAGE[i].SEQ_ID[i],retranscount[i]-1);
//                                                                             putsUart0(temp1);
//                                                                         }
//                                                                    }
            }                         // Else Checksum bracket

        }                         //Switch Case Start Bracket

    }                             // Message in progress=0 bracket Start Bracket

}                                     // Send Rs 485 byte bracket

//                                           }
//                           }
//}
//}

void greenFlag()
{
    static uint8_t v;

    {
        v--;
        if (v == 0)
        {

            green_led_onBoard = 0;
        }
    }
}
void redFlag()
{
    {
        u--;
        if (u == 0)
        {

            red_led_onBoard = 0;
        }
    }
}

void send_RS_485(uint8_t DST_ADDRESS, uint8_t CHANNEL, uint8_t COMM,
                 uint8_t SIZE, uint8_t val[])
{
    int x = 0;
    int q = 0;
//uint8_t checksum=0;

    for (q = 0; q <= 255; q++)
    {
        if (TX485MESSAGE[q].VALID == 0)
        {
            TX485MESSAGE[q].DST_ADDRESS = DST_ADDRESS;
            TX485MESSAGE[q].SRC_ADDRESS = SRC_ADD;
            TX485MESSAGE[q].SEQ_ID++ & 255;
            TX485MESSAGE[q].CHANNEL = CHANNEL;
//            if(ack==1)
//            {
//                TX485MESSAGE[q].COMMAND=COMM;
//                //7thbitis 1
//                TX485MESSAGE[q].COMMAND|=(1<<7);                                                    //Command with Ack putting queue
//            }
//            else
//            {
            TX485MESSAGE[q].COMMAND = COMM;
            //7th bit is 0
            //                 TX485MESSAGE[q].COMMAND|=(0<<7);
//            }
            TX485MESSAGE[q].SIZE = SIZE;               //size putting into Queue
            for (x = 0; x <= SIZE - 1; x++)
            {
                TX485MESSAGE[q].DATA[x] = *val;        //Data putting into Queue
                sumdata = sumdata + TX485MESSAGE[q].DATA[x];
                TX485MESSAGE[q].CHECKSUM = TX485MESSAGE[q].CHECKSUM + sumdata;
                *val++;
            }
            TX485MESSAGE[q].CHECKSUM = ~(TX485MESSAGE[q].DST_ADDRESS
                    + TX485MESSAGE[q].SRC_ADDRESS + TX485MESSAGE[q].SEQ_ID
                    + TX485MESSAGE[q].CHANNEL + TX485MESSAGE[q].COMMAND
                    + TX485MESSAGE[q].SIZE + TX485MESSAGE[q].CHECKSUM); //Checksum putting into Queue

            TX485MESSAGE[q].VALID = 1;
            if (UART1_FR_R && UART_FR_TXFE)

            {
                send_rs_485_byte();
            }

            break;

        }
    }
}

void Acknowledgemsg()
{
    uint8_t o[MAX_MSGS] = { 0 };
    o[0] = rxdata[2];
    send_RS_485(rxdata[1], MY_NULL, CMD_ACK, DATA_SIZE_ACK, o);
}

bool checksum_check()
{
    check_temp = 0;
    static uint8_t z = 0;
    for (z = 0; z < rxdata[5] + 6; z++)
    {
        check_temp = check_temp + rxdata[z];
        check_temp = ~check_temp;
    }
    if (rxdata[3] == 0 ||rxdata[3]==128)
    {
        check_temp = check_temp - 4;
    }
    else if (rxdata[3] == 48)
    {
        check_temp = 37;
    }

    if (check_temp == rxdata[6 + rxdata[5]])
    {
        return true;
    }
    else
        return false;
}

void processmsg()
{
    //code to turn on/off RED LED in channel 1 through SET (channel 1)
    if ((rxdata[3] == CMD_SET || rxdata[3] == 0x80) && (rxdata[4] == 1))
    {
        if (checksum_check())
        {
            if (rxdata[6] == 1)
            {

                PWM1_2_CMPB_R = 255;
            }
            else
                PWM1_2_CMPB_R = 0;
            if ((rxdata[3] & 0x80) == 0x80)
                Acknowledgemsg();
        }

    }

    //code for receiving set command in channel 1 through 3
    if ((rxdata[3] == CMD_SET || rxdata[3] == 0x80) && (rxdata[4] == 2)
            && (rxdata[5] == 1))
    {
        if (checksum_check())
        {
            if (rxdata[6] == 1)
                putsUart0("I am here");

                PWM1_3_CMPB_R = 230;
            else
                PWM1_3_CMPB_R = 0;
            if ((rxdata[3] & 0x80) == 0x80)
                Acknowledgemsg();
        }

    }

    if ((rxdata[3] == CMD_SET || rxdata[3] == 0x80) && (rxdata[4] == 3)&& (rxdata[5] == 1))
    {
        if (checksum_check())
        {
            if (rxdata[6] == 1)
            {
                PWM1_3_CMPA_R = 240;
            }
            else
            {
                PWM1_3_CMPA_R = 0;
            }
            if ((rxdata[3] & 0x80) == 0x80)
                Acknowledgemsg();
        }

    }
    if ((rxdata[3] == CMD_RGB || rxdata[3] == 0xC8) && (rxdata[4] == 4))
    {
        if (checksum_check())
        {
            PWM1_2_CMPB_R = rxdata[6];
            PWM1_3_CMPB_R = rxdata[7];
            PWM1_3_CMPA_R = rxdata[8];
            if ((rxdata[3] & 0x80) == 0x80)
            {
                Acknowledgemsg();
            }
        }

    }
    //Code to respond to the get message
    if ((rxdata[3] == 48 || rxdata[3] == 0xB0)
            && (rxdata[5] == DATA_SIZE_GET_REQ))
    {
        if (checksum_check())
        {
            if (rxdata[4] == 1)
            {
                value[0] = PWM1_2_CMPB_R;
                if ((rxdata[3] & 0x80) == 0x80)
                    Acknowledgemsg();
                send_RS_485(rxdata[1], rxdata[4], CMD_GET_RES,DATA_SIZE_GET_RES, value);
            }

        }
    }

}

void send_UI_message(char *str)
{
    uint8_t x = 0;
    static uint8_t ReadIndex;
    static uint8_t WriteIndex;

    while (str[x] != '\0')
    {
        d[x] = str[x];
        x++;
        WriteIndex++;
    }

    if (UART0_FR_R & UART_FR_TXFE)
    {      //  if(ReadIndex != WriteIndex)
        {
//            UART0_DR_R = (d);
            ReadIndex++;
        }
    }
}
void Uart0_Isr()
{
    if (UART0_FR_R && UART_FR_TXFE)
        if (ReadIndex != WriteIndex)
        {
//            UART0_DR_R = d;
            ReadIndex++;
        }
    ReadIndex = (ReadIndex + 1) % 32;
    UART0_ICR_R |= UART_ICR_TXIC;
}
//void userinterface(void)
//{
//    UART0_DR_R=str[readindex];
//    readindex=((read index +1) % (32));
//}

void Uart1_Isr()
{
    {
        if (UART1_RIS_R & UART_RIS_TXRIS)
        {
            if (UART1_FR_R & UART_FR_TXFE)
            {
                send_rs_485_byte();
                UART1_ICR_R |= UART_ICR_TXIC;
            }
        }

    }

    if (!(UART1_RIS_R & UART_RIS_TXRIS))

    {
        uint16_t d = 0;
        d = UART1_DR_R;

        if ((d & UART_DR_PE) && ((d & 0xFF) == SRC_ADD)
                || ((d & 0xFF) == ALL_ADD))
        {
            v = 50;
            green_led_onBoard = 1;

            //         UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS;
            green_led_onBoard = 1;
            rxphase = 0;
            rxdata[rxphase] = (d & 0xFF);
            rxphase++;


        }
        else
        {
            if (rxphase != 0)
            {
                putsUart0("L");
                rxdata[rxphase] = (d & 0xFF);
                rxphase++;
            }
            if (rxphase == (rxdata[5] + 7))
            {
                rxphase = 0;

                processmsg();

            }
        }
    }
    UART1_ICR_R |= UART_ICR_TXIC;
}

void Timer1Isr()
{
    {
        if (green_led_onBoard == 1)
        {
            greenFlag();

        }

        if (red_led_onBoard == 1)
        {

            redFlag();
        }
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
    }
//uint8_t e;
//    if(message_in_progress==-1)
//        {for(e=0;e<255;e++)
//        {
//            if((TX485MESSAGE[e].VALID == 1)&&(retranstimeout[e] == 0))
//                {
////                    RED_BOARD = 1;
////                    u = 50;
////                    red_flag = true;
//                message_in_progress = 0;
//
//                message_phase = 0x00;
//                    if(retranscount[e] == 0)
//                        {
//                        sprintf(temp7,"Transmitting message %d\r\n",TX485MESSAGE[e].SEQ_ID);
//                        putsUart0(temp7);
//                        }
//                    break;
//                }
//        }
//    }
//    if(message_in_progress==0)
//    {
//        send_rs_485_byte();
//    }
}

// Initialize Hardware

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure LED pins GPIO PF1, PF2, PF3 for PWM 5, PWM 6, PWM 7
    GPIO_PORTF_DIR_R |= 0x0E;      // make bits 3,2,1 an output
    GPIO_PORTF_DR2R_R |= 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x0E;      // enable Green, Red, Blue LEDs on board
    GPIO_PORTF_AFSEL_R |= 0x0E;
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6
            | GPIO_PCTL_PF3_M1PWM7;

    // Conigure On board LEDs Green Led and Red Led
    GPIO_PORTE_DIR_R |= green_led_onBoard_mask | red_led_onBoard_mask; // make bits 3,2,1 an output
    GPIO_PORTE_DR2R_R |= green_led_onBoard_mask | red_led_onBoard_mask; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= green_led_onBoard_mask | red_led_onBoard_mask;

    //Configuring PWM 2b,3a,3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    SYSCTL_SRPWM_R = 0;
    PWM1_2_CTL_R = 0;
    PWM1_3_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

    PWM1_2_LOAD_R = 1024;
    PWM1_3_LOAD_R = 1024;
    PWM1_INVERT_R =PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

    PWM1_2_CMPB_R = 0;
    PWM1_3_CMPB_R = 0;
    PWM1_3_CMPA_R = 0;

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

    // Configure Timer 1 as the time base
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x00009C40; // set load value to 100 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void getsUart0(USER_DATA *data)
{
    uint32_t count = 0;
    char c;
    getc: ;
    c = getcUart0();
    if (c == 8 || c == 127)
    {
        if (count > 0)
        {

            count--;
            putcUart0(c);
            goto getc;
        }
        else
        {
            goto getc;
        }
    }

    else if (c == 13 || c == 10)
    {
        putcUart0('\0');
        //putcUart0('\n');
        goto exit;

    }
    else if (c >= 32)
    {
        data->buffer[count] = c;
        count++;
        if (c >= 48 && c <= 57)
        {

        }
        putcUart0(c);
        if (count == MAX_CHARS)
        {
            goto exit;
        }
        else
        {
            goto getc;

        }
    }
    else
    {
        goto getc;
    }
    exit: ;
}

void parseFields(USER_DATA *data)
{
    uint32_t count = 0;
    uint8_t p = 0;
    uint8_t q = 0;
    uint8_t w = 0;
    while (data->buffer[w] != '\0')
    {
        if ((data->buffer[w] >= 32 && data->buffer[w] <= 47))
        {
            data->buffer[w] = '\0';
        }
        else
        {
        }
        w++;
    }
    int l;
    for (l = 0; l <= 50; l++)
    {
        data->fieldType[l] = '\0';
    }

    if ((data->buffer[0] >= 65 && data->buffer[0] <= 90)
            || (data->buffer[0] >= 97 && data->buffer[0] <= 122))
    {
        data->fieldType[0] = 'A';
        data->fieldPosition[0] = 0;
        p = p + 1;
        count++;
        q = q + 1;
    }
    else
    {
        data->fieldType[p] = 'N';
        data->fieldPosition[0] = 0;
        p = p + 1;
        count++;
        q = q + 1;
    }
    for (count = 1; count <= 80; count++)
    {
        if ((data->buffer[count - 1] == '\0')
                && (data->buffer[count] >= 47 && data->buffer[count] <= 58))

        {
            data->fieldType[p] = 'N';
            data->fieldPosition[q] = count;
            putcUart0(data->fieldPosition[q]);
            q++;
            p++;

        }

        else if ((data->buffer[count - 1] == '\0')
                && ((data->buffer[count] >= 65 && data->buffer[count] <= 90)
                        || (data->buffer[count] >= 97
                                && data->buffer[count] <= 122)))
        {

            data->fieldType[p] = 'A';
            data->fieldPosition[q] = count;

            p++;
            q++;
        }
        else
        {
        }

        data->fieldCount = p;
    }

}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    uint8_t m = 0;
    uint8_t j = 0;
    uint8_t l = 0;
    uint8_t k = 1;
    bool valid;

    while (data->buffer[j] != '\0')
    {
        if (strCommand[m] != data->buffer[j])
        {
            valid = false;
            return (valid);

        }
        m++;
        j++;
    }
    while (data->fieldType[k] != '\0')
    {
        if (data->fieldType[k] == 78 || data->fieldType[k] == 65)
        {
            l++;
        }
        k++;
    }
    if (l == minArguments)
    {
        valid = true;
    }
    return (valid);
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    int i = fieldNumber;

    if (i == fieldNumber)
    {
        return &data->buffer[data->fieldPosition[i]];
    }
    else
    {
        return (0);
    }
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int num = 0;
    int i;
    if (fieldNumber <= data->fieldCount)
    {
        for (i = data->fieldPosition[fieldNumber]; data->buffer[i] != '\0'; i++)

            num = num * 10 + (data->buffer[i] - 48);
        return num;

    }
    else
    {
        return 0;
    }
}

// Blocking function that writes a serial character when the UART buffer is not full
extern void putcUart0(char c);

// Blocking function that writes a string when the UART buffer is not full
extern void putsUart0(char *str);

// Blocking function that returns with serial data once the buffer is not empty

extern char getcUart0();

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool valid;

    // Initialize hardware
    initHw();
    initUart1();
    initUart0();

    u = 200;
    red_led_onBoard = 1;

    putsUart0("\r\nReady\r\n");
    v = 50;
    green_led_onBoard = 1;

    //  int x=0;
    int i = 0;
//    uint32_t checksum=0;
    for (i = 0; i < 255; i++)
    {
        TX485MESSAGE[i].DST_ADDRESS = '\0';
        TX485MESSAGE[i].SRC_ADDRESS = '\0';
        TX485MESSAGE[i].SEQ_ID = '\0';
        TX485MESSAGE[i].COMMAND = '\0';
        TX485MESSAGE[i].SIZE = '\0';
        TX485MESSAGE[i].CHANNEL = '\0';
        TX485MESSAGE[i].VALID = 0;

    }
    USER_DATA data;
    while (true)
    {
        int8_t o;
        for (o = 0; o <= 80; o++)
        {
            data.buffer[o] = '\0';
        }

        // Setup UART0 baud rate
        setUart0BaudRate(115200, 40e6);
//    uint8_t value[20];
        putcUart0('\r');
        putcUart0('\n');
        putsUart0("Serial Example\r\n");
        putsUart0("Press '0' or '1'\r\n");
        putcUart0('>');
        getsUart0(&data);
        parseFields(&data);
        putcUart0('\n');
        uint8_t Z;
        for (Z = 0; Z < data.fieldCount; Z++)
        {
            //   putcUart0('\t');
            putcUart0('\r');
            putcUart0(data.fieldType[Z]);
            putcUart0('\t');
            putsUart0(&data.buffer[data.fieldPosition[Z]]);
            putcUart0('\n');

        }

        if (isCommand(&data, "alert", 1))
        {
            char *str = getFieldString(&data, 1);

            char alert[] = "on";
            char alert1[] = "off";
            int8_t n = 1, l0 = 0, l1 = 0;
            if (*str == 'o')
            {
                str++;
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    else if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 1)
            {
                alertEnabled = true;
                valid = true;
            }
            else if (l1 == 2)
            {
                valid = true;
                alertEnabled = true;
            }
            else
            {
                valid = false;
            }
        }

        if (isCommand(&data, "set", 3))
        {
            uint8_t COM;
            uint8_t DST_ADDRESS = getFieldInteger(&data, 1);
            uint8_t CHANNEL = getFieldInteger(&data, 2);
            uint8_t SIZE = 1;
            value[0] = getFieldInteger(&data, 3);

            COM = 0;
            if (ackEnabled == 1)
            {
                COM = 0x80;
            }

            send_RS_485(DST_ADDRESS, CHANNEL, COM, SIZE, value);
            valid = true;
        }

        if (isCommand(&data, "get", 2))
        {
            uint32_t B = getFieldInteger(&data, 1);
            uint32_t C = getFieldInteger(&data, 2);
            uint8_t D = (0x30);             //48;//(0x30);
            send_RS_485(B, C, D, DATA_SIZE_POLL_REQ, MY_NULL);
            valid = true;
        }

        if (isCommand(&data, "reset", 0))
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            valid = true;
        }

        if (isCommand(&data, "cs", 1))
        {
            char *str = getFieldString(&data, 1);

            char alert[] = "on";
            char alert1[] = "off";
            int8_t n = 1, l0 = 0, l1 = 0;
            if (*str == 'o')
            {
                str++;
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    else if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 1)
            {
                csEnabled = true;
                valid = true;
            }
            else if (l1 == 2)
            {
                csEnabled = true;
                valid = true;
            }
            else
            {
                valid = false;
            }
        }

        if (isCommand(&data, "random", 1))
        {
            char *str = getFieldString(&data, 1);

            char alert[] = "on";
            char alert1[] = "off";
            int8_t n = 1, l0 = 0, l1 = 0;
            if (*str == 'o')
            {
                str++;
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    else if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 1)
            {
                randomEnabled = true;
                valid = true;
            }
            else if (l1 == 2)
            {
                randomEnabled = true;
                valid = true;
            }
            else
            {
                valid = false;
            }
        }

        if (isCommand(&data, "ack", 1))
        {
            char *str = getFieldString(&data, 1);

            char alert[] = "on";
            char alert1[] = "off";
            int8_t n = 1, l0 = 0, l1 = 0;
            if (*str == 'o')
            {
                str++;
                while (*str != '\0')
                {
                    if (*(str) == alert[n])
                    {
                        l0++;
                    }
                    else if (*(str) == alert1[n])
                    {
                        l1++;
                    }
                    else
                    {

                    }
                    n++;
                    str++;

                }
            }

            if (l0 == 1)
            {
                ackEnabled = true;
                valid = true;
            }
            else if (l1 == 2)
            {
                ackEnabled = true;
                valid = true;
            }
            else
            {
                valid = false;
            }
        }

        if (isCommand(&data, "poll", 0))                     //POLL command loop
        {
            valid = true;
            address = ALL_ADD;
            send_RS_485(address, MY_NULL, CMD_POLL, DATA_SIZE_POLL, MY_NULL);

        }

        if (isCommand(&data, "reset", 1))                   //RESET command loop
        {
            if (data.fieldType[1] == 'N')
            {
                valid = 1;
                address = getFieldInteger(&data, 1);
                uint8_t f = 0x7F;
                send_RS_485(address, MY_NULL, f, DATA_SIZE_RESET, value);
                address = 0;
            }
            else
                valid = false;
        }

        if (!valid)
        {
            putsUart0("Invalid Command\n");
        }

    }
}

