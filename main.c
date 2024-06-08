// Standard includes
#include <stdio.h>
#include <stdint.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "systick.h"
#include "pin.h"
#include "spi.h"
#include "uart.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "gpio_if.h"
#include "common.h"

// Custom includes
#include "utils/network_utils.h"
#include "pin_mux_config.h"


//*****************************************************************************
//                 SYSTICK MACROS -- Start
//*****************************************************************************
// some helpful macros for systick
// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// systick reload value set to 40ms period
#define SYSTICK_RELOAD_VAL 3200000UL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\
//*****************************************************************************
//                 SYSTICK MACROS -- End
//*****************************************************************************

//*****************************************************************************
//                 AWS MACROS -- Start
//*****************************************************************************
#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1
//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                5    /* Current Date */
#define MONTH               6     /* Month 1-12 */
#define YEAR                2024  /* Current year */
#define HOUR                6    /* Time - hours */
#define MINUTE              0    /* Time - minutes */
#define SECOND              0     /* Time - seconds */


#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "a1f2ug7b07xujp-ats.iot.us-east-1.amazonaws.com" // CHANGE ME
#define GOOGLE_DST_PORT       8443


#define POSTHEADER "POST /things/jade_cc3200_board/shadow HTTP/1.1\r\n"             // CHANGE ME
#define HOSTHEADER "Host: a1f2ug7b07xujp-ats.iot.us-east-1.amazonaws.com\r\n"  // CHANGE ME
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"var\" :\""                                           \
                        "Hello Jade, "                                     \
                        "Gararge door has been opened/closed"                  \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"
//*****************************************************************************
//                 AWS MACROS -- End
//*****************************************************************************

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//extern void (* const g_pfnVectors[])(void);

volatile uint64_t ulsystick_delta_us = 0;
volatile int systick_cnt = 0;
volatile int systick_expired = 0;
volatile int count = 0;
volatile uint64_t delta_buffer[20];
volatile int start = 0;
volatile int readReady = 0;

#define ZERO  4 //1100000000000100
#define ONE   132 //1100000010000100
#define TWO   68 //1100000001000100
#define THREE 196 //1100000011000100
#define FOUR  36 //1100000000100100
#define FIVE  164 //1100000010100100
#define SIX   100 //1100000001100100
#define SEVEN 228 //1100000011100100
#define EIGHT 20 //1100000000010100
#define NINE  148 //1100000010010100
#define ENTER 238 //1100000011101110
#define MUTE  56 //1100000000111000
#define LAST  160 //1100000010100000

#define SPI_IF_BIT_RATE  100000
#define MAX_BUFFER  100
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//extern void (* const g_pfnVectors[])(void);

volatile long PIN8_intcount;
volatile unsigned char PIN8_intflag;

volatile unsigned long tmp;
volatile static tBoolean bRxDone;

volatile long currentPress;
volatile long previousPress;
volatile long currentButton;
volatile long previousButton;
char message[MAX_BUFFER]; //message to send to AWS
char buffer[1000]; // buffer to store IR Data
int isButton = -1;  // check if it is a valid button
int sameButton = 0; // check if press the same button consecutively flag
int bufferSize = 0; // the buffer size for composing message
int receiveSize = 0; // the buffer size for received message
int composing_x = 5, composing_y = 68; // composing message position
int received_x = 5, received_y = 4; // received message position
int messageReady = 0; // indicate if the received message is read to display
int previousSize = 0;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

typedef struct Letter {
    unsigned int x;
    unsigned int y;
    char letter;
} Letter;
static PinSetting PIN8  = { .port = GPIOA2_BASE, .pin = 0x2 }; // GPIOPIN8 for IR_OUT
static PinSetting OPEN  = { .port = GPIOA2_BASE, .pin = 0x40 }; // GPIOPIN15 for SENDING OPEN
static PinSetting CLOSE  = { .port = GPIOA0_BASE, .pin = 0x20 }; // GPIOPIN21 for SENDING CLOSE
Letter ComposingLetter[MAX_BUFFER];
Letter ReceivedLetter[MAX_BUFFER];
Letter PreviousLetter[MAX_BUFFER];
//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
//static void BoardInit(void);
static int set_time();
static int http_post(int);
static void BoardInit(void);
//unsigned long Decode(unsigned long* buffer);
void Display(unsigned long value);
void MasterMain();
char ToLetter(unsigned long value);
char forwardLetter(char letter, unsigned long value);
void ClearComposingMessage();
void ClearReceivedMessage();
void SendMessage();
void CheckMessage();
void DisplayMessage();
void ProcessIR(long lRetVal);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************
//Reset SysTick Counter
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;

    systick_expired = 0;
}

//SysTick Interrupt Handler
//Keep track of whether the systick counter wrapped
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
    systick_expired = 1;
    ulsystick_delta_us = 0;
//    expired_num++;
}

/**
 * GPIO Interrupt Handler (Pin 8)
 *
 * Triggered at every rising edge of Pin 8.
 * Computes the 17 pulse width duration of pulse including and after the start pulse
 * The start pulse is ~12ms
 */
static void GPIOA0IntHandler(void) {    // Pin 8 Handler

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(PIN8.port, true);
    MAP_GPIOIntClear(PIN8.port, ulStatus);

    if (ulStatus & PIN8.pin) {

        // read the countdown register and compute elapsed cycles
        uint64_t ulsystick_delta = SYSTICK_RELOAD_VAL - SysTickValueGet();
        uint64_t time = TICKS_TO_US(ulsystick_delta);

        // Captures the start pulse only after ~12ms pulse
        if (10000 < time && time < 13500) start = 1;
        if (start) {
            delta_buffer[count] = time;
            count++;
            if (count >= 17) {
                start = 0;
                count = 0;
                // Set readReady high so main can print it
                readReady = 1;
                PIN8_intflag = 1;
            }
        }
        // reset the countdown register
        SysTickReset();
    }
}

//*****************************************************************************
//                      INITIALIZING FUNCTIONS
//*****************************************************************************
static void BoardInit(void) {
    //MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
      //
      // Set vector table base
      //
    #if defined(ccs)
        MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}


//Initializes SysTick Module
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

static void OLEDInit(void)
{
    //Initializes the OLED screen to black with a horizontal line midway through
    Adafruit_Init();
    fillScreen(0);
    drawLine(0, 64, 127, 64, 0x07FF);
}

//*****************************************************************************
//                      OTHER FUNCTIONS
//*****************************************************************************

void sendSignal(int order){ //1: open, 0: close, 2: nothing)
    if (order == 1){ //open
        MAP_GPIOPinWrite(OPEN.port, OPEN.pin, 0);
        //MAP_UtilsDelay(100000);
        //MAP_GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);
        printf("sent signal\n");
    }else{//close
//        MAP_GPIOPinWrite(CLOSE.port, CLOSE.pin, CLOSE.pin);
//        MAP_UtilsDelay(80000);
        MAP_GPIOPinWrite(CLOSE.port, CLOSE.pin, CLOSE.pin);
//        MAP_UtilsDelay(800000);
//        MAP_GPIOPinWrite(CLOSE.port, CLOSE.pin, CLOSE.pin);
    }
}



static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

unsigned long Decode() {
    unsigned long sum = 0x0;
    int i;
    for (i = 9; i < 17; i++) {
        sum = sum << 1;
        if (delta_buffer[i] > 2000) {
            sum = sum | 0x1;
        }
    }
    return sum;
}


void UART0_Display(unsigned long sum) {
    switch(sum) {
        case ZERO:
            Report("You pressed 0.\n\r");
            isButton = 0;
            break;
        case ONE:
            Report("You pressed 1.\n\r");
            isButton = 1;
            break;
        case TWO:
            Report("You pressed 2.\n\r");
            isButton = 2;
            break;
        case THREE:
            Report("You pressed 3.\n\r");
            isButton = 3;
            break;
        case FOUR:
            Report("You pressed 4.\n\r");
            isButton = 4;
            break;
        case FIVE:
            Report("You pressed 5.\n\r");
            isButton = 5;
            break;
        case SIX:
            Report("You pressed 6.\n\r");
            isButton = 6;
            break;
        case SEVEN:
            Report("You pressed 7.\n\r");
            isButton = 7;
            break;
        case EIGHT:
            Report("You pressed 8.\n\r");
            isButton = 8;
            break;
        case NINE:
            Report("You pressed 9.\n\r");
            isButton = 9;
            break;
        case ENTER:
            Report("You pressed Enter.\n\r");
            isButton = 10;
            break;
        case MUTE:
            Report("You pressed MUTE.\n\r");
            isButton = 11;
            break;
        case LAST:
            Report("You pressed LAST.\n\r");
            isButton = 12;
            break;
        default:
            isButton = -1;
            break;
    }
}

char toLetter(unsigned long sum) {
    char letter;
    switch(sum) {
        case ZERO:
            letter = ' ';
            break;
        case ONE:
            letter = '?';
            break;
        case TWO:
            letter = 'a';
            break;
        case THREE:
            letter = 'd';
            break;
        case FOUR:
            letter = 'g';
            break;
        case FIVE:
            letter = 'j';
            break;
        case SIX:
            letter = 'm';
            break;
        case SEVEN:
            letter = 'p';
            break;
        case EIGHT:
            letter = 't';
            break;
        case NINE:
            letter = 'w';
            break;
        case MUTE:
            letter = '-';
            break;
        case LAST:
            letter = '+';
            break;
        default:
            letter = '/';
            break;
    }
    return letter;
}

char forwardLetter(char letter, unsigned long sum) {

    char newLetter;
    switch(sum) {
        case ONE:
            if(letter == '?') {
                newLetter = '.';
            }

            else if(letter == '.') {
                newLetter = ',';
            }

            else {
                newLetter = '?';
            }
            break;
        case TWO:
            if(letter == 'c') {
                newLetter = 'a';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case THREE:
            if(letter == 'f') {
                newLetter = 'd';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FOUR:
            if(letter == 'i') {
                newLetter = 'g';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FIVE:
            if(letter == 'l') {
                newLetter = 'j';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SIX:
            if(letter == 'o') {
                newLetter = 'm';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SEVEN:
            if(letter == 's') {
                newLetter = 'p';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case EIGHT:
            if(letter == 'v') {
                newLetter = 't';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case NINE:
            if(letter == 'z') {
                newLetter = 'w';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        default:
            newLetter = letter;
            break;
    }
    return newLetter;
}

void MasterMain()
{
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    // Initialize Adafruit
    //Adafruit_Init();
}

void SetupCommunication()
{
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PinTypeUART(PIN_58, PIN_MODE_6); //UART1_TX
    PinTypeUART(PIN_59, PIN_MODE_6); //UART1_RX
    PinTypeUART(PIN_55, PIN_MODE_3); //UART0_TX
    PinTypeUART(PIN_57, PIN_MODE_3); //UART0_RX

    //UART Setup
    UARTConfigSetExpClk(UARTA1_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UARTA0_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTEnable(UARTA1_BASE);
    UARTEnable(UARTA0_BASE);
    UARTDMADisable(UARTA1_BASE, (UART_DMA_RX | UART_DMA_TX));
    UARTFIFODisable(UARTA1_BASE) ;
}

void ClearComposingMessage() {
    int i;
    for(i = 0; i < bufferSize; i++) {
        drawChar(ComposingLetter[i].x, ComposingLetter[i].y, ComposingLetter[i].letter, BLACK, BLACK, 1);
    }
    bufferSize = 0;
    composing_x = 5;
    composing_y = 68;
}

void ClearPreviousMessage() {
    int i;
    for(i = 0; i < previousSize; i++) {
        drawChar(PreviousLetter[i].x, PreviousLetter[i].y, PreviousLetter[i].letter, BLACK, BLACK, 1);
    }
}

void SendMessage() {

    // if there's something to send
    if(bufferSize > 0) {

        int i;
        for(i = 0; i < bufferSize; i++) {
            // wait for UART to be available
            while(UARTBusy(UARTA1_BASE));
            UARTCharPut(UARTA1_BASE, ComposingLetter[i].letter);
        }

        // end line character
        UARTCharPut(UARTA1_BASE,'\0');
        ClearComposingMessage();

    }
}


//void sendMessageToAWS(){
//
//}


void CheckMessage() {

    // clear UART interrupt
    UARTIntClear(UARTA1_BASE,UART_INT_RX);

    // when UART is available
    while(UARTCharsAvail(UARTA1_BASE))
    {
        char c = UARTCharGet(UARTA1_BASE);

        if(c == '\0') {
            messageReady = 1;
        }
        else {
            ReceivedLetter[receiveSize].letter = c;
            ReceivedLetter[receiveSize].x = received_x;
            ReceivedLetter[receiveSize].y = received_y;
            // increase buffer size
            receiveSize++;
            // increment pixel position
            received_x += 7;
            // position boundaries
            if(received_x >= 124) {
                received_x = 5;
                received_y += 10;
            }
        }
    }
}

void DisplayMessage() {
    if(messageReady) {
        // clear flag
        messageReady = 0;

        int i;
        ClearPreviousMessage();
        for(i = 0; i < receiveSize; i++) {
            drawChar(ReceivedLetter[i].x, ReceivedLetter[i].y, ReceivedLetter[i].letter, RED, RED, 1);
            PreviousLetter[i] = ReceivedLetter[i];
        }
        previousSize = receiveSize;
        received_x = 5;
        received_y = 4;
        receiveSize = 0;
    }
}

void ProcessIR(long lRetVal) {
    if (PIN8_intflag) {
        PIN8_intflag = 0;  // clear flag
        currentButton = Decode();
        UART0_Display(currentButton);

        // set up same button flag
        if(previousButton == currentButton) {
            sameButton = 1;
        }

        else {
            sameButton = 0;
        }
        // print out composing message on OLED

        // valid button from remote
//        if(isButton != -1) {
//            // delete a character
//            if(toLetter(currentButton) == '-') {
//
//                sendSignal(0);
//                // draw previous black
//                if(bufferSize > 0) {
//                    bufferSize--;
//                    drawChar(ComposingLetter[bufferSize].x, ComposingLetter[bufferSize].y, ComposingLetter[bufferSize].letter, BLACK, BLACK, 1);
//                }
//
//                // set new composing position for letters
//                if(composing_x >= 12) {
//                    composing_x -= 7;
//                }
//                else if(composing_x == 5) {
//                    if(composing_y >= 78) {
//                        composing_y -= 10;
//                        composing_x = 117;
//                    }
//                }
//            }
//
//            // send a character
//            else
        if(toLetter(currentButton) == '+') {
                //SendMessage();
                sendSignal(1);
                testthedoorclose();
                //MAP_UtilsDelay(24000000);
                http_post(lRetVal);
                //memset(message, 0, MAX_BUFFER);
            }

            // add a character
//            else {
//                char letter;
//                letter = toLetter(currentButton);
//
//                if(bufferSize < MAX_BUFFER) {
//
//                    // consecutive button for switching character
//                    if(previousPress == currentPress && sameButton) {
//                        int index = bufferSize - 1;
//                        char l = ComposingLetter[index].letter;
//
//                        // clear previous letter
//                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, l, BLACK, BLACK, 1);
//
//                        // draw the next letter
//                        ComposingLetter[index].letter = forwardLetter(l, currentButton);
//                        message[index] = forwardLetter(l, currentButton);
//                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, ComposingLetter[index].letter, BLUE, BLUE, 1);
//                    }
//
//                    else {
//                        Letter CL;
//                        CL.x = composing_x;
//                        CL.y = composing_y;
//                        CL.letter = letter;
//                        ComposingLetter[bufferSize] = CL;
//                        message[bufferSize] = letter;
//
//                        drawChar(composing_x, composing_y, letter, BLUE, BLUE, 1);
//
//                        // adjust pixel positions
//                        composing_x += 7;
//                        if(composing_x >= 124) {
//                            composing_x = 5;
//                            composing_y += 10;
//                        }
//
//                        // increase buffer size for next input
//                        bufferSize++;
//                    }
//                }
//            }
//        }
        // update press flag and button
        previousPress = currentPress;
        previousButton = currentButton;
    }
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {

    long lRetVal = -1;

    BoardInit();
    PinMuxConfig();
    OLEDInit();
    SetupCommunication();
    InitTerm();
    ClearTerm();
    SysTickInit();
    MAP_GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);
    MAP_GPIOPinWrite(GPIOA0_BASE, 0x20, 0);

    UART_PRINT("My terminal works!\n\r");
    // initialize global default app configuration
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    //Connect the CC3200 to the local access point
        lRetVal = connectToAccessPoint();
        //Set time so that encryption can be used
        lRetVal = set_time();
        if(lRetVal < 0) {
            UART_PRINT("Unable to set time in the device");
            LOOP_FOREVER();
        }
        //Connect to the website with TLS encryption
        lRetVal = tls_connect();
        if(lRetVal < 0) {
            ERR_PRINT(lRetVal);
        }


    // Register the interrupt handlers
    MAP_GPIOIntRegister(PIN8.port, GPIOA0IntHandler);
    // Configure interrupts on rising edges
    MAP_GPIOIntTypeSet(PIN8.port, PIN8.pin, GPIO_RISING_EDGE); // read ir_output
    uint64_t ulStatus = MAP_GPIOIntStatus(PIN8.port, false);
    MAP_GPIOIntClear(PIN8.port, ulStatus);
    // Enable interrupts on ir_output
    MAP_GPIOIntEnable(PIN8.port, PIN8.pin);

    // clear global variables
    PIN8_intcount = 0;
    PIN8_intflag = 0;
    currentPress = 0;
    previousPress = 1;
    currentButton = -2;
    previousButton = -1;

//    // UART_Interrupt
//    Message("\t\t****************************************************\n\r");
//    Message("\t\t\tWaveform\n\r");
//    Message("\t\t ****************************************************\n\r");
//    Message("\n\n\n\r");

    MasterMain();
    //fillScreen(BLACK);

//    // UART interrupt
//    UARTIntRegister(UARTA1_BASE, CheckMessage);
//    UARTIntEnable( UARTA1_BASE, UART_INT_RX) ;
//    printf ("Passed UART interrupt");


    // main for loop
    while (1) {
        MAP_GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);
        MAP_GPIOPinWrite(GPIOA0_BASE, 0x20, 0);
        ProcessIR(lRetVal);
        DisplayMessage();
    }

//    sl_Stop(SL_STOP_TIMEOUT);
//    LOOP_FOREVER();

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(DATA1);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}
