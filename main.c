/*
 *  ======= main ========
 *
 *  Created on: 20.11.2017
 *  Author:     Roman Grewenig M.Sc.
 *  Company:	Hochschule Trier
 *
 *  Target: TM4C123GH6PM
 *  JTAG Chain: Stellaris In-Circuit Debug Interface (on-board)
 *
 *  Compiler: TI-ARM v17.9
 *  IDE: CCS v6.2.0.00050
 *
 *  Description:
 *  ------------
 *  Minimal example for the EK-TM4C123GXL Launch Pad.
 *  Use as a starting point for your own projects.
 *  This example blinks all three colours of the RGB LED using a state machine and a
 *  cyclic task. The current state is transmitted every time step (CAN-ID 0x1).
 *
 *  Important note: Check Project > Properties > Resource > Linked Resources for relative
 *  file paths.
 *
 *  Needs TI-RTOS (an the included TivaWare) to be installed from:
 *  http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/tirtos/2_16_01_14/exports/tirtos_tivac_setupwin32_2_16_01_14.exe
 */

/* XDCtools Header files */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* Standard IO stream etc. */
#include <stdbool.h>			// Defines for standard boolean representation.
#include <stdio.h>				// IO-stream for console display etc.

/* Drivers etc. */
#include "inc/hw_ints.h"		// Macros defining the interrupts.
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"		// Macros defining the memory map of the device.
#include "inc/hw_sysctl.h"		// Macros for usage with SysCtl.
#include "inc/hw_types.h"		// Macros for common types.
#include "inc/hw_can.h"			// Macros for usage with the CAN controller peripheral.
#include "driverlib/debug.h"	// Macros to debug the driverlib.
#include "driverlib/gpio.h"		// Macros defining the GPIO assignments.
#include "driverlib/pin_map.h"	// Macros defining the pin map of the device.
#include "driverlib/sysctl.h"	// Prototypes for the SysCtl driver.
#include "driverlib/can.h"		// Defines and macros for the CAN controller.
#include "driverlib/qei.h"      // for quadratur encoder
#include "driverlib/interrupt.h"//for use of interrupt
#include "driverlib/pwm.h"      //use for pwm Output
#include "EPOS_command_send.h"

/* Global variables */
// Put your global variables here.
tCANMsgObject MsgObjectTx;
tCANMsgObject MsgObjectTx1;
tCANMsgObject MsgObjectTx2;
tCANMsgObject MsgObjectTx3;
tCANMsgObject sMsgObjectDataTx;
tCANMsgObject sMsgObjectDataRx0;
tCANMsgObject sMsgObjectDataRx1;
tCANMsgObject sMsgObjectDataRx2;
tCANMsgObject sMsgObjectDataRx3;

uint8_t pui8TxBuffer[8];
uint8_t pui8TxBufferWinkel[8];
uint8_t pui8RxBuffer[8];
uint8_t pui8RxBufferMagServo[8];
uint8_t TestVariable;
uint32_t Position;
uint8_t Position1;
uint8_t Position2;
uint8_t Position3;
uint8_t Position4;
uint8_t Position1W;
uint8_t Position2W;
uint8_t Position3W;
uint8_t Position4W;

bool test = true;
bool initStart = false;
bool initMag = false;
bool initFahrt = false;
uint8_t checkHallIR = 0;
uint8_t Strecke = 0;

/* Function prototypes */
void Init_Clock(void);
void Init_CAN(void);
void ConfigureQEI0(void);
void Init_PWM(void);
void Init_Winkel(void);
void Init_Magnet(void);
void Init_Hall(void);
void pack(uint8_t, uint16_t, uint8_t, int32_t);
uint32_t unpack(uint8_t *SDO_Byte, uint16_t *index, uint8_t *sub_index, int32_t *value);

void ISR_GPIOD(void){
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
    if(initMag == false && checkHallIR == 0)
    {
        pack(WRITING_SEND, CURRENT_MODE_SETTING_VALUE,0,1500);
        QEIPositionSet(QEI0_BASE, 0);
        CANMessageSet(CAN0_BASE, 9, &sMsgObjectDataTx, MSG_OBJ_TYPE_TX);
        checkHallIR++;
    }
    else if(initMag == false && checkHallIR == 1)
    {
        checkHallIR++;
        initMag = true;
        initFahrt = true;
        pack(WRITING_SEND, CURRENT_MODE_SETTING_VALUE,0,-1500);
        CANMessageSet(CAN0_BASE, 9, &sMsgObjectDataTx, MSG_OBJ_TYPE_TX);
    }
    else
    {
        GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
        CANMessageSet(CAN0_BASE, 5, &MsgObjectTx2, MSG_OBJ_TYPE_TX);    //am besten Id möglichst hoch prior noch anpassen !!!
    }
}
void CAN0IntHandler(void)
{
    static uint32_t ui32Status = 0;

    /*gibt das Message Objekt (0-31) raus, welches eine Nachricht enthält*/
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    /* Switch for message object */
    switch(ui32Status)
    {
        /* TX handler */
        case 1:     CANIntClear(CAN0_BASE, 1);
                    break;

        /* Message Object for encoder + winkelencoder */
        case 2:    CANIntClear(CAN0_BASE, 2);
                   // ui32CanRxFlags = (ui32CanRxFlags | 0b10);
                   sMsgObjectDataRx0.pui8MsgData = pui8RxBuffer;
                   CANMessageGet(CAN0_BASE, 2, &sMsgObjectDataRx0, 0);
                   TestVariable = pui8RxBuffer[0];
                   if(TestVariable == 200)
                   {
                       Position = QEIPositionGet(QEI0_BASE);
                       Position1 = Position;
                       Position2 = Position >> 8;
                       Position3 = Position >> 16;
                       Position4 = Position >> 24;
                       pui8TxBuffer[0] = Position1;
                       pui8TxBuffer[1] = Position2;
                       pui8TxBuffer[2] = Position3;
                       pui8TxBuffer[3] = Position4;
                       CANMessageSet(CAN0_BASE, 1, &MsgObjectTx, MSG_OBJ_TYPE_TX);
                       //for (i = 0; i < 100000; i++);
                       // Wait for previous transmission to complete (mailbox 1), im Forum erfragter Code
                       while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) & 1) == 1);
                       //Teil für Winkel
                       Position = QEIPositionGet(QEI1_BASE);
                       Position1W = Position;
                       Position2W = Position >> 8;
                       Position3W = Position >> 16;
                       Position4W = Position >> 24;
                       pui8TxBufferWinkel[0] = Position1W;
                       pui8TxBufferWinkel[1] = Position2W;
                       pui8TxBufferWinkel[2] = Position3W;
                       pui8TxBufferWinkel[3] = Position4W;
                       CANMessageSet(CAN0_BASE, 3, &MsgObjectTx1, MSG_OBJ_TYPE_TX);
                   }
                   break;
        case 4:     CANIntClear(CAN0_BASE, 4);
                    // damit der Buffer genau mit dem Message Objekt verknüpft wird
                    sMsgObjectDataRx1.pui8MsgData = pui8RxBufferMagServo;
                    CANMessageGet(CAN0_BASE, 4, &sMsgObjectDataRx1, 0);
                    TestVariable = pui8RxBufferMagServo[0];
                    if(TestVariable == 1)
                    {
                        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);   //schaltet den Magneten an
                    }
                    else
                    {
                        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);  //macht den Magneten aus
                    }
                    break;

        case 6:     CANIntClear(CAN0_BASE, 6);
                    // damit der Buffer genau mit dem Message Objekt verknüpft wird
                    sMsgObjectDataRx2.pui8MsgData = pui8RxBufferMagServo;
                    CANMessageGet(CAN0_BASE, 6, &sMsgObjectDataRx2, 0);
                    TestVariable = pui8RxBufferMagServo[0];
                    if(TestVariable == 1)
                    {
                        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 2500);
                    }
                    else
                    {
                        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1250);
                    }
                    break;

        case 7:     CANIntClear(CAN0_BASE, 7);
                    sMsgObjectDataRx3.pui8MsgData = pui8RxBuffer;
                    CANMessageGet(CAN0_BASE, 7, &sMsgObjectDataRx3, 0);
                    TestVariable = pui8RxBuffer[0];
                    if(TestVariable == 100)
                    {
                        initStart = true;
                    }
                    break;

        case 8:     CANIntClear(CAN0_BASE, 8);
                    break;

        case 9:     CANIntClear(CAN0_BASE, 9);
                    break;

        case 17:    CANIntClear(CAN0_BASE, 17);
                    break;

        /*pending status error */
        case 32768: CANIntClear(CAN0_BASE, CAN_INT_INTID_STATUS);
                  //  printf("Status Error \n");
                    break;

        /* Unused message objects: These should never trigger */
        default:    break;
    }
    return;
}
/*
 * ======== main ========
 */
void main(void)
{
    printf("BIOS starting...\n");

    // Initialize the main clock/oscillator.
    Init_Clock();

    // Initialize your peripherals (GPIOs etc.).
    //Init_GPIO();

    // Initialize the CAN peripheral.
    Init_CAN();

    //QEI Module aktivieren
    ConfigureQEI0();

    //PWM Module aktivieren
    Init_PWM();

    //Winkel Enoceder aktivieren
    Init_Winkel();

    //Magnet Pin vorm Transistor als Output initialisieren
    Init_Magnet();

    //initialisierung des Hall Interrupt
    Init_Hall();

    printf("Finished.\n");

    // Start TI-RTOS. Everything is now controlled by the BIOS, as defined in your project .CFG file.
    BIOS_start();

}

/*
 * ======== tasks ========
 */

// This is the background task.
void Task_Idle(void)
{
	//Initialisierung
    //Teil an dem Laufkatze den Magneten erkannt hat und sich nach links gewegt
    if(initFahrt == true)
    {
        Position = QEIPositionGet(QEI0_BASE);
        if(Position <= 1000)
        {
            pack(WRITING_SEND, CURRENT_MODE_SETTING_VALUE,0,0);
            CANMessageSet(CAN0_BASE, 9, &sMsgObjectDataTx, MSG_OBJ_TYPE_TX);
            // Wait for previous transmission to complete (mailbox 1), im Forum erfragter Code
            while((CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) & 1) == 1);
            //Nachricht für den Mikrocontroller, das die initialisierung abgeschlossen ist
            pui8TxBuffer[0] = 75;
            pui8TxBuffer[1] = 0;
            pui8TxBuffer[2] = 0;
            pui8TxBuffer[3] = 0;
            CANMessageSet(CAN0_BASE, 8, &MsgObjectTx3, MSG_OBJ_TYPE_TX);
            initFahrt = false;
            QEIPositionSet(QEI0_BASE,0);
        }
    }
  //  System_flush();
}

// Timing can be modified in the .CFG file.
void Task_100ms(void)
{
    //initialisierung
    //fahre als erstes nach rechts, bis der Magnet erkannt wurde
    if(initStart == true)
    {
        //fahre langsam mit 1000ma nach rechts
        pack(WRITING_SEND, CURRENT_MODE_SETTING_VALUE,0,-1500);
        CANMessageSet(CAN0_BASE, 9, &sMsgObjectDataTx, MSG_OBJ_TYPE_TX);
        initStart = false;
    }
    else
    {
        //tue nichts
    }
}

/*
 * ======== user functions ========
 */

// This function initializes the clock/oscillator used in the project.
void Init_Clock(void)
{
	// Settings for 80 MHz.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	return;
}

// This function initializes the CAN controller resources used in the project.
// Modify to your demands.
void Init_CAN(void)
{
	printf("Initializing CAN...");
	// Enable the peripheral.
	SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
		// Wait until it is ready...
		while (! SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
	// Set the GPIO special functions (CANTX, CANRX) for the respective pins.
	GPIOPinConfigure(GPIO_PB5_CAN0TX);
	GPIOPinConfigure(GPIO_PB4_CAN0RX);
	GPIOPinTypeCAN (GPIO_PORTB_BASE , GPIO_PIN_4 | GPIO_PIN_5);
	// Enable the CAN peripheral CAN0.
	SysCtlPeripheralEnable (SYSCTL_PERIPH_CAN0);
	// Reset the CAN controller.
	CANInit(CAN0_BASE);
	// Set the baud rate to 250 kBaud/s based on the system clocking. Modify if needed.
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000);
	// Disable auto-retry if no ACK-bit is received by the CAN controlled.
	CANRetrySet(CAN0_BASE, 0);
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER);
	IntEnable(INT_CAN0);
	CANEnable (CAN0_BASE);

	// TestMessageObjekt
	//benötigt für encoder info
	MsgObjectTx.ui32MsgID = 0x0012;
	MsgObjectTx.ui32Flags = 0x0000;			// No flags are used on this message object.
	MsgObjectTx.ui32MsgIDMask = 0x0000;		// No masking is used for TX.
	MsgObjectTx.ui32MsgLen = 8;			    // Set the DLC to '8' (8 bytes of data)
	MsgObjectTx.pui8MsgData = pui8TxBuffer; // A buffer, to which this message object points for data storage.
	CANMessageSet(CAN0_BASE, 1, &MsgObjectTx, MSG_OBJ_TYPE_TX);

	// benötigt für winkel encoder
	MsgObjectTx1.ui32MsgID = 0x0013;         // Message ID is '1'.
	MsgObjectTx1.ui32Flags = 0x0000;         // No flags are used on this message object.
	MsgObjectTx1.ui32MsgIDMask = 0x0000;     // No masking is used for TX.
	MsgObjectTx1.ui32MsgLen = 8;             // Set the DLC to '8' (8 bytes of data)
	MsgObjectTx1.pui8MsgData = pui8TxBufferWinkel; // A buffer, to which this message object points for data storage.
	CANMessageSet(CAN0_BASE, 3, &MsgObjectTx1, MSG_OBJ_TYPE_TX);

	//message bject for data encoder and winkel encoder
	sMsgObjectDataRx0.ui32MsgID = 0x0011;
	sMsgObjectDataRx0.ui32MsgIDMask = 0xFFFF;
	sMsgObjectDataRx0.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
	sMsgObjectDataRx0.ui32MsgLen = 8;
	CANMessageSet ( CAN0_BASE , 2, &sMsgObjectDataRx0 ,MSG_OBJ_TYPE_RX );

	//message Objekt für Magneten
	sMsgObjectDataRx1.ui32MsgID = 0x0029;
	sMsgObjectDataRx1.ui32MsgIDMask = 0xFFFF;
	sMsgObjectDataRx1.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
	sMsgObjectDataRx1.ui32MsgLen = 8;
	CANMessageSet ( CAN0_BASE , 4, &sMsgObjectDataRx1 ,MSG_OBJ_TYPE_RX );

	//message objekt für die initialisierung der kompletten Laufkatze
	sMsgObjectDataRx3.ui32MsgID = 0x0030;
	sMsgObjectDataRx3.ui32MsgIDMask = 0xFFFF;
	sMsgObjectDataRx3.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
	sMsgObjectDataRx3.ui32MsgLen = 8;
	CANMessageSet ( CAN0_BASE , 7, &sMsgObjectDataRx3 ,MSG_OBJ_TYPE_RX );

	//message objekt für die initialisierung der kompletten Laufkatze
	MsgObjectTx3.ui32MsgID = 0x0031;
	MsgObjectTx3.ui32Flags = 0x0000;
	MsgObjectTx3.ui32MsgIDMask = 0x0000;
	MsgObjectTx3.ui32MsgLen = 8;
	MsgObjectTx3.pui8MsgData = pui8TxBuffer;
	CANMessageSet(CAN0_BASE, 8, &MsgObjectTx3, MSG_OBJ_TYPE_TX);

	//message Objekt für Servo Motor
	sMsgObjectDataRx2.ui32MsgID = 0x0027;
	sMsgObjectDataRx2.ui32MsgIDMask = 0xFFFF;
	sMsgObjectDataRx2.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
	sMsgObjectDataRx2.ui32MsgLen = 8;
	CANMessageSet ( CAN0_BASE , 6, &sMsgObjectDataRx2 ,MSG_OBJ_TYPE_RX );

	//message Objekt für Hall Sensor
	MsgObjectTx2.ui32MsgID = 0x0026;         // Message ID is '1'.
	MsgObjectTx2.ui32Flags = 0x0000;         // No flags are used on this message object.
	MsgObjectTx2.ui32MsgIDMask = 0x0000;     // No masking is used for TX.
	MsgObjectTx2.ui32MsgLen = 8;             // Set the DLC to '8' (8 bytes of data)
	MsgObjectTx2.pui8MsgData = pui8TxBuffer; // A buffer, to which this message object points for data storage.
	CANMessageSet(CAN0_BASE, 5, &MsgObjectTx2, MSG_OBJ_TYPE_TX);

	//Message objects für Steuergerät
	sMsgObjectDataTx.ui32MsgID = 0x0600+0b00000001;
	sMsgObjectDataTx.ui32Flags = 0x0000;
	sMsgObjectDataTx.ui32MsgIDMask = 0x0000;
	sMsgObjectDataTx.ui32MsgLen = 8;
	sMsgObjectDataTx.pui8MsgData = pui8TxBuffer;
	CANMessageSet(CAN0_BASE, 9, &sMsgObjectDataTx, MSG_OBJ_TYPE_TX);

	printf("done.\n");
	printf("System clock used for CAN bus timing: %d Hz\n", SysCtlClockGet());
	return;
}

void ConfigureQEI0(){
/* ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
 * GPIO Konfiguration
 * PHA = PF0
 * PHB = PF1
*/

    printf("Initializing QEI ...");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);         // Enable the QEI0 peripheral
   // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);         // Enable the QEI0 peripheral
   // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));  // Wait for the QEI0 module to be ready.
    //Set Pins to be PHA0 and PHB0

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinConfigure(GPIO_PF0_PHA0);
    GPIOPinConfigure(GPIO_PF1_PHB0);

    GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypeQEI(GPIO_PORTF_BASE, GPIO_PIN_1);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 100000);

    //Enable velocity capture QEI Module 0
    //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() * TIME_TO_COUNT);
    // 3rd Parameter = Number of clock cycles to count ticks
    // ex. 80000000 = sysctlclock = 1 sek
    //QEIVelocityEnable(QEI0_BASE);

    // enable QEI module
    QEIEnable(QEI0_BASE);

    //Set Register start Values
    QEIPositionSet(QEI0_BASE,50000);

    // Configure velocity measurement
    //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEI_TIME_TO_COUNT*SysCtlClockGet());
    //QEIVelocityEnable(QEI0_BASE);

    // enable gpio interrupts to allow period measurement
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    printf(" done.\n");
    return;
}


void Init_PWM(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

  //  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  //  HWREG(GPIO_PORTF_BASE + GPIO_O_CR)   |= 0x01;

    //Systemtakt durch 64 teilen
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;


    //Seite 1233 im Datenblatt
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 25000);

    //!!!!2000 hatte hier schon mal als wert geklappt!!!!
    //range von 1250 bis 2500
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1250);

    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT , true);
}

void Init_Winkel(void)
{
    /* ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
     * GPIO Konfiguration
     * PHA = PC5
     * PHB = PC6
     * IDX = PC4 -> not used at the moment
    */

        printf("Initializing QEI-Winkel ...");
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);         // Enable the QEI0 peripheral
       // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
        SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);         // Enable the QEI0 peripheral
       // while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));  // Wait for the QEI0 module to be ready.
        //Set Pins to be PHA0 and PHB0

        GPIOPinConfigure(GPIO_PC5_PHA1);
        GPIOPinConfigure(GPIO_PC6_PHB1);

        GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);
        GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);

        //DISable peripheral and int before configuration
        QEIDisable(QEI1_BASE);
        QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

        QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, 60000);

        //Enable velocity capture QEI Module 0
        //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() * TIME_TO_COUNT);
        // 3rd Parameter = Number of clock cycles to count ticks
        // ex. 80000000 = sysctlclock = 1 sek
        //QEIVelocityEnable(QEI0_BASE);

        // enable QEI module
        QEIEnable(QEI1_BASE);

        //Set Register start Values
        QEIPositionSet(QEI1_BASE,30000);

        // Configure velocity measurement
        //QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, QEI_TIME_TO_COUNT*SysCtlClockGet());
        //QEIVelocityEnable(QEI0_BASE);

        // enable gpio interrupts to allow period measurement
        //GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_0);
        //GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
        //GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_RISING_EDGE);
        //GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
        printf(" done.\n");
        return;
}
void Init_Magnet(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
   // GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, 0);  //macht den Magneten aus
   // GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, GPIO_PIN_2);   //schaltet den Magneten an
}
void Init_Hall(void)
{
    // Enable the GPIOD peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Wait for the GPIOA module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    // Black magic, uncomment if error
    /*HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;*/

    // Set GPIO to input
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);

    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_3);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_3);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3,GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_3);

    //GPIOIntRegister(GPIO_PORTD_BASE, ISR_GPIOD);
    //Initialize the GPIO pin configuration
    //Set Pin 3 as a Input
    /*GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);
    //Make pin 3 low level triggered interrupt.
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_LOW_LEVEL);
    //das muss raus weil ich ja die Interrupt von dem Betriebssystem nutze
    // Enable the pin interrupts.
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_3);*/
}
// This function packs the different communication objects and the value in CANopen format.
void pack(uint8_t SDO_Byte, uint16_t index, uint8_t sub_index, int32_t value)
{
    uint8_t *pointer_index_Tx = &index;
    int8_t *pointer_value_Tx = &value;
    pui8TxBuffer [0] = SDO_Byte;
    pui8TxBuffer [1] = *pointer_index_Tx;
    pui8TxBuffer [2] = *(pointer_index_Tx+1);
    pui8TxBuffer [3] = sub_index;
    pui8TxBuffer [4] = *pointer_value_Tx;
    pui8TxBuffer [5] = *(pointer_value_Tx+1);
    pui8TxBuffer [6] = *(pointer_value_Tx+2);
    pui8TxBuffer [7] = *(pointer_value_Tx+3);
    return;
}

// This function unpacks the incoming CANopen massage and hand the received communication objects to global variables.
uint32_t unpack(uint8_t *SDO_Byte, uint16_t *index, uint8_t *sub_index, int32_t *value)
{
    uint16_t *pointer_index_Rx;
    int32_t *pointer_value_Rx;
    uint32_t *pointer_display;
    uint8_t index_array_Rx [2];
    int8_t value_array_Rx [4];
    uint8_t display_array [4];
    *SDO_Byte = pui8RxBuffer [0];
    index_array_Rx [0] = pui8RxBuffer [1];
    index_array_Rx [1]= pui8RxBuffer [2];
    *sub_index = pui8RxBuffer [3];
    value_array_Rx [0] = pui8RxBuffer [4];
    value_array_Rx [1] = pui8RxBuffer [5];
    value_array_Rx [2] = pui8RxBuffer [6];
    value_array_Rx [3] = pui8RxBuffer [7];
    display_array [0] = pui8RxBuffer [3];
    display_array [1] = pui8RxBuffer [1];
    display_array [2] = pui8RxBuffer [2];
    display_array [3] = pui8RxBuffer [0];
    pointer_index_Rx = index_array_Rx;
    pointer_value_Rx = value_array_Rx;
    pointer_display = display_array;
    *index = *pointer_index_Rx;
    *value = *pointer_value_Rx;
    return(*pointer_display);
}
// End of file.

