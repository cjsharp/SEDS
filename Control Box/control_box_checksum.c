//battery
//checksum
//oled communication
//tones
//communication timeout
//continue from autosolenoids  when override HIGH
/*************************************************************************
 *  @File Descption: Read states of manual override buttons of control_box
 *	and bi-directional communication via UART with main_board.
 *	@File: control_box_firmware.c
 *  @author(s):
 *  -Tawfic Rabbani, Embedded Systems Engineer, head of software
 *	-Matthew Santos, Electrical and Firmware Engineer
 *  @Property of SEDS UCSD
 *  @since: 10/2016
 *************************************************************************/
//#include "reg60s2.h" //Include reg file for 8051 architecure
#include "STC/STC12C5A60S2.H"
#include "intrins.h" //for ADC
#include "math.h"
#define OUT_COUNT 16 //1 override + 12 solenoidstates + 1 encoder + 2 checkSum
#define IN_COUNT 71 //1 override + 12 solenoidstates + 56 IN_DAQ COUNT + 2 checkSum
#define OLED_COUNT 63 //1 override + 1 tone + 2 battlvl + 3 commsgood + 56 IN_DAQ COUNT
#define STARTKEY 0x55 //indicates beginning of an array message
#define NUM_SWITCHES 12
#define COM_TIMEOUT 2400	//how long communications must cease to warn user in units of 1.25 ms
/*Define UART operation const*/
#define FOSC 11059200L //system oscillator frequency
#define BAUD 9600 //baud rate
#define S2RI 0x01 //S2CON.0
#define S2TI 0x02 //S2CON.1
/*Define ADC operation const for ADC_CONTR*/
#define ADC_POWER   0x80 //ADC power control bit
#define ADC_FLAG    0x10 //ADC complete flag
#define ADC_START   0x08 //ADC start control bit
#define ADC_SPEEDLL 0x00 //540 clocks
#define ADC_SPEEDL  0x20 //360 clocks
#define ADC_SPEEDH  0x40 //180 clocks
#define ADC_SPEEDHH 0x60 //90 clocks

/*************************************************************************
 *                            -- VARIABLES --
 *  Each variable is a pin on the MCU that can be read/write 1/0.
 *  Syntax:
 *       varaible-type  variable-name = pin;
 *************************************************************************/
//sfr P4SW = 0xBB; //special register for using P4 pins as I/O
//sfr BRT = 0x9C; //special register for using baud rate timer

sbit LED1 = P0^0;
sbit LED2 = P0^1;
sbit LED3 = P0^2;
sbit LED4 = P0^3;
sbit LED5 = P0^4;
sbit LED6 = P0^5;
sbit LED12 = P0^6;
sbit LED11 = P0^7;

sbit E0 = P1^0; //encoder pin 0
sbit E1 = P1^1; //encoder pin 1

sbit GPIO1 = P1^4;
sbit GPIO2 = P1^5;
sbit BATTLVL = P1^6; //analog voltage reading of battery
sbit sw1 = P1^7;

sbit LED7 = P2^7;
sbit sw12 = P2^6;
sbit sw11 = P2^5;
sbit sw10 = P2^4;
sbit sw9 = P2^3;
sbit sw8 = P2^2;
sbit swTOG = P2^1; //main override toggle switch
sbit LEDtog = P2^0; //main override indicator

sbit sw2 = P3^2;
sbit sw3 = P3^3;
sbit sw4 = P3^4;
sbit sw5 = P3^5;
sbit sw6 = P3^6;
sbit sw7 = P3^7;

sbit LED10 = P4^6;
sbit LED9 = P4^5;
sbit LED8 = P4^4;

/*************************************************************************
 *                           --PROTOTYPES--
 *************************************************************************/
void uart_init(); //Initialize UART1 and UART2 sfr
void uart1_tx(unsigned char dat); //UART1 transmit single byte
void send(); //UART1 send output array
void uart2_tx(unsigned char dat); //UART2 transmit single byte
void update_OLED(); //update OLED
void adc_init(); //Initialize ADC sfr
void timer0_init(); //Initialize Timer0 sfr
void generateCheckSum();
void button_check();
void update_LEDS();
unsigned char readBattery(unsigned char ch);
void checkCheckSum();

/*************************************************************************
 *                          --GLOBAL VARIABLES--
 *************************************************************************/
/* Communication Arrays */
//sequencestage,criticaldata,errormessages is just forwarded from input to arduino
unsigned char input[IN_COUNT]; //stores all recieved inputs
/* [0,override][1-14,solenoid states][13-16,errors][17-20,warnings][21-24,pressure valves][25-26,checkSum] */
//SOLENOIDSTATES CAN BE 0=INACTIVE, 1=ACTIVE, 2=PENDING
unsigned char output[OUT_COUNT]; //stores all commands to be sent
/* [0,override][1-14,buttonpresses][5,encoder][6-7,checkSum] */
unsigned char oled[OLED_COUNT]; //stores info for oled to display
/* [0,override][1,tone][2,battlvl][3,commsgood][4,thrust][5,tank1p][6,tank1t][7,tank2p][8,tank2t][9-12,kbot1-4][13-22,errormessages] */
//TONE CAN BE 0=none, 1=buttonpress, 2=overrideswitch, 3=error

unsigned int i = 0; //for loops
unsigned int timerCounter = 0; //for timer0 isr
unsigned int comCounter = 0;
bit inputChanged = 0;
bit busy = 0; //boolean for UART TX holding
bit storing = 0; //1 if currently updating RX array, 0 otherwise
bit readSwitchesFlag = 0; //1 if input data has been updated, 0 otherwise
bit updateLEDFlag = 0;
bit checkBit1 = 0;
bit checkBit2 = 0;
unsigned char com;
unsigned char ch = 6; //ADC channel number for BATTLVL
unsigned char LED[NUM_SWITCHES+1];
unsigned char lastState[NUM_SWITCHES+1];
unsigned char currentState[NUM_SWITCHES+1];
unsigned char debounceFlag[NUM_SWITCHES+1];
unsigned char debounceCounter[NUM_SWITCHES+1];
unsigned char LEDState = 0;
unsigned int index = 0; //for UART RX array location
//////////////////////////////////////////////////////////////////unsigned int debugsum=0;
/*----------------------------
Main loop.
Initializes MCU registers, checks for constant communication,
reads switches and buttons, sends commands with checkSum, updates
leds and oled with new info.
----------------------------*/
void main() {
	P4SW = 0x70; //enable IO for all of P4
	uart_init(); //Initialize UART1 and UART2 sfr.
	adc_init(); //Initialize ADC sfr
	timer0_init(); //Initialize Timer0 sfr
	EA = 1; //Open master interrupt switch
	while(1) { //loop forever
		if(checkBit1){uart1_tx(0x99);}
		else {uart1_tx(0x77);}
		if(checkBit2){uart1_tx(0x99);}
		else {uart1_tx(0x77);}
//////////////////		if(checkBit){
		for(i=0;i<=3000;i++){nop_();}	  //////////////////////////////////////////////////////////
		if(readSwitchesFlag==1){button_check(); readSwitchesFlag=0;} 
		if(updateLEDFlag==1){update_LEDS();}
//////////////////////////////////////////////////////////////////////////////////////}
//METHOD TO COPY INPUT STATES TO OUTPUT STATES
		send(); //transmit output 
//		if (((LEDState/4)%2)&1==0 && checkBit){update_OLED();} //update OLED
	} //end while
} //end main


/*----------------------------
Update OLED screen with info.
----------------------------*/
void update_OLED() {							//update array:
	oled[0] = output[0];		//override					   
	//oled[1]					//tone doesn't need to be updated, updated elsewhere in code
	oled[2] = readBattery(1);	//battery level
	oled[3] = com;				//communication updated elsewhere	
	for(i = 0; i < OLED_COUNT; i++) {uart2_tx(oled[i]);}		//send the oled array
	oled[1]=0; 		//set tone  back to zero
}

/*----------------------------
Initialize UART1 and UART2 sfr.
Data buffer is set to 8 bit length,
baud rate set to 9600 baud,
uart interrupt is enabled.
----------------------------*/
void uart_init() {
	SCON = 0x50; //8-bit variable UART1
	TMOD = 0x21; //timer1 in mode2. timer0 in mode1
	TH1 = TL1 = -(FOSC/12/32/BAUD); //Set auto-reload vaule
	TR1 = 1; //Timer 1 enable
	ES = 1; //Enable UART interrupt
	
	S2CON = 0x50; //8-bit variable UART2
	BRT = -(FOSC/32/BAUD); //Set auto-reload vaule of baudrate generator
	AUXR = 0xF5; //see datashee pg185
	IE2 = 0x01; //Enable UART2 interrupt
	
	for(i = 0; i < OUT_COUNT; ++i) //initialize output array to zeros
		output[i] = 0;
	for(i = 0; i < IN_COUNT; ++i) //initialize input array to zeros
		input[i] = 0;
	for(i = 0; i < OLED_COUNT; ++i) //initialize oled array to zeros
		oled[i] = 0;
}

/*----------------------------
UART1 interrupt routine.
Updates the input array.
----------------------------*/
void Uart_Isr() interrupt 4 using 1 {
	unsigned char c;
	if(RI) { //receive1 flagged
		RI = 0; //reset receive flag
		c = SBUF; //store buffer in c
		if(storing) { //we are in storing mode
			input[index] = c; //store SBUF in current index of array
			index++; //increment array index
		} else if(c == STARTKEY) { //start key is received
			storing = 1; //set that we are now storing
			index = 0; //start from beginning of array by resetting index
		}
		if(index >= IN_COUNT) { //read in enough values
			checkCheckSum();
			storing = 0; //set that we are done storing
			inputChanged = 1; //tell the program that new data has been delivered
		}	
	}	
	if(TI) { //transmit1 flagged
		TI = 0; //Clear transmit interrupt flag
		busy = 0; //Clear transmit busy flag
	}
}

/*----------------------------
UART1 transmit single byte
----------------------------*/
void uart1_tx(unsigned char dat) {
	while(busy); //Wait for the completion of the previous data is sent
	busy = 1; //set transmit busy flag
	SBUF = dat; //Send data to UART buffer
}

/*----------------------------
UART1 send output array
----------------------------*/
void send() {
	uart1_tx(STARTKEY);
	generateCheckSum(); 
	for(i = 0; i < OUT_COUNT; ++i) {uart1_tx(output[i]);}
}

/*----------------------------
UART2 interrupt routine
----------------------------*/
void Uart2() interrupt 8 using 1 {
    if(S2CON & S2RI) { //Does nothing with received data
        S2CON &= ~S2RI; //Clear receive interrupt flag
    }
    if(S2CON & S2TI) { //transmit2 flagged
        S2CON &= ~S2TI; //Clear transmit interrupt flag
        busy = 0; //Clear transmit busy flag
    }
}

/*----------------------------
UART2 transmit single byte
----------------------------*/
void uart2_tx(unsigned char dat) {
    while(busy); //Wait for the completion of the previous data is sent
    busy = 1; //set transmit flag to busy
    S2BUF = dat; //Send data to UART2 buffer
}

/*----------------------------
Initialize ADC sfr
----------------------------*/
void adc_init() {
	P1ASF = 0x03; //set P1.0 and P1.1 as analog input ports
	ADC_RES = 0; //Clear previous result
	ADC_CONTR = ADC_POWER | ADC_SPEEDLL; //set register
	for(i=0;i<10;i++){nop_();} //ADC power-on delay and Start A/D conversion
	IE = 0xb0; //Enable ADC and UART interrupt and Open master interrupt switch
}

/*----------------------------
Initialize Timer0 sfr
----------------------------*/
void timer0_init() {
	TL0 = 0xFF; //initialize timer0 low byte	  65535-(11059200/12/800)=FB7F
	TH0 = 0xED; //initialize timer0 high byte
	TR0 = 1; //timer0 start running
	ET0 = 1;} //enable timer0 interrupt


void writeOutput(char index){
	if (output[0]==1){
		if (debounceFlag[index]==1 && currentState[index]==1 && lastState[index]==1){debounceCounter[index]++;}
		else if (debounceFlag[index]==1 && currentState[index]==0 && lastState[index]==1 && debounceCounter[index]>=3){
			debounceFlag[index]=0; debounceCounter[index]=0; oled[1]=1;
			if (output[index]==0){output[index]=1;}
			else if (output[index]==1){output[index]=0;}}
		else if (currentState[index]==1 && lastState[index]==0){debounceFlag[index]=1;}
		lastState[index]=currentState[index];}
	else{output[index]=input[index];}}

void button_check(){
	for(i=1; i<=NUM_SWITCHES;i++){writeOutput(i);}
	output[0]=swTOG;
	currentState[1]=sw1;	
	currentState[2]=sw2;
	currentState[3]=sw3;
	currentState[4]=sw4;
	currentState[5]=sw5;
	currentState[6]=sw6;
	currentState[7]=sw7;
	currentState[8]=sw8;
	currentState[9]=sw9;
	currentState[10]=sw10;
	currentState[11]=sw11;
	currentState[12]=sw12;
//	sw13=currentState[13];
//	sw14=currentState[14];
}

void update_LEDS(){
	LEDtog=input[0]&1;
	if (input[1] <= 1){LED1=!(bit)(input[1]&1);}
	else {LED1=((LEDState/4)%2)&1;}
	if (input[2] <= 1){LED2=!(bit)(input[2]&1);}
	else {LED2=((LEDState/4)%2)&1;}
	if (input[3] <= 1){LED3=!(bit)(input[3]&1);}
	else {LED3=((LEDState/4)%2)&1;}
	if (input[4] <= 1){LED4=!(bit)(input[4]&1);}
	else {LED4=((LEDState/4)%2)&1;}
	if (input[5] <= 1){LED5=!(bit)(input[5]&1);}
	else {LED5=((LEDState/4)%2)&1;}
	if (input[6] <= 1){LED6=!(bit)(input[6]&1);}
	else {LED6=((LEDState/4)%2)&1;}
	if (input[7] <= 1){LED7=!(bit)(input[7]&1);}
	else {LED7=((LEDState/4)%2)&1;}
	if (input[8] <= 1){LED8=!(bit)(input[8]&1);}
	else {LED8=((LEDState/4)%2)&1;}
	if (input[9] <= 1){LED9=!(bit)(input[9]&1);}
	else {LED9=((LEDState/4)%2)&1;}
	if (input[10] <= 1){LED10=!(bit)(input[10]&1);}
	else {LED10=((LEDState/4)%2)&1;}
	if (input[11] <= 1){LED11=!(bit)(input[11]&1);}
	else {LED11=((LEDState/4)%2)&1;}
	if (input[12] <= 1){LED12=!(bit)(input[12]&1);}
	else {LED12=((LEDState/4)%2)&1;}
//	if (input[13] <= 1){LED13=!input[13]&1;}
//	else {LED13=((LEDState/4)%2)&1;}
//	if (input[14] <= 1){LED14=!input[14]&1;}
//	else {LED14=((LEDState/4)%2)&1;}
}

/*----------------------------
Timer0 interrupt routine
----------------------------*/
void tm0_isr() interrupt 1 using 1 {
	TL0 = 0xFF;   //initialize timer0 low byte       65535-(11059200/12/200)=EDFF
	TH0 = 0xED;   //initialize timer0 high byte
	timerCounter++;
	readSwitchesFlag=1;
	if (inputChanged==0){comCounter++;
		if(comCounter>=COM_TIMEOUT){com=1; oled[1]=2;}}
	else{comCounter=0; inputChanged=0; com=0;}
	if(timerCounter%50==0){updateLEDFlag=1; LEDState++;}
	if(timerCounter==800){timerCounter=0;}}


unsigned char readBattery(unsigned char ch){
    ADC_CONTR = ADC_POWER | ADC_SPEEDLL | ch | ADC_START;
    _nop_(); _nop_(); _nop_(); _nop_(); 
    while (!(ADC_CONTR & ADC_FLAG));//Wait complete flag
    ADC_CONTR &= ~ADC_FLAG;         //Close ADC
    return ADC_RES;}                //Return ADC result

void generateCheckSum(){
    unsigned int sum=0; unsigned int b;
    for (b = 0; b < OUT_COUNT-2; ++b) {sum+=output[b];}
	output[OUT_COUNT-2]=(unsigned char)(sum>>8); 
	output[OUT_COUNT-1]=(unsigned char)sum;}

void checkCheckSum(){
    unsigned int sum=0; unsigned int b;
	for (b = 0; b < IN_COUNT-2; ++b) {sum+=input[b];}////////////////////////////////////////////////////////////// debugsum=sum;
    if (input[IN_COUNT-2]==(unsigned char)(sum>>8)){checkBit1=1;}
	else {checkBit1=0;}
	if (input[IN_COUNT-1]==(unsigned char)sum){checkBit2=1;}
	else {checkBit2=0;}}