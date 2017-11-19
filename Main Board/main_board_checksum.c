//
/*************************************************************************
 *  @File Descption: Read states of manual override buttons of control_box,
 *  forward them to solenoids, receives feedback from DAQ and forwards to 
 *  control board along with solenoid state information
 *	@File: main_board_firmware.c
 *  @author(s):
 *  -Cristian Sharp, Merp
 *	-Matthew Santos, Electrical and Software Engineer
 *  @Property of SEDS UCSD
 *  @since: 4/2017
 *************************************************************************/
//#include "reg60s2.h" //Include reg file for 8051 architecure
#include "STC/STC12C5A60S2.H"
#include "intrins.h"
#include "math.h"
#define IN_CONTROL_COUNT 16 //1 override + 12 solenoidstates + 1 encoder + 2 checkSumckSum
#define IN_DAQ_COUNT 56 //34 errors or warnings + 4 T tank1 + 4 T tank2 + 4 P tank1 + 4 P tank2 + 6 bytes for kbot1-3
#define NUM_OF_SWITCHES 12 //total number of switches not including toggle
#define OUT_COUNT 71 //1 override + 12 switches + 56 IN_DAQ COUNT + 2 checkSum
#define STARTKEY 0x55 //indicates beginning of an array message
/*Define UART operation const*/
#define FOSC 11059200L //system oscillator frequency
#define BAUD 9600 //baud rate
#define S2RI 0x01 //S2CON.0
#define S2TI 0x02 //S2CON.1
/*************************************************************************
 *                            -- VARIABLES --
 *  Each variable is a pin on the MCU that can be read/write 1/0.
 *  Syntax:
 *       varaible-type  variable-name = pin;
 *************************************************************************/
// CHANGE ALL VARS AND ADJUST
sbit ACT_MUX_S = P3^5;

sbit A1 = P0^0;
sbit A2 = P0^1;
sbit A3 = P0^2;
sbit A4 = P0^3;
sbit A5 = P0^4;
sbit A6 = P0^5;
sbit A7 = P0^6;
sbit A8 = P0^7;
sbit A9 = P2^7;
sbit A10 = P2^6;
sbit A11 = P2^5;
sbit A12 = P2^4;

sbit IN_MUX_S = P1^0;
sbit IN_MUX_E = P1^1;
sbit MUX_OUT1 = P1^4;
sbit MUX_OUT2 = P1^5;
sbit MUX_OUT3 = P1^6;
sbit MUX_OUT4 = P1^7;
/*************************************************************************
 *                                   --PROTOTYPES--
 *************************************************************************/

void uart_init(); //Initialize UART1 and UART2 sfr
void uart1_tx(unsigned char dat); //UART1 transmit single byte
void send(); //UART1 send output array
void generateCheckSum();  //in send() function
void checkCheckSum();
void update_output_array();
void solenoid_data();

/*************************************************************************
 *                          --GLOBAL VARIABLES--
 *************************************************************************/

/* Communication Arrays */
//sequencestage,criticaldata,errormessages is just forwarded from input_control to arduino
unsigned char output[OUT_COUNT]; //stores all recieved inputs
/* [0,override][1-12,solenoid states][13-16,errors][17-20,warnings][21-24,pressure valves][25-26,checkSum] */
//SOLENOIDSTATES CAN BE 0=INACTIVE, 1=ACTIVE, 2=PENDING
unsigned char input_control[IN_CONTROL_COUNT]; //stores all commands to be sent
/* [0,override][1-12,buttonpresses][14-15,checkSum] */
unsigned char input_DAQ[IN_DAQ_COUNT]; //stores all commands to be sent
/* [0-3,errors][4-7,warnings][8-11,pressure valves] */
unsigned char switchs[NUM_OF_SWITCHES+1];
unsigned char i = 0; //for loops
bit busy; //boolean for UART TX holding
bit inputChanged = 0;
xdata unsigned char index_control = 0; //for UART RX array location
unsigned char index_DAQ = 0; //for UART RX array location
bit storing_DAQ = 0; //1 if currently updating RX2 (input_DAQ) array, 0 otherwise
bit storing_control = 0; //1 if currently updating RX1 (input_control) array, 0 otherwise
bit checkBit;

/*************************************************************************
 *                          -- MAIN FUNCTION --
 *  @Descption: The unsigned char array "input_control" stores commands from 
 *              control board. "input_DAQ" stores info from DAQ. "output"
 *              stores info about solenoid states and info from the DAQ 			
 *
 *  @PRECONDITION: none
 *
 *  @POSTCONDITION: -connenction between two MCU's will be complete
 *                  -commands will be received and delt with
 *                  -commands will be sent to other MCU
 *
 *  @PARAMETER: none
 *
 *  @RETURN: none
 *************************************************************************/
void main() {	
	P4SW = 0x70; //enable IO for all of P4
	uart_init(); //Initialize UART1 and UART2 sfr
	EA = 1; //Open master interrupt switch
	while(1) {
		solenoid_data();		 //collect data about solenoid states										    
		update_output_array();	 //update output array using that info							
		send();					 //and send that composite data to control board
	}	 
} //end main

/*----------------------------
Takes info from DAQ (stored in input_DAQ)
and info from control board (stored in 
input_control) and updates output array.
checkSum_generator (called at end of function) also 
changes last 2 values of output array.
output array is later sent to control board
through UART1 in the send() function 
----------------------------*/
void update_output_array() {
	 if (checkBit){
	 	uart1_tx(0x99);  ////////////////////////////////////////////////////////////
		output[0]=input_control[0];			   
		if(input_control[0]==1)	//if override is 1 and checkSum_check is 1, send override 
		{									 		//values to A1 through A12. 
			ACT_MUX_S=0;	for(i=0;i<=20;i++){nop_();}/////////////////////////////////					    //forward override switch values to solenoids
			for(i=1;i<=NUM_OF_SWITCHES;i++)
			{
				if(switchs[i]==0 && input_control[i]==0)	   		 //Compare values of A1-A12 to actual solenoid states
				{										   		     //and store that comparison in output array so that it 		
					output[i] = 0;									 //gets sent to control board through UART1. 
				}										   		   		
				else if (switchs[i]==1 && input_control[i]==1)
				{							//1=active
					output[i] = 1;
				}
				else if (switchs[i] == 1 && input_control[i] == 0)
				{						   //2=pending (switched HIGH, but solenoid is not actually HIGH yet)
					output[i] = 2;
				} 
				else if (switchs[i] == 0 && input_control[i] == 1)
				{						   //2=pending (switched HIGH, but solenoid is not actually HIGH yet)
					output[i] = 3;
				}
			}								  
		}
		else{
			ACT_MUX_S=1; for(i=0;i<=20;i++){nop_();}					//and set ACT_MUX_S to HIGH so solenoids take commands from DAQ
			for(i=1;i<=NUM_OF_SWITCHES;i++)	//instead of control board
			{	  
				output[i] = switchs[i];
			}
		}
		A1=!(output[1]&1); A2=!(output[2]&1); A3=!(output[3]&1); 
		A4=!(output[4]&1); A5=!(output[5]&1); A6=!(output[6]&1); 
		A7=!(output[7]&1); A8=!(output[8]&1); A9=!(output[9]&1); 
		A10=!(output[10]&1); A11=!(output[11]&1); A12=!(output[12]&1);
	//	A13=output[13]; A14=output[14];
	}
	else{uart1_tx(0x77);}  ////////////////////////////////////////////////////////////												
	for(i=1;i<=IN_DAQ_COUNT;i++)			 //update output array with info from DAQ (UART2) to control board via UART1
	{									 
		output[NUM_OF_SWITCHES+i] = input_DAQ[i-1];  
	}			    
}

/*----------------------------
Uses IN_MUX pins to select info about
solenoids and their states and stores
that info in switchs array
----------------------------*/
void solenoid_data() {
	IN_MUX_S = 1; 			  						  
	IN_MUX_E = 1;	  	for(i=0;i<=20;i++){nop_();}
  	switchs[2] = !MUX_OUT1;	  //even top
  	switchs[4] = !MUX_OUT2;	  
	switchs[6] = !MUX_OUT3;
	switchs[8] = !MUX_OUT4;	
	IN_MUX_S = 1;
	IN_MUX_E = 0;		 for(i=0;i<=20;i++){nop_();}
	switchs[10] = !MUX_OUT1;
	switchs[12] = !MUX_OUT2;
//	switchs[14] = MUX_OUT3;	  //maybe adding more solenoids in the future?
    IN_MUX_S = 0;
  	IN_MUX_E = 1;	   for(i=0;i<=20;i++){nop_();}
  	switchs[1] = !MUX_OUT1;	  //odd bottom
  	switchs[3] = !MUX_OUT2;
  	switchs[5] = !MUX_OUT3;
	switchs[7] = !MUX_OUT4;
	IN_MUX_S = 0;
	IN_MUX_E = 0; 	  for(i=0;i<=20;i++){nop_();}
  	switchs[9] = !MUX_OUT1;
  	switchs[11] = !MUX_OUT2;	  
//  switchs[13] = MUX_OUT3;		//maybe adding more solenoids in the future?		    
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
	//IE2   = 0xaf;
	for(i = 0; i < OUT_COUNT; i++) //initialize output array to zeros
		output[i] = 0;
	for(i = 0; i < IN_CONTROL_COUNT; i++) //initialize input_control array to zeros
		input_control[i] = 0;
	for(i = 0; i < IN_DAQ_COUNT; i++) //initialize input_control array to zeros
		input_DAQ[i] = 0;
	for(i = 0; i < NUM_OF_SWITCHES; i++) //initialize input_control array to zeros
		switchs[i] = 0;
}





/*----------------------------
UART1 interrupt routine.
Updates the input_control array.
----------------------------*/
void Uart_Isr() interrupt 4 {
	unsigned char c=0;
	if(RI) { //receive1 flagged
		RI = 0; //reset receive flag
		c = SBUF; //store buffer in c
		if(storing_control) { //we are in storing_control mode
			input_control[index_control] = c; //store SBUF in current index_control of array
			index_control++; //increment array index
		}
		else if(c == STARTKEY) { //start key is received
			storing_control = 1; //set that we are now storing_control
			index_control = 0; //start from beginning of array
		} 
		if(index_control >= IN_CONTROL_COUNT) { //read in enough values
			checkCheckSum();
			storing_control = 0; //set that we are done storing_control
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
	uart1_tx(STARTKEY);					 //send startbyte						 
	generateCheckSum();
	for(i = 0; i < OUT_COUNT; i++) {	 //send entire output array 
		uart1_tx(output[i]);			 //include "override" char so that control board knows 
	}									 //if it is interpreting solenoid state values or the state 
}										 //of the solenoid compared to its corresponding button

/*----------------------------
UART2 interrupt service routine
----------------------------*/
void Uart2() interrupt 8 using 1
{
unsigned char d;
	if(S2CON & S2RI) { //receive2 flagged
		S2CON &= ~S2RI; //reset receive flag
		d = S2BUF; //store buffer in d
		if(storing_DAQ) { //we are in storing_DAQ mode
			input_DAQ[index_DAQ] = d; //store SBUF in current index_control of array
			index_DAQ++; //increment array index
		} else if(d == STARTKEY) { //start key is received
			storing_DAQ = 1; //set that we are now storing_DAQ
			index_DAQ = 0; //start from beginning of array
		} 
		if(index_DAQ >= IN_DAQ_COUNT) { //read in enough values
			storing_DAQ = 0; //set that we are done storing_DAQ
		}
	}	
	if(S2CON & S2TI) { //transmit1 flagged
		S2CON &= ~S2TI; //Clear transmit interrupt flag
		busy = 0; //Clear transmit busy flag
	}
}

void generateCheckSum(){
    unsigned int sum=0;	unsigned int b;
    for (b = 0; b < OUT_COUNT-2; ++b) {sum+=output[b];}
	output[OUT_COUNT-2]=(unsigned char)(sum>>8); 
	output[OUT_COUNT-1]=(unsigned char)sum;}

void checkCheckSum(){
    unsigned int sum=0; unsigned int b;
	for (b = 0; b < IN_CONTROL_COUNT-2; ++b) {sum+=input_control[b];}
    if (input_control[IN_CONTROL_COUNT-2]==(unsigned char)(sum>>8) && input_control[IN_CONTROL_COUNT-1]==(unsigned char)sum){checkBit=1;}
	else {checkBit=0;}}
