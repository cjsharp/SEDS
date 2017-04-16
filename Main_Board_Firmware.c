//Cris's board
/*************************************************************************
 *  @File Descption: Read states of manual override buttons of control_box,
 *  forward them to solenoids, receives feedback from DAQ and forwards to 
 *  control board along with solenoid state information
 *	@File: main_board_firmware.c
 *  @author(s):
 *  -Cristian Sharp, Embedded Systems Engineer, head of softwar
 *	-Matthew Santos, Electrical and Software Engineer
 *  @Property of SEDS UCSD
 *  @since: 4/2017
 *************************************************************************/
#include "reg60s2.h" //Include reg file for 8051 architecure
#include "crc.h" //for crc generator
#define OUT_COUNT 27 //1 startkey + 1 tog + 12 sw + 2 pot + 4 crc
#define IN_CONTROL_COUNT 16 //1 override + 12 solenoidstates + 3 errormessages + 4 crc
#define IN_DAQ_COUNT 12 
#define NUM_OF_SWITCHES 12 //total number of switches 
#define STARTKEY 255 //indicates beginning of an array message
#define TIMEOUT 3000 //time until communication is considered missing
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
sbit Rx2_3 = P1^2;
sbit Tx2_4 = P1^3;

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
void timer0_init(); //Initialize Timer0 sfr
void delay(unsigned int n); //Software delay function
void CRC_generator();  //in send() function
bit CRC_check();

/*************************************************************************
 *                          --GLOBAL VARIABLES--
 *************************************************************************/

/* Communication Arrays */
//sequencestage,criticaldata,errormessages is just forwarded from input_control to arduino
unsigned char output[OUT_COUNT]; //stores all recieved inputs
/* [0,override][1-12,solenoid states][13-16,errors][17-20,warnings][21-24,pressure valves][25-26,crc] */
//SOLENOIDSTATES CAN BE 0=INACTIVE, 1=ACTIVE, 2=PENDING
unsigned char input_control[IN_CONTROL_COUNT]; //stores all commands to be sent
/* [0,override][1-12,buttonpresses][14-15,crc] */
unsigned char input_DAQ[IN_DAQ_COUNT]; //stores all commands to be sent
/* [0-3,errors][4-7,warnings][8-11,pressure valves] */
unsigned char switchs[NUM_OF_SWITCHES];


int i = 0; //for loops
bit busy; //boolean for UART TX holding
unsigned char index_control = 0; //for UART RX array location
unsigned char index_DAQ = 0; //for UART RX array location
bit storing_DAQ = 0; //1 if currently updating RX2 (input_DAQ) array, 0 otherwise
bit storing_control = 0; //1 if currently updating RX1 (input_control) array, 0 otherwise
unsigned int comms_timeout_count = TIMEOUT; //countdown to timeout, 3sec

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
	unsigned int counter = 0;	
	unsigned int x = 200;
	crc_init();
	P4SW = 0x70; //enable IO for all of P4
	uart_init(); //Initialize UART1 and UART2 sfr.
	timer0_init(); //Initialize Timer0 sfr
	EA = 1; //Open master interrupt switch
	while(1) {
		// Even odd
		// Top bottom			  //use IN_MUX pins to select info about 
		IN_MUX_S = 1; 			  //certain solenoids and their states
  		IN_MUX_E = 1;	
	  	switchs[2] = MUX_OUT1;
	  	switchs[4] = MUX_OUT2;
  		switchs[6] = MUX_OUT3;
  		switchs[8] = MUX_OUT4;	

	 	delay(200);
  		IN_MUX_S = 1;
  		IN_MUX_E = 0;	
  		switchs[10] = MUX_OUT1;
  		switchs[12] = MUX_OUT2;
//		switchs[14] = MUX_OUT3;	  //maybe adding more solenoids in the future?
	  	delay(200);
			  
	    IN_MUX_S = 0;
	  	IN_MUX_E = 1;	
	  	switchs[1] = MUX_OUT1;
	  	switchs[3] = MUX_OUT2;
	  	switchs[5] = MUX_OUT3;
		switchs[7] = MUX_OUT4;	
		delay(200);					//why this delay?
		
		IN_MUX_S = 0;
  		IN_MUX_E = 0;	
	  	switchs[9] = MUX_OUT1;
	  	switchs[11] = MUX_OUT2;	  //maybe adding more solenoids in the future?
//  		switchs[13] = MUX_OUT3;			   //if override is 1 and CRC_check is 1, send override values to A1 
											   //through A12. Compare values of A1-A12 to actual solenoid states 
		output[0]=input_control[0];			   //and store that comparison in output array so that it gets sent to
		if(input_control[0]==1 & CRC_check()==1)	 //control board through UART1. 
		{									 
			ACT_MUX_S=0;
			A1=input_control[1]; A2=input_control[2]; A3=input_control[3]; A4=input_control[4];
			A5=input_control[5]; A6=input_control[6]; A7=input_control[7]; A8=input_control[8];
			A9=input_control[9]; A10=input_control[10]; A11=input_control[11]; A12=input_control[12];
			for(i=1;i<NUM_OF_SWITCHES;++i)
			{
				if(switchs[i]==1 & input_control[i]==1)	   
				{										   
					output[i]=1;						   
				}										   
				else if (switchs[i]==0 &  input_control[i]==0)
				{										   
					output[i]=0;						   
				}
				else if (switchs[i]==0 &  input_control[i]==1)
				{
					output[i]=2;
				}
			}								//if "if statement" fails, then forward solenoid state 
		}									//values from the MUX. Do nothin w/ pins A1-A12
		else 
		{
			ACT_MUX_S=1;
			for(i=1;i<=NUM_OF_SWITCHES;++i)
			{
				output[i]=switchs[i];
			}
		}
		
		for(i=0;i<IN_DAQ_COUNT;++i)			 //update output array with info from DAQ
		{
			output[13+i]=input_DAQ[i];
		}
		
		send();								 //and send that composite data (CRC in send() function)
	}	 
} //end main

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
	for(i = 0; i < OUT_COUNT; ++i) //initialize output array to zeros
		output[i] = 0;
	for(i = 0; i < IN_CONTROL_COUNT; ++i) //initialize input_control array to zeros
		input_control[i] = 0;
	for(i = 0; i < IN_DAQ_COUNT; ++i) //initialize input_control array to zeros
		input_DAQ[i] = 0;
	output[0] = STARTKEY;
}

/*----------------------------
UART1 interrupt routine.
Updates the input_control array.
----------------------------*/
void Uart_Isr() interrupt 4 using 1 {
	unsigned char c;
	if(RI) { //receive1 flagged
		comms_timeout_count = TIMEOUT;
		RI = 0; //reset receive flag
		c = SBUF; //store buffer in c
		if(c == STARTKEY) { //start key is received
			storing_control = 1; //set that we are now storing_control
			index_control = 0; //start from beginning of array
		} else if(storing_control) { //we are in storing_control mode
			input_control[index_control] = c; //store SBUF in current index_control of array
			++index_control; //increment array index
		}
		if(index_control >= IN_CONTROL_COUNT) { //read in enough values
			storing_control = 0; //set that we are done storing_control
			index_control = 0; //reset index
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
	CRC_generator();
	uart1_tx(0xFF);							 //send startbyte
	for(i = 0; i < OUT_COUNT; ++i) {		 //send entire output array 
		uart1_tx(output[i]);				 //include "override" char so that control board knows 
	}										 //if it is interpreting solenoid state values or the state 
}											 //of the solenoid compared to its corresponding button

/*----------------------------
UART2 interrupt service routine
----------------------------*/
void Uart2() interrupt 8 using 1
{
unsigned char c;
	if(S2CON & S2RI) { //receive2 flagged
		comms_timeout_count = TIMEOUT;
		S2CON &= ~S2RI; //reset receive flag
		c = S2BUF; //store buffer in c
		if(c == STARTKEY) { //start key is received
			storing_DAQ = 1; //set that we are now storing_DAQ
			index_DAQ = 0; //start from beginning of array
		} else if(storing_DAQ) { //we are in storing_DAQ mode
			input_DAQ[index_DAQ] = c; //store SBUF in current index_control of array
			++index_DAQ; //increment array index
		}
		if(index_DAQ >= IN_DAQ_COUNT) { //read in enough values
			storing_DAQ = 0; //set that we are done storing_DAQ
			index_DAQ = 0; //reset index
		}
	}	
	if(S2CON & S2TI) { //transmit1 flagged
		S2CON &= ~S2TI; //Clear transmit interrupt flag
		busy = 0; //Clear transmit busy flag
	}
}

/*----------------------------
Initialize Timer0 sfr
----------------------------*/
void timer0_init() {
	TL0 = (65536-FOSC/1000); //initialize timer0 low byte
	TH0 = (65536-FOSC/1000) >> 8; //initialize timer0 high byte
	TR0 = 1; //timer0 start running
	ET0 = 1; //enable timer0 interrupt
	comms_timeout_count = 0; //initial counter
}

/*----------------------------
Timer0 interrupt routine
----------------------------*/
void tm0_isr() interrupt 1 using 1 {
	TL0 = (65536-FOSC/1000); //initialize timer0 low byte
	TH0 = (65536-FOSC/1000) >> 8; //initialize timer0 high byte
	if(comms_timeout_count != 0) { //1ms * 1000 -> 1s
		--comms_timeout_count;
	}
}

/*----------------------------
Software delay function
----------------------------*/
void delay(unsigned int n) {
  unsigned int j;
  while(n--) {
    j = 5000;
    while(j--);
  }
}

/*----------------------------
Append crc bytes to end of output array.
----------------------------*/
void CRC_generator() {
	unsigned short crcval = crcFast(output, OUT_COUNT-2); //compute the crc value from output array up to the actual crc elements
	output[25] = (unsigned char)(crcval>>8); //append the first byte of the crcvalue to the output array
	output[26] = (unsigned char)(crcval); //append the second byte of the crcvalue to the output array
}
/*----------------------------
Check crc of input_control array.
1 if good data, 0 otherwise.
----------------------------*/
bit CRC_check() {
	unsigned short crcval = crcFast(input_control, IN_CONTROL_COUNT-2); //compute the crc value from input_control array up to the actual crc elements
	if(((unsigned char)(crcval>>8) == input_control[14])
		&& ((unsigned char)(crcval) == input_control[15])) {
			return 1;
	}
	return 0;
}
