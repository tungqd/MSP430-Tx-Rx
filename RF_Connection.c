/******************************************************************************
* This example realize simple RF connection between 2 MSP430-CCRF boards.    *
* Both boards are sending ~4 packets per second with lenght 10 bytes. 
* Output power is set by default to -30dBm!
* Whenever any board has received a valid packet, it switches-over its LED!
* Packet sending can be enabled/disabled individually for everyone board by pressing its button BUT.
* Warning: This RF code example is setup to operate at 868Mhz frequency, 
* which might be out of allowable range of operation in certain countries. Please
* refer to the appropriate legal sources before performing tests with this code 
* example. 
* 
* The RF packet engine settings specify variable-length-mode with CRC check enabled
* The RX packet also appends 2 status bytes regarding CRC check, RSSI and LQI info.
* For specific register settings please refer to the comments for each register in
* RF1A_REGISTER_CONFIG[] or the CC430x613x User's Guide and the SmartRF Studio
* 
* All required changes, which enable this code to be portable for MSP430-CCRF,
* were made by Penko T. Bozhkov -> Olimex LTD
******************************************************************************/
#include "cc430x513x.h"
#include "HAL/RF1A.h"
#include "HAL/cc430x613x_PMM.h"
#include "RF_Connection.h"

#ifdef MHZ_915
  #include "HAL/RF_config_Olimex/smartrf_CC430F5137_915MHz_38k4Baud.h"
#elif defined MHZ_868
  #include "HAL/RF_config_Olimex/smartrf_CC430F5137_868MHz_38k4Baud.h"
#endif


#define LED_Togg    P1OUT ^= 0x01;  P1DIR |= 0x01;
#define timer1_A3_Stop_Mode     TA1CTL &= (~0x0030)
#define timer1_A3_Up_Mode       TA1CTL |= (0x0010)

unsigned char button_on = 0;


// Set All 47 Configuration Registers!!!! 
// Two are reserved - writtne to "0". All other 45 are exported by SmartRF Studio!
const unsigned char RF1A_REGISTER_CONFIG[CONF_REG_SIZE]=
{
  SMARTRF_SETTING_IOCFG2  ,  // IOCFG2: GDO2 signals on RF_RDYn     
  SMARTRF_SETTING_IOCFG1  ,  // IOCFG1: GDO1 signals on RSSI_VALID     
  SMARTRF_SETTING_IOCFG0  ,  // IOCFG0: GDO0 signals on PA power down signal to control RX/TX switch         
  SMARTRF_SETTING_FIFOTHR , // FIFOTHR: RX/TX FIFO Threshold: 33 bytes in TX, 32 bytes in RX    
  SMARTRF_SETTING_SYNC1   , // SYNC1: high byte of Sync Word
  SMARTRF_SETTING_SYNC0   , // SYNC0: low byte of Sync Word
  SMARTRF_SETTING_PKTLEN  , // PKTLEN: Packet Length in fixed mode, Maximum Length in variable-length mode      
  SMARTRF_SETTING_PKTCTRL1, // PKTCTRL1: No status bytes appended to the packet    
  SMARTRF_SETTING_PKTCTRL0, // PKTCTRL0: Fixed-Length Mode, No CRC       
  SMARTRF_SETTING_ADDR    , // ADDR: Address for packet filtration       
  SMARTRF_SETTING_CHANNR  , // CHANNR: 8-bit channel number, freq = base freq + CHANNR * channel spacing          
  SMARTRF_SETTING_FSCTRL1 , // FSCTRL1: Frequency Synthesizer Control (refer to User's Guide/SmartRF Studio) 
  SMARTRF_SETTING_FSCTRL0 , // FSCTRL0: Frequency Synthesizer Control (refer to User's Guide/SmartRF Studio) 
  SMARTRF_SETTING_FREQ2   , // FREQ2: base frequency, high byte      
  SMARTRF_SETTING_FREQ1   , // FREQ1: base frequency, middle byte      
  SMARTRF_SETTING_FREQ0   , // FREQ0: base frequency, low byte      
  SMARTRF_SETTING_MDMCFG4 , // MDMCFG4: modem configuration (refer to User's Guide/SmartRF Studio)     
  SMARTRF_SETTING_MDMCFG3 , // MDMCFG3:                "                      "    
  SMARTRF_SETTING_MDMCFG2 , // MDMCFG2:                "                      "        
  SMARTRF_SETTING_MDMCFG1 , // MDMCFG1:                "                      "        
  SMARTRF_SETTING_MDMCFG0 , // MDMCFG0:                "                      "        
  SMARTRF_SETTING_DEVIATN , // DEVIATN: modem deviation setting (refer to User's Guide/SmartRF Studio)         
  SMARTRF_SETTING_MCSM2   , // MCSM2: Main Radio Control State Machine Conf. : timeout for sync word search disabled      
  SMARTRF_SETTING_MCSM1   , // MCSM1: CCA signals when RSSI below threshold, stay in RX after packet has been received      
  SMARTRF_SETTING_MCSM0   , // MCSM0: Auto-calibrate when going from IDLE to RX or TX (or FSTXON )      
  SMARTRF_SETTING_FOCCFG  , // FOCCFG: Frequency Offset Compensation Conf.     
  SMARTRF_SETTING_BSCFG   , // BSCFG: Bit Synchronization Conf.       
  SMARTRF_SETTING_AGCCTRL2, // AGCCTRL2: AGC Control   
  SMARTRF_SETTING_AGCCTRL1, // AGCCTRL1:     "   
  SMARTRF_SETTING_AGCCTRL0, // AGCCTRL0:     "   
  SMARTRF_SETTING_WOREVT1 , // WOREVT1: High Byte Event0 Timeout    
  SMARTRF_SETTING_WOREVT0 , // WOREVT0: High Byte Event0 Timeout
  SMARTRF_SETTING_WORCTRL , // WORCTL: Wave On Radio Control ****Feature unavailable in PG0.6****
  SMARTRF_SETTING_FREND1  , // FREND1: Front End RX Conf.    
  SMARTRF_SETTING_FREND0  , // FREND0: Front End TX Conf.               
  SMARTRF_SETTING_FSCAL3  , // FSCAL3: Frequency Synthesizer Calibration (refer to User's Guide/SmartRF Studio)    
  SMARTRF_SETTING_FSCAL2  , // FSCAL2:              "      
  SMARTRF_SETTING_FSCAL1  , // FSCAL1:              "     
  SMARTRF_SETTING_FSCAL0  , // FSCAL0:              "     
  0x00                    , // Reserved *read as 0*
  0x00                    , // Reserved *read as 0*
  SMARTRF_SETTING_FSTEST  , // FSTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_PTEST   , // PTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_AGCTEST , // AGCTEST: For test only, irrelevant for normal use case
  SMARTRF_SETTING_TEST2   , // TEST2  : For test only, irrelevant for normal use case    
  SMARTRF_SETTING_TEST1   , // TEST1  : For test only, irrelevant for normal use case
  SMARTRF_SETTING_TEST0     // TEST0  : For test only, irrelevant for normal use case    
};


extern unsigned char packetReceived;
extern unsigned char packetTransmit; 

unsigned char RxBuffer[255], RxBufferLength = 0;
const unsigned char TxBuffer[6]= {0x06, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
unsigned char buttonPressed = 0;
unsigned int i = 0; 


/**********************************************************************************/
/*  Function name: InitButtonLed               	                                  */
/*  	Parameters                                                                */
/*          Input   :  No       			                          */
/*          Output  :  No	                                                  */
/*	Action: Initialize Led and Button directions.                          	  */
/**********************************************************************************/
void InitButtonLed(void)
{
  // Set up the button as interruptible 
  P1DIR &= ~BIT1;
  P1REN |= BIT1;
  P1IES &= BIT1;
  P1IFG = 0;
  P1OUT |= BIT1;
  P1IE  |= BIT1; 

  // Set up LED 
  P1OUT &= ~BIT0;
  P1DIR |= BIT0;
}


/**********************************************************************************/
/*  Function name: Init_RF               	                                  */
/*  	Parameters                                                                */
/*          Input   :  No       			                          */
/*          Output  :  No	                                                  */
/*	Action: Initialize RF radio core.                               	  */
/**********************************************************************************/
void Init_RF(void){
  
  // Increase PMMCOREV level to 2 in order to avoid low voltage error 
  // when the RF core is enabled
  SetVCore(2);
  ResetRadioCore();     
  WriteBurstReg(IOCFG2, (unsigned char*)RF1A_REGISTER_CONFIG, CONF_REG_SIZE);
  WritePATable();
  InitButtonLed();
  ReceiveOn();  
  //Wait for RX status to be reached
  while((Strobe(RF_SNOP) & 0x70) != 0x10);
  
}


/**********************************************************************************/
/*  Function name: RF_Connection_Test         	                                  */
/*  	Parameters                                                                */
/*          Input   :  No       			                          */
/*          Output  :  True or False                                              */
/*	Action: Send packet when BUT is pressed and receive echo.       	  */
/**********************************************************************************/
void RF_Connection_Test(void)
{  
  Init_RF();
  timer1_A3_Up_Mode;
  
  while(1){
    
    if (buttonPressed)                      // Process a button pressed --> transmit 
    {
      //P3OUT |= BIT6;                        // Pulse LED during Transmit                          
      buttonPressed = 0; 
      P1IFG = 0; 
      
      ReceiveOff();
      Transmit( (unsigned char*)TxBuffer, sizeof TxBuffer);      
      
      //Wait for TX status to be reached before going back to low power mode
      while((Strobe(RF_SNOP) & 0x70) != 0x20);    
       
      P1IE |= BIT1;                         // Re-enable button press  
    }
    else if (packetTransmit)
    {
      RF1AIE &= ~BIT9;                      // Disable RFIFG9 TX end-of-packet interrupts
      //P3OUT &= ~BIT6;                       // Turn off LED after Transmit 
      
      ReceiveOn();
      
      //Wait for RX status to be reached
      while((Strobe(RF_SNOP) & 0x70) != 0x10); 
    
      packetTransmit = 0; 
    }
    else if(packetReceived)                 // Process a received packet 
    {
      char toggle = 1; 
      
      // Read the length byte from the FIFO       
      RxBufferLength = ReadSingleReg( RXBYTES );               
      ReadBurstReg(RF_RXFIFORD, RxBuffer, RxBufferLength); 
    
      // Check the packet contents and don't toggle LED if they are different 
      for(i = 0; i < RxBuffer[0]; i++)
      {
        if(RxBuffer[i] != TxBuffer[i]) toggle = 0; 
      }      
        
      if(toggle){
        LED_Togg;                  // Toggle LED
        packetReceived = 0; 
      }
      
    }
  }
}


/**********************************************************************************/
/*      PORT1, interrupt service routine.                                         */
/**********************************************************************************/
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
  switch(__even_in_range(P1IV, 4))
  {
    case  0: break;
    case  2: break;                         // P1.0 IFG
    case  4:
      //P1IE = 0;                             // Debounce by disabling buttons
      if(button_on){
        button_on = 0;
        timer1_A3_Stop_Mode;
      }
      else{
        button_on = 1;
        timer1_A3_Up_Mode;
      }
      //buttonPressed = 1;   
      break;                                // P1.1 IFG
    case  6: break;                         // P1.2 IFG
    case  8: break;                         // P1.3 IFG
    case 10: break;                         // P1.4 IFG
    case 12: break;                         // P1.5 IFG
    case 14: break;                         // P1.6 IFG
    case 16: break;                         // P1.7 IFG 
      
  }
}


/****************************************************************************/
/*  Timer1_A3 CC0 interrupt service routine.                                */
/****************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_Capture_Compare_ISR(void)
{
  buttonPressed = 1;
}

