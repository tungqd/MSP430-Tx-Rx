
/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */

/* Define whether or not to check for RF1AERROR flags during debug */ 
//#define DEBUG 1              // For debug of error flags ONLY

/* Define frequency of operation by commenting the symbol, SELECT_MHZ_FOR_OPERATION,
 * then uncommenting the appropriate frequency symbol (868 or 915)*/ 
//#define SELECT_MHZ_FOR_OPERATION  1 
//#define MHZ_868   1            // Select either 868 or 915MHz
#define MHZ_915   1

#ifdef SELECT_MHZ_FOR_OPERATION
  #error: "Please define frequency of operation in RF1A.h" 
#endif

#define CONF_REG_SIZE	 47	   /* There are 47 8-bit configuration registers */ 

unsigned char Strobe2(unsigned char strobe);
unsigned char ReadSingleReg2(unsigned char addr);
void WriteSingleReg2(unsigned char addr, unsigned char value);
void ReadBurstReg2(unsigned char addr, unsigned char *buffer, unsigned char count);
void WriteBurstReg2(unsigned char addr, unsigned char *buffer, unsigned char count);
void ResetRadioCore2 (void);
void WritePATable2(void);
void Transmit2(unsigned char *buffer, unsigned char length);
void ReceiveOn2(void);
void ReceiveOff2(void);
void WriteSmartRFReg2(const unsigned char SmartRFSetting[][2], unsigned char size);
