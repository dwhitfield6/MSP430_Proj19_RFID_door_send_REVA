#include <msp430.h>
#include "msprf24.h"
#include "nrf_userconfig.h"
#include "stdint.h"

//Vcc -> Vcc
//GND -> GND

//SCK -> P1.5
//MO -> P1.7
//MI -> P1.6
//(note-- MO and MI should be reversed for the G2xx2 and G2xx1 chips)
//CE -> P2.0
//CSN -> P2.1
//IRQ -> P2.2

/*
MOSI LP -> Pin 3 of the module
 MISO LP -> Pin 4 of the module
 PIn 1.3 LP -> Pin 1 of the module (SS)
 Pin 2.2 LP -> Pin 7 of the module (RST)
 Pin 1.5 LP -> Pin 2 of the module (SCK)
 GND LP -> Pin 6 of the module
 VCC LP -> Pin 8 of the module
 Pin 5 is not connected.

 */

#define chipSelectPin BIT3  //SDA 1.3
#define NRSTPD BIT3  //2.3 RST
#define LED BIT5  //2.0 is SPI CS

#define MAX_LEN 16

//MF522 - Commands
#define PCD_IDLE              0x00
#define PCD_AUTHENT           0x0E
#define PCD_RECEIVE           0x08
#define PCD_TRANSMIT          0x04
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESETPHASE        0x0F
#define PCD_CALCCRC           0x03

//Mifare_One - Commands
#define PICC_REQIDL           0x26
#define PICC_REQALL           0x52
#define PICC_ANTICOLL         0x93
#define PICC_SElECTTAG        0x93
#define PICC_AUTHENT1A        0x60
#define PICC_AUTHENT1B        0x61
#define PICC_READ             0x30
#define PICC_WRITE            0xA0
#define PICC_DECREMENT        0xC0
#define PICC_INCREMENT        0xC1
#define PICC_RESTORE          0xC2
#define PICC_TRANSFER         0xB0
#define PICC_HALT             0x50

//MF522 - Status
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

//MF522 - Registers
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F

//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F

//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F

//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F




void WriteReg(unsigned char addr, unsigned char val);
unsigned char ReadReg(unsigned char addr);
void SetBitMask(unsigned char reg, unsigned char mask);
void ClearBitMask(unsigned char reg, unsigned char mask);
void AntennaOn(void);
void AntennaOff(void);
void Reset(void);
void Init(void);
unsigned char Request(unsigned char reqMode, unsigned char *TagType);
unsigned char ToCard(unsigned char command, unsigned char *sendData, unsigned char sendLen, unsigned char *backData, unsigned int *backLen);
unsigned char Anticoll(unsigned char *serNum);
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);
unsigned char SelectTag(unsigned char *serNum);
unsigned char Auth(unsigned char authMode, unsigned char BlockAddr, unsigned char *Sectorkey, unsigned char *serNum);
unsigned char ReadBlock(unsigned char blockAddr, unsigned char *recvData);
unsigned char WriteBlock(unsigned char blockAddr, unsigned char *writeData);
void Halt(void);
void spiA_init();
uint8_t spiA_transfer(uint8_t);  // SPI xfer 1 byte
uint16_t spiA_transfer16(uint16_t);  // SPI xfer 2 bytes
uint16_t spiA_transfer9(uint16_t);   // SPI xfer 9 bits (courtesy for driving LCD screens)

unsigned char serNum[5];
unsigned char sectorKey[6] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned char readData[16];
unsigned char status;
unsigned char cardNum[MAX_LEN];
unsigned char RealCardNum[MAX_LEN];
unsigned char i;

unsigned char CardNumber =0;
volatile unsigned int user;
int ledcount =0;


int main()
{
  uint8_t addr[5];


  WDTCTL = WDTHOLD | WDTPW;
  DCOCTL = CALDCO_16MHZ;
  BCSCTL1 = CALBC1_16MHZ;
  BCSCTL2 = DIVS_1;  // SMCLK = DCOCLK/2
  // SPI (USCI) uses SMCLK, prefer SMCLK < 10MHz (SPI speed limit for nRF24 = 10MHz)

  P2DIR |= NRSTPD;
  P2DIR |= LED;
  P2OUT |= LED;
  spiA_init();
  P1OUT &= ~chipSelectPin;
  Init();

  user = 0xFE;

  /* Initial values for nRF24L01+ library config variables */
  //rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
  rf_addr_width      = 5;
  rf_speed_power     = RF24_SPEED_2MBPS | RF24_POWER_0DBM;
  rf_channel         = 124;

  msprf24_init();  // All RX pipes closed by default
  msprf24_set_pipe_packetsize(0, 1);
  msprf24_open_pipe(0, 0);  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs

  // Transmit to 'rad01' (0x72 0x61 0x64 0x30 0x31)
  msprf24_standby();
  user = msprf24_current_state();
  addr[0] = 0xDE;
  addr[1] = 0xAD;
  addr[2] = 0xBE;
  addr[3] = 0xEF;
  addr[4] = 0x00;
  w_tx_addr(addr);
  w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
  // needs to listen to the TX addr on pipe#0 to receive them.

  while(1){

    status = Request(PICC_REQIDL, cardNum);
    if (status == MI_OK)
    {

    }

    status = Anticoll(cardNum);
    if (status == MI_OK)
    {
      Init();
      for(i = 16; i>0; i--)
      {
        RealCardNum[i-1] = cardNum[i-1];
      }
      __delay_cycles(10);
      CardNumber =0;
      if(RealCardNum[0]== 145 && RealCardNum[1]== 47 && RealCardNum[2]== 195 && RealCardNum[3]== 180 && RealCardNum[4]== 201)
      {
        CardNumber = 1; //close
      }
      else if(RealCardNum[0]== 52 && RealCardNum[1]== 224 && RealCardNum[2]== 88 && RealCardNum[3]== 146 && RealCardNum[4]== 30)
      {
        CardNumber = 2; //open
      }
      for(i=5;i>0;i--)
      {
        w_tx_payload(1, &CardNumber);
        msprf24_activate_tx();
        __delay_cycles(20000);
      }
      P2OUT |= LED;
      ledcount =0;


    }

    __delay_cycles(200);
    Halt();
    for(i = 16; i>0; i--)
    {
      RealCardNum[i-1] =0;
    }

    if(ledcount == 90)
    {
      P2OUT &= ~LED;
    }
    ledcount++;
    if(ledcount >100)
    {
      ledcount =100;
    }

  }
}

void WriteReg(unsigned char addr, unsigned char val)
{
  P1OUT &= ~(chipSelectPin);

  spiA_transfer((addr<<1)&0x7E);
  spiA_transfer(val);

  P1OUT |= (chipSelectPin);
}

/*
 * Function：ReadReg
 * Description：read a byte data into one register of MR RC522
 * Input parameter：addr--register address
 * Return：return the read value
 */
unsigned char ReadReg(unsigned char addr)
{
  unsigned char val;
  P1OUT &= ~(chipSelectPin);

  spiA_transfer(((addr<<1)&0x7E) | 0x80);
  val =spiA_transfer(0x00);

  P1OUT |= (chipSelectPin);

  return val;
}

/*
 * Function：SetBitMask
 * Description：set RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void SetBitMask(unsigned char reg, unsigned char mask)
{
  unsigned char tmp;
  tmp = ReadReg(reg);
  WriteReg(reg, tmp | mask);  // set bit mask
}

/*
 * Function：ClearBitMask
 * Description：clear RC522 register bit
 * Input parameter：reg--register address;mask--value
 * Return：null
 */
void ClearBitMask(unsigned char reg, unsigned char mask)
{
  unsigned char tmp;
  tmp = ReadReg(reg);
  WriteReg(reg, tmp & (~mask));  // clear bit mask
}


/*
 * Function：AntennaOn
 * Description：Turn on antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOn(void)
{
  unsigned char temp;

  temp = ReadReg(TxControlReg);
  if (!(temp & 0x03))
  {
    SetBitMask(TxControlReg, 0x03);
  }
}


/*
 * Function：AntennaOff
 * Description：Turn off antenna, every time turn on or shut down antenna need at least 1ms delay
 * Input parameter：null
 * Return：null
 */
void AntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function：Reset
 * Description： reset RC522
 * Input parameter：null
 * Return：null
 */
void Reset(void)
{
  WriteReg(CommandReg, PCD_RESETPHASE);
}


/*
 * Function：Init
 * Description：initilize RC522
 * Input parameter：null
 * Return：null
 */
void Init(void)
{
  P2OUT |= NRSTPD;

  Reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  WriteReg(TModeReg, 0x8D);       //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  WriteReg(TPrescalerReg, 0x3E);  //TModeReg[3..0] + TPrescalerReg
  WriteReg(TReloadRegL, 30);
  WriteReg(TReloadRegH, 0);

  WriteReg(TxAutoReg, 0x40);      //100%ASK
  WriteReg(ModeReg, 0x3D);

  //ClearBitMask(Status2Reg, 0x08);       //MFCrypto1On=0
  //WriteReg(RxSelReg, 0x86);     //RxWait = RxSelReg[5..0]
  //WriteReg(RFCfgReg, 0x7F);         //RxGain = 48dB

  AntennaOn();
}


/*
 * Function：Request
 * Description：Searching card, read card type
 * Input parameter：reqMode--search methods，
 *           TagType--return card types
 *              0x4400 = Mifare_UltraLight
 *              0x0400 = Mifare_One(S50)
 *              0x0200 = Mifare_One(S70)
 *              0x0800 = Mifare_Pro(X)
 *              0x4403 = Mifare_DESFire
 * return：return MI_OK if successed
 */
unsigned char Request(unsigned char reqMode, unsigned char *TagType)
{
  unsigned char status;
  unsigned int backBits;

  WriteReg(BitFramingReg, 0x07);      //TxLastBists = BitFramingReg[2..0] ???

  TagType[0] = reqMode;
  status = ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
  {
    status = MI_ERR;
  }

  return status;
}

/*
 * Function：ToCard
 * Description：communicate between RC522 and ISO14443
 * Input parameter：command--MF522 command bits
 *           sendData--send data to card via rc522
 *           sendLen--send data length
 *           backData--the return data from card
 *           backLen--the length of return data
 * return：return MI_OK if successed
 */
unsigned char ToCard(unsigned char command, unsigned char *sendData, unsigned char sendLen, unsigned char *backData, unsigned int *backLen)
{
  unsigned char status = MI_ERR;
  unsigned char irqEn = 0x00;
  unsigned char waitIRq = 0x00;
  unsigned char lastBits;
  unsigned char n;
  unsigned int i;

  switch (command)
  {
  case PCD_AUTHENT:
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
  case PCD_TRANSCEIVE:
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
  default:
    break;
  }

  WriteReg(CommIEnReg, irqEn|0x80);
  ClearBitMask(CommIrqReg, 0x80);
  SetBitMask(FIFOLevelReg, 0x80);

  WriteReg(CommandReg, PCD_IDLE);

  for (i=0; i<sendLen; i++)
  {
    WriteReg(FIFODataReg, sendData[i]);
  }

  WriteReg(CommandReg, command);
  if (command == PCD_TRANSCEIVE)
  {
    SetBitMask(BitFramingReg, 0x80);        //StartSend=1,transmission of data starts
  }

  i = 2000;
  do
  {
    n = ReadReg(CommIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  ClearBitMask(BitFramingReg, 0x80);          //StartSend=0

    if (i != 0)
  {
    if(!(ReadReg(ErrorReg) & 0x1B)) //BufferOvfl Collerr CRCErr ProtecolErr
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
      {
        status = MI_NOTAGERR;           //??
      }

      if (command == PCD_TRANSCEIVE)
      {
        n = ReadReg(FIFOLevelReg);
        lastBits = ReadReg(ControlReg) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1)*8 + lastBits;
        }
        else
        {
          *backLen = n*8;
        }
        if (n == 0)
        {
          n = 1;
        }
        if (n > MAX_LEN)
        {
          n = MAX_LEN;
        }

        for (i=0; i<n; i++)
        {
          backData[i] = ReadReg(FIFODataReg);
        }
      }
    }
    else
    {
      status = MI_ERR;
    }
  }

  //SetBitMask(ControlReg,0x80);           //timer stops
  //WriteReg(CommandReg, PCD_IDLE);

  return status;
}


/*
 * Function：MFRC522_Anticoll
 * Description：Prevent conflict, read the card serial number
 * Input parameter：serNum--return the 4 bytes card serial number, the 5th byte is recheck byte
 * return：return MI_OK if successed
 */
unsigned char Anticoll(unsigned char *serNum)
{
  unsigned char status;
  unsigned char i;
  unsigned char serNumCheck=0;
  unsigned int unLen;


  //ClearBitMask(Status2Reg, 0x08);       //TempSensclear
  //ClearBitMask(CollReg,0x80);           //ValuesAfterColl
  WriteReg(BitFramingReg, 0x00);      //TxLastBists = BitFramingReg[2..0]
  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == MI_OK)
  {
    for (i=0; i<4; i++)
    {
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i])
    {
      status = MI_ERR;
    }
  }

  //SetBitMask(CollReg, 0x80);        //ValuesAfterColl=1

  return status;
}


/*
 * Function：CalulateCRC
 * Description：Use MF522 to calculate CRC
 * Input parameter：pIndata--the CRC data need to be read，len--data length，pOutData-- the caculated result of CRC
 * return：Null
 */
void CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
  unsigned char i, n;

  ClearBitMask(DivIrqReg, 0x04);          //CRCIrq = 0
  SetBitMask(FIFOLevelReg, 0x80);
  //WriteReg(CommandReg, PCD_IDLE);

  for (i=0; i<len; i++)
  {
    WriteReg(FIFODataReg, *(pIndata+i));
  }
  WriteReg(CommandReg, PCD_CALCCRC);

  i = 0xFF;
  do
  {
    n = ReadReg(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));            //CRCIrq = 1

  pOutData[0] = ReadReg(CRCResultRegL);
  pOutData[1] = ReadReg(CRCResultRegM);
}


/*
 * Function：SelectTag
 * Description：Select card, read card storage volume
 * Input parameter：serNum--Send card serial number
 * return：return the card storage volume
 */
unsigned char SelectTag(unsigned char *serNum)
{
  unsigned char i;
  unsigned char status;
  unsigned char size;
  unsigned int recvBits;
  unsigned char buffer[9];

  //ClearBitMask(Status2Reg, 0x08);           //MFCrypto1On=0

  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  for (i=0; i<5; i++)
  {
    buffer[i+2] = *(serNum+i);
  }
  CalulateCRC(buffer, 7, &buffer[7]);     //??
  status = ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

  if ((status == MI_OK) && (recvBits == 0x18))
  {
    size = buffer[0];
  }
  else
  {
    size = 0;
  }

  return size;
}


/*
 * Function：Auth
 * Description：verify card password
 * Input parameters：authMode--password verify mode
 0x60 = verify A password key
 0x61 = verify B password key
 BlockAddr--Block address
 Sectorkey--Block password
 serNum--Card serial number ，4 bytes
 * return：return MI_OK if successed
 */
unsigned char Auth(unsigned char authMode, unsigned char BlockAddr, unsigned char *Sectorkey, unsigned char *serNum)
{
  unsigned char status;
  unsigned int recvBits;
  unsigned char i;
  unsigned char buff[12];

  buff[0] = authMode;
  buff[1] = BlockAddr;
  for (i=0; i<6; i++)
  {
    buff[i+2] = *(Sectorkey+i);
  }
  for (i=0; i<4; i++)
  {
    buff[i+8] = *(serNum+i);
  }
  status = ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

  if ((status != MI_OK) || (!(ReadReg(Status2Reg) & 0x08)))
  {
    status = MI_ERR;
  }

  return status;
}


/*
 * Function：ReadBlock
 * Description：Read data
 * Input parameters：blockAddr--block address;recvData--the block data which are read
 * return：return MI_OK if successed
 */
unsigned char ReadBlock(unsigned char blockAddr, unsigned char *recvData)
{
  unsigned char status;
  unsigned int unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  CalulateCRC(recvData,2, &recvData[2]);
  status = ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

  if ((status != MI_OK) || (unLen != 0x90))
  {
    status = MI_ERR;
  }

  return status;
}


/*
 * Function：WriteBlock
 * Description：write block data
 * Input parameters：blockAddr--block address;writeData--Write 16 bytes data into block
 * return：return MI_OK if successed
 */
unsigned char WriteBlock(unsigned char blockAddr, unsigned char *writeData)
{
  unsigned char status;
  unsigned int recvBits;
  unsigned char i;
  unsigned char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  CalulateCRC(buff, 2, &buff[2]);
  status = ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

  if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
  {
    status = MI_ERR;
  }

  if (status == MI_OK)
  {
    for (i=0; i<16; i++)
    {
      buff[i] = *(writeData+i);
    }
    CalulateCRC(buff, 16, &buff[16]);
    status = ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
      status = MI_ERR;
    }
  }

  return status;
}


/*
 * Function：Halt
 * Description：Command the cards into sleep mode
 * Input parameters：null
 * return：null
 */
void Halt(void)
{
  unsigned char status;
  unsigned int unLen;
  unsigned char buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  CalulateCRC(buff, 2, &buff[2]);

  status = ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}

void spiA_init()
{
  /* Configure ports on MSP430 device for USCI_A */
  P1SEL |= BIT1 | BIT2 | BIT4;
  P1SEL2 |= BIT1 | BIT2 | BIT4;
  P1DIR |= chipSelectPin;

  /* USCI-A specific SPI setup */
  UCA0CTL1 |= UCSWRST;
  UCA0MCTL = 0x00;  // Clearing modulation control per TI user's guide recommendation
  UCA0CTL0 = UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // SPI mode 0, master
  UCA0BR0 = 0x01;  // SPI clocked at same speed as SMCLK
  UCA0BR1 = 0x00;
  UCA0CTL1 = UCSSEL_2;  // Clock = SMCLK, clear UCSWRST and enables USCI_A module.
}

uint8_t spiA_transfer(uint8_t inb)
{
  UCA0TXBUF = inb;
  while ( !(IFG2 & UCA0RXIFG) )  // Wait for RXIFG indicating remote byte received via SOMI
    ;
  return UCA0RXBUF;
}

uint16_t spiA_transfer16(uint16_t inw)
{
  uint16_t retw;
  uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

  UCA0TXBUF = inw8[1];
  while ( !(IFG2 & UCA0RXIFG) )
    ;
  retw8[1] = UCB0RXBUF;
  UCB0TXBUF = inw8[0];
  while ( !(IFG2 & UCA0RXIFG) )
    ;
  retw8[0] = UCA0RXBUF;
  return retw;
}


