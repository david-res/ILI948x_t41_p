#ifndef _ILI948x_t41_p_H_
#define _ILI948x_t41_p_H_


#include "Arduino.h"
#include "FlexIO_t4.h"

#define BUS_WIDTH 8   /*Available options are 8 or 16 */


#define SHIFTNUM 8 // number of shifters used (up to 8)
#define BYTES_PER_BEAT (sizeof(uint8_t))
#define BEATS_PER_SHIFTER (sizeof(uint32_t)/BYTES_PER_BEAT)
#define BYTES_PER_BURST (sizeof(uint32_t)*SHIFTNUM)
#define SHIFTER_IRQ (SHIFTNUM-1)
#define TIMER_IRQ 0
#define FLEXIO_ISR_PRIORITY 64 // interrupt is timing sensitive, so use relatively high priority (supersedes USB)


#define _TFTWIDTH   320      // ILI9488 TFT width in default rotation
#define _TFTHEIGHT  480      // ILI9488 TFT height in default rotation

#define ILI9488_NOP         0x00  // No-op
#define ILI9488_SWRESET     0x01  // Software reset
#define ILI9488_RDDID       0x04  // Read display ID
#define ILI9488_RDDST       0x09  // Read display status

#define ILI9488_SLPIN       0x10  // Enter Sleep Mode
#define ILI9488_SLPOUT      0x11  // Sleep Out
#define ILI9488_PTLON       0x12  // Partial Mode ON
#define ILI9488_NORON       0x13  // Normal Display Mode ON

#define ILI9488_RDMODE      0x0A  // Read Display Power Mode
#define ILI9488_RDMADCTL    0x0B  // Read Display MADCTL
#define ILI9488_RDCOLMOD    0x0C  // Read Display Pixel Format
#define ILI9488_RDIMGFMT    0x0D  // Read Display Image Mode
#define ILI9488_RDDSM       0x0E  // Read Display Signal Mode
#define ILI9488_RDSELFDIAG  0x0F  // Read Display Self-Diagnostic Result

#define ILI9488_INVOFF      0x20  // Display Inversion OFF
#define ILI9488_INVON       0x21  // Display Inversion ON
#define ILI9488_GAMMASET    0x26  // Gamma Set
#define ILI9488_DISPOFF     0x28  // Display OFF
#define ILI9488_DISPON      0x29  // Display ON

#define ILI9488_CASET       0x2A  // Column Address Set 
#define ILI9488_PASET       0x2B  // Page Address Set 
#define ILI9488_RAMWR       0x2C  // Memory Write 
#define ILI9488_RAMRD       0x2E  // Memory Read

#define ILI9488_PTLAR       0x30  // Partial Area
#define ILI9488_TEOFF       0x34  // Tearing effect line off
#define ILI9488_TEON        0x35  // Tearing effect line on
#define ILI9488_MADCTL      0x36  // Memory Access Control
#define ILI9488_VSCRSADD    0x37  // Vertical Scrolling Start Address
#define ILI9488_COLMOD      0x3A  // Interface pixel format

#define ILI9488_TESLWR      0x44  // Write tear scan line

#define ILI9488_FRMCTR1     0xB1  // Frame Rate Control (Normal Mode / Full Colors)
#define ILI9488_FRMCTR2     0xB2  // Frame Rate Control (Idle Mode / 8 Colors)
#define ILI9488_FRMCTR3     0xB3  // Frame Rate Control (Partial Mode / Full Colors)
#define ILI9488_INVCTR      0xB4  // Display Inversion Control
#define ILI9488_DFUNCTR     0xB6  // Display Function Control
#define ILI9488_ETMOD       0xB7  // Entry Mode Set

#define ILI9488_PWCTR1      0xC0  // Power Control 1
#define ILI9488_PWCTR2      0xC1  // Power Control 2
#define ILI9488_PWCTR3      0xC2  // Power Control 3 (For Normal Mode)
#define ILI9488_PWCTR4      0xC3  // Power Control 4 (For Idle Mode)
#define ILI9488_PWCTR5      0xC4  // Power Control 5 (For Partial Mode)
#define ILI9488_VMCTR1      0xC5  // VCOM Control
#define ILI9488_CABCCTRL1   0xC6  // CABC Control 1
#define ILI9488_CABCCTRL2   0xC8  // CABC Control 2

#define ILI9488_PGAMCTRL    0xE0  // Positive Gamma Control
#define ILI9488_NGAMCTRL    0xE1  // Negative Gamma Control
#define ILI9488_SETIMAGE    0xE9  // Set Image Function

#define ILI9488_RDID1       0xDA  // Read ID1 value
#define ILI9488_RDID2       0xDB  // Read ID2 value
#define ILI9488_RDID3       0xDC  // Read ID3 value

#define MADCTL_MY  0x80  // Bottom to top
#define MADCTL_MX  0x40  // Right to left
#define MADCTL_MV  0x20  // Row/Column exchange
#define MADCTL_ML  0x10  // LCD refresh Bottom to top
#define MADCTL_RGB 0x00  // Red-Green-Blue pixel order
#define MADCTL_BGR 0x08  // Blue-Green-Red pixel order
#define MADCTL_MH  0x04  // LCD refresh right to left
#define MADCTL_GS  0x01
#define MADCTL_SS  0x02

//MADCTL 0,1,2,3 for setting rotation and 4 for screenshot
#define MADCTL_ARRAY { MADCTL_MX | MADCTL_BGR, MADCTL_MV | MADCTL_BGR, MADCTL_MY | MADCTL_BGR, MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR } // ILI9488/9486

#ifdef __cplusplus

PROGMEM const uint8_t ILI9488_DISP[] = 
{
  ILI9488_PWCTR1,     2, 0x19, 0x1A,              // Power Control 1
  ILI9488_PWCTR2,     2, 0x45, 0X00,              // Power Control 2
  ILI9488_PWCTR3,     1, 0x33,                    // Power Control 3 (For Normal Mode)
	ILI9488_VMCTR1,     3, 0x00, 0x12, 0x80,        // VCOM control
	ILI9488_INVCTR,     1, 0x02,                    // Display Inversion Control
	ILI9488_DFUNCTR,    3, 0x00, 0x02, 0x3B,	      // Display Function Control  RGB/MCU Interface Control
	ILI9488_ETMOD,      1, 0x07,                    // Entry Mode Set
  // Gamma Setting	   
  ILI9488_PGAMCTRL,  15, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F,
	ILI9488_NGAMCTRL,  15, 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F,
  // Other commands  
 	ILI9488_MADCTL,     1, 0x48,                    // Memory Access Control : 0x48 is equivalent to _rotation = 0
  ILI9488_COLMOD,     1, 0x55,                    // Set bit depth to 16-bit (RGB565)
	0
};


class ILI948x_t41_p {
  public:
    ILI948x_t41_p(const uint8_t * init_commands, int8_t dc, int8_t cs = -1, int8_t rst = -1);
    void begin(uint8_t baud_div = 20);
    uint8_t getBusSpd();


    uint8_t setBitDepth(uint8_t bitDepth);
    uint8_t getBitDepth();

    void setFrameRate(uint8_t frRate);
    uint8_t getFrameRate();

    void setTearingEffect(bool tearingOn);
    bool getTearingEffect();

    void setTearingScanLine(uint16_t scanLine);
    uint16_t getTearingScanLine();

    void setRotation(uint8_t r);
    void invertDisplay(bool invert);
    void displayInfo();
    void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

    void pushPixels16bit(const uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    void pushPixels16bitAsync(const uint16_t * pcolors, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
    uint8_t readCommand(const uint8_t cmd);
    

    typedef void(*CBF)();
    CBF _callback;
    void onCompleteCB(CBF callback);


  
    
  private:

  FlexIOHandler *pFlex;
  IMXRT_FLEXIO_t *p;
  const FlexIOHandler::FLEXIO_Hardware_t *hw;
   
    uint8_t _baud_div = 20; 

    uint8_t _bitDepth = 16;
    uint8_t _rotation = 0;
    const uint8_t MADCTL[5] = MADCTL_ARRAY;

    uint8_t _frameRate = 60;

    bool _bTearingOn = false;
    uint16_t _tearingScanLine = 0;

    int16_t _width, _height;
    int8_t  _dc, _cs, _rst;
    const uint8_t * _init_commands;

    uint8_t _dummy;
    uint8_t _curMADCTL;

    uint16_t _lastx1, _lastx2, _lasty1, _lasty2;

    volatile bool WR_IRQTransferDone = true;
    uint32_t MulBeatCountRemain;
    uint16_t *MulBeatDataRemain;
    uint32_t TotalSize; 

    /* variables used by ISR */
    volatile uint32_t bytes_remaining;
    volatile unsigned int bursts_to_complete;
    volatile uint32_t *readPtr;
    uint32_t finalBurstBuffer[SHIFTNUM];

    void displayInit();
    void CSLow();
    void CSHigh();
    void DCLow();
    void DCHigh();
    void gpioWrite();
    void gpioRead();
    
    void FlexIO_Init();
    void FlexIO_Config_SnglBeat();
    void FlexIO_Config_MultiBeat();
    void FlexIO_Config_SnglBeat_Read();

    void SglBeatWR_nPrm_8(uint32_t const cmd, uint8_t const *value , uint32_t const length);
    void SglBeatWR_nPrm_16(uint32_t const cmd, const uint16_t *value, uint32_t const length);
    void MulBeatWR_nPrm_IRQ(uint32_t const cmd,  const void *value, uint32_t const length);
    
    void microSecondDelay();

    static void ISR();
    void flexIRQ_Callback();

    bool isCB = false;
    void _onCompleteCB();
    
    static ILI948x_t41_p *IRQcallback;
    
};
#endif //__cplusplus
#endif //_IILI948x_t41_p_H_
