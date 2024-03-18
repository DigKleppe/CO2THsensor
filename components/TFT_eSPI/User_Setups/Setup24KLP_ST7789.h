// ST7789 240 x 240 display with no chip select line
#define USER_SETUP_ID 24

#define TFT_ST7789_2_DRIVER     // Configure all registers

//#define TFT_WIDTH  240
//#define TFT_HEIGHT 240

//#define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue
//#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red

//#define TFT_INVERSION_ON
//#define TFT_INVERSION_OFF

#include "sdkconfig.h"

#define TFT_DC    CONFIG_TFT_DC
#define TFT_RST   CONFIG_TFT_RST
#define TFT_MOSI  CONFIG_TFT_MOSI
#define TFT_SCLK  CONFIG_TFT_SCLK


//// Generic ESP32 setup  in User_Setup.h
//#define TFT_MISO 	 -1 // 19
//#define TFT_MOSI 	23
//#define TFT_SCLK 	18
//#define TFT_CS    	-1  // Not connected
//#define TFT_DC      0  // 0  //	5  // DMM 17
//#define TFT_RST   	33  //26  // Connect reset to ensure display initialises

// For NodeMCU - use pin numbers in the form PIN_Dx where Dx is the NodeMCU pin designation
//#define TFT_CS   -1      // Define as not used
//#define TFT_DC   17		 // Data Command control pin
//#define TFT_RST  26  // TFT reset pin (could connect to NodeMCU RST, see next line)
//#define TFT_RST  -1    // TFT reset pin connect to NodeMCU RST, must also then add 10K pull down to TFT SCK


#define LOAD_GLCD   CONFIG_TFT_LOAD_GLCD// Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2	CONFIG_TFT_LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  CONFIG_TFT_LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  CONFIG_TFT_LOAD_FONT6 // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  CONFIG_TFT_LOAD_FONT7 // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
#define LOAD_FONT8  CONFIG_TFT_LOAD_FONT8 // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF   CONFIG_TFT_LOAD_GFXFF// FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT CONFIG_TFT_SMOOTH_FONT

#define SPI_FREQUENCY  CONFIG_TFT_SPI_FREQUENCY
#define SPI_READ_FREQUENCY  CONFIG_TFT_SPI_READ_FREQ
#define SPI_TOUCH_FREQUENCY  CONFIG_SPI_TOUCH_FREQUENCY

// #define SUPPORT_TRANSACTIONS
