#pragma once


// Colours for Parallelmixer
#define TFT_LOGOBACKGROUND       0x85BA // note, if you pick another color from the image, note that you will have to flip the bytes here
#define TFT_LOGOBLUE             0x5497
#define TFT_DARKERBLUE           0x3A97 // A muted steel blue
#define TFT_DEEPBLUE             0x1A6F // A darker steel blue
#define TFT_SLATEBLUE            0x2B4F // A lighter steel blue
#define TFT_MIDNIGHTBLUE         0x1028 // A light steel blue
#define TFT_REDDISH_TINT         0xA4B2   
#define TFT_GREENISH_TINT        0x5DAD

#define parallelVerLbl "P-mixer 2026-01-07 0.9.0"
#define VERSION "0.9.0"

// Pin definitions
#define Flow_Input_Analogue_pin 1
#define Flow_Output_Analogue_pin 3
#define Valve_ctrl_Analogue_pin 6

#define INTERACTION_BUTTON_PIN 14 // GPIO14, key 2
#define LONG_KEY_PRESS_DURATION 1000 // long press, in [ms]

#define hostCom Serial // Use Serial for host communication

#define SET_UI_UPDATE_TIME 250000 // 100ms in [us] for UI update time
#define PMIXERSSID "ParallelMixer" // WiFi SSID for the Parallel Mixer
#define PMIXERPWD "" // WiFi Password for the Parallel Mixer


// in the TFT_eSPI TFT_eSPI_User_setup_Select.H ensure that the 
// line 133 #include <User_Setups/Setup206_LilyGo_T_Display_S3.h>     // For the LilyGo T-Display S3 based ESP32S3 with ST7789 170 x 320 TFT

// In the TFT_eSPI_User_setup.H 
// line 55 #define ST7789_DRIVER      // Full configuration option, define additional parameters below for this display
// line 87 #define TFT_WIDTH  170 // ST7789 170 x 320
// line 92 #define TFT_HEIGHT 320 // ST7789 240 x 320
