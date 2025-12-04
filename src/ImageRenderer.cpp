#include "ImageRenderer.hpp"
#include "Free_Fonts.h"
#include "main.hpp"

ImageRenderer::ImageRenderer(TFT_eSPI &display) : tft(display) {}

void ImageRenderer::begin() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_LOGOBACKGROUND); // Clear the display
    tft.setTextColor(TFT_WHITE, TFT_LOGOBACKGROUND  ); // Set text color and background
    tft.setCursor(10, 10); // Set cursor position
    tft.println("P-mixer Ready"); // Print a message on the display
    tft.setCursor(10, 30); // Set cursor position
    tft.println(parallelVerLbl); // Print version on the display
    tft.setTextSize(defaultTextSize);
    initPositions();
}

void ImageRenderer::clear() {
    tft.fillScreen(TFT_LOGOBACKGROUND); // Clear the display with the background color
    tft.setTextColor(TFT_WHITE, TFT_LOGOBACKGROUND); // Reset text color and background
}

void ImageRenderer::drawString(const String& text, int x, int y, int font) {
    tft.drawString(text, x, y, font);
}

void ImageRenderer::drawString(const char* text, int x, int y, int font) {
    drawString(String(text), x, y, font);
}

void ImageRenderer::drawCenteredText(const String& text, int y) {
    int x = (tft.width() - tft.textWidth(text)) / 2;
    tft.drawString(text, x, y);
}

void ImageRenderer::drawSwatch(int x, int y, int width, int height, uint16_t color, const char* label) {
    tft.fillRect(x, y, width, height, color);
    tft.setTextColor(TFT_WHITE, color);
    tft.setCursor(x + 5, y + (height / 2) - 8);
    tft.print(label);
}

void ImageRenderer::drawSwatch(int x, int y, uint16_t color, const char* label, bool rounded) {
    int textWidth  = tft.textWidth(label);
    int textHeight = tft.fontHeight();

    int padding = 10;
    int boxWidth  = textWidth + padding * 2;
    int boxHeight = textHeight + padding;

    if (rounded) {
        tft.fillRoundRect(x, y, boxWidth, boxHeight, 5, color);
    } else {
        tft.fillRect(x, y, boxWidth, boxHeight, color);
    }

    tft.setTextColor(TFT_WHITE, color);
    tft.setCursor(x + padding, y + (boxHeight - textHeight) / 2);
    tft.print(label);
}

void ImageRenderer::drawImage(int x, int y, int w, int h, const uint16_t* img) {
    tft.startWrite();
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            uint16_t color = img[row * w + col];
            tft.drawPixel(x + col, y + row, color);
        }
    }
    tft.endWrite();
}

void ImageRenderer::pushFullImage(int x, int y, int w, int h, const uint16_t* img) {
    tft.pushImage(x, y, w, h, img);
}

void ImageRenderer::drawLabel() {
    tft.setFreeFont(FSSB18);    
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND); // Set text color and background
    tft.setTextSize(1); // Set text size
    // tft.setCursor(labelPos.x, labelPos.y); // Set cursor position
    tft.drawString("P-Mixer", labelPos.x, labelPos.y); // Print a message on the display
    // tft.setCursor(versionPos.x, versionPos.y); // Set cursor position for next line
    tft.setFreeFont(FSS9); 
    // tft.setTextSize(smallTextSize); // Set text size for the next line
    tft.drawString(VERSION, versionPos.x, versionPos.y, 2); // Print version on the display
}

void ImageRenderer::drawWiFiField() {
    tft.setFreeFont(FSS9);   
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(wiFiRectPos.x, wiFiRectPos.y, 150, 70, 10, TFT_WHITE); // White border around the screen
    tft.drawString("WiFi  ", wiFiLabelPos.x, wiFiLabelPos.y); // Print a message on the display  
}
void ImageRenderer::drawStatusField() {
    tft.setFreeFont(FSS9);   
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(statusRectPos.x, statusRectPos.y, 150, 110, 10, TFT_WHITE); // White border around the screendelay(10000);
    tft.drawString("Status  ", statusLabelPos.x, statusLabelPos.y); // Print a message on the display  
}

void ImageRenderer::drawWiFiAPIP(String WiFiAPIP, String wiFiSSID) {
    tft.setFreeFont(FSS9);  
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawString(WiFiAPIP, wiFiAPIPPos.x, wiFiAPIPPos.y, 2); // Print another message on the display
    tft.setTextSize(1);
    tft.drawString("SSID: " + wiFiSSID, wiFiSSIDPos.x, wiFiSSIDPos.y, 2); // Print another message on the display
}

void ImageRenderer::drawWiFiPromt(String WiFiPrompt) {
    tft.setFreeFont(FSS9);  
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawString(WiFiPrompt,  wiFiPromptPos.x, wiFiPromptPos.y , 2); // Print another message on the display, small font
}

void ImageRenderer::drawControllerMode(const String& mode) {
    static String oldMode = "Valve Set";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);  
    tft.drawString("Mode: " + oldMode, statusControllerModePos.x, statusControllerModePos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the mode string
    tft.drawString("Mode: " + mode, statusControllerModePos.x, statusControllerModePos.y, 2);
}

void ImageRenderer::drawFlow(const String& flow) {
    static String oldFlow = "0.0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);  
    tft.drawString("Flow: " + oldFlow, statusFlowPos.x, statusFlowPos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the flow string
    tft.drawString("Flow: " + flow, statusFlowPos.x, statusFlowPos.y, 2);
    oldFlow = flow;
}

void ImageRenderer::drawPressure(const String& pressure) {
    static String oldPressure = "0.0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);  
    tft.drawString("Pressure: " + oldPressure, statusPressurePos.x, statusPressurePos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the pressure string
    tft.drawString("Pressure: " + pressure, statusPressurePos.x, statusPressurePos.y, 2);
    oldPressure = pressure;
}

void ImageRenderer::drawValveCtrlSignal(const String& signal) {
    static String oldSignal = "0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);  
    tft.drawString("Valve: " + oldSignal, statusValveCtrlSignalPos.x, statusValveCtrlSignalPos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the valve control signal string
    tft.drawString("Valve: " + signal, statusValveCtrlSignalPos.x, statusValveCtrlSignalPos.y, 2);
    oldSignal = signal;
}

void ImageRenderer::initPositions() {

    logoPos.x = 0;
    logoPos.y = 0;

    labelPos.x = 70;
    labelPos.y = 10;

    versionPos.x = 290;
    versionPos.y = 25; 
  
    servoIDPos.x = labelPos.x;
    servoIDPos.y = 60;  
  
    timePos.x = labelPos.x;
    timePos.y = 80;
  
    wiFiRectPos.x = 5;
    wiFiRectPos.y = 100; 
  
    wiFiLabelPos.x = wiFiRectPos.x + 5;
    wiFiLabelPos.y = wiFiRectPos.y - 10;
     
    wiFiAPIPPos.x = wiFiLabelPos.x;
    wiFiAPIPPos.y = wiFiLabelPos.y + 20; 

    wiFiSSIDPos.x = wiFiLabelPos.x;
    wiFiSSIDPos.y = wiFiLabelPos.y + 20 + 20;

    wiFiPromptPos.x = wiFiLabelPos.x;
    wiFiPromptPos.y = wiFiLabelPos.y + 20 + 20 + 20; 

    statusRectPos.x = 170;
    statusRectPos.y = 60;

    statusLabelPos.x = statusRectPos.x + 5 ;
    statusLabelPos.y = statusRectPos.y - 10 ;  
  
    statusCOMPos.x = 0;               // will be overridden by function
    statusCOMPos.y = 108; 
  
    statusSDPos.x = 0;                // will be overridden by function
    statusSDPos.y = 138; 

    statusFlowPos.x = statusLabelPos.x;
    statusFlowPos.y = statusLabelPos.y + 20;

    statusPressurePos.x = statusLabelPos.x;
    statusPressurePos.y = statusLabelPos.y + 40;

    statusValveCtrlSignalPos.x = statusLabelPos.x;
    statusValveCtrlSignalPos.y = statusLabelPos.y + 60;

    statusControllerModePos.x = statusLabelPos.x;
    statusControllerModePos.y = statusLabelPos.y + 80;

    phasePos.x = 180;
    phasePos.y = 140;
}