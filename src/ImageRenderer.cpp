#include "ImageRenderer.hpp"
#include "Free_Fonts.h"
#include <main.hpp>

ImageRenderer::ImageRenderer(TFT_eSPI &display) : tft(display) {}

void ImageRenderer::begin() {
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_LOGOBACKGROUND);
    tft.setTextColor(TFT_WHITE, TFT_LOGOBACKGROUND);
    tft.setTextSize(defaultTextSize);
    initPositions();
}

void ImageRenderer::showBootScreen(const char* version, const char* compileDate, const char* compileTime) {
    tft.fillScreen(TFT_LOGOBACKGROUND);
    tft.setTextColor(TFT_WHITE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);

    // Center the boot screen content
    int centerY = tft.height() / 2;

    // Draw title
    tft.setFreeFont(&FreeSansBold12pt7b);
    drawCenteredText("P-Mixer", centerY - 60);

    // Draw version label
    tft.setFreeFont(&FreeSans9pt7b);
    drawCenteredText(String(version), centerY - 20);

    // Draw compile date and time
    tft.setFreeFont(&FreeSans9pt7b);
    String buildInfo = String("Build: ") + compileDate;
    drawCenteredText(buildInfo, centerY + 10);
    drawCenteredText(String(compileTime), centerY + 35);

    // Reset to default font
    tft.setTextSize(defaultTextSize);
}

void ImageRenderer::showLinesOnScreen(const char* line1, const char* line2, const char* line3) {
    tft.fillScreen(TFT_LOGOBACKGROUND);
    tft.setTextColor(TFT_WHITE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);

    // Center the boot screen content
    int centerY = tft.height() / 2;

    // Draw title
    tft.setFreeFont(&FreeSansBold12pt7b);
    drawCenteredText("Info:", centerY - 60);

    // Draw Line1
    tft.setFreeFont(&FreeSans9pt7b);
    drawCenteredText(String(line1), centerY - 20);
    // Draw Line2
    drawCenteredText(String(line2), centerY + 10);
    // Draw Line3
    drawCenteredText(String(line3), centerY + 40);
    // Reset to default font
    tft.setTextSize(defaultTextSize);
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
    tft.drawString("ParallellMixer", labelPos.x, labelPos.y); // Print a message on the display
    // tft.setCursor(versionPos.x, versionPos.y); // Set cursor position for next line
    tft.setFreeFont(FSS9); 
    // tft.setTextSize(smallTextSize); // Set text size for the next line
    tft.drawString(VERSION, versionPos.x, versionPos.y, 2); // Print version on the display
}

void ImageRenderer::drawWiFiField() {
    tft.setFreeFont(FSS9);   
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(wiFiRectPos.x, wiFiRectPos.y, 130, 70, 10, TFT_WHITE); // White border around the screen
    tft.drawString("WiFi  ", wiFiLabelPos.x, wiFiLabelPos.y); // Print a message on the display  
}

void ImageRenderer::drawStatusField() {
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(statusRectPos.x, statusRectPos.y, 175, 120, 10, TFT_WHITE); // White border (increased height for current)
    tft.drawString("Status  ", statusLabelPos.x, statusLabelPos.y); // Print a message on the display
}

void ImageRenderer::drawWiFiAPIP(String WiFiAPIP, String wiFiSSID) {
    static String oldIP = "";
    static String oldSSID = "";
    tft.setFreeFont(FSS9);
    tft.setTextSize(1);
    // Clear previous IP text by drawing in background color
    if (oldIP.length() > 0) {
        tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
        tft.drawString(oldIP, wiFiAPIPPos.x, wiFiAPIPPos.y, 2);
    }
    // Draw new IP text
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.drawString(WiFiAPIP, wiFiAPIPPos.x, wiFiAPIPPos.y, 2);
    oldIP = WiFiAPIP;
    // Clear previous SSID text by drawing in background color
    if (oldSSID.length() > 0) {
        tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
        tft.drawString("SSID: " + oldSSID, wiFiSSIDPos.x, wiFiSSIDPos.y, 2);
    }
    // Draw new SSID text
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.drawString("SSID: " + wiFiSSID, wiFiSSIDPos.x, wiFiSSIDPos.y, 2);
    oldSSID = wiFiSSID;
}

void ImageRenderer::drawWiFiPromt(String WiFiPrompt) {
    static String oldPrompt = "";
    tft.setFreeFont(FSS9);
    tft.setTextSize(1);
    // Clear previous prompt text by drawing in background color
    if (oldPrompt.length() > 0) {
        tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
        tft.drawString(oldPrompt, wiFiPromptPos.x, wiFiPromptPos.y, 2);
    }
    // Draw new prompt text
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.drawString(WiFiPrompt, wiFiPromptPos.x, wiFiPromptPos.y, 2);
    oldPrompt = WiFiPrompt;
}

void ImageRenderer::drawControllerMode(const String& mode) {
    static String oldMode = "";
    tft.setFreeFont(FSS9);
    tft.setTextSize(1);
    // Clear previous text by drawing in background color
    if (oldMode.length() > 0) {
        tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
        tft.drawString("Mode: " + oldMode, statusControllerModePos.x, statusControllerModePos.y, 2);
    }
    // Draw new mode text
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.drawString("Mode: " + mode, statusControllerModePos.x, statusControllerModePos.y, 2);
    oldMode = mode;
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
    tft.drawString(oldPressure, statusPressurePos.x, statusPressurePos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the pressure string
    tft.drawString(pressure, statusPressurePos.x, statusPressurePos.y, 2);
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

void ImageRenderer::drawCurrent(const String& current) {
    static String oldCurrent = "0.0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);
    tft.drawString("Current: " + oldCurrent, statusCurrentPos.x, statusCurrentPos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the current string
    tft.drawString("Current: " + current, statusCurrentPos.x, statusCurrentPos.y, 2);
    oldCurrent = current;
}

void ImageRenderer::initPositions() {
    labelPos.x = 10;
    labelPos.y = 10;

    versionPos.x = 285;
    versionPos.y = 25;

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

    statusRectPos.x = 140;
    statusRectPos.y = 50;

    statusLabelPos.x = statusRectPos.x + 5;
    statusLabelPos.y = statusRectPos.y - 10;

    statusFlowPos.x = statusLabelPos.x;
    statusFlowPos.y = statusLabelPos.y + 20;

    statusPressurePos.x = statusLabelPos.x;
    statusPressurePos.y = statusLabelPos.y + 40;

    statusValveCtrlSignalPos.x = statusLabelPos.x;
    statusValveCtrlSignalPos.y = statusLabelPos.y + 60;

    statusControllerModePos.x = statusLabelPos.x;
    statusControllerModePos.y = statusLabelPos.y + 80;

    statusCurrentPos.x = statusLabelPos.x;
    statusCurrentPos.y = statusLabelPos.y + 100;
}