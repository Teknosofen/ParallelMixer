#include "ImageRenderer.hpp"
#include "Free_Fonts.h"
#include <main.hpp>

ImageRenderer::ImageRenderer(TFT_eSPI &display) : tft(display) {}

void ImageRenderer::begin() {
    tft.init();
    tft.setRotation(0);  // Portrait mode (170x320)
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

void ImageRenderer::showVentilatorSettings(const char* line1, const char* line2, const char* line3, const char* line4) {
    tft.fillScreen(TFT_LOGOBACKGROUND);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);  // Same color as real-time data
    tft.setTextSize(1);

    // Center the content
    int centerY = tft.height() / 2;

    // Draw title - use same font as status labels
    tft.setFreeFont(FSS9);
    drawCenteredText("Ventilator Settings", centerY - 70);

    // Draw settings lines - smaller spacing for compact display
    drawCenteredText(String(line1), centerY - 40);  // Vent: ON/OFF State
    drawCenteredText(String(line2), centerY - 15);  // RR, VT, IE
    drawCenteredText(String(line3), centerY + 10);  // PI, PE, MF
    drawCenteredText(String(line4), centerY + 35);  // Breath count, peak pressure, etc.

    // Footer hint
    tft.setTextColor(TFT_SLATEBLUE, TFT_LOGOBACKGROUND);
    drawCenteredText("Short press: back", centerY + 70);

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
    tft.setFreeFont(FSSB12);  // Smaller font for portrait header
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawString("P-Mixer", labelPos.x, labelPos.y);
    tft.setFreeFont(FSS9);
    tft.drawString(VERSION, versionPos.x, versionPos.y, 2);
}

void ImageRenderer::drawWiFiField() {
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(wiFiRectPos.x, wiFiRectPos.y, 160, 70, 10, TFT_WHITE);
    tft.drawString("WiFi", wiFiLabelPos.x, wiFiLabelPos.y);
}

void ImageRenderer::drawStatusField() {
    tft.setFreeFont(FSS9);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    tft.drawRoundRect(statusRectPos.x, statusRectPos.y, 160, 160, 10, TFT_WHITE);  // Increased height for Flow2
    tft.drawString("Status", statusLabelPos.x, statusLabelPos.y);
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

void ImageRenderer::drawFlow2(const String& flow) {
    static String oldFlow2 = "0.0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);
    tft.drawString("Flow2: " + oldFlow2, statusFlow2Pos.x, statusFlow2Pos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the Bus 1 flow string
    tft.drawString("Flow2: " + flow, statusFlow2Pos.x, statusFlow2Pos.y, 2);
    oldFlow2 = flow;
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

void ImageRenderer::drawO2(const String& o2) {
    static String oldO2 = "0.0";
    // Clear previous text by drawing background rectangle
    tft.setTextColor(TFT_LOGOBACKGROUND, TFT_LOGOBACKGROUND);
    tft.setFreeFont(FSS9);
    tft.drawString("pO2: " + oldO2, statusO2Pos.x, statusO2Pos.y, 2);
    tft.setTextColor(TFT_DEEPBLUE, TFT_LOGOBACKGROUND);
    tft.setTextSize(1);
    // Draw the O2 string
    tft.drawString("pO2: " + o2, statusO2Pos.x, statusO2Pos.y, 2);
    oldO2 = o2;
}

void ImageRenderer::initPositions() {
    // Portrait layout (170x320)
    // Header at top
    labelPos.x = 10;
    labelPos.y = 5;

    versionPos.x = 110;
    versionPos.y = 12;

    // Status section in middle
    statusRectPos.x = 5;
    statusRectPos.y = 40;

    statusLabelPos.x = statusRectPos.x + 5;
    statusLabelPos.y = statusRectPos.y - 10;

    statusFlowPos.x = statusLabelPos.x;
    statusFlowPos.y = statusLabelPos.y + 20;

    statusFlow2Pos.x = statusLabelPos.x;
    statusFlow2Pos.y = statusLabelPos.y + 40;  // Bus 1 flow below Bus 0 flow

    statusPressurePos.x = statusLabelPos.x;
    statusPressurePos.y = statusLabelPos.y + 60;

    statusValveCtrlSignalPos.x = statusLabelPos.x;
    statusValveCtrlSignalPos.y = statusLabelPos.y + 80;

    statusControllerModePos.x = statusLabelPos.x;
    statusControllerModePos.y = statusLabelPos.y + 100;

    statusCurrentPos.x = statusLabelPos.x;
    statusCurrentPos.y = statusLabelPos.y + 120;

    statusO2Pos.x = statusLabelPos.x;
    statusO2Pos.y = statusLabelPos.y + 140;

    // WiFi section at bottom
    wiFiRectPos.x = 5;
    wiFiRectPos.y = 240;

    wiFiLabelPos.x = wiFiRectPos.x + 5;
    wiFiLabelPos.y = wiFiRectPos.y - 10;

    wiFiAPIPPos.x = wiFiLabelPos.x;
    wiFiAPIPPos.y = wiFiLabelPos.y + 20;

    wiFiSSIDPos.x = wiFiLabelPos.x;
    wiFiSSIDPos.y = wiFiLabelPos.y + 40;

    wiFiPromptPos.x = wiFiLabelPos.x;
    wiFiPromptPos.y = wiFiLabelPos.y + 60;
}