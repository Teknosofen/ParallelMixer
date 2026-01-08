#ifndef IMAGE_RENDERER_HPP
#define IMAGE_RENDERER_HPP

#include <TFT_eSPI.h>

class ImageRenderer {
public:
    ImageRenderer(TFT_eSPI &display);
    
    // Initialization
    void begin();
    void clear();
    void initPositions();
    void showBootScreen(const char* version, const char* compileDate, const char* compileTime);
    void showLinesOnScreen(const char* line1, const char* line2, const char* line3);
    
    // Basic drawing
    void drawLabel();
    void drawString(const String& text, int x, int y, int font = 2);
    void drawString(const char* text, int x, int y, int font = 2);
    void drawCenteredText(const String& text, int y);
    
    // Swatch drawing (colored label boxes)
    void drawSwatch(int x, int y, int width, int height, uint16_t color, const char* label);
    void drawSwatch(int x, int y, uint16_t color, const char* label, bool rounded = false);
    
    void drawStatusField();
    void drawWiFiField();
    void drawWiFiAPIP(String drawWiFiAPIP, String wiFiSSID);
    void drawWiFiPromt(String WiFiPrompt);
    void drawControllerMode(const String& mode);
    void drawFlow(const String& flow);
    void drawPressure(const String& pressure);
    void drawValveCtrlSignal(const String& signal);

    // Image drawing
    void drawImage(int x, int y, int w, int h, const uint16_t* img);
    void pushFullImage(int x, int y, int w, int h, const uint16_t* img);
    
    // Access to underlying display
    TFT_eSPI& getDisplay() { return tft; }

    struct DisplayPos {
        int x = 0;
        int y = 0;
    };

    DisplayPos logoPos;
    DisplayPos labelPos;
    DisplayPos versionPos;
    DisplayPos servoIDPos;
    DisplayPos timePos;
    DisplayPos wiFiRectPos;
    DisplayPos wiFiLabelPos;
    DisplayPos wiFiAPIPPos;
    DisplayPos wiFiSSIDPos;
    DisplayPos wiFiPromptPos;
    DisplayPos statusRectPos;
    DisplayPos statusLabelPos;
    DisplayPos statusCOMPos;
    DisplayPos statusSDPos;
    DisplayPos phasePos;
    DisplayPos statusControllerModePos;
    DisplayPos statusFlowPos;
    DisplayPos statusPressurePos;
    DisplayPos statusValveCtrlSignalPos;


private:
    TFT_eSPI &tft;
    static constexpr int defaultTextSize = 1;
};

#endif // IMAGE_RENDERER_HPP