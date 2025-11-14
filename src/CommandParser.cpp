#include "CommandParser.h"

void CommandParser::begin() {
    buffer.reserve(32);
}

bool CommandParser::feed(char c) {
    if (c == '\n') return true;
    buffer += c;
    return false;
}

void CommandParser::parse(const String &cmd) {
    if (cmd.length() == 0) return;

    char C = cmd[0];
    String param = cmd.substring(1);

    switch (C) {
        case 'Q': case 'q':
            quiet = param.toInt();
            Serial.println("OK");
            break;

        case 'F': case 'f':
            digitalFlowRef = param.toFloat();
            Serial.println("OK");
            break;

        case 'I': case 'i':
            Igain = param.toFloat();
            Serial.println("OK");
            break;

        case 'P': case 'p':
            Pgain = param.toFloat();
            Serial.println("OK");
            break;

        case 'D': case 'd':
            Dgain = param.toFloat();
            Serial.println("OK");
            break;

        case 'C': case 'c':
            controllerMode = param.toInt();
            Serial.println("OK");
            break;

        case 'E': case 'e':
            externalPWM = param.toInt();
            Serial.println("OK");
            break;

        default:
            Serial.println("Err");
            break;
    }
    buffer = "";
}
