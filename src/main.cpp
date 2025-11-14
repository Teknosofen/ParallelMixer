




































/*
#include <Arduino.h>
#include "Wire.h"
#include "math.h"

// I2C sensor control stuff, SFM = flow meter, SPD = diff press sensor
#define I2Cadr_SFM 0x40
#define I2Cadr_SPD 0x25
uint8_t err;
#define SFM_com0 0x20 // SW reset
#define SFM_com1 0x10 // start cont measurement
#define SFM_com2 0x00
#define SPD_com1 0x36 // start cont measurement
#define SPD_com2 0x08
float SFM_flow ;
float SPD_press;

#define I2Cadr_SSC 0x58
float SSC_press;

#define I2Cadr_MCP4725 0x60
uint16_t Analogue_out;

// I2C traffic stuff
int16_t combined;  //32 bit variable to store the msb and lsb
uint8_t msb; 
uint8_t lsb;

// Sampled signals
float Press_now;  // press fr sensirion SPD
float Flow_now; // flow fr sensirion flow sensor
float Supply_press; // the press from SSC sensor

// analogue and IO pin definition
#define Flow_Input_Analogue_pin 1 // analogue control signal
#define Flow_Output_Analogue_pin 3 // measured flow signal out
#define Valve_ctrl_Analogue_pin 6 // check that this pin is OK for PWM
uint32_t past_time; // used for loop timing

// Command parser stuff
String CommandStr = "";
bool CommandComplete = false;

// Parameters to set and read
int16_t Flow_setting_selection_is_Analog = 0; // Command "A" default is digital flow reference
float digital_Flow_reference = 0.0;       // Command "F" in [L/min]
int16_t Quiet = 0;                            // Command "Q" Used to select quiet or verbose [default] output on serial channel
int16_t Param_C = 3;                          // Not yet used
int16_t DeltaT = 750;                         // Command "T" sampling time in [micros]
float Flow_reference = 0;
float Fused_Flow_value = 0;
uint16_t Flow_ref_analogue = 0;
uint8_t external_PWM = 1; // used to flag internal or external PWM signal, ext is default


// Control stuff
float ctrl_I_gain = 1;  // integration constant, Command "I"
float ctrl_P_gain = 1;  // integration constant, Command "P"
float ctrl_D_gain = 0;  // integration constant, Command "D"
float ctrl_I = 0;       // controller integrated error
float ctrl_err = 0;     // controller error
float Valve_Offset = 0; // Identified valve offset
float ctrl_output = 0;  // output sgnal from the controller
uint16_t Valve_ctrl_signal = 0; // this is the actual PWM to the valve

int Controller_mode = 0; // used to select controller bahaviour, see help text
uint16_t Offset = 1024; // Offset for sine and step functions
uint16_t Amplitude = 256; // Amplitude for sine and step functions
uint16_t Valve_ctrl_signal_externally_set = 0;
uint16_t Valve_ctrl_signal_sine_or_pulse = 0;
#define PID_Control 0
#define Valve_set_Value_Control 1
#define Sine_Control 2
#define Step_Control 3
uint16_t Samples_per_sine_or_pulse = 256; // sets the number of internal ticks to use for a sine or pulse period
uint16_t Index_in_period = 0; // caounter for position in pulses


// ------------------- setup stuff
void setup() {
  Wire.begin();
  Wire.setClock(1000000);
  Serial.begin(500000);

  // SW reset Sensirion flow measurement
  Wire.beginTransmission(I2Cadr_SFM);
  Wire.write(SFM_com0);
  Wire.write(SFM_com2);
  err =   Wire.endTransmission();
  if ( err != 0) {
    Serial.print("err SFM SW reset "); // print err
    Serial.println(err); // print err
  }
  delay(70);

  // SW reset Sensirion press measurement
  Wire.beginTransmission(I2Cadr_SPD);
  Wire.write(0x00);
  Wire.write(0x06);
  err =   Wire.endTransmission();
  if ( err != 0) {
    Serial.print("err SPD SW reset "); // print err
    Serial.println(err); // print err
  }
  delay(10);

  // start continous sensirion press measurement
  Wire.beginTransmission(I2Cadr_SPD);
  Wire.write(SPD_com1);
  Wire.write(SPD_com2);
  err =   Wire.endTransmission();
  if ( err != 0) {
    Serial.print("err SPD cont meas setup "); // print err
    Serial.println(err); // print err
  }
  delay(10);

  // start continous sensirion flow measurement
  Wire.beginTransmission(I2Cadr_SFM);
  Wire.write(SFM_com1);
  Wire.write(SFM_com2);
  err =   Wire.endTransmission();
  if ( err != 0) {
    Serial.print("err SFM cont meas setup "); // print err
    Serial.println(err); // print err
  }
  delay(10);

  //  ---- start timing stiff
  past_time = micros();

} // setup

// ---------------------- Main loop
void loop() {
  if ((micros() - past_time) >= DeltaT) { // time do go to work?
    past_time = micros(); // save the present time before spending it
    // sample pressure and Flow
    Press_now = get_SPD_Press();
    Flow_now = get_Flow();
    Supply_press = get_SSC_Press ();
    Flow_ref_analogue = analogRead(Flow_Input_Analogue_pin);    // read the input pin
    Fused_Flow_value = Sensor_Fuse(Press_now, Flow_now); // calculate a flow from the two sensors
    // -*-*-* ADD APPROPRIATE SCALING for the flow output!

    if (Quiet == 0) { // output data if not set to quiet
      // debug
      //      Serial.print(Flow_ref_analogue); // print the flow
      //      Serial.print(" ");
      // end debug // dP, Flow, SupplyP, Fused Flow
      Serial.print(Press_now, 1); // print the flow
      Serial.print(" ");
      Serial.print(Flow_now, 1); // print the flow
      Serial.print(" ");
      Serial.print(Supply_press, 3); // print the flow
      Serial.print(" ");
      Serial.println(Fused_Flow_value, 2); // print the flow
    }
    analogWrite(Flow_Output_Analogue_pin, int(Fused_Flow_value)); // Output the flow value to host

    // Selector for controller mode,

    Index_in_period++;
    if (Index_in_period >= Samples_per_sine_or_pulse) Index_in_period = 0; // roll around the in-period counter

    switch (Controller_mode) {
      case PID_Control:
        // use AD-value or digital set value for flow
        if (Flow_setting_selection_is_Analog == true) {  // use the analogue signal to calculate the set value
          // Flow_reference = something times the ADC
          Flow_reference = Flow_ref_analogue; // Add a decent scale factor when it is known
        }
        else {
          Flow_reference = digital_Flow_reference; // use the digital setting
        } // else
        // Here is the place to control the thing

        ctrl_err = (Flow_reference - Fused_Flow_value); // calc controller error
        ctrl_I += ctrl_err; // add the integrator

        ctrl_output = ctrl_P_gain * ctrl_err + ctrl_I_gain * ctrl_I; // start with PI only
        // -*-*-* ADD APPROPRIATE SCALING for the valve control output!
        Valve_ctrl_signal = uint16_t(ctrl_output + Valve_Offset); // Add the valve offset as well
        if (Valve_ctrl_signal > 4095) Valve_ctrl_signal = 4095;
        if (Valve_ctrl_signal <= 0) Valve_ctrl_signal = 0;
        if (Flow_reference == 0) {
          Valve_ctrl_signal = 0; // Ensure no valve output if the flow setting is zero
          ctrl_I = 0 ; // keep the integrator at zero
        }
        if (external_PWM == 1)     {
          analog_out_MCP4725(Valve_ctrl_signal);
        }
        else  {
          analogWrite(Valve_ctrl_Analogue_pin, Valve_ctrl_signal);
        }

        if (Quiet == 2) { // output debug data
          Serial.print("I ");
          Serial.print(ctrl_I, 1); // print the integrator
          Serial.print(" E ");
          Serial.print(ctrl_err, 1); // print the err
          Serial.print(" V ");
          Serial.print(Valve_ctrl_signal); // print the valve ctrl
          Serial.print(" F ");
          Serial.println(Fused_Flow_value);
          //      Serial.print(" Flow ref ");
          //      Serial.println(Flow_reference, 1); // print the flow
        } // if quiet

        break;

      case Valve_set_Value_Control:
        if (external_PWM == 1)     {
          analog_out_MCP4725(Valve_ctrl_signal_externally_set);
        }
        else  {
          analogWrite(Valve_ctrl_Analogue_pin, Valve_ctrl_signal_externally_set);
        }
        if (Quiet == 3) { // output data if not set to quiet
          Serial.print("Set value ");
          Serial.println(Valve_ctrl_signal_externally_set);
        }
        break;

      case Sine_Control:
        // Samples_per_sine_or_pulse
        Valve_ctrl_signal_sine_or_pulse = (float)Amplitude / 2 * (1 + sin(6.28 * (float)Index_in_period / (float)Samples_per_sine_or_pulse));
        Valve_ctrl_signal_sine_or_pulse += Offset;
        if (Quiet == 3) { // output data if not set to quiet
          Serial.print("Sine ");
          Serial.println(Valve_ctrl_signal_sine_or_pulse);
        }
        // Output the stuff
        if (external_PWM == 1)     {
          analog_out_MCP4725(Valve_ctrl_signal_sine_or_pulse);
        }
        else  {
          analogWrite(Valve_ctrl_Analogue_pin, Valve_ctrl_signal_sine_or_pulse);
        }
        break;

      case Step_Control:
        if (Index_in_period <= Samples_per_sine_or_pulse / 2) {
          // Start with the high pulse
          Valve_ctrl_signal_sine_or_pulse = Offset + Amplitude;
        }
        else
        { // time for the low level
          Valve_ctrl_signal_sine_or_pulse = Offset;
        }
        if (Quiet == 3) { // output data if not set to quiet
          Serial.print("pulse ");
          Serial.println(Valve_ctrl_signal_sine_or_pulse);
        }
        // Output the stuff
        if (external_PWM == 1)     {
          analog_out_MCP4725(Valve_ctrl_signal_sine_or_pulse);
        }
        else  {
          analogWrite(Valve_ctrl_Analogue_pin, Valve_ctrl_signal_sine_or_pulse);
        }
        break;

      default:
        break;

    } // switch (controller_mode)


  } // time to go to work

  // Check for commands
  if (CommandComplete == true )
  { Parse_CMD(CommandStr);
  } // if CMD complete

}   // Loop

// --------------------- functions and stuff
void analog_out_MCP4725(uint16_t Output) { // controls the external analogue outfuntion.
  Wire.beginTransmission(I2Cadr_MCP4725);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write(Output >> 4);        // the 8 most significant bits...
  Wire.write((Output & 15) << 4); // the 4 least significant bits...
  Wire.endTransmission();
} // analog_out_MCP4725

float get_SPD_Press()
{ //   Get SPD data
  Wire.requestFrom(I2Cadr_SPD, 3);    // contents of your first two bytes
  if (Wire.available() >= 3 );        // check when there is data
  { msb = Wire.read();                //byte1 is msb
    lsb = Wire.read();                //byte2 is lsb
    combined = msb;                   //assign msb to combined variable
    combined = msb << 8;              //assign msb to combined variable
    combined |= lsb;                  //add the lsb to the combined variable
    //    lsb = Wire.read();          //byte3 is crc
    //    Serial.print(combined);     //print the differential pressure
    //    Serial.print(" ");          //print the differential pressure
    //    Serial.print(combined, HEX);//print the differential pressure
    //    Serial.print(" ");          //print the differential pressure
    //    Serial.print(lsb, HEX);     // print the CRC
    return (float(combined) * 0.95 / 240); // 0.95 = 966mBar/1013mBar
  } // wire.available
} // get press

float get_Flow() {
  Wire.requestFrom(I2Cadr_SFM, 3);    // contents of your first two bytes
  if (Wire.available() >= 3 );        // check when there is data
  { msb = Wire.read();                //byte1 is msb
    lsb = Wire.read();                //byte2 is lsb
    combined = msb;                   //assign msb to combined variable
    combined = msb << 8;              //assign msb to combined variable
    combined |= lsb;                  //add the lsb to the combined variable
    //    lsb = Wire.read();          //byte3 is crc
    //    Serial.print(combined);     //print the differential pressure
    //    Serial.print(" ");          //print the differential pressure
    //    Serial.print(combined, HEX);//print the differential pressure
    //    Serial.print(" ");          //print the differential pressure
    //    Serial.print(lsb, HEX);     // print the CRC
    //    Serial.print(flow, 2);      // print the flow
    //    Serial.print(" ");
    return  (float(combined) - 10000) / 120.0;
  } // wire.available
} // get _Flow

float get_SSC_Press()
{ //   Get SSC data
  uint8_t s; // status bits, not used here
  uint16_t p, t; // raw data for press and temp
  Wire.requestFrom(I2Cadr_SSC, 4);    // contents of your first two bytes
  if (Wire.available() >= 3 );        // check when there is data
  { msb = Wire.read();                //byte1 is msb
    lsb = Wire.read();                //byte2 is lsb
    s = msb >> 6;               // maska ut de tvÃ¥ hÃ¶gsta statusbitarna
    p = ((msb & 0x3f) << 8) | lsb;  // maska de tvÃ¥ hÃ¶gsta bitarna, sen x*256 + y
    // SSC_press = (p - 1638.0) * (6.895) / (13107.0); // (p - 1638.0) * (6.895 - 0.0) / (14745.0 - 1638.0) + 0.0;
    msb = Wire.read();
    lsb = Wire.read();
    t = (( msb << 8) | lsb) >> 5; // (x*256 + y)/32
    Wire.endTransmission();
    //         T = (t / 2047.0) * 200.0 - 50.0;
    return (p - 1638.0) * (6.895) / (13107.0); // (p - 1638.0) * (6.895 - 0.0) / (14745.0 - 1638.0) + 0.0;
  } // wire.available
} // get press

// --------------------- Support stuff

float Sensor_Fuse(float diffPress, float flow) {
    // the two variables below should be sent here from a calibraion procedure somewhere
    float diffPressScalefactor = 10.84;     // From Marios XL-sheet
    float diffPressExponent = 0.558;          // From Marios XL-sheet
    float flowLowLimit = 1.0;                // low range where fusion starts
    float flowHighLimit = 10.0;              // high range where fusion stops
    float dPCalculatedFlow = 0;             // flow calculated from the diff pressure
    float weighingFactor = 0.5;             // used to weigh the two flows in an intermediate range
    
    if (diffPress <= 0.0) diffPress = 0.0; // do not allow negative flows

    if (flow < flowLowLimit) // low flow range - return the dP bases flow calculation
        {
            dPCalculatedFlow = diffPressScalefactor * pow(diffPress, diffPressExponent); 
            return dPCalculatedFlow;
        }
        else    // Flow is high enough to start working
        {
            if (flow < flowHighLimit) // Flow is in the fusing range
            {
                weighingFactor = (flow-flowLowLimit) / (flowHighLimit-flowLowLimit);
                dPCalculatedFlow = diffPressScalefactor * pow(diffPress, diffPressExponent); 
                Serial.println(dPCalculatedFlow, 3);
                return (dPCalculatedFlow * (1-weighingFactor) + flow * weighingFactor); // 
            }
            else // flow is above the fusing range, return the pure high flow
            {
                return flow; 
            }   
        }   
} // Sensor_Fuse()

void Parse_CMD(String CommandStr_local)
{ int16_t CommandStr_length  = 0;
  CommandStr_length = CommandStr_local.length();
  switch (CommandStr_local[0])
  {
    case 'N': case 'n':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Flow_setting_selection_is_Analog = CommandStr_local.toInt(); // Get the number
        Serial.print(Flow_setting_selection_is_Analog);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Flow_setting_selection_is_Analog != 0) Flow_setting_selection_is_Analog = 1;
      }
      else // respond with the present setting
      { Serial.print("N= ");
        Serial.println(Flow_setting_selection_is_Analog);
      }
      break;

    case 'F': case 'f':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        digital_Flow_reference = CommandStr_local.toFloat(); // Get the number, a float in this case
        Serial.print(digital_Flow_reference, 4);
        Serial.println("OK");
      }
      else // respond with the present setting
      { Serial.print("F-ref= ");
        Serial.println(digital_Flow_reference, 4);
      }
      break;

    case 'T': case 't':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        DeltaT = CommandStr_local.toInt(); // Get the number
        Serial.println("OK");
      }
      else // respond with the present setting
      { Serial.print("Dt= ");
        Serial.println(DeltaT);
      }
      break;

    case 'Q': case 'q': // quiet or verbose on serial channel
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Quiet = CommandStr_local.toInt(); // Get the number
        if (Quiet > 3) Quiet = 1; // Q=1 is quiet, Q=0 gives raw data out, Q= 2 gives debug data
        Serial.println("OK");
      }
      else // respond with the present setting
      { Serial.print("Q= ");
        Serial.println(Quiet);
      }
      break;

    // Controller settings
    case 'I': case 'i': // quiet or verbose on serial channel
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        ctrl_I_gain = CommandStr_local.toFloat(); // Get the number, a float in this case
        ctrl_I = 0; // zero the integrator, will behave strange if used for new I-gain
        Serial.print(ctrl_I_gain, 4);
        Serial.println("OK");
      }
      else // respond with the present setting
      { Serial.print("I= ");
        Serial.println(ctrl_I_gain, 4);
      }
      break;

    case 'P': case 'p': // quiet or verbose on serial channel
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        ctrl_P_gain = CommandStr_local.toFloat(); // Get the number, a float in this case
        Serial.print(ctrl_P_gain, 4);
        Serial.println("OK");
        // notify that an OK command was received
      }
      else // respond with the present setting
      { Serial.print("P= ");
        Serial.println(ctrl_P_gain, 4);
      }
      break;

    case 'D': case 'd': // quiet or verbose on serial channel
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        ctrl_D_gain = CommandStr_local.toFloat(); // Get the number, a float in this case
        Serial.print(ctrl_D_gain, 4);
        Serial.println("OK");
        // notify that an OK command was received
      }
      else // respond with the present setting
      { Serial.print("D= ");
        Serial.println(ctrl_D_gain, 4);
      }
      break;

    case 'E': case 'e': // External or internal PWM
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        external_PWM = CommandStr_local.toInt(); // Get the number
        if (external_PWM > 1) external_PWM = 1; // E = 1 is external PWM
        Serial.println("OK");
      }
      else // respond with the present setting
      { Serial.print("E= ");
        Serial.println(external_PWM);
      }
      break;

    //    case '1': // No extra digits after this command
    //      // notify that an OK command was received for
    //      Serial.println("OK");
    //      break;

    //    case '0': // No extra digits after this command
    //      // notify that an OK command was received for
    //      Serial.println("OK");
    //      break;

    case 'Z': case 'z':
      ctrl_err = 0; // zero flow error measure
      ctrl_I = 0; // zero the integrator
      Serial.println("Z OK");
      break;

    case 'C': case 'c':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Controller_mode = CommandStr_local.toInt(); // Get the number
        Serial.print(Controller_mode);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Controller_mode < 0) Controller_mode = 0;
        if (Controller_mode > 3) Controller_mode = 3;
      }
      else // respond with the present setting
      { Serial.print("C= ");
        Serial.println(Controller_mode);
      }
      Index_in_period = 0; // start the period from zero at controller type or period change
      break;

    case 'O': case 'o':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Offset = CommandStr_local.toInt(); // Get the number
        Serial.print(Offset);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Offset < 0) Offset = 0;
        if (Offset > 4095) Offset = 4095;
      }
      else // respond with the present setting
      { Serial.print("O= ");
        Serial.println(Offset);
      }
      break;

    case 'A': case 'a':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Amplitude = CommandStr_local.toInt(); // Get the number
        Serial.print(Amplitude);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Amplitude < 0) Amplitude = 0;
        if (Amplitude > 4095) Amplitude = 4095;
      }
      else // respond with the present setting
      { Serial.print("A= ");
        Serial.println(Amplitude);
      }
      break;

    case 'V': case 'v':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Valve_ctrl_signal_externally_set = CommandStr_local.toInt(); // Get the number
        Serial.print(Valve_ctrl_signal_externally_set);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Valve_ctrl_signal_externally_set < 0) Valve_ctrl_signal_externally_set = 0;
        if (Valve_ctrl_signal_externally_set > 4095) Valve_ctrl_signal_externally_set = 4095;
      }
      else // respond with the present setting
      { Serial.print("V= ");
        Serial.println(Valve_ctrl_signal_externally_set);
      }
      break;
    case 'S': case 's':
      if (CommandStr_length > 1)  // More than 'A' & <CR> was received, get the number
      { CommandStr_local.remove(0, 1); // Remove the first command character
        Samples_per_sine_or_pulse = CommandStr_local.toInt(); // Get the number
        Serial.print(Samples_per_sine_or_pulse);            // Echo the setting
        Serial.println("OK");
        // the setting shall be 0 or 1, if not zero - set to 1
        if (Samples_per_sine_or_pulse < 0) Samples_per_sine_or_pulse = 0;
        if (Samples_per_sine_or_pulse > 4095) Samples_per_sine_or_pulse = 4095;
      }
      else // respond with the present setting
      { Serial.print("S= ");
        Serial.println(Samples_per_sine_or_pulse);
      }
      Index_in_period = 0; // start the period from zero at controller type or period change
      break;

    case '?': // No extra digits after this command
      // Help command
      Serial.println("N = 0 for aNalog flow reference");
      Serial.println("T for sampl Time [us] int");
      Serial.println("Q = 1 for quiet, Q = 0 verbose");
      Serial.println("dP, Flow, SupplyP, Fused Flow");
      Serial.println("Z zeroes controller integrator");
      Serial.println("E selects Ext/Int PWM");
      Serial.println("F for flow ref value [L/min] float");
      Serial.println("I for int const [-] float");
      Serial.println("P for propo const [-] float");
      Serial.println("D for integrat const [-] float");
      Serial.println("C for controller mode");
      Serial.println("  0 = Valve set value, 1 = PI controller");
      Serial.println("  2 = Sine, 3 = step");
      Serial.println("O for Offset in [DAC steps], Int");
      Serial.println("A for Amplitude in [DAC steps], int");
      Serial.println("S for Samples per period of pulses or sine, int");
      Serial.println("V for valve outut set value [DAC counts]");
      Serial.println("R for rise time could be added another day");
      break;

    default:
      // Erroneous command if we end up here, manage accordingly
      Serial.println("Err");
      break;

  }         // Switch
  CommandComplete = false; // prepare for the next command
  CommandStr = "";         // clear the old string
}   // Parse_CMD

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();   // get the new byte:
    if (inChar == '\n') {                // if the incoming character is a newline, set a flag to main loop
      CommandComplete = true;             // but do not add the \n to the string
    } // If end of string
    else {
      CommandStr += inChar; // add it to the CommandStr:
    }   // else
  } // While there are chars available
} // SerialEvent

int lookup = 0;//varaible for navigating through the tables

int sintab2[512] =
{
  2048, 2073, 2098, 2123, 2148, 2174, 2199, 2224,
  2249, 2274, 2299, 2324, 2349, 2373, 2398, 2423,
  2448, 2472, 2497, 2521, 2546, 2570, 2594, 2618,
  2643, 2667, 2690, 2714, 2738, 2762, 2785, 2808,
  2832, 2855, 2878, 2901, 2924, 2946, 2969, 2991,
  3013, 3036, 3057, 3079, 3101, 3122, 3144, 3165,
  3186, 3207, 3227, 3248, 3268, 3288, 3308, 3328,
  3347, 3367, 3386, 3405, 3423, 3442, 3460, 3478,
  3496, 3514, 3531, 3548, 3565, 3582, 3599, 3615,
  3631, 3647, 3663, 3678, 3693, 3708, 3722, 3737,
  3751, 3765, 3778, 3792, 3805, 3817, 3830, 3842,
  3854, 3866, 3877, 3888, 3899, 3910, 3920, 3930,
  3940, 3950, 3959, 3968, 3976, 3985, 3993, 4000,
  4008, 4015, 4022, 4028, 4035, 4041, 4046, 4052,
  4057, 4061, 4066, 4070, 4074, 4077, 4081, 4084,
  4086, 4088, 4090, 4092, 4094, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4094, 4092, 4090, 4088,
  4086, 4084, 4081, 4077, 4074, 4070, 4066, 4061,
  4057, 4052, 4046, 4041, 4035, 4028, 4022, 4015,
  4008, 4000, 3993, 3985, 3976, 3968, 3959, 3950,
  3940, 3930, 3920, 3910, 3899, 3888, 3877, 3866,
  3854, 3842, 3830, 3817, 3805, 3792, 3778, 3765,
  3751, 3737, 3722, 3708, 3693, 3678, 3663, 3647,
  3631, 3615, 3599, 3582, 3565, 3548, 3531, 3514,
  3496, 3478, 3460, 3442, 3423, 3405, 3386, 3367,
  3347, 3328, 3308, 3288, 3268, 3248, 3227, 3207,
  3186, 3165, 3144, 3122, 3101, 3079, 3057, 3036,
  3013, 2991, 2969, 2946, 2924, 2901, 2878, 2855,
  2832, 2808, 2785, 2762, 2738, 2714, 2690, 2667,
  2643, 2618, 2594, 2570, 2546, 2521, 2497, 2472,
  2448, 2423, 2398, 2373, 2349, 2324, 2299, 2274,
  2249, 2224, 2199, 2174, 2148, 2123, 2098, 2073,
  2048, 2023, 1998, 1973, 1948, 1922, 1897, 1872,
  1847, 1822, 1797, 1772, 1747, 1723, 1698, 1673,
  1648, 1624, 1599, 1575, 1550, 1526, 1502, 1478,
  1453, 1429, 1406, 1382, 1358, 1334, 1311, 1288,
  1264, 1241, 1218, 1195, 1172, 1150, 1127, 1105,
  1083, 1060, 1039, 1017,  995,  974,  952,  931,
  910,  889,  869,  848,  828,  808,  788,  768,
  749,  729,  710,  691,  673,  654,  636,  618,
  600,  582,  565,  548,  531,  514,  497,  481,
  465,  449,  433,  418,  403,  388,  374,  359,
  345,  331,  318,  304,  291,  279,  266,  254,
  242,  230,  219,  208,  197,  186,  176,  166,
  156,  146,  137,  128,  120,  111,  103,   96,
  88,   81,   74,   68,   61,   55,   50,   44,
  39,   35,   30,   26,   22,   19,   15,   12,
  10,    8,    6,    4,    2,    1,    1,    0,
  0,    0,    1,    1,    2,    4,    6,    8,
  10,   12,   15,   19,   22,   26,   30,   35,
  39,   44,   50,   55,   61,   68,   74,   81,
  88,   96,  103,  111,  120,  128,  137,  146,
  156,  166,  176,  186,  197,  208,  219,  230,
  242,  254,  266,  279,  291,  304,  318,  331,
  345,  359,  374,  388,  403,  418,  433,  449,
  465,  481,  497,  514,  531,  548,  565,  582,
  600,  618,  636,  654,  673,  691,  710,  729,
  749,  768,  788,  808,  828,  848,  869,  889,
  910,  931,  952,  974,  995, 1017, 1039, 1060,
  1083, 1105, 1127, 1150, 1172, 1195, 1218, 1241,
  1264, 1288, 1311, 1334, 1358, 1382, 1406, 1429,
  1453, 1478, 1502, 1526, 1550, 1575, 1599, 1624,
  1648, 1673, 1698, 1723, 1747, 1772, 1797, 1822,
  1847, 1872, 1897, 1922, 1948, 1973, 1998, 2023
};
*/