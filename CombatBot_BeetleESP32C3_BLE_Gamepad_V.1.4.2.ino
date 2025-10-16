// Version
String version = "1.4.2";
/******************************************************/
/*   CombatBot for Beetle ESP32-C3 - BLE - GamePad    */
/******************************************************

  Free script designed to provide the platform for hobbyists and educational use as a starting point for
  controlling 150-450 g Combat Robots with gamepad via Bluetooth Low Energy (BLE).
  
  Script and controller can also be used to control basic Arduino based bots with servo.
  
  *****
  FEATURES:
  *****
  - High accuracy tank steering with center brake.
  - Channel mixing between fwd/bwd and left/right channels.
  - Drive motor rotation inverting and trim.
  - Supports DC motors using HR8833 or TB6612FNG DC motor driver or Brushless motors using ESC's for driving.
  - Support for single servo or brushless motor controlled by ESC as weapon motor.
  - 1-2S LiPo battery voltage monitoring.
  - Low voltage guard.
  - Failsafe & ouf of range protection.
  - Signal safety (Bot won't accept any signals until signal safety switch is pressed from controller).
  - Customizable AI slot (ch4, void runAI).
  - Buildin automatic support in Custom AI slot for flippers using DFRobot VL6180X ToF Distance Ranging Sensor.
  - Weapon/servo presets, drive motor directions and trim levels can be set from script presets. Trim can also be changed from controller.
  - Signal and variable debugging through serial monitor.
  - Onboard led indicates error-, standby-, bluetooth- and weapon statuses as followed:
      - Blink, dim/slow     =   Standby
      - Blink, bright/fast  =   Error: Wrong weapon parameter / Telemetry not enabled when using Low voltage guard / Battery voltage not recognized by telemetry / Telemetry voltage divider pin overload (Happens when ground resistor is broken or cut loose and pin receives battery voltage.)
      - Solid, dim          =   Bluetooth connected
      - Solid, bright       =   Weapon signal active
  - Thanks to the awesome Bluepad32 library can be controlled with max. 4 controllers at the same time.
  
  *****
  HARDWARE EXAMPLE (150g With DC Motors):
  *****
  - DFRobot - Beetle ESP32-C3
  - DFRobot - Fermion: HR8833 Thumbnail Sized DC Motor Driver 2x1.5A or TB6612FNG 2x1.2A DC Motor Driver
  - DFRobot - Fermion: VL6180X ToF Distance Ranging Sensor 5-100mm (Optional, for flippers.)
  - Rebuild Robotics - ESP32-C3 CombatBot Expansion Board (Prototype) or 5V BEC to convert battery voltage for boards
  - Rebuild Robotics - TinySwitch (Prototype)
  - Rebuild Robotics - TinyLED (Prototype)
  - 6V 600/1000RPM Geared N10/N20 DC motors
  - Servo for flipper or brushless motor and ESC for spinning weapon
  - 2S 180-300mAh LiPo battery
  - 22AWG Wires for drive motors, calculate main power and ESC needs separately
  - BT2.0 Connector pair (Replace battery JST with BT2.0 male connector. Remember to prevent battery shortcut!)
  - 10kOhm pull-down resistor for pin 6 with HR8833 and TB6612 (Included in ESP32-C3 CombatBot Expansion Board)

  *****
  OUTPUTS:
  *****
  - Driving:
    - DC motors:          2 x PWM + 2 digital outputs
    - Brushless motors:   2 x PWM outputs
  - Weapon ESC or servo:  1 x PWM output
  - Weapon PWM signals are generated with servo library.
  - If using brushless ESC's they has to be configured separately!

  *****
  HOW TO INSTALL:
  *****
  1. Installing Arduino IDE, Board manager and libraries:
    1.1. Install Arduino IDE from https://www.arduino.cc/en/software. It is the main program what you will use to manage this script and connection to ESP.
    1.2. Install board manager by following guide in bluepad32 wiki: https://github.com/ricardoquesada/bluepad32
    1.3. Install proper Arduino library versions mentioned below.
  2. Installing script:
    2.1. Download script from Githubs button: "Code" > Download Zip and unzip it.
    2.2. Set up pins and presets from script.
    2.3. Set up board manager presets as mentioned below.
    2.4. Upload script as mentioned below.
  3. Connect and have fun.
      
  *****
  ARDUINO IDE BOARD MANAGER & LIBRARIES:
  *****
  Tested stable versions:
  - Board & Gamepad:          Bluepad32 v.4.1.0           Install from https://github.com/ricardoquesada/bluepad32    (Uses ESP32 core 2.x)
  - Servo/Weapon control:     ESP32Servo v.3.0.9          Install from Arduino IDE
  - Distance sensor:          DFRobot_VL6180X v.1.0.1     Install from Arduino IDE
   
  *****
  UPLOADING AND SERIAL MONITORING:
  *****
  Presets needed in Arduino IDE for serial monitoring and uploading script:
  - Tools > Board: > esp32_bluepad32 > DFRobot Beetle ESP32-C3
  - Tools > USB CDC On Boot > Enabled
  - Tools > Upload Speed > 115200
  - Tools > Erase All Flash Before Sketch Upload > Enabled = Erases all bot presets from Eeprom / Disabled = Keeps bot presets in memory.
      
  When updating new version of script flash should always be erased!

  *****
  PRESETS:
  *****
  - Basic bot presets can be changed from below.
  - Channelmix and trim can be set from controller.
  - Presets will be saved into Eeprom when controller is disconnected from bot.
  - Presets saved in Eeprom will be erased in upload if "Tools > Erase All Flash Before Sketch Upload" is enabled.

  *****
  DEBUG:
  *****
  - Debug mode and serial monitoring can be enabled from below by changing "#define DEBUG" to true and reuploading script to ESP.
  - You might have to reset ESP once from button to get clean serial monitor output in start.
  - If facing problems uploading script and having "exit status 2..." error keep Arduino IDE open, plug ESP off from USB port, keep pressing Boot button while plugging ESP back into computer,
    reupload script and let go off from the Boot button when uploading has been started. https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3

  *****  
  COMMUNICATION:
  *****
  - Script uses 2,4GHz Bluetooth Low Energy (BLE) to communicate with gamepad.
  - axisRY, axisRX and axisY values are in scale of -511 to 511, 0 is stop. Weapon pad is 10 steps of +/-18 (=180deg). Buttons On/Off. axisRY and axisRX Signals are converted to PWM value scale 0 to 255. Negative numbers are absoluted.

  *****
  FAILSAFE & OUT OF RANGE:
  *****
  - Script includes failsafe and out of range functions to prevent up accidents and bot running away.
  - Functions stops bot from using AI, weapon and motors when controllers signal is lost or bot is out of range.
  - Using failsafe is highly recommended in robotics and it's required in combat robotics. Same kind of functions are used in normal RC transmitters.
  - Failsafe is deactivated when controller is connected into bot, so if you go out of range you have to reconnect into bot for to controlling it.
  - Remember to set pins correctly, if they are not set right script and failsafe won't work properly!

  *****
  LOW VOLTAGE GUARD:
  *****
 - Protects battery by activating failsafe when battery is running low while keeping controller connected to bot.
 - Failsafe will be deactivated when voltage becomes higher than shutdown level.
 - If facing problems with voltage drops and guard is stopping motors with close to charged battery try to use quality connectors like BT2.0 or XT-30 (not JST), proper cable sizes and possible capacitor near weapons power drain.
  
  *****
  CONTROLLER:
  *****
  - Script uses Bluepad32 library/board manager to handle bluetooth connection to controller.
  - Developed and tested with Google Stadia and XBox 360 gamepads supporting BLE. See more from: https://github.com/ricardoquesada/bluepad32/blob/main/docs/supported_gamepads.md
  - Use DEBUG to retrieve connected gamepads bluetooth address into serial monitor if needed.
  - Bluetooth pairing instructions can be found from bluepad32 pages. After once pairing device controller should automaticly reconnect to bot.
  - Controls:
      - Controller keymapping can be found from root of the directory.
      - Signal lock off (Basic controls):
        - A                   =   AI on/off
        - X                   =   Invert drive on/off
        - Right bumper        =   Signal lock on/off
        - Left bumper         =   Weapon on/off
        - Pad up/down         =   Weapon
        - Left stick          =   Weapon
        - Right stick         =   Driving
        - Y + Pad left/right  =   Trim
      - Signal lock on (Programming mode):
        - Y + B               =   Channel mixing on/off
        - Y + A               =   Add connected gamepads bluetooth address to allowList // Vibrates when added to list
        - Y + X               =   Clear gamepads from allowList // Vibrates when cleared
        - Y + Menu            =   Activate / disable allowList // Vibrates when enabled // If more than one pads are connected use this only once from any of the them
  - Haptic confirmations:
    - Left side             =   Weapon on/off button or weapon pad pressed
    - Left side, short      =   Trim button pressed from left pad
    - Right side, short     =   Add to allowList or enable/disable allowList buttons pressed
    - Right side, long      =   Signal lock button pressed
    - Both sides, strong    =   Bot battery low (Vibrates only when signal lock is off and controls are used.)
  - Controller doesn't vibrate when min/max value has been reached.
  - Notice: If no other weapon control haven't been used before pressing pad then first time pressing up/down calibrates weapon angle to idle.
  
  *****
  ALLOWLIST:
  *****
  - Allowlist allows only gamepads added into it to connect. It is crucial for security because it prevents unwanted connections.
  - Connected controllers can be added to list or removed from it with buttons above in controls list.
  - List will be reseted when reuploading script into ESP and "Erase All Flash Before Sketch Upload" is enabled from tools.
  - List can be hard reseted by grounding GPIO 2 shortly when esp has power turned on.
  
    *****
    Guide:  
    *****
    Always make adjustments before entering public areas for to avoid unwanted connections and injuries!

    Adding controller into bots allowList:
    1. Connect controller into bot.
    2. Make sure that Signal lock is ON.
    3. Press Y + A (Controller vibrates).
    4. Press Y + Menu (Controller vibrates).
    5. Power off controller (Bot saves controller into memory at disconnect).
    6. Wait that esp blue light is blinking again and your done..
    7. Now you can test is list working by shutting down bot and restarting bot and controller. Controller should reconnect into bot and other controllers won't.

    Removing controller from list:
    1. Connect controller into bot.
    2. Make sure that Signal lock is ON.
    3. Press Y + X (Controller vibrates).
    4. Power off controller (Bot saves controller into memory at disconnect).
    5. Wait that esp blue light is blinking again and your done.
    
    Problems:
    - If you are totally locked out from list hard reset list with jumper cable.

    AllowList is part of Bluepad32 library which is separate project and has nothing to do with this project except this script is using their board manager for gamepads.
    Without them this project wouldn't been achieved.

    More from allowlist:
    - https://bluepad32.readthedocs.io/en/latest/FAQ/
    - https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32/include/bt/uni_bt_allowlist.h
  
  *****
  SAFETY NOTICE:
  *****
  - Combat robotics is fun and extremely educating but dangerous hobby, always think safety first!
  - Do not keep battery connected at the same time when USB is connected into powersource!
  - Accidentally motor spins will happen when ESP is powered, script is uploading or wrong board or library version is installed!
    Motor spins at startup can be prevented by using pull-up/pull down resistors or our designed ESP32-C3 CombatBot Expansion Board and tested IDE library versions.
  - Configure ESC properly before connecting controller into bot, or it causes serious hazard!
  - Test bot with precaution and only weapon motor attached, not weapon itself!
  - Do not use any other board manager or library versions than those which are mentioned as tested and safe ones in this scripts read me area!
    It might cause serious injuries because board functionalities are changing. Future updates to this script includes fresher and tested information from later versions.
  - Remember always check that you have correct versions installed before updating script into board.
  - Modify script only if you know what you are doing!
  - Script has been tested only in close range at Combat arenas and inside. When driving outside use precaution.

  *****
  SUPPORTING PROJECTS:
  *****
  We do not search any financial benefit from scripts and apps made, we just want to offer platforms to help you to get started with robotics and learn how things are made.
  You are free to support other projects which are used in our programs and have.
  - Bluepad32, Ricardo Quesada: https://github.com/ricardoquesada/bluepad32
  
  *****
  SOURCES:
  *****
  Hardware:
  - https://wiki.dfrobot.com/SKU_DFR0868_Beetle_ESP32_C3
  - https://wiki.dfrobot.com/2x1.2A_DC_Motor_Driver__TB6612FNG__SKU__DRI0044
  - https://wiki.dfrobot.com/Dual_1.5A_Motor_Driver_-_HR8833_SKU__DRI0040
  - https://wiki.dfrobot.com/DFRobot_VL6180X_TOF_Distance_Ranging_Sensor_Breakout_Board_SKU_SEN0427

  Driving:
  - https://youtu.be/Bx0y1qyLHQ4?si=U1tk3dPtz3ZfaxqV
  - https://gist.github.com/ShawnHymel/ccc28335978d5d5b2ce70a2a9f6935f4
  - https://github.com/ricardoquesada/bluepad32
 
  This script has been made respecting the regulations of combat robotics and they should be usable all around the world.
  Script builders and component manufacturers are not responsiple from possible damages.
  Third party script writers hasn't got nothing to do with this project except I'm using their libraries.

  Copyright (C) Ville Ollila (RoboticsIsForNerds, Rebuild Robotics).
  
  This work is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  To view a copy of this license, visit http://creativecommons.org/licenses/by-nc/4.0/

  Examples used are linked in functions.

  Third party copyrights:
  Bluepad32 Copyright (C) 2016-2024 Ricardo Quesada and contributors.
  BTstack Copyright (C) 2017 BlueKitchen GmbH.
  ESP32Servo Copyright (C) 2017 John K. Bennett

  Bug reports through https://github.com/RebuildRobotics /program/Issues. Other contacts related this program: support@rebuildrobotics.fi.
*/
// -------------------- Verify board -------------------
#if !defined( ARDUINO_ESP32C3_DEV )
	#error This code is build for ESP32-C3! Check Board manager settings.
#endif
/******************************************************/
/* ------------------ BASIC PRESETS ----------------- */
/******************************************************/
// --------------------- Debug mode --------------------
#define DEBUG                       false     // To enable debug mode and serial monitoring change to true, to disable false.
// -------------------- Drive motors -------------------
// Motor driver type
#define HR8833                                // HR8833 / TB6612 / BRUSHL
// Rotation
bool INV_MOT_L = false;                       // Invert left drive motor rotation
bool INV_MOT_R = false;                       // Invert right drive motor rotation
// Speed
byte SPD_Y = 100;                             // Fwd/bwd (ch1) speed multiplier // 0 - 100%
byte SPD_X = 100;                             // Left/right (ch2) speed multiplier // 0 - 100%
// Trim
byte TRIM_L = 100;                            // Left motors rotation value for trim // 0 = zero rotation, 100 = full rotation
byte TRIM_R = 100;                            // Right motors rotation value for trim // 0 = zero rotation, 100 = full rotation
// Channel mixing
bool CHMIX = true;                            // Defines are fwd/bwd (ch1) and left/right (ch2) channel signals mixed together
// ----------------- Weapon ESC / Servo ----------------
bool USE_BIDIR = false;                       // Use BiDirectional servo/weapon signal // true = signal width is from -100 to 100, false = 0 to 100
bool USE_ESC = false;                         // Is script using ESC or servo // true = For spinners, signal is sent continuously to ESC. false = For flippers and grabbers, signal is sent to servo only when value has been changed.
// Servo angles / Brushless ESC speed controlled by Servo library (0-180° // 90° = 50%, 180° = 100% etc.) // Weapon(normal mode): 0° = Stop, 1-180° = run // Servo/Weapon(BiDir): 0° = Left, 180° = Right, 90° = Middle.
byte SRV_ANG_IDLE = 0;                        // Idle angle // Has to be 0 when using ESC and NOT using BiDirectional signal!
byte SRV_ANG_MAX = 180;                       // Max angle  // Has to be bigger than 0 when using ESC and NOT using BiDirectional signal!
byte SRV_ANG_MIN = 0;                         // Min angle  // Used only in BiDirectional mode
// --------------------- Telemetry ---------------------
#define useTelemetry                true      // Use battery voltage monitoring // Set false if voltage divider is not attached, otherwise bot is sending random values to controller and it will launch low battery alarm
#define PIN_VDIV                    0         // Voltage divider pin
#define monitorVoltageInterval      5000      // Delay in ms // Do not change smaller than 15sec, ADC and bluetooth can't handle it!
// ----------------- Low voltage guard -----------------
#define useLowVoltageGuard          true      // Use battery low voltage guard // Stops drive- and weapon motors if low voltage limit is reached // Telemetry has to be enabled when used.
#define lowVoltageLimit             0         // Voltage monitors low level limit in percents (6.5V - 8.4V = 0 - 100%)
// ---------- Distance sensor / AI (Optional) ----------
#define useSensor                   false     // Define is optional VL6180X distance sensor used
#define triggerRange                20        // Distance sensors trigger range in millimeters
#define sensorScanFreq              20        // Sensor scan and weapon angle change interval in ms
// --------------------- Connection --------------------
#define timeoutDelay                1000      // Timeout delay // If packages are not received from controller after delay stop and disconnect bot from all controllers.
// ------------------- Gamepad stick -------------------
#define enableWepStick              true      // Enables/Disables weapon joystick
#define invWepStick                 false     // Invert weapon joystick when using BiDir signal
#define stickOffset                 50        // Joystick offset area
/******************************************************/
/* ----------------- PWM PARAMETERS ----------------- */
/******************************************************/
// Motordrivers
#if defined(BRUSHL)
#define MIN_MICROS_MD               500       // min. 500us
#define MAX_MICROS_MD               2500      // max. 2500us
#endif
// Servo // Corona CS-939MG 600/2300us // BristolBotBuilders 2S HV High Speed Metal Geared Antweight Servo 500/2500us // DMS-MG90 500/2500us
#define MIN_MICROS_SERVO            500       // min. 500us
#define MAX_MICROS_SERVO            2500      // max. 2500us
// Brushless ESC (Default BLHeli S Littlebee 20A values)
#define MIN_MICROS_ESC              1148      // min. 500us
#define MAX_MICROS_ESC              1832      // max. 2500us
/******************************************************/
/* -------------- PIN & PWM PARAMETERS -------------- */
/******************************************************
 Pin layouts, pwm parameters etc. below are designed for to either connect motor drivers listed in info directly into DFRobots ESP32-C3 or when using Rebuild Robotics - ESP32-C3 CombatBot Expansion Board (Prototype).
 Presets are not necessary to change if so, but be sure that they are right.
 If connecting HR8833 or TB6612 motor driver into ESP directly, use 10kOhm pull-down resistor in pin 6. This prevents motor from spinning in startup and upload!
******************************************************/
// -------------------- Drive motors -------------------
// PWM channel parameters for DC motor drivers (Using ESP LED PWM)
#if defined(HR8833) || defined(TB6612)
#define CH_PWM_IA1                  2         // Left motor
#define CH_PWM_IA2                  4         // Left motor
#define CH_PWM_IB1                  3         // Right motor
#define CH_PWM_IB2                  5         // Right motor
#define FREQ_PWM_MD                 500       // PWM frequency
#define RESO_PWM_MD                 8         // 8-bit resolution
#endif
// Output pins // Use 10kOhm pull-down resistor in pin 6 with HR8833 and TB6612!
#if defined(HR8833) || defined(TB6612)
#define PIN_MD_IB1                  4         // Right motor
#define PIN_MD_IB2                  1         // Right motor
#endif
#if defined(HR8833)
#define PIN_MD_IA1                  5         // Left motor
#define PIN_MD_IA2                  6         // Left motor
#elif defined(TB6612)
#define PIN_MD_IA1                  6         // Left motor
#define PIN_MD_IA2                  5         // Left motor
#elif defined(BRUSHL)
#define PIN_M1                      1         // Left motor
#define PIN_M2                      4         // Right motor
#endif
// ----------------- Weapon ESC / Servo ----------------
#define PIN_WEP                     7         // Weapon pin (Servo / Brushless ESC) // Also allocates PWM channel 0, this is done by ESP32Servo library.
// -------------------- OnBoard LED --------------------
#define PIN_LED_ONBOARD             10        // OnBoard led pin
#if defined(HR8833) || defined(TB6612)
#define CH_PWM_LED                  1         // PWM channel for OnBoard led pulse generation with DC motor setup (ESP32Servo library is reserving channel 0 for weapon and channels 2-5 for drive motors).
#elif defined(BRUSHL)
#define CH_PWM_LED                  3         // PWM channel for OnBoard led pulse generation with brushless setup (ESP32Servo library is reserving channels 0-2 for weapon and drive motors).
#endif
#define FREQ_PWM_LED                5000      // PWM frequency
#define RESO_PWM_LED                8         // 8-bit resolution
/******************************************************/
/* ------------------- END OF PRESETS --------------- */
/******************************************************/
// ---------------------- Libraries --------------------
// Eeprom
#include <EEPROM.h>
// Gamepad
#include <Bluepad32.h>
#include <uni.h>
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Servos
#include <ESP32Servo.h>
Servo weapon;
#if defined(BRUSHL)
Servo motorLeft;
Servo motorRight;
#endif
// Distance sensor
#if defined(useSensor)
#include <DFRobot_VL6180X.h>
DFRobot_VL6180X VL6180X;
#endif
// ---------------------- Values -----------------------
// OnBoard LED
byte ledDuty;
unsigned long ledBlinkDelay;
bool ledOn;
unsigned long ledToggled;
// EEPROM
#define EEPROM_SIZE 5 // Amount of bytes(=rows) in EEPROM. 1 byte is for eeprom itself and 2 is for presets.
byte varsCount;
// Telemetry
unsigned long voltageLevel;
unsigned long voltageMonitored;
byte cells; // 1 or 2 // 1S or 2S LiPo battery // Defined in setup from batteryvoltage
bool sendAlarm;
bool batteryEmpty;
// Bluetooth
bool deviceConnected;
bool allowList;
unsigned long lastUpdate;
// Channel values
int ch1; // Drive channel: Forward/Backward drive speed
int ch2; // Drive channel: Left/Right turn speed
// Previous signal values
bool prevR1;
bool prevL1;
bool prevY;
bool prevB;
bool prevA;
bool prevX;
int prevMisc;
int prevPad;
int prevAxisY;
// AI
bool AIActive;
// Weapon
bool weaponActive;
int angPrev = -1;
unsigned long angleSetTime;
static uint8_t weaponPadStep = 18; // (10 steps from 0 to 180 deg)
// Signal safety
bool safetyActive = true;
// Invert drive
bool invert;
// Distance sensor
uint8_t range;
// Failsafe
bool failsafeActive = true;
// ---------------------- Onboard led ------------------
void setLed(byte duty, int delay)
{
  ledDuty = duty;
  ledBlinkDelay = delay;
  ledcWrite(CH_PWM_LED, ledDuty);
  (ledDuty > 0) ? ledOn = true : ledOn = false;
  if (ledBlinkDelay > 0) ledToggled = millis();
}

void blinkLed()
{
  if (ledBlinkDelay > 0 && (millis() - ledToggled) > ledBlinkDelay)
  {
    (ledOn) ? ledcWrite(CH_PWM_LED, 0) : ledcWrite(CH_PWM_LED, ledDuty);
    ledOn = !ledOn;
    ledToggled = millis();
  }
}
// ---------------- Parameters error check -------------
String errors;
void checkPresetErrors()
{
  // Invalid motor driver type
  #if !defined(HR8833) && !defined(TB6612) && !defined(BRUSHL)
    errors += "Warning: Incorrect motor driver type!\n";
  #endif
  // Wrong cell count
  if (useTelemetry && cells == 0)
    errors += "Warning: Battery voltage not recognized!\n";
  // Low voltage guard - telemetry disabled
  if (!useTelemetry && useLowVoltageGuard)
    errors += "Warning: Telemetry has to be enabled when using Low voltage guard!\n";
  // Wrong speed parameter
  if (SPD_Y <= 0 || SPD_Y > 100 || SPD_X <= 0 || SPD_X > 100)
    errors += "Warning: Speed value out of range!\n";
  // Wrong weapon parameter
  if ((USE_ESC && !USE_BIDIR && SRV_ANG_IDLE != 0) || (USE_ESC && USE_BIDIR && (SRV_ANG_IDLE == 0 || SRV_ANG_IDLE == 180)))
    errors += "Warning: Incorrect weapon idle angle!\n";
  // Servo value out of range
  if (SRV_ANG_MIN < 0 || SRV_ANG_MIN > 180 || SRV_ANG_IDLE < 0 || SRV_ANG_IDLE > 180 || SRV_ANG_MAX < 0 || SRV_ANG_MAX > 180)
    errors += "Warning: Servo value out of range!\n";
  // Trim out of range
  if (TRIM_L < 0 || TRIM_L > 100 || TRIM_R < 0 || TRIM_R > 100)
    errors += "Warning: Trim level out of range!\n";
}
// ------------------------ Setup ----------------------
void setup()
{
  // Init eeprom
  EEPROM.begin(EEPROM_SIZE); // Start EEPROM
  // Return data from EEPROM : Save default data to EEPROM if empty
  (EEPROM_Exists()) ? EEPROM_Load() : EEPROM_Save();
  // Set OnBoard led PWM
  #if defined(CH_PWM_LED)
    ledcSetup(CH_PWM_LED, FREQ_PWM_LED, RESO_PWM_LED);
    ledcAttachPin(PIN_LED_ONBOARD, CH_PWM_LED);
  #endif
  // Set allowList clear pin
  pinMode(2, INPUT);
  // Allocate timers for servos
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  // Init weapon
	weapon.setPeriodHertz(50); // 50hz
  // Init brushless drive motors
  #if defined(BRUSHL)
    motorLeft.setPeriodHertz(50);
    motorRight.setPeriodHertz(50);
  // Init DC Motor Driver PWM channels
  #elif defined(HR8833) || defined(TB6612)
    ledcSetup(CH_PWM_IA1, FREQ_PWM_MD, RESO_PWM_MD);
    ledcSetup(CH_PWM_IB1, FREQ_PWM_MD, RESO_PWM_MD);
    #if defined(HR8833)
      ledcSetup(CH_PWM_IA2, FREQ_PWM_MD, RESO_PWM_MD);
      ledcSetup(CH_PWM_IB2, FREQ_PWM_MD, RESO_PWM_MD);
    #endif
  #endif
  // Stop drive motors (Notice: At default ESP pin6 is HIGH in powerup and after that. This causes left motor to keep spinning until pin status has been changed. ESP powerup motor spinning can be prevented with pull-up/pull-down resistors if not using CombatBot Expansion Board.)
  stopMotors();
  // Disable Motor Driver output pins
  disableOutputs();
  // Start AI stopped
  setAI(false);
  // Init battery monitor
  if (useTelemetry)
  {
    // Voltage monitor // Uses voltage divider with 56kOhm resistor in GND and 100kOhm resistor in POS
    pinMode(PIN_VDIV, INPUT);
    analogSetPinAttenuation(PIN_VDIV, ADC_11db); // ESP32-C3 range 0-2.5V // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html#analogsetattenuation
    // Init LiPo cell count from battery voltage (Using voltage divider)
    int mA = analogReadMilliVolts(PIN_VDIV);
    if (mA >= 1000 && mA < 1550) // 2,8 - 4,3 V
      cells = 1;
    else if (mA >= 2150 && mA <= 3100) // 6,0 - 8,65V
      cells = 2;
  }
  // Start debug mode
  if (DEBUG)
  {
    // Start serial monitor connection and wait connection
    Serial.begin(115200);
    while (!Serial);
    delay(10);
    // Add debug header text
    Serial.printf("--------------------------------------\nCombatBot for Beetle ESP32-C3 V.%s\n----------- [ DEBUG MODE ] -----------\n", version);
  }
  // Check parameter errors
  checkPresetErrors();
  // Init distance sensor
  if (useSensor && !(VL6180X.begin()))
  {
    if (DEBUG) errors += "Warning: Distance sensor not found!\n";
  }
  // Init Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // Bluepad allowList
  (allowList) ? uni_bt_allowlist_set_enabled(true) : uni_bt_allowlist_set_enabled(false);
  // Print data
  if (DEBUG)
  {
    if (errors == ""){ printBotData(); }
    else
    {
      Serial.println(errors);
      Serial.println("--------- [ Setup failed ] ---------");
    }
  }
  // Indicate parameter errors
  while (errors != "")
    (ledBlinkDelay > 0) ? blinkLed() : setLed(255, 100); // Blink led and do not continue if errors found
  // Add debug footer text
  if (DEBUG)
    Serial.println("--------- [ Setup complete ] ---------");
  setLed(50, 1000);
}
// ---------------------- Main loop --------------------
void loop()
{
  // Gamepad
  bool dataUpdated = BP32.update();
  if (dataUpdated) processControllers();
  // Check has controller data been received and stop after delay if not
  runWatchdog();
  // Onboard led blink
  blinkLed();
  // Voltage monitoring
  if (useTelemetry && millis() - voltageMonitored >= monitorVoltageInterval)
    monitorVoltage();
  // Run AI
  if (deviceConnected && AIActive && !safetyActive && !batteryEmpty)
    runAI();
  // Hard reset allowList
  if (digitalRead(2) == LOW) resetAllowList();
  // Slow down loop, otherwise runWatchdog() check will fail
  delay(1);
}
// ---------------------- Functions --------------------
void EEPROM_Save()
{
  EEPROM.write(0, 1); // Boolean for EEPROM_Exists()
  EEPROM.write(1, CHMIX);
  EEPROM.write(2, TRIM_L);
  EEPROM.write(3, TRIM_R);
  EEPROM.write(4, allowList);
  EEPROM.commit();
}

void EEPROM_Load()
{
  CHMIX = EEPROM.read(1);
  TRIM_L = EEPROM.read(2);
  TRIM_R = EEPROM.read(3);
  allowList = EEPROM.read(4);
}

bool EEPROM_Exists()
{
  if (EEPROM.read(0) == 1) { return true; }
  else { return false; }
}

void drive(int spdL, int spdR)
{
  // Fermion: TB6612FNG
  #if defined(TB6612)
    // Left motor direction
    if (spdL > 0) // Forward
    {
      (INV_MOT_L) ? digitalWrite(PIN_MD_IA2, 1) : digitalWrite(PIN_MD_IA2, 0);
    }
    else if (spdL < 0) // Backward
    {
      (INV_MOT_L) ? digitalWrite(PIN_MD_IA2, 0) : digitalWrite(PIN_MD_IA2, 1);
    }
    else // Brake
    {
      digitalWrite(PIN_MD_IA2, 1);
    }
    // Right motor direction
    if (spdR > 0) // Forward
    {
      (INV_MOT_R) ? digitalWrite(PIN_MD_IB2, 1) : digitalWrite(PIN_MD_IB2, 0);
    }
    else if (spdR < 0) // Backward
    {
      (INV_MOT_R) ? digitalWrite(PIN_MD_IB2, 0) : digitalWrite(PIN_MD_IB2, 1);
    }
    else // Brake
    {
      digitalWrite(PIN_MD_IB2, 1);
    }
    // Speed
    ledcWrite(CH_PWM_IA1, abs(spdL));
    ledcWrite(CH_PWM_IB1, abs(spdR));
  #endif
  // Fermion: HR8833
  #if defined(HR8833)
    // Left motor direction
    if (spdL > 0) // Forward
    {
      if (INV_MOT_L)
      {
        ledcWrite(CH_PWM_IA1, 255-abs(spdL));
        ledcWrite(CH_PWM_IA2, 255);
      }
      else
      {
        ledcWrite(CH_PWM_IA2, 255-abs(spdL));
        ledcWrite(CH_PWM_IA1, 255);
      }
    }
    else if (spdL < 0) // Backward
    {
      if (INV_MOT_L)
      {
        ledcWrite(CH_PWM_IA2, 255-abs(spdL));
        ledcWrite(CH_PWM_IA1, 255);
      }
      else
      {
        ledcWrite(CH_PWM_IA1, 255-abs(spdL));
        ledcWrite(CH_PWM_IA2, 255);
      }
    }
    else // Brake
    {
      ledcWrite(CH_PWM_IA1, 255);
      ledcWrite(CH_PWM_IA2, 255);
    }
    // Right motor direction
    if (spdR > 0) // Forward
    {
      if (INV_MOT_R)
      {
        ledcWrite(CH_PWM_IB1, 255-abs(spdR));
        ledcWrite(CH_PWM_IB2, 255);
      }
      else
      {
        ledcWrite(CH_PWM_IB2, 255-abs(spdR));
        ledcWrite(CH_PWM_IB1, 255);
      }
    }
    else if (spdR < 0) // Backward
    {
      if (INV_MOT_R)
      {
        ledcWrite(CH_PWM_IB2, 255-abs(spdR));
        ledcWrite(CH_PWM_IB1, 255);
      }
      else
      {
        ledcWrite(CH_PWM_IB1, 255-abs(spdR));
        ledcWrite(CH_PWM_IB2, 255);
      }
    }
    else // Brake
    {
      ledcWrite(CH_PWM_IB1, 255);
      ledcWrite(CH_PWM_IB2, 255);
    }
  #endif
  // Brushless motors
  #if defined(BRUSHL)
    motorLeft.write(spdL);
    motorRight.write(spdR);
  #endif
}

void setWeaponAngle(int angle)
{
  // Rotate servo  
  weapon.write(angle);
  // Change build in led state and send confirm to controller if channel value has been changed
  if (angle == SRV_ANG_IDLE)
  {
    (deviceConnected) ? setLed(50, 0) : setLed(50, 1000);
    weaponActive = false;
  }
  else
  {
    setLed(255, 0);
    weaponActive = true;
  }
  angPrev = angle;
  angleSetTime = millis();
}

void enableOutputs()
{
  // Drive Motors failsafe
  #if defined(HR8833)
    // Attach Motor Driver pins to pwm channels
    ledcAttachPin(PIN_MD_IA1, CH_PWM_IA1);
    ledcAttachPin(PIN_MD_IA2, CH_PWM_IA2);
    ledcAttachPin(PIN_MD_IB1, CH_PWM_IB1);
    ledcAttachPin(PIN_MD_IB2, CH_PWM_IB2);
  #endif
  #if defined(TB6612)
    // Attach Motor Driver pins to pwm channels
    ledcAttachPin(PIN_MD_IA1, CH_PWM_IA1);
    ledcAttachPin(PIN_MD_IB1, CH_PWM_IB1);
    // Set direction pins as output
    pinMode(PIN_MD_IA2, OUTPUT);
    pinMode(PIN_MD_IB2, OUTPUT);
  #endif
  #if defined(BRUSHL)
    // Enable motor signals
    motorLeft.attach(PIN_M1, MIN_MICROS_MD, MAX_MICROS_MD); 
    motorRight.attach(PIN_M2, MIN_MICROS_MD, MAX_MICROS_MD);
  #endif
  // Enable weapon signal
  weapon.attach(PIN_WEP, (USE_ESC) ? MIN_MICROS_ESC : MIN_MICROS_SERVO, (USE_ESC) ? MAX_MICROS_ESC : MAX_MICROS_SERVO); 
}

void disableOutputs()
{
  // Drive Motors failsafe
  #if defined(HR8833)
    // Detach Motor Driver pins from pwm channels
    ledcDetachPin(PIN_MD_IA1);
    ledcDetachPin(PIN_MD_IA2);
    ledcDetachPin(PIN_MD_IB1);
    ledcDetachPin(PIN_MD_IB2);
    // Set pins as input to stop motors from rotating
    pinMode(PIN_MD_IA1, INPUT);
    pinMode(PIN_MD_IB1, INPUT);
    pinMode(PIN_MD_IA2, INPUT);
    pinMode(PIN_MD_IB2, INPUT);
  #endif
  #if defined(TB6612)
    // Detach Motor Driver pins from pwm channels
    ledcDetachPin(PIN_MD_IA1);
    ledcDetachPin(PIN_MD_IB1);
    // Set pins as input to stop motors from rotating
    pinMode(PIN_MD_IA1, INPUT);
    pinMode(PIN_MD_IB1, INPUT);
    pinMode(PIN_MD_IA2, INPUT);
    pinMode(PIN_MD_IB2, INPUT);
  #endif
  #if defined(BRUSHL)
    motorLeft.detach();
    motorRight.detach();
  #endif
  // Disable weapon signal
  weapon.detach();
}

void stopMotors()
{
  // Fermion: TB6612FNG
  #if defined(TB6612)
    ledcWrite(CH_PWM_IA1, 0);
    ledcWrite(CH_PWM_IB1, 0);
    digitalWrite(PIN_MD_IA2, 1);
    digitalWrite(PIN_MD_IB2, 1);
  #endif
  // Fermion: HR8833
  #if defined(HR8833)
    ledcWrite(CH_PWM_IA1, 255);
    ledcWrite(CH_PWM_IA2, 255);
    ledcWrite(CH_PWM_IB1, 255);
    ledcWrite(CH_PWM_IB2, 255);
  #endif
  // Brushless motors  
  #if defined(BRUSHL)
    motorLeft.write(90);
    motorRight.write(90);
  #endif
}

void resetChannels()
{
  ch1 = 0;
  ch2 = 0;
}

void onConnectedController(ControllerPtr ctl) // This callback gets called any time a new gamepad is connected. // Up to 4 gamepads can be connected at the same time.
{
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
  {
    if (myControllers[i] == nullptr)
    {
      ControllerProperties properties = ctl->getProperties();
      if (DEBUG)
      {
        Serial.printf("Controller is connected, index=%d\n", i);
        const uint8_t* ctlAddr = properties.btaddr; // Get controllers bluetooth adddress
        Serial.printf("ctlAddr: %2X:%2X:%2X:%2X:%2X:%2X\n", ctlAddr[0], ctlAddr[1], ctlAddr[2], ctlAddr[3], ctlAddr[4], ctlAddr[5]);
        //Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      }
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot && DEBUG)
    Serial.println("Controller connected, but could not found empty slot");
  deviceConnected = true;
  setLed(50, 0);
  failsafe(false);
  if (DEBUG) printBotData();
}

void onDisconnectedController(ControllerPtr ctl)
{
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
  {
    if (myControllers[i] == ctl)
    {
      if (DEBUG)
        Serial.printf("Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController)
  {
    if (DEBUG)
      Serial.println("Controller disconnected, but not found in myControllers");
  }
  deviceConnected = false;
  setLed(50, 1000);
  failsafe(true);
  safetyActive = true;
  EEPROM_Save();
  if (DEBUG) printBotData();
}

void failsafe(bool active)
{
  if (active) // Enable, stops all
  {
    stopMotors();
    if (USE_ESC) setWeaponAngle(SRV_ANG_IDLE);
    setAI(false);
    disableOutputs();
    resetChannels();
    failsafeActive = true;
  }
  else // Disable, allow all
  {
    enableOutputs();
    failsafeActive = false;
  }
}

void runWatchdog()
{
  if ((ch1 != 0 || ch2 != 0 || AIActive || weaponActive) && !failsafeActive && (millis() - lastUpdate) > timeoutDelay)
  {
    failsafe(true);
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
      myControllers[i]->disconnect();
    if (DEBUG) Serial.println("[!] No data received");
  }
}

void monitorVoltage()
{
  // Turn battery voltage into percents // Using voltage divider in PIN 0 with 100kOhm resistor in POS and 56kOhm in GND
  voltageLevel = analogReadMilliVolts(PIN_VDIV); // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
  if (cells == 1)
    voltageLevel = map(voltageLevel, 1150, 1500, 0, 100);
  else if (cells == 2)
    voltageLevel = map(voltageLevel, 2350, 3000, 0, 100);
  else
    voltageLevel = 0;
  voltageLevel = constrain(voltageLevel, 0, 100);
  // Alarm & low voltage guard
  if (voltageLevel <= lowVoltageLimit)
  {
    if (deviceConnected) sendAlarm = true;
    if (useLowVoltageGuard && !batteryEmpty)
    {
      batteryEmpty = true;
      failsafe(true);
    }
  }
  else
  {
    if (sendAlarm) sendAlarm = false;
    if (batteryEmpty)
    {
      batteryEmpty = false;
      if (useLowVoltageGuard && deviceConnected)
        failsafe(false);
    }
  }
  voltageMonitored = millis();
}

void processGamepad(ControllerPtr ctl)
{
  // Battery low alarm
  if (!safetyActive && sendAlarm)
    ctl->playDualRumble(0, 500, 0x80, 0x40);
  // Get signals
  bool currentR1 = ctl->r1();
  bool currentL1 = ctl->l1();
  bool currentY = ctl->y();
  bool currentB = ctl->b();
  bool currentA = ctl->a();
  bool currentX = ctl->x();
  int currentMisc = ctl->miscButtons();
  int currentPad = ctl->dpad();
  int currentAxisY = ctl->axisY();
  // Signal safety
  if (currentR1 && !prevR1)
    ctl->playDualRumble(0, 300, 0x00, 0x40);
  if (currentR1 && !prevR1 && !safetyActive) // Signal safety On
  {
    stopMotors();
    if (USE_ESC) setWeaponAngle(SRV_ANG_IDLE);
    setAI(false);
    resetChannels();
    safetyActive = true;
  }
  else if (currentR1 && !prevR1 && safetyActive) // Signal safety Off
  {
    safetyActive = false;
  }
  // Programming mode
  if (safetyActive && currentY && prevY)
  {
    // Channel mix on/off
    if (currentB && !prevB)
      CHMIX = !CHMIX;
    // Bluetooth allowList, add address to list
    if (currentA && !prevA) // Y + A
    {
      if (uni_bt_allowlist_add_addr(ctl->getProperties().btaddr))
        ctl->playDualRumble(0, 200, 0x80, 0x40);
    }
    // Bluetooth allowList, clear list
    if (currentX && !prevX) // Y + X
    {
      uni_bt_allowlist_remove_all();
      ctl->playDualRumble(0, 200, 0x80, 0x40);
    }
    // Bluetooth allowList, enable / disable list
    if (currentMisc == 4 && prevMisc != 4) // Y + menu
    {
      allowList = !allowList;
      uni_bt_allowlist_set_enabled(allowList);
      if (allowList) ctl->playDualRumble(0, 200, 0x80, 0x40);
    }
  }
  // Normal mode
  if (!failsafeActive && !safetyActive && !batteryEmpty)
  {
    // AI
    if (currentA && !prevA)
      setAI(!AIActive);
    // Weapon On/Off button
    if (currentL1 && !prevL1)
      ctl->playDualRumble(0, 100, 0x80, 0x00);
    if (currentL1 && !prevL1 && !weaponActive) // Wep On/Full
    {
      if (!USE_BIDIR) setWeaponAngle(SRV_ANG_MAX);
    }
    else if (currentL1 && !prevL1 && weaponActive) // Wep Off
    {
      setWeaponAngle(SRV_ANG_IDLE);
    }
    // Weapon stick (Left joystick)
    else if (enableWepStick && currentAxisY != prevAxisY)
    {
      // Convert weapon signal to servo position
      int srvAng = ctl->axisY();
      if (invWepStick)
        srvAng = -srvAng;
      if (abs(srvAng) <= stickOffset)
        srvAng = 0;
      if (USE_BIDIR)
      {
        srvAng = map(srvAng, 511, -511, SRV_ANG_MIN, SRV_ANG_MAX);
        srvAng = constrain(srvAng, (SRV_ANG_MIN < SRV_ANG_MAX) ? SRV_ANG_MIN : SRV_ANG_MAX, (SRV_ANG_MIN < SRV_ANG_MAX) ? SRV_ANG_MAX : SRV_ANG_MIN);
      }
      else
      {
        srvAng = map(srvAng, 0, -511, SRV_ANG_IDLE, SRV_ANG_MAX);
        srvAng = constrain(srvAng, (SRV_ANG_IDLE < SRV_ANG_MAX) ? SRV_ANG_IDLE : SRV_ANG_MAX, (SRV_ANG_IDLE < SRV_ANG_MAX) ? SRV_ANG_MAX : SRV_ANG_IDLE);
      }
      setWeaponAngle(srvAng);
    }
    // Weapon pad
    else if (currentPad == 1 && prevPad != 1) // Up // Increase in steps of 18
    {
      if (angPrev == -1) // Calibrate to zero if no other weapon control haven't been used before pressing pad
      {
        setWeaponAngle(SRV_ANG_IDLE);
      }
      else // Set angle
      {
        if (SRV_ANG_IDLE < SRV_ANG_MAX)
        {
          if (SRV_ANG_MAX - angPrev >= weaponPadStep)
            setWeaponAngle(angPrev + weaponPadStep);
          else if (SRV_ANG_MAX - angPrev < weaponPadStep && SRV_ANG_MAX - angPrev > 0)
            setWeaponAngle(SRV_ANG_MAX);
        }
        else
        {
          if (abs(SRV_ANG_MAX - angPrev) >= weaponPadStep)
            setWeaponAngle(angPrev - weaponPadStep);
          else if (abs(SRV_ANG_MAX - angPrev) < weaponPadStep && abs(SRV_ANG_MAX - angPrev) > 0)
            setWeaponAngle(SRV_ANG_MAX);
        }
      }
    }
    else if (currentPad == 2 && prevPad != 2) // Down // Decrease in steps of 18
    {
      if (angPrev == -1) // Calibrate to zero if no other weapon control haven't been used before pressing pad
      {
        setWeaponAngle(SRV_ANG_IDLE);
      }
      else // Set angle
      {
        if (SRV_ANG_IDLE < SRV_ANG_MAX)
        {
          if ((USE_BIDIR && SRV_ANG_MIN + angPrev >= weaponPadStep) || (!USE_BIDIR && SRV_ANG_IDLE + angPrev >= weaponPadStep))
            setWeaponAngle(angPrev - weaponPadStep);
          else if (USE_BIDIR && SRV_ANG_MIN + angPrev < weaponPadStep && SRV_ANG_MIN + angPrev > SRV_ANG_MIN)
            setWeaponAngle(SRV_ANG_MIN);
          else if (!USE_BIDIR && SRV_ANG_IDLE + angPrev < weaponPadStep && SRV_ANG_IDLE + angPrev > SRV_ANG_IDLE)
            setWeaponAngle(SRV_ANG_IDLE);
        }
        else
        {
          if ((USE_BIDIR && SRV_ANG_MIN - angPrev >= weaponPadStep) || (!USE_BIDIR && SRV_ANG_IDLE - angPrev >= weaponPadStep))
            setWeaponAngle(angPrev + weaponPadStep);
          else if (USE_BIDIR && SRV_ANG_MIN - angPrev < weaponPadStep && SRV_ANG_MIN + angPrev > SRV_ANG_MIN)
            setWeaponAngle(SRV_ANG_MIN);
          else if (!USE_BIDIR && SRV_ANG_IDLE - angPrev < weaponPadStep && SRV_ANG_IDLE + angPrev > SRV_ANG_IDLE)
            setWeaponAngle(SRV_ANG_IDLE);
        }
      }
    }
    if ((currentPad == 1 && prevPad != 1) || (currentPad == 2 && prevPad != 2)) // Vibrate
    {
      if (
        USE_BIDIR && SRV_ANG_IDLE < SRV_ANG_MAX && angPrev > SRV_ANG_MIN && angPrev < SRV_ANG_MAX
        || USE_BIDIR && SRV_ANG_MAX < SRV_ANG_IDLE && angPrev < SRV_ANG_MIN && angPrev > SRV_ANG_MAX        
        || !USE_BIDIR && SRV_ANG_IDLE < SRV_ANG_MAX && angPrev > SRV_ANG_IDLE && angPrev < SRV_ANG_MAX        
        || !USE_BIDIR && SRV_ANG_MAX < SRV_ANG_IDLE && angPrev < SRV_ANG_IDLE && angPrev > SRV_ANG_MAX
        )
          ctl->playDualRumble(0, 100, 0x80, 0x00);
    }
    // Trim
    static int trimPadStep = 5;
    if (currentY && prevY)
    {
      if ((currentPad == 8 && prevPad != 8 || currentPad == 4 && prevPad != 4) && TRIM_L > 0 && TRIM_L < 100 && TRIM_R > 0 && TRIM_R < 100)
        ctl->playDualRumble(0, 100, 0x80, 0x00);
      if (currentPad == 8 && prevPad != 8) // Left button
      {
        if (TRIM_L > 0 && TRIM_R == 100)
        {
          if (TRIM_L >= trimPadStep) // Decrease left motor
            TRIM_L -= trimPadStep;
          else
            TRIM_L -= TRIM_L;
        }
        else if (TRIM_R < 100 && TRIM_L == 100)
        {
          if (100-TRIM_R >= trimPadStep) // Increase right motor
            TRIM_R += trimPadStep;
          else
            TRIM_R += (100-TRIM_R);
        }
      }
      else if (currentPad == 4 && prevPad != 4) // Right button
      {
        if (TRIM_R > 0 && TRIM_L == 100)
        {
          if (TRIM_R >= trimPadStep) // Decrease right motor
            TRIM_R -= trimPadStep;
          else
            TRIM_R -= TRIM_R;
        }
        else if (TRIM_L < 100 && TRIM_R == 100)
        {
          if (100-TRIM_L >= trimPadStep) // Increase left motor
            TRIM_L += trimPadStep;
          else
            TRIM_L += (100-TRIM_L);
        }
      }
    }
    // Invert drive
    if (currentX && !prevX)
      invert = !invert;
    // Read driving channels
    if (!invert && ctl->axisRY())
      ch1 = ctl->axisRY();
    else if (invert && ctl->axisRY())
      ch1 = -(ctl->axisRY());
    if (ctl->axisRX())
      ch2 = ctl->axisRX();
    // Set speed multiplier
    ch1 = round(ch1 * ((float)SPD_Y / 100));
    ch2 = round(ch2 * ((float)SPD_X / 100));
    // If in offset area set to zero
    if (abs(ctl->axisRY()) <= stickOffset)
      ch1 = 0;
    if (abs(ctl->axisRX()) <= stickOffset)
      ch2 = 0;
    // Mix driving channels & invert ch1 because of controller fwd is negative not positive
    int throttleLeft; // Left motor speed
    int throttleRight; // Right motor speed
    if (CHMIX)
    {
      throttleLeft = -ch1 + ch2; // Original: y + x
      throttleRight = -ch1 - ch2; // Original: y - x
    }
    else // No channel mixing
    {
      if (abs(ch1) >= abs(ch2)) // Forward/backward
      {
        throttleLeft = -ch1;
        throttleRight = -ch1;
      }
      else // Full turns
      {
        throttleLeft = ch2;
        throttleRight = -ch2;
      }
    }
    // Constrain 
    throttleLeft = constrain(throttleLeft, -500, 500 ); 
    throttleRight = constrain(throttleRight, -500, 500 );
    // Trim
    throttleLeft = round(throttleLeft * ((float)TRIM_L / 100));
    throttleRight = round(throttleRight * ((float)TRIM_R / 100));
    // Convert to PWM
    #if defined(HR8833) || defined(TB6612)
      throttleLeft = map(throttleLeft, -500, 500, -255, 255);
      throttleRight = map(throttleRight, -500, 500, -255, 255);
      throttleLeft = constrain(throttleLeft, -255, 255);
      throttleRight = constrain(throttleRight, -255, 255);
    #endif
    // Convert to Angle & invert rotation
    #if defined(BRUSHL)
      throttleLeft = (INV_MOT_L) ? map(throttleLeft, -500, 500, 180, 0) : map(throttleLeft, -500, 500, 0, 180);
      throttleRight = (INV_MOT_R) ? map(throttleRight, -500, 500, 0, 180) : map(throttleRight, -500, 500, 180, 0);
      throttleLeft = constrain(throttleLeft, 0, 180);
      throttleRight = constrain(throttleRight, 0, 180);
    #endif
    // Run drive motors
    #if defined(HR8833) || defined(TB6612) || defined(BRUSHL)
      drive(throttleLeft, throttleRight);
    #endif
  }
  // Print debug data
  if (DEBUG)
  {
    printPadData(ctl);
    if (prevR1 != currentR1 || prevL1 != currentL1 || prevY != currentY || prevB != currentB || prevA != currentA || prevX != currentX || prevMisc != currentMisc || prevPad != currentPad || prevAxisY != currentAxisY)
      printBotData();
  }
  // Store previous signal values
  prevR1 = currentR1;
  prevL1 = currentL1;
  prevY = currentY;
  prevB = currentB;
  prevA = currentA;
  prevX = currentX;
  prevMisc = currentMisc;
  prevPad = currentPad;
  prevAxisY = currentAxisY;
}

void processControllers()
{
  for (auto myController : myControllers)
  {
    if (myController && myController->isConnected())
    {
      if (myController->hasData())
      {
        if (myController->isGamepad())
        {
          processGamepad(myController);
          lastUpdate = millis();
        }
        else { if (DEBUG) errors += "Unsupported controller\n"; }
      }
    }
  }
}

void setAI(bool active)
{
  AIActive = active;
}

void runAI() // Modifiable ch4 channel slot, designed for to run optional AI. Uses signal from 0 to 100
{
  // Distance sensor for semiautomatic flipper
  if (useSensor)
  {
    if ((millis() - angleSetTime) >= sensorScanFreq) // Trigger after delay
    {
      range = VL6180X.rangePollMeasurement();
      if (range > 0 && range < 255 && range <= triggerRange && !weaponActive) // Set weapon to max angle
        setWeaponAngle(SRV_ANG_MAX);
      else if ((range == 0 || range >= 255 || range > triggerRange) && weaponActive) // Return to idle position
        setWeaponAngle(SRV_ANG_IDLE);
      else
        angleSetTime = millis();
    }
    if (DEBUG) // Print errors
    {
      uint8_t status = VL6180X.getRangeResult();
      switch (status)
      {
        case VL6180X_EARLY_CONV_ERR:
          errors += "RANGE ERR: ECE check failed!\n";
          break;
        case VL6180X_MAX_CONV_ERR:
          errors += "RANGE ERR: System did not converge before the specified max!\n";
          break;
        case VL6180X_IGNORE_ERR:
          errors += "RANGE ERR: Ignore threshold check failed!\n";
          break;
        case VL6180X_MAX_S_N_ERR:
          errors += "RANGE ERR: Measurement invalidated!\n";
          break;
        case VL6180X_RAW_Range_UNDERFLOW_ERR:
          errors += "RANGE ERR: RESULT_RANGE_RAW < 0!\n";
          break;
        case VL6180X_RAW_Range_OVERFLOW_ERR:
          errors += "RESULT_RANGE_RAW is out of range!\n";
          break;
        case VL6180X_Range_UNDERFLOW_ERR:
          errors += "RANGE ERR: RESULT__RANGE_VAL < 0!\n";
          break;
        case VL6180X_Range_OVERFLOW_ERR:
          errors += "RANGE ERR: RESULT__RANGE_VAL is out of range!\n";
          break;
        default:
          "RANGE ERR: System err!\n";
          break;
      }
      delay(1000);
    }
  }
}

void printBotData()
{
  Serial.printf(
    "BOT: Battery: %d%%, Safety: %d, Distance: %dmm, Weapon: %d, Wep.angle: %d, AI: %d, Invert: %d, TRIM_L: %d, TRIM_R: %d, Allowlist: %d\n",
    voltageLevel,
    safetyActive,
    range,
    weaponActive,
    angPrev,
    AIActive,
    invert,
    TRIM_L,
    TRIM_R,
    allowList
  );
}

void printPadData(ControllerPtr ctl)
{
  Serial.printf(
  "PAD: idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

void resetAllowList()
{
  uni_bt_allowlist_remove_all();
  BP32.forgetBluetoothKeys();
  allowList = false;
  uni_bt_allowlist_set_enabled(allowList);
  EEPROM_Save();
}
