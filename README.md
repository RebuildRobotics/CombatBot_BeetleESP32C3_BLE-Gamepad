****************************************
# CombatBot_BeetleESP32C3_BLE-Gamepad
****************************************

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
  - Low voltage guard (Protects battery by activating failsafe when battery is running low while keeping controller connected to bot. Failsafe will be deactivated when voltage becomes higher than shutdown level).
    (If facing problems with voltage drops and guard is stopping motors with close to charged battery try to use quality connectors like BT2.0 or XT-30 not JST's, proper cable sizes and possible capacitor near weapons power drain.)
  - Failsafe (Stops all functions when disconnected from controller. Same function is used with low voltage guard).
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
    - Brushless motors:   2 x PPM outputs
  - Weapon ESC or servo:  1 x PPM output
  - PPM signals are generated with servo library.
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
  - Servo/Weapon control:     ESP32Servo v.3.0.5          Install from Arduino IDE
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
  CONTROLLER:
  *****
  - Script uses Bluepad32 library/board manager to handle bluetooth connection to controller.
  - Developed and tested with Googles Stadia gamepad but it should work with following gamepads supporting BLE (Test with precaution!):
    Xbox Wireless, Xbox Adaptive, Steam, Stadia and Android. See more from: https://github.com/ricardoquesada/bluepad32/blob/main/docs/supported_gamepads.md
  - Use DEBUG to retrieve connected gamepads bluetooth address into serial monitor if needed.
  - Bluetooth pairing in Stadia gamepad is done by turning controller on and pressing google button + Y button 2 seconds until light turns orange and after that static white when connected to ESP.
    After pairing device once controller will automaticly reconnect to bot when both are turned on (Atleast with Stadia it seems to take a forever or never).
  - Controls:
      - Signal lock off (Basic controls):
        - Stadia but. + Y 2s  =   Pair device (Light turns orange and after that solid white when connected.)
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
        - Y + A               =   Add connected gamepads bluetooth address to allowList
        - Y + Menu            =   Toggles activate / disable + clear allowList // If more than one pads are connected use this only once from any of the them.
  - Haptic confirmations:
    - Left side             =   Weapon on/off button or weapon pad pressed
    - Left side, short      =   Trim button pressed from left pad
    - Right side, short     =   Add to allowList or enable/disable allowList buttons pressed
    - Right side, long      =   Signal lock button pressed
    - Both sides, strong    =   Bot battery low (Vibrates only when signal lock is off and controls are used.)
  - Controller doesn't vibrate when min/max value has been reached.
  - Notice: If no other weapon control haven't been used before pressing pad then first time pressing up/down calibrates weapon angle to idle.
  
  *****
  BLUEPAD32 ALLOWLIST
  *****
  - Allowlist allows only gamepads added into it to connect. It is crucial for security because it prevents unwanted connections.
  - Connected controllers can be added to list or removed from it with buttons above in controls list.
  - List will be reseted when reuploading script into ESP and "Erase All Flash Before Sketch Upload" is enabled from tools.
    If having problems and list is not reseted from controller or from script upload go to line: "// Bluepad bluetooth allowlist" in setup() and make changes mentioned and reupload script.
  - More from allowlist:
    https://bluepad32.readthedocs.io/en/latest/FAQ/
    https://github.com/ricardoquesada/bluepad32/blob/main/src/components/bluepad32/include/bt/uni_bt_allowlist.h

  ****************************************
  Bluepad32 is separate project from this program and has nothing to do with this project except this script is using their board manager for gamepads.
  If you have need to support this project remember them before me, because without them this project wouldn't been achieved.
  ****************************************

  *****
  SAFETY NOTICE:
  *****
  - Combat robotics is fun and extremely educating but dangerous hobby, always think safety first!
  - Script includes failsafe function to prevent up accidents. Function shuts down drive- and weapon motors when controllers signal is lost.
    This is same kind of option than good RC receivers have and it's required in combat robotics.
  - Script is programmed to accept controller signals only when it is approved from controllers Signal lock button.
  - Remember to set pins correctly, if they are not set right script and failsafe won't work properly!
  - Do not keep battery connected at the same time when USB is connected into powersource!
  - Accidentally motor spins will happen when ESP is powered, script is uploading or wrong board or library version is installed!
    Motor spins at startup can be prevented by using pull-up/pull down resistors or our designed ESP32-C3 CombatBot Expansion Board and tested IDE library versions.
  - Configure ESC properly before connecting controller into bot, or it causes serious hazard!
  - Test bot with precaution and only weapon motor attached, not weapon itself!
  - Do not use any other board manager or library versions than those which are mentioned as tested and safe ones in this scripts read me area!
    It might cause serious injuries because board functionalities are changing. Future updates to this script includes fresher and tested information from later versions.
  - Remember always check that you have correct versions installed before updating script into board.
  - Modify script only if you know what you are doing!
  - Script has been tested only in close range at Combat arenas. When driving outside use precaution.
