# SerialDebug Library for Arduino

<a>![build badge](https://img.shields.io/badge/version-v0.9.2-blue.svg)</a> [![Codacy Badge](https://api.codacy.com/project/badge/Grade/5ddb5c53fa29416eb1d1eaaf6f201ec6)](https://app.codacy.com/app/JoaoLopesF/SerialDebug?utm_source=github.com&utm_medium=referral&utm_content=JoaoLopesF/SerialDebug&utm_campaign=Badge_Grade_Settings) 
<a>![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)</a>


Improved serial debugging to Arduino, with simple software debugger,
to see/change global variables, to add watch for these variables,
or call a function, in runtime, using serial monitor.

## Contents
 - [About](#about)
 - [Beta version](#beta-version)
 - [Github](#github)
 - [Benefits](#benefits)
 - [Commands](#commands)
 - [How it looks](#how-it-looks-1)
 - [Install](#install)
 - [Usage](#usage)
 - [Watches](#watches)
 - [Release](#releases)
 - [Resources](#resources)
 - [Thanks](#thanks)

## About

Generally debugs messages in Arduino is done by Serial.print*,
and show in this monitor serial, or another serial tool, as screen, pyserial, coolterm, etc.

The Arduino official IDE, not have debugger.
Only way, until now, is using a another IDE, that can be paid, and a hardware debugger, as JTAG.

__SerialDebug__ is a library to Improved serial debugging and simple software debugger to Arduino.

Yes, now we can use one debugger, simple but functional,
and not need a extra hardware to do it.

## Beta version

This is a beta version. 
Not yet fully tested, optimized, and documented.

This is a previous documentation.
It will be better documented before first RC version.

## Github 

Contribute to this libray development by creating an account on GitHub.

Please give a star, if you find this library usefull, 
this help a another people, discover it too.

Please add a issue for problems or suggestion.

I suggest you use a Github Desktop New app to clone, 
it help to keep updated.

## Benefits

__SerialDebug__ is bether than Arduino default serial debugging:

  - This is __optimized__

        The initial status of SerialDebug is inactive,
        where no normal debug outputs, and no CPU waste time for debugs.
        Well, there may not be anyone seeing it.
        It is good for not always USB connected project,
        as one powered by battery or external power supply.

        Only messages that are processed and displayed,
        are of type Error or Always (important ones).
        
        After first command received, SerialDebug will become active

        For boards with Serial.printf native, as ESP8266 and ESP32,
        all routines to show debug is a C/C++ precompiler macros, 
        so no extra functions calls, only Serial.print*. 

        For simple software debugger, have memory optimizations:

          - No fixed arrays, is used C++ Vector to dynamic arrays

          - Is used void* pointer to store values, when it is need.
            Is more complicate, but it dramatically reduces use of memory, 
            compared to store 17 variables for support 17 kinds of this.

        In future versions will more otimized, for CPU and memory

  - It is good for __any__ Arduino

        Is good for new boards, that have good CPU and memory,
        like Espressif (ESP8266 and ESP32) and ARM arch (Due, Teensy, etc.).

        But it runs in older Arduino, as UNO, Leonardo, Mega, ...
        In UNO or similar some features as Watches in debugger is not implemented,
        due full library is huge for it (more than 7k lines of code).
        For the Mega, some features are reduced, but have watches.
        
        If debugger is disabled, SerialDebug in UNO,
        consumes only about 170 bytes of memory.
        And it not fully otimized yet.

        The default speed of serial is 250000, for Espressif, ARM or Mega boards
        and 115200 for UNO, Leonardo, etc.

        Only exception is boards with Tiny* AVR MCU, 
        due it not have CPU and memory to this library.

  - Have __debug levels__ 
    
        During the development, we can put a lot of debug messages...

        But with SerialDebug, we can put a level in each one.

        For all messages (except any (debug\*A) or error (debug\*E)),
        the message only is processed and showed,
        if debug level is equal or higher than it level

        SerialDebug have 7 debug levels, in order of priority:

          Alway showed:
              Error:    Critical errors
              Always:   Important messages 

          No debug:
              None:     No debug output 

          Another levels (showed if level is equal or higher that actual one):
              Warning:  Error conditions but not critical 
              Info:     Information messages 
              Debug:    Extra information 
              Verbose:  More information than the usual  

        So We can change the level to Verbose, to see all messages. 
        Or to Debug to see only debug or higher level, etc.

        Is very good to reduce a quantity of messages that a project can generate,
        in serial monitor.

  - Have __printf__ support to serial

        Regardless of whether the board has it native or not.
        That I know, only Espressif boards have it native

        For example:

          	Serial.print("*** Example - varA = ");
            Serial.print(varA);
            Serial.print(" varB =  ");
            Serial.print(varB);
            Serial.print(" varC =  ");
            Serial.print(varC);
            Serial.println(); 

        Can be converted to a single command:

            debugD("*** Example - varA = %d varB = %d varC = %d", varA, varB, varC);

        In future we will have a converter to migrate old codes to __SerialDebug__

        Note: __SerialDebug__ follows the same concept, 
        that modern debug/logging messages model,
        as ESP-IDF, Android, iOS, etc.
        In these, each call generates a formatted output line.

  - Have __auto__ function name and simple __profiler__

        A simple debug:

          debugV("* Run time: %02u:%02u:%02u (VERBOSE)", mRunHours, mRunMinutes, mRunSeconds);

        Can generate this output in serial monitor:

          (V p:^3065)(loop)(C1) * Run time: 00:41:23 (VERBOSE)

        Where:  V: is the level
                p: is a profiler time, elased, between this and previous debug
                (loop): is a function name, that executed this debug
                (C1): is a core that executed this debug (and a function of this) (only for ESP32)
                The remaining is the message formatted (printf)

        Note how printf is powerfull, %02u means a unsigned integer with minimum lenght of 2,
        and leading by zeros

        For ESP32, the core id in each debug is very good to optimizer multicore programming.

  - Have __commands__ to execute from serial monitor

        __SerialDebug__ takes care of inputs from serial, and process predefined commands

        For example:

           Change the level of debug, to show less or more messages.
           See memory
           Reset the board

        See about __SerialDebug__ commands below.

  - Have a simple __software debugger__ 

        If enabled, you can command in serial monitor:

          - Show and change values of global variables
          - Call a function
          - Add or change watches for global variables

        It not have some features than a real hardware debugger,
        but is good features, for when yet have none of this ...

        See about this below.

  - Ready for __production__ (release compiler))

        For release your device, just uncomment DEBUG_DISABLED in your project
        Done this, and no more serial messages, or debug things. A
        And better for DEBUG_DISABLED, __SerialDebug__ have ZERO overhead, 
        due is nothing of this is compiled

## Commands

__SerialDebug__ takes care of inputs from serial, and process predefined commands as:

      ? or help -> display these help of commands
      m -> show free memory
      n -> set debug level to none
      v -> set debug level to verbose
      d -> set debug level to debug
      i -> set debug level to info
      w -> set debug level to warning
      e -> set debug level to errors
      s -> silence (Not to show anything else, good for analysis)
      p -> profiler:
        p      -> show time between actual and last message (in millis)
        p min  -> show only if time is this minimal
      r -> repeat last command (in each debugHandle)
        r ? -> to show more help 
      reset -> reset the Arduino board
    
      Only if debugger is enabled: 
        f -> call the function
            f ?  -> to show more help 
        g -> see/change global variables
            g ?  -> to show more help 
        wa -> see/change watches for global variables
            wa ?  -> to show more help 
    
      Not yet implemented:
        gpio -> see/control gpio

For debugger:

    - For functions:

      - To show help: f ?
      - To show: f [name|num]
      - To search with start of name (case insensitive): f name
      - To call it, just command: f [name|number] [arg]

    - For global variables:

      - To show help: g ?
      - To show: g [name|num]
      - To search by start of name: g name
      - To change global variable, g [name|number] = value [y]
        note: y is to confirm it (without confirm message)

    - For watches:

      - To show help: wa ?
      - To show: wa [num]
      - To add: wa a {global [name|number]} [==|!=|<|>|<=|>=|change] [value] [as]
        notes: change not need a value, and as -> set watch to stop always
      - To add cross (2 globals): wa ac {global [name|number]} [=|!=|<|>|<=|>=] {global [name|number]} [as]
      - To change: wa u {global [name|number]} [==|!=|<|>|<=|>=|change] [value] [as]
      - To change cross (not yet implemented)
      - To disable: wa d [num|all]
      - To enable: wa e [num|all]
      - To nonstop on watches: wa ns
      - To stop on watches: wa s



Notes: 
  - watches is not for low memory boards, as Uno.
  - memory and reset, yet implemented only to AVR, Espressif, Teensy, and Arm (in test).
  
## How it looks

[![IMAGE ALT TEXT HERE](https://www.youtube.com/watch?v=EfvF55Ww-lU/0.jpg)](https://www.youtube.com/watch?v=EfvF55Ww-lU)


## Install 

Just download or clone this repository.

For install help, please see: https://www.arduino.cc/en/Guide/Libraries

Note: In some boards, after upload if you see only dirty characteres in serial monitor,
please reset the board. There is possibly some glitch in the serial monitor of Arduino

## Usage

    Please open the examples to see it working:

      - Basic -> for basic usage, without debugger

      - Advanced/Avr -> for Arduino AVR arch. uses F() to save memory

      - Advanded/Others -> for new boards, with enough memory,
                          not use F(), due RAM is more faster than Flash memory

      - Disabled -> example of how disable features, or entire SerialDebug

### include

Place it, in top of code:

```cpp
#include "SerialDebug.h" //https://github.com/JoaoLopesF/SerialDebug
   
```

### setup

    Setup code is only necessary for debugger elements.
    As this library not uses a hardware debugger,
    this codes are necessary to add this elements,
    into "simple software debugger" of SerialDebug. 

For example, for functions:

````cpp
// Add functions that can called from SerialDebug

if (debugAddFunctionVoid("benchInt", &benchInt) >= 0) {
  debugSetLastFunctionDescription("To run a benchmark of integers");
}
if (debugAddFunctionVoid("benchFloat", &benchFloat) >= 0) {
  debugSetLastFunctionDescription("To run a benchmark of float");
}
if (debugAddFunctionVoid("benchGpio", &benchGpio) >= 0) {
  debugSetLastFunctionDescription("To run a benchmark of Gpio operations");
}
if (debugAddFunctionVoid("benchAll", &benchAll) >= 0) {
  debugSetLastFunctionDescription("To run all benchmarks");
}

if (debugAddFunctionVoid("benchSerial", &benchSerial) >= 0) {
  debugSetLastFunctionDescription("To benchmarks standard Serial debug");
}
if (debugAddFunctionVoid("benchSerialDebug", &benchSerialDebug) >= 0) {
  debugSetLastFunctionDescription("To benchmarks SerialDebug");
}

if (debugAddFunctionStr("funcArgStr", &funcArgStr) >= 0) {
  debugSetLastFunctionDescription("To run with String arg");
}
if (debugAddFunctionChar("funcArgChar", &funcArgChar) >= 0) {
  debugSetLastFunctionDescription("To run with Character arg");
}
if (debugAddFunctionInt("funcArgInt", &funcArgInt) >= 0) {
  debugSetLastFunctionDescription("To run with Integer arg");
}
`````

Or short use, if you not want descriptions:

````cpp
// Add functions that can called from SerialDebug

debugAddFunctionVoid("benchInt", &benchInt);
debugAddFunctionVoid("benchFloat", &benchFloat);
debugAddFunctionVoid("benchGpio", &benchGpio);
debugAddFunctionVoid("benchAll", &benchAll);
debugAddFunctionVoid("benchSerial", &benchSerial);
debugAddFunctionVoid("benchSerialDebug", &benchSerialDebug);
debugAddFunctionStr("funcArgStr", &funcArgStr);
debugAddFunctionChar("funcArgChar", &funcArgChar);
debugAddFunctionInt("funcArgInt", &funcArgInt);
debugSetLastFunctionDescription("To run with Integer arg");
`````

Note: If it is for old boards, as UNO, Leornardo, etc.
You must use F() to save memory:

````cpp
// Add functions that can called from SerialDebug

debugAddFunctionVoid(F("benchInt"), &benchInt);
````

Notes: It is too for all examples showed below

- For global variables (note: only global ones):

````cpp

// Add global variables that can showed/changed from SerialDebug
// Note: Only globlal, if pass local for SerialDebug, can be dangerous

if (debugAddGlobalUInt8_t("mRunSeconds", &mRunSeconds) >= 0) {
  debugSetLastGlobalDescription("Seconds of run time");
}
if (debugAddGlobalUInt8_t("mRunMinutes", &mRunMinutes) >= 0) {
  debugSetLastGlobalDescription("Minutes of run time");
}
if (debugAddGlobalUInt8_t("mRunHours", &mRunHours) >= 0) {
  debugSetLastGlobalDescription("Hours of run time");
}

// Note: easy way, no descriptions ....

debugAddGlobalBoolean("mBoolean", 	&mBoolean);
debugAddGlobalChar("mChar", 		&mChar);
debugAddGlobalByte("mByte", 		&mByte);
debugAddGlobalInt("mInt", 			&mInt);
debugAddGlobalUInt("mUInt", 		&mUInt);
debugAddGlobalLong("mLong", 		&mLong);
debugAddGlobalULong("mULong", 		&mULong);
debugAddGlobalFloat("mFloat", 		&mFloat);
debugAddGlobalDouble("mDouble", 	&mDouble);

debugAddGlobalString("mString", 	&mString);

// Note: For char arrays, not use the '&'

debugAddGlobalCharArray("mCharArray", mCharArray);

// Note, here inform to show only 20 characteres of this string or char array

debugAddGlobalString("mStringLarge", &mStringLarge, 20);

debugAddGlobalCharArray("mCharArrayLarge",
                  mCharArrayLarge, 20);

// For arrays, need add for each item (not use loop for it, due the name can not by a variable)
// Notes: Is good added arrays in last order, to help see another variables
//        In next versions, we can have a helper to do it in one command

debugAddGlobalInt("mIntArray[0]", 	&mIntArray[0]);
debugAddGlobalInt("mIntArray[1]", 	&mIntArray[1]);
debugAddGlobalInt("mIntArray[2]", 	&mIntArray[2]);
debugAddGlobalInt("mIntArray[3]",	&mIntArray[3]);
debugAddGlobalInt("mIntArray[4]",	&mIntArray[4]);

````

And for watches (not for low memory boards, as UNO):

````cpp
// Add watches for some global variables
// Note: watches can be added/changed in serial monitor too

// Watch -> mBoolean when changed (put 0 on value)

debugAddWatchBoolean("mBoolean", DEBUG_WATCH_CHANGED, 0);

// Watch -> mRunSeconds == 10

debugAddWatchUInt8_t("mRunSeconds", DEBUG_WATCH_EQUAL, 10);

// Watch -> mRunMinutes > 3

debugAddWatchUInt8_t("mRunMinutes", DEBUG_WATCH_GREAT, 3);

// Watch -> mRunMinutes == mRunSeconds (just for test)

debugAddWatchCross("mRunMinutes", DEBUG_WATCH_EQUAL, "mRunSeconds");
````

### loop
- In the begin of loop function

```cpp
	// SerialDebug handle
	// NOTE: if in inactive mode (until receive anything from serial),
	// it show only messages of always or errors level type
	// And the overhead during inactive mode is very much low

	debugHandle();

```
### How use __SerialDebug__ macros

Instead _Serial.print*_, use __debug*__ macros:

See example of how convert it, in [Benefits](#benefits) topic below.

Using macros to show debug:
 
  - For always show (for example in setup, here the debug output is disabled) 

```` cpp
debugA("**** Setup: initialized.");
````

  - For errors: 

````cpp
debugE("* This is a message of debug level ERROR");
`````

  - For another levels:

````cpp
debugV("* This is a message of debug level VERBOSE");
debugD("* This is a message of debug level DEBUG");
debugI("* This is a message of debug level INFO");
debugW("* This is a message of debug level WARNING");
````

### printf formatting

__SerialDebug__ use prinf native (for Espressif boards), 
or implements it in _depugPrintf_ function.

For Example:

```cpp
debugA("This is a always - var %02d", var);
debugV("This is a verbose - var %02d", var);
debugD("This is a debug - var %02d", var);
debugI("This is a information - var %02d", var);
debugW("This is a warning - var %02d", var);
debugE("This is a error - var %02d", var);
```

See more about printf formatting: http://www.cplusplus.com/reference/cstdio/printf/

Notes:
- __SeriaDebug__ use the standard printf of Arduino

  - Some features can be not implemented, depending of board arch.

  - For String variables, you must use the c_str() method:

````cpp
debugA("*** called with arg.: %s", str.c_str());
````

  - For AVR MCUs, as UNO, Leonardo, Mega, etc.,
    no have support to %f (format floats)

    If you need this, use: 

````cpp
#ifndef ARDUINO_ARCH_AVR // Native float printf support
	debugV("mFloat = %0.3f", mFloat);
#else // For AVR, it is not supported, using String instead
	debugV("mFloat = %s", String(mFloat).c_str());
#endif
````
    (in future versions of __SerialDebug__, can be have a better solution)

## Watches

Watches is usefull to warning when the content of global variable,
is changed or reaches a certain condition.

How this works, without a real hardware debugger? :

  - If have any watch (enabled),

  - Is verified this content is changed,
    
  - And is verified the watch,
    And trigger it, if value is changed, or the operation is true

This is done before each _debug*_ show messages or in _debugHandle_ function.


## Releases

#### 0.9.2 - 29/09/18

    - Few adjustments

#### 0.9.1 - 28/09/18

    - Few adjustments

#### 0.9.0 - 26/09/18

    - First beta

## Resources

If you need a remote debug for ESP8266 or ESP32, 
see my other library -> https://github.com/JoaoLopesF/RemoteDebug

## Thanks

Special thanks to:

    - Arduino, for bring open hardware to us.

    - Good people, that work hard, to bring to us excellent open source,
      as libraries, examples, etc..
      That inspire me to make new libraries, as SerialDebug, RemoteDebug, etc.
      
    - Makers people, that works together as a big family.
