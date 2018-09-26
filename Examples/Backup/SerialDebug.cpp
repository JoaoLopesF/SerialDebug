/*****************************************
 * Library   : SerialDebug - Improved and lightweight serial debugging to Arduino
 * Programmer: Joao Lopes
 * Comments  : Tips on gcc logging seen at http://www.valvers.com/programming/c/logging-with-gcc/
 *             For remote debugging -> RemodeDebug: https://github.com/JoaoLopesF/SerialDebug
 * 			   Based on RemoteDebug library and ESP-IDF logging debug levels
 * Versions  :
 * ------ 	-------- 	-------------------------
 * 0.9.0  	26/08/18	First beta
 *****************************************/

/*
 * Source for SerialDebug
 *
 * Copyright (C) 2018  Joao Lopes https://github.com/JoaoLopesF/SerialDebug
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This file contains the code for SerialDebug.
 *
 */

/*
 * TODO list:
 * - see all warnings
 * - more optimizations
 */

/////// Includes

#include <Arduino.h>

// Utilities

#include "Util/Util.h"
#include <Util/Fields.h>

// This project

#include "SerialDebug.h"

// Vector (to reduce memory and no fixed limit)

#ifndef DEBUG_DISABLE_DEBUGGER

// Arduino arch have C++ std vector ?

#if defined ESP8266 || defined ESP32 || defined _GLIBCXX_VECTOR

	#define VECTOR_STD true

	// C++ std vector

	#include <vector>

	using namespace std;

#else // This Arduino arch is not compatible with std::vector

	// Using a lightweight Arduino_Vector library: https://github.com/zacsketches/Arduino_Vector/

	#include "Util/Vector.h"

#endif

#endif // DEBUG_DISABLE_DEBUGGER

// ESP8266 SDK

#if defined ESP8266
	extern "C" {
		bool system_update_cpu_freq(uint8_t freq);
	}
#endif

/////// Not for production (releases)

#ifndef DEBUG_DISABLED

// Version

#define DEBUG_VERSION "0.9.0"                   	// Version of this library

/////// Variables - public

boolean _debugActive = false;		        		// Debug is only active after receive first data from Serial
uint8_t _debugLevel = DEBUG_LEVEL_NONE;         	// Current level of debug (init as disabled)

bool _debugSilence = false;							// Silent mode ?

bool _debugShowProfiler = true;						// Show profiler time ?
uint16_t _debugMinTimeShowProfiler = 0;				// Minimum time to show profiler
unsigned long _debugLastTime = millis(); 			// Last time show a debug

char _debugShowISR = ' ';							// Can show ISR (only if in 115200 bps)

/////// Variables - private

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

	// Type of global or function arg

	typedef enum {
		DEBUG_TYPE_BOOLEAN,									// Basic types
		DEBUG_TYPE_CHAR,
		DEBUG_TYPE_BYTE,
		DEBUG_TYPE_INT,
		DEBUG_TYPE_U_INT,
		DEBUG_TYPE_LONG,
		DEBUG_TYPE_U_LONG,
		DEBUG_TYPE_FLOAT,
		DEBUG_TYPE_DOUBLE,

		DEBUG_TYPE_INT8_T,									// Integers size _t
		DEBUG_TYPE_INT16_T,
		DEBUG_TYPE_INT32_T,
	//#ifdef ESP32
	//		DEBUG_TYPE_INT64_T,
	//#endif
		DEBUG_TYPE_UINT8_T,									// Unsigned integers size _t
		DEBUG_TYPE_UINT16_T,
		DEBUG_TYPE_UINT32_T,
	//#ifdef ESP32
	//		DEBUG_TYPE_UINT64_T
	//#endif

		DEBUG_TYPE_CHAR_ARRAY,								// Strings
		DEBUG_TYPE_STRING,

		DEBUG_TYPE_FUNCTION_VOID							// For function void

	} debugEnumTypes_t;

	// Debug functions

	struct debugFunction_t {
		const char* name = 0;							// Name
		const char* description = 0;					// Description
		const __FlashStringHelper *nameF = 0;			// Name (in flash)
		const __FlashStringHelper *descriptionF = 0;	// Description (in flash)
		void (*callback)() = 0;							// Callbacks
		uint8_t argType = 0;							// Type of argument
	};

	#ifdef VECTOR_STD  // Arduino arch have C++ std vector

		vector<debugFunction_t> _debugFunctions;		// Vector array of functions

	#else // Using a Arduino_Vector library: https://github.com/zacsketches/Arduino_Vector/

		Vector<debugFunction_t> _debugFunctions;		// Vector array of functions

	#endif

	static uint8_t _debugFunctionsAdded = 0;			// Number of functions added

	// Debug global variables

	struct debugGlobal_t {
		const char* name = 0;							// Name
		const char* description = 0;					// Description
		const __FlashStringHelper *nameF = 0;			// Name (in flash)
		const __FlashStringHelper *descriptionF = 0;	// Description (in flash)
		uint8_t type = 0;								// Type of variable (see enum below)
		void *pointer = 0;								// Generic pointer
		uint8_t showLength = 0;							// To show only a part (strings)
		boolean changed = false;						// Value change (between 2 debugHandle call)
		void *pointerOld = 0;							// Generic pointer for old value
	};

	#ifdef VECTOR_STD  // Arduino arch have C++ std vector

		vector<debugGlobal_t> _debugGlobals;		// Vector array of globals

	#else // Using a Arduino_Vector library: https://github.com/zacsketches/Arduino_Vector/

		Vector<debugGlobal_t> _debugGlobals;		// Vector array of globals

	#endif

	static uint8_t _debugGlobalsAdded = 0;				// Number of globals added

	typedef enum {
			DEBUG_SHOW_GLOBAL, 							// For globals
			DEBUG_SHOW_GLOBAL_WATCH						// For watches
	} debugEnumShowGlobais_t;


	struct debugWatch_t {
		uint8_t globalNum;								// Global id
		void *pointerOldvalue = 0;						// Generic pointer to old value of global
		boolean valueChanged = false;					// Value has changed ?
		uint8_t operation;								// Operation
		void *pointer = 0;								// Generic pointer to value (to do operation)
		uint8_t globalNumCross = 0;						// To watch cross - compare two globals
		boolean enabled = true;							// Enabled ?
		boolean alwaysStop = false;						// Always stop, even _debugWatchStop = false
	};

	#ifdef VECTOR_STD  // Arduino arch have C++ std vector

		vector<debugWatch_t> _debugWatches;		// Vector array of watches

	#else // Using a Arduino_Vector library: https://github.com/zacsketches/Arduino_Vector/

		Vector<debugWatch_t> _debugWatches;		// Vector array of watches

	#endif

	static uint8_t _debugWatchesAdded = 0;				// Number of watches added
	static boolean _debugWathesEnabled = false;			// Watches is enabled (only after add any)?
	static boolean _debugWatchStop = true;				// Causes a stop in debug for any positive watches ?

	// To show found name in Flash - to F()

	static String _debugLastNameF = "";					// Used store last name found (flash) - function or global

	// To process debug break response

	static String _debugBreakResponse = "";

	// To show help (uses PROGMEM)
	// Note: Using PROGMEM in large string (even for Espressif boards)

	static const char debugHelp[] PROGMEM = \
	"\
	*\r\n\
	* SerialDebug commands:\r\n\
		? or help -> display these help of commands\r\n\
		m -> Show free memory\r\n\
		n -> set debug level to none\r\n\
		v -> set debug level to verbose\r\n\
		d -> set debug level to debug\r\n\
		i -> set debug level to info\r\n\
		w -> set debug level to warning\r\n\
		e -> set debug level to errors\r\n\
		s -> silence (Not to show anything else, good for analysis)\r\n\
		profiler:\r\n\
			p      -> show time between actual and last message (in millis)\r\n\
			p min  -> show only if time is this minimal\r\n\
		filter:\r\n\
			filter <string> -> show only debugs with this\r\n\
			nofilter        -> disable the filter\r\n\
		f -> call the function\r\n\
			f ?  -> to show more help \r\n\
		g -> see/change global variables\r\n\
			g ?  -> to show more help \r\n\
		wa -> see/change watches for global variables\r\n\
			wa ?  -> to show more help \r\n\
		r -> repeat command (in eaxh debugHandle)\r\n\
			r ? -> to show more help \r\n\
		reset  -> Reset the Arduino board\r\n\
	";

	static String _debugLastCommand = "";		    	// Last Command received

	static String _debugFilter = "";				    // Filter
	static boolean _debugFilterActive = false;

	/////// Prototypes - private

	static void processCommand();
	static void showHelp();

	// For functions

	static int8_t debugAddFunction(const char* name, uint8_t argType);
	static int8_t debugAddFunction(const __FlashStringHelper *name, uint8_t argType);

	static void processFunctions(String& options);
	static int8_t showFunctions(String& options, boolean one);
	static void callFunction(String& options);

	// For globais

	static int8_t debugAddGlobal(const char* name,  void* pointer, uint8_t type, uint8_t showLength);
	static int8_t debugAddGlobal(const __FlashStringHelper* name, void* pointer, uint8_t type, uint8_t showLength);

	static void processGlobals(String& options);
	static int8_t showGlobals(String& options, boolean one);
	static boolean showGlobal(uint8_t globalNum, debugEnumShowGlobais_t mode, boolean getLastNameF);
	static void changeGlobal(Fields& fields);

	static boolean debugFindGlobal (const char* globalName, uint8_t* globalNum);

	// For void* pointer values

	static void getStrValue(uint8_t type, void* pointer, uint8_t showLength, String& response, String& responseType);

	static boolean apllyOperation(uint8_t type, void* pointer1, uint8_t operation, void* pointer2);

	// For watches

	static int8_t debugAddWatch(uint8_t globalNum, uint8_t operation, boolean allwaysStop = false);

	static void processWatches(String& options);
	static int8_t showWatches(String& options);
	static boolean showWatch(uint8_t watchNum);
	static void changeWatches(Fields& fields);


	// Only for AVR boards

	#ifdef ARDUINO_ARCH_AVR

		static int freeMemory(); // Based on MemoryFree library: https://github.com/maniacbug/MemoryFree

		static void(* resetArduino) (void) = 0;	// Based on https://www.instructables.com/id/two-ways-to-reset-arduino-in-software/

	#endif

#endif // DEBUG_DISABLE_DEBUGGER

/////// Defines (private)

#ifdef DEBUG_NOT_USE_FLASH_F  // Not using flash to string and functions (good for board with memory  - more faster)
	#undef F
 	#define F(str) str
#endif

// Internal printf (no time, auto func, etc, just print it)

#ifdef DEBUG_USE_NATIVE_PRINTF // For Espressif boards, have Serial.printf

	#define PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)

#else // Use debugPrintf

	#define PRINTF(fmt, ...) debugPrintf(' ', 0, fmt, ##__VA_ARGS__)

#endif

/////// Methods 

// Note: SerialDebug not is a C++ object class, as RemoteDebug
//       due all show debug is precompiler macros, 
//       for more performance.

////// Handles

// SerialDebug handle for debug deactivate
// Only activate it where receive first data from serial
// to show periodic messages and activate the debugs at first data is arrivved
// And only is call by debugHandler macro if debugs still deactive

void debugHandleInactive() {

    static uint8_t execCount = 5; // Count debugHandleInactive calls

    // Test if first data is arrived, to enable debugs

    if (Serial.available() > 0) {

        _debugActive = true;

        Serial.println(F("* SerialDebug: Debug now is active."));

        // Show helps
        // Uncomment this if You want the help here
        //showHelp();

        // Go to initial level

        debugSetLevel (DEBUG_INITIAL_LEVEL);

        return;
    }

    // Show message initial (only after n executions, due where board is booting, some messages can be lost

	if (execCount > 0) { // To count executions. You can change it

    	execCount--;

    	if (execCount == 0) {

#ifndef DEBUG_DISABLE_DEBUGGER
			Serial.println(F("* SerialDebug: Please press any key and enter to activate debugs"));
			Serial.println(F("(Please verify if is in Newline mode in monitor serial)"));
#else
			Serial.println(F("* SerialDebug: Please press any key and enter to activate debugs"));
#endif
		}
    }
}

// Set actual levef of debug

void debugSetLevel(uint8_t level) {

	if (level < DEBUG_LEVELS_SIZE) {
		_debugLevel = level;
		Serial.print(F("* SerialDebug: Debug level set to "));
		switch (_debugLevel) {
		case DEBUG_LEVEL_NONE:
			Serial.println(F("None"));
			break;
		case DEBUG_LEVEL_ERROR:
			Serial.println(F("Error"));
			break;
		case DEBUG_LEVEL_WARN:
			Serial.println(F("Warning"));
			break;	\
		case DEBUG_LEVEL_INFO:
			Serial.println(F("Information"));
			break;	\
		case DEBUG_LEVEL_DEBUG:
			Serial.println(F("Debug"));
			break;
		case DEBUG_LEVEL_VERBOSE:
			Serial.println(F("Verbose"));
			break;
		}
	}
}

// Filter

void debugSetFilter(String& filter) {

	_debugFilter = filter;
	_debugFilter.toLowerCase(); // TODO: option to case insensitive ?
	_debugFilterActive = true;

	PRINTF(F("* SerialDebug: Filter active: %s\r\n"), _debugFilter.c_str());
}

void debugSetNoFilter() {

	_debugFilter = "";
	_debugFilterActive = false;

	Serial.println(F("* SerialDebug: Filter disabled"));

}

// Profiler

void debugShowProfiler(boolean activate, uint16_t minTime, boolean showMessage) {

	_debugShowProfiler = activate;

	if (activate) {

		_debugMinTimeShowProfiler = minTime;

		if (showMessage) {

			if (minTime == 0) {
				Serial.println(F("* SerialDebug: Show profiler: On (without minimal time)\r\n"));

			} else {

				PRINTF(F("* SerialDebug: Show profiler: On (with minimal time: %u\r\n"), _debugMinTimeShowProfiler);

			}
		}

	} else {

		_debugMinTimeShowProfiler = 0;

		if (showMessage) {

			Serial.println(F("* SerialDebug: Show profiler: Off\r\n"));
		}

	}
}

// Silence

void debugSilence(boolean activate, boolean showMessage) {

	_debugSilence = activate;

	if (_debugSilence && showMessage) {

		Serial.println(F("* SerialDebug: Debug now is in silent mode"));
		Serial.println(F("* SerialDebug: Press enter or another command to return show debugs"));

	} else if (!_debugSilence && showMessage) {

		Serial.println(F("* SerialDebug: Debug now exit from silent mode"));
	}

}

// Debug printf (used for not Espressif boards (that have it) or to use flash strings
// Based on Arduino Espressif Print.printf (thanks a lot)

#ifndef DEBUG_USE_NATIVE_PRINTF // Not for native Serial.printf

void debugPrintf(const char level, const char* function, const char* format, ...) {

	// Buffer

	const size_t bufSize = 64; 	// Size of buffer
	char buffer[bufSize];		// Buffer

	// Process the initial format of SerialDebug

	if (level != ' ') { // ' ' is used internally in SerialDebug to do printf, if Arduino no have, or for flash F support

		if (function) { // Auto function

			if (_debugShowProfiler) {
				snprintf(buffer, bufSize,
						"(%c p:^%04lu)" DEBUG_TAG_FMT "(%s) ",
						level, (millis() - _debugLastTime),
						DEBUG_TAG, function);
				_debugLastTime = millis();
			} else {
				snprintf(buffer, bufSize,
						"(%c) (%lu)" DEBUG_TAG_FMT "(%s) ",
						level, millis(),
						DEBUG_TAG, function);
			}

		} else { // No auto function

			if (_debugShowProfiler) {
				snprintf(buffer, bufSize,
						"(%c p:^%04lu)" DEBUG_TAG_FMT,
						level, (millis() - _debugLastTime),
						DEBUG_TAG);
				_debugLastTime = millis();
			} else {
				snprintf(buffer, bufSize,
						"(%c) (%lu)" DEBUG_TAG_FMT,
						level, millis(),
						DEBUG_TAG);
			}
		}

		// Send it to serial

		Serial.print(buffer);
	}

	// Process the var arg to process the custom format

	va_list arg;
	va_list copy;
	char* temp = buffer;

	va_start(arg, format);
	va_copy(copy, arg);
	size_t len = vsnprintf(NULL, 0, format, arg);
	va_end(copy);
	if(len >= bufSize){
	    temp = new char[len+1];
	    if(temp == NULL) {
	        return;
	    }
	}
	len = vsnprintf(temp, len+1, format, arg);

	// Send it to serial

	Serial.write((uint8_t*)temp, len);

	// Clean

	va_end(arg);
	if(len > bufSize){
	    delete[] temp;
	}
	return;
}

void debugPrintf(const char level, const char* function, const __FlashStringHelper *format, ...) {

	// For Flash string variables
	// Based on Arduino String.concat - see in https://gist.github.com/sticilface/e54016485fcccd10950e93ddcd4461a3

	// Variables

	const size_t bufInitSize = 64; 	// Initial size of buffers

	char flash[bufInitSize];		// Buffer to get flash content
	size_t flashSize = bufInitSize; // Size of this buffer
	char* pFlash = flash;			// This buffer pointer

	char buffer[bufInitSize];		// Buffer to printf
	size_t bufferSize = bufInitSize;// Size of buffer
	char* pBuffer = buffer;			// Buffer pointer

	size_t written = 0;				// Bytes written to buffer

	// Process the initial format of SerialDebug

	if (level != ' ') { // ' ' is used internally in SerialDebug to do printf, if Arduino no have, or for flash F support

		memset(buffer, '\0', bufferSize);

		if (function) { // Auto function

			if (_debugShowProfiler) {
				 written = snprintf(buffer, bufferSize,
						"(%c p:^%04lu)" DEBUG_TAG_FMT "(%s) ",
						level, (millis() - _debugLastTime),
						DEBUG_TAG, function);
				_debugLastTime = millis();
			} else {
				written = snprintf(buffer, bufferSize,
						"(%c) (%lu)" DEBUG_TAG_FMT "(%s) ",
						level, millis(),
						DEBUG_TAG, function);
			}

		} else { // No auto function

			if (_debugShowProfiler) {
				snprintf(buffer, bufferSize,
						"(%c p:^%04lu)" DEBUG_TAG_FMT,
						level, (millis() - _debugLastTime),
						DEBUG_TAG);
				_debugLastTime = millis();
			} else {
				snprintf(buffer, bufferSize,
						"(%c) (%lu)" DEBUG_TAG_FMT,
						level, millis(),
						DEBUG_TAG);
			}

		}

		// Send it to serial

		Serial.print(buffer);


	}

	// Copy Flash content to a local variable

	written = strlen_P((PGM_P)format);
	if(written >= flashSize){
	    pFlash = new char[written+1];
		flashSize = written;
	    if(pFlash == NULL) {
	        return;
	    }
	}

	memset(pFlash, '\0', flashSize);

	strncpy_P(pFlash, (PGM_P)format, written);

	// Process the var arg to process the custom format

	va_list arg;
	va_list copy;

	va_start(arg, flash);
	va_copy(copy, arg);
	written = vsnprintf(NULL, 0, flash, arg);
	va_end(copy);
	if(written >= bufferSize){
	    pBuffer = new char[written+1];
		bufferSize = written;
	    if(pBuffer == NULL) {
	        return;
	    }
	}

	memset(buffer, '\0', bufferSize);
	written = vsnprintf(pBuffer, written+1, flash, arg);

	// Send it to serial

	Serial.write((uint8_t*)pBuffer, written);

	// Clean stuff

	va_end(arg);

	if(flashSize > bufInitSize){
	    delete[] pFlash;
	}
	if(bufferSize > bufInitSize){
	    delete[] pBuffer;
	}
	return;

}

#endif // DEBUG_USE_NATIVE_PRINTF

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

// SerialDebug handle, to process data receipts

void debugHandleEvent() {

	static String command = ""; 		 	// Buffer for command receipt
	static uint32_t lastTime = millis(); 	// Time of last receipt
	static char last = ' ';					// Last char received

	// Process serial data

	while (Serial.available()) {

	    // Get the new char:

	    char character = (char)Serial.read();

	    // Clear buffer if is a long time of last receipt

	    if (command.length() > 0 && (millis() - lastTime) > 5000) {
	    	command = "";
	    }
	    lastTime = millis(); // Save it

		// Newline (CR or LF) - once one time if (\r\n)

		if (isCRLF(character) == true) {

			if (isCRLF(last) == false) {

				// Process the command

				if (command.length() > 0) { // Exist command

					_debugLastCommand = command; // Store the last command

					processCommand();

				} else if (_debugSilence) { // Exit from silence mode

					debugSilence(false, true);
				}
			}

			command = ""; // Init it for next command

		} else if (isPrintable(character)) { // Only valid

			// Concat

			command.concat(character);

		}

		// Last char

		last = character;
	}
}

// Add a function

int8_t debugAddFunctionVoid(const char* name, void (*callback)()) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_FUNCTION_VOID);

	if (pos != -1) {
		_debugFunctions[pos].callback = callback;
	}
	return pos;
}

int8_t debugAddFunctionStr(const char* name, void (*callback)(String)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_STRING);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void)) callback;
	}
	return pos;
}

int8_t debugAddFunctionChar(const char* name, void (*callback)(char)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_CHAR);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void)) callback;
	}
	return pos;
}

int8_t debugAddFunctionInt(const char* name, void (*callback)(int)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_INT);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void))callback;
	}
	return pos;
}

// For Flash F()

int8_t debugAddFunctionVoid(const __FlashStringHelper* name, void (*callback)()) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_FUNCTION_VOID);

	if (pos != -1) {
		_debugFunctions[pos].callback = callback;
	}
	return pos;
}

int8_t debugAddFunctionStr(const __FlashStringHelper* name, void (*callback)(String)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_STRING);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void)) callback;
	}
	return pos;
}

int8_t debugAddFunctionChar(const __FlashStringHelper* name, void (*callback)(char)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_CHAR);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void)) callback;
	}
	return pos;
}

int8_t debugAddFunctionInt(const __FlashStringHelper* name, void (*callback)(int)) {

	int8_t pos = debugAddFunction(name, DEBUG_TYPE_INT);

	if (pos != -1) {
		_debugFunctions[pos].callback = (void(*) (void)) callback;
	}
	return pos;
}

// Add a function description for last added

void debugSetFunctionDescription(const char *description) {

	_debugFunctions[_debugFunctionsAdded - 1].description = description;

}

void debugSetLastFunctionDescription(const __FlashStringHelper *description) {

	_debugFunctions[_debugFunctionsAdded - 1].descriptionF = description;

}

// Add a global variable

// Basic types

int8_t debugAddGlobalBoolean (const char* name, boolean* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_BOOLEAN, 0);
}
int8_t debugAddGlobalChar (const char* name, char* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR, 0);
}
int8_t debugAddGlobalByte (const char* name, byte* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_BYTE, 0);
}
int8_t debugAddGlobalInt (const char* name, int* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_INT, 0);
}
int8_t debugAddGlobalUInt (const char* name, unsigned int* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_U_INT, 0);
}
int8_t debugAddGlobalLong (const char* name, long* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_LONG, 0);
}
int8_t debugAddGlobalULong (const char* name, unsigned long* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_U_LONG, 0);
}
int8_t debugAddGlobalFloat (const char* name, float* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_FLOAT, 0);
}
int8_t debugAddGlobalDouble (const char* name, double* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_DOUBLE, 0);
}

// Integer C size t

int8_t debugAddGlobalInt8_t (const char* name, int8_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT8_T, 0);
}
int8_t debugAddGlobalInt16_t (const char* name, int16_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT16_T, 0);
}
int8_t debugAddGlobalInt32_t (const char* name, int32_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT32_T, 0);
}
//#ifdef ESP32
//int8_t debugAddGlobalInt64_t (const char* name, int64_t* pointer) {
//
//	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT64_T);
//}
//#endif
int8_t debugAddGlobalUInt8_t (const char* name, uint8_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT8_T, 0);
}
int8_t debugAddGlobalUInt16_t (const char* name, uint16_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT16_T, 0);
}
int8_t debugAddGlobalUInt32_t (const char* name, uint32_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT32_T, 0);
}
//#ifdef ESP32
//int8_t debugAddGlobalUInt64_t (const char* name, uint64_t* pointer) {
//
//	debugAddGlobal(name, pointer, DEBUG_TYPE_UINT64_T);
//}
//#endif

// Strings

int8_t debugAddGlobalCharArray (const char* name, char* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR_ARRAY, 0);
}
int8_t debugAddGlobalCharArray (const char* name, char* pointer, uint8_t showLength) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR_ARRAY, showLength);
}
int8_t debugAddGlobalString (const char* name, String* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_STRING, 0);
}
int8_t debugAddGlobalString (const char* name, String* pointer, uint8_t showLength) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_STRING, showLength);
}

// For Flash F()

// Basic types

int8_t debugAddGlobalBoolean (const __FlashStringHelper* name, boolean* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_BOOLEAN, 0);
}
int8_t debugAddGlobalChar (const __FlashStringHelper* name, char* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR, 0);
}
int8_t debugAddGlobalByte (const __FlashStringHelper* name, byte* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_BYTE, 0);
}
int8_t debugAddGlobalInt (const __FlashStringHelper* name, int* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_INT, 0);
}
int8_t debugAddGlobalUInt (const __FlashStringHelper* name, unsigned int* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_U_INT, 0);
}
int8_t debugAddGlobalLong (const __FlashStringHelper* name, long* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_LONG, 0);
}
int8_t debugAddGlobalULong (const __FlashStringHelper* name, unsigned long* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_U_LONG, 0);
}
int8_t debugAddGlobalFloat (const __FlashStringHelper* name, float* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_FLOAT, 0);
}
int8_t debugAddGlobalDouble (const __FlashStringHelper* name, double* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_DOUBLE, 0);
}

// Integer C size t

int8_t debugAddGlobalInt8_t (const __FlashStringHelper* name, int8_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT8_T, 0);
}
int8_t debugAddGlobalInt16_t (const __FlashStringHelper* name, int16_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT16_T, 0);
}
int8_t debugAddGlobalInt32_t (const __FlashStringHelper* name, int32_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT32_T, 0);
}
//#ifdef ESP32
//int8_t debugAddGlobalInt64_t (const char* name, int64_t* pointer) {
//
//	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT64_T);
//}
//#endif
int8_t debugAddGlobalUInt8_t (const __FlashStringHelper* name, uint8_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT8_T, 0);
}
int8_t debugAddGlobalUInt16_t (const __FlashStringHelper* name, uint16_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT16_T, 0);
}
int8_t debugAddGlobalUInt32_t (const __FlashStringHelper* name, uint32_t* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_UINT32_T, 0);
}
//#ifdef ESP32
//int8_t debugAddGlobalUInt64_t (const char* name, uint64_t* pointer) {
//
//	debugAddGlobal(name, pointer, DEBUG_TYPE_UINT64_T);
//}
//#endif

// Strings

int8_t debugAddGlobalCharArray (const __FlashStringHelper* name, char* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR_ARRAY, 0);
}
int8_t debugAddGlobalCharArray (const __FlashStringHelper* name, char* pointer, uint8_t showLength) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_CHAR_ARRAY, showLength);
}
int8_t debugAddGlobalString (const __FlashStringHelper* name, String* pointer) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_STRING, 0);
}
int8_t debugAddGlobalString (const __FlashStringHelper* name, String* pointer, uint8_t showLength) {

	return debugAddGlobal(name, pointer, DEBUG_TYPE_STRING, showLength);
}

// Add a description for global in last added

void debugSetGlobalDescription(const char *description) {

	_debugGlobals[_debugGlobalsAdded - 1].description = description;
}

void debugSetLastGlobalDescription(const __FlashStringHelper *description) {

	_debugGlobals[_debugGlobalsAdded - 1].descriptionF = description;
}

//// Not allowed
//
//int8_t debugAddGlobalCharArray (const char* name, const char* pointer) {
//
//	PRINTF(F("* SerialDebug: const char array is not allowed. Name=%s\r\n"), name);
//}
//int8_t debugAddGlobalCharArray (const char* name, const char* pointer, uint8_t showLength) {
//
//	PRINTF(F("* SerialDebug: const char array is not allowed. Name=%s\r\n"), name);
//}

// Add a watch

int8_t debugAddWatchBoolean (const char* globalName, uint8_t operation, boolean& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchBoolean(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}

int8_t debugAddWatchBoolean (uint8_t globalNum, uint8_t operation, boolean& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchChar (const char* globalName, uint8_t operation, char& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchChar(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchChar (uint8_t globalNum, uint8_t operation, char& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchByte (const char* globalName, uint8_t operation, byte& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchByte(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}

int8_t debugAddWatchByte (uint8_t globalNum, uint8_t operation, byte& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchInt (const char* globalName, uint8_t operation, int& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchInt(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchInt (uint8_t globalNum, uint8_t operation, int& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchUInt (const char* globalName, uint8_t operation, unsigned int& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchUInt(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchUInt (uint8_t globalNum, uint8_t operation, unsigned int& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchLong (const char* globalName, uint8_t operation, long& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchLong(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchLong (uint8_t globalNum, uint8_t operation, long& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchULong (const char* globalName, uint8_t operation, unsigned long& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchULong(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchULong (uint8_t globalNum, uint8_t operation, unsigned long& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchFloat (const char* globalName, uint8_t operation, float& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchFloat(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchFloat (uint8_t globalNum, uint8_t operation, float& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchDouble (const char* globalName, uint8_t operation, double& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchDouble(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchDouble (uint8_t globalNum, uint8_t operation, double& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchInt8_t (const char* globalName, uint8_t operation, int8_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchInt8_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchInt8_t (uint8_t globalNum, uint8_t operation, int8_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchInt16_t (const char* globalName, uint8_t operation, int16_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchInt16_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchInt16_t (uint8_t globalNum, uint8_t operation, int16_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchInt32_t (const char* globalName, uint8_t operation, int32_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchInt32_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchInt32_t (uint8_t globalNum, uint8_t operation, int32_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

//#ifdef ESP32
//int8_t debugAddWatchInt64_t (uint8_t globalNum, uint8_t operation, int64_t& pointer);
//#endif

int8_t debugAddWatchUInt8_t (const char* globalName, uint8_t operation, uint8_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchUInt8_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchUInt8_t (uint8_t globalNum, uint8_t operation, uint8_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchUInt16_t (const char* globalName, uint8_t operation, uint16_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchUInt16_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchUInt16_t (uint8_t globalNum, uint8_t operation, uint16_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchUInt32_t (const char* globalName, uint8_t operation, uint32_t& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchUInt32_t(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchUInt32_t (uint8_t globalNum, uint8_t operation, uint32_t& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

//#ifdef ESP32
//int8_t debugAddWatchUInt64_t (uint8_t globalNum, uint8_t operation, uint64_t& pointer);
//#endif

int8_t debugAddWatchCharArray (const char* globalName, uint8_t operation, char& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchCharArray(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}

int8_t debugAddWatchCharArray (uint8_t globalNum, uint8_t operation, char& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

int8_t debugAddWatchString (const char* globalName, uint8_t operation, String& pointer, boolean allwaysStop) {

	uint8_t globalNum;

	if (debugFindGlobal(globalName, &globalNum)) {
		return debugAddWatchString(globalNum, operation, pointer, allwaysStop);
	} else {
		return -1;
	}
}
int8_t debugAddWatchString (uint8_t globalNum, uint8_t operation, String& pointer, boolean allwaysStop) {

	int8_t ret = -1;

	ret = debugAddWatch(globalNum, operation, allwaysStop);

	if (ret != -1) {
		_debugWatches[ret].pointer = (void*) &pointer;
	}

	return ret;
}

// For watches cross - between 2 globals

int8_t debugAddWatchCross(uint8_t globalNum, uint8_t operation, uint8_t anotherGlobalNum, boolean allwaysStop) {

	int8_t ret = -1;

	// Validate

	if (globalNum > _debugGlobalsAdded) {

		PRINTF(F("* SerialDebug: First global number must between 1 and %u\r\n"), _debugGlobalsAdded);
		return -1;
	}

	if (anotherGlobalNum > _debugGlobalsAdded) {

		PRINTF(F("* SerialDebug: Second global number must between 1 and %u\r\n"), _debugGlobalsAdded);
		return -1;
	}

	if (globalNum == anotherGlobalNum) {

		Serial.println(F("* SerialDebug: Globals numbers (first and second) can not be equaln"));
		return -1;
	}

	// Adjust the numbers

	globalNum--;
	anotherGlobalNum--;

	// Add this

	debugWatch_t watch;

	watch.globalNum = globalNum;
	watch.operation = operation;
	watch.globalNumCross = anotherGlobalNum;
	watch.alwaysStop = allwaysStop;
	_debugWatches.push_back(watch);

	ret = _debugWatchesAdded;

	// Count it

	if (ret != -1) {

		_debugWatchesAdded++;
	}

	// Return index of this

	return ret;

}

#endif // DEBUG_DISABLE_DEBUGGER

// debugBreak - show a message and wait for response

void debugBreak(String str, uint32_t timeout) {
	debugBreak(str.c_str(), timeout);
}

void debugBreak(__FlashStringHelper *ifsh, uint32_t timeout) {
	debugBreak(String(ifsh).c_str(), timeout);

}
void debugBreak(const char* str, uint32_t timeout) {

	// Show a message

	Serial.println(str);

	// Clear response buffer

	_debugBreakResponse = "";

	// Wait for response // Note: the Arduino will wait for it, to continue runs

	uint32_t lastTime = millis(); 			// Time of last receipt
	char last = ' ';						// Last char received

	// Enter in silence (if is not yet)

	boolean oldSilence = _debugSilence;		// In silence mode ?

	if (!_debugSilence) {
		debugSilence(true, false);
	}

	// Process serial data (until timeout, if informed)

	while (timeout == 0 ||
			((millis() - lastTime) <= timeout)){

		if (Serial.available()) {

		    // Get the new char:

		    char character = (char)Serial.read();

		    // Clear buffer if is a long time of last receipt

		    if (_debugBreakResponse.length() > 0 && (millis() - lastTime) > 5000) {
		    	_debugBreakResponse = "";
		    }
		    lastTime = millis(); // Save it

			// Newline (CR or LF) - once one time if (\r\n)

			if (isCRLF(character) == true) {

				if (isCRLF(last) == false) { // New line -> return the command

					break;
				}

			} else if (isPrintable(character)) { // Only valid

				// Concat

				_debugBreakResponse.concat(character);

			}

			// Last char

			last = character;
		}

		delay(10); // Give a time
	}

	// Is in silence ? (restore it)

	if (_debugSilence && !oldSilence) {
		debugSilence(false, false);
	}

	// Response (tolower always)

	_debugBreakResponse.toLowerCase();

}

/////// Private code (Note: this not starts with debug)

// Private function used for all types of functions

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

// Only for debugger enabled

// Add function

static int8_t debugAddFunction(const char* name, uint8_t argType) {

	int8_t ret = -1;

	// Add this

	debugFunction_t function;

	function.name = name;
	function.argType = argType;

	_debugFunctions.push_back(function);

	ret = _debugFunctionsAdded;

	// Count it

	if (ret != -1) {

		_debugFunctionsAdded++;
	}

	// Return index of this

	return ret;
}

static int8_t debugAddFunction(const __FlashStringHelper* name, uint8_t argType) {

	int8_t ret = -1;

	debugFunction_t function;

	// Add this

	function.nameF = name;
	function.argType = argType;

	_debugFunctions.push_back(function);

	ret = _debugFunctionsAdded;

	// Count it

	if (ret != -1) {

		_debugFunctionsAdded++;
	}

	// Return index of this

	return ret;
}

// Add Global variable

static int8_t debugAddGlobal(const char* name, void* pointer, uint8_t type, uint8_t showLength) {

	int8_t ret = -1;

	debugGlobal_t global;

	// Add this

	global.name = name;
	global.pointer = pointer;
	global.type = type;
	global.showLength = showLength;

	_debugGlobals.push_back(global);

	ret = _debugGlobalsAdded;

	// Count it

	if (ret != -1) {

		_debugGlobalsAdded++;
	}

	// Return index of this

	return ret;
}

static int8_t debugAddGlobal(const __FlashStringHelper* name, void* pointer, uint8_t type, uint8_t showLength) {

	int8_t ret = -1;

	debugGlobal_t global;

	// Add this

	global.nameF = name;
	global.pointer = pointer;
	global.type = type;
	global.showLength = showLength;

	_debugGlobals.push_back(global);

	ret = _debugGlobalsAdded;

	// Count it

	if (ret != -1) {

		_debugGlobalsAdded++;
	}

	// Return index of this

	return ret;
}

// For watches

static int8_t debugAddWatch(uint8_t globalNum, uint8_t operation, boolean allwaysStop) {

	int8_t ret = -1;

	debugWatch_t watch;

	// Validate

	if (globalNum > _debugGlobalsAdded) {

		PRINTF(F("* SerialDebug: Global number must between 1 and %u\r\n"), _debugGlobalsAdded);
		return -1;
	}

	// Adjust the number

	globalNum--;

	// Add this

	watch.globalNum = globalNum;
	watch.operation = operation;
	watch.alwaysStop = allwaysStop;
	_debugWatches.push_back(watch);

	ret = _debugWatchesAdded;

	// Count it

	if (ret != -1) {

		_debugWatchesAdded++;
	}

	// Return index of this

	return ret;

}

// Process watches commands

static void processWatches(String& options) {

#if DEBUG_USE_FLASH_F

	__FlashStringHelper* errorSintax = F("* SerialDebug: Invalid sintax for watches. use w ? to show help");

#else

	const char* errorSintax = "* SerialDebug: Invalid sintax for watches. use w ? to show help";

#endif

	if (_debugWatchesAdded > 0) {

		// Get fields of command options

		Fields fields(options, ' ', true);

		if (fields.size() > 4) {
			Serial.println(errorSintax);
			return;
		}

		String firstOption = (fields.size() >= 1)? fields.getString(1) : "";
		firstOption.toLowerCase();

		if (firstOption.length() == 0 || firstOption == "?") {

			// Just show globals and help

			showWatches(firstOption);

		} else if (fields.size() == 1 && fields.isNum(1)) {

			// Search by number

			uint8_t watchNum = fields.getInt(1);

			showWatch(watchNum);

		} else {

			// Process commands

			if (firstOption == "a") {

				// Add

				if (fields.size() < 3) {
					Serial.println(errorSintax);
					return;
				}

				// Add watch

			} if (firstOption == "ac") {

				// Add cross watch

				if (fields.size() != 4) {
					Serial.println(errorSintax);
					return;
				}

				// Add watch

			} if (firstOption == "d") {

				// Disable watch

				if (fields.size() != 3) {
					Serial.println(errorSintax);
					return;
				}

				// Disable watch

			} if (firstOption == "e") {

				// enable watch

				if (fields.size() != 3) {
					Serial.println(errorSintax);
					return;
				}

				// Disable watch

			} if (firstOption == "ns" || firstOption == "nonstop") {

				// Nonstop watch

				if (fields.size() != 2) {
					Serial.println(errorSintax);
					return;
				}

				// Set to nonstop for watches

			} if (firstOption == "s" || firstOption == "stop") {

				// stop for watches

				if (fields.size() != 2) {
					Serial.println(errorSintax);
					return;
				}

				// Set to nonstop watch

			} else {

				Serial.println(errorSintax);
			}
		}

	} else {

		Serial.println(F("* SerialDebug: Watches not added to SerialDebug in your project"));
		Serial.println(F("* SerialDebug: See how do it in advanced example"));
	}
}

// Show list of watches for globals available
// Used to search and show all (return total showed)
// Or to search and show one (return number showed)

static int8_t showWatches(String& options) {

	// Show all ?

	boolean showAll = false;
	int8_t byNumber = -1;

	// Searching ?

	if (options.length() == 0) {

		showAll = true;

	} else {

		// Is integer ?

		if (strIsNum(options)) {

			uint8_t num = options.toInt();

//				PRINTF("byNumber %s = %d\r\n", options.c_str(), num);

			if (num > 0) { // Find by number, else by exact name

				byNumber = num;
				showAll = false;

			} else {

				Serial.println(F("* SerialDebug: option not is a number valid (>0)"));
				return -1;

			}

		} else {

			Serial.println(F("* SerialDebug: option not is a number"));
			return -1;
		}

	}

	// Show global(s)

	if (showAll) {
		PRINTF(F("\r\n* SerialDebug: Showing all watches for global variables (%u):\r\n\r\n"), _debugWatchesAdded);
	} else {
		Serial.println(F("\r\n* SerialDebug: Searching and showing watch for global variable\r\n"));
	}

	int8_t showed = 0;

	// Process

	for (uint8_t i=0; i < _debugWatchesAdded; i++) {

		String name = "";
		boolean show = showAll;

		// Show ?

		if (byNumber != -1) {
			show = ((i + 1) == byNumber);
		}

		if (show) {

			// Get global num

			uint8_t globalNum = _debugWatches[i].globalNum;

			if (globalNum >= _debugGlobalsAdded) {

				Serial.println(F("\r\n* SerialDebug: invalid index for global variable in watch\r\n"));
				return -1;
			}

			// Show

			PRINTF(F("* %02u {"), (i + 1));

			if (showGlobal(globalNum, DEBUG_SHOW_GLOBAL_WATCH, false)) {

				PRINTF(F("} "));
			}

			showed++;

		}
	}

	// Help

	if (showed > 0) {

		Serial.println(F("\r\n* SerialDebug: To show: wa [num]"));
		Serial.println(F("* SerialDebug: To add: wa {global [name|number]} [=|!=|<|>|<=|>=|change] [value]\r\n"));
		Serial.println(F("* SerialDebug: To add cross (2 globals): wa {global [name|number]} [=|!=|<|>|<=|>=] {global [name|number]}\r\n"));
		Serial.println(F("* SerialDebug: To disable : wa d [num|a|all]\r\n"));
		Serial.println(F("* SerialDebug: To enable: wa e [num|a|all]\r\n"));
		Serial.println(F("* SerialDebug: To nonstop on watches: wa [ns|nonstop]\r\n"));
		Serial.println(F("* SerialDebug: To stop on watches: wa [s|stop]\r\n"));
		Serial.println(F("* SerialDebug: To \r\n"));
		Serial.println(F("* SerialDebug: To \r\n"));
		Serial.println(F("* SerialDebug: To \r\n"));

	} else {

		Serial.println(F("\r\n* SerialDebug: watch not found."));
	}

	// Enter in silence mode

	debugSilence(true);

#ifndef DEBUG_NOT_USE_FLASH_F  // For Espressif boards, have a lot of memory, not using flash, to better performance
	// Cleanup

	if(flashSize > bufInitSize){
		delete[] pFlash;
	}
#endif


	// Return

	return showed;

}

// Show watch

static boolean showWatch(uint8_t watchNum) {

	if (watchNum >= _debugWatchesAdded) {

		Serial.println(F("\r\n* SerialDebug: invalid index for watch \r\n"));
		return false;
	}

	// Show

	PRINTF(F("* %02u {global "), (watchNum + 1));

	if (showGlobal(_debugWatches[watchNum].globalNum, DEBUG_SHOW_GLOBAL_WATCH, false)) {

		// Operation

		String oper = "";

		switch (_debugWatches[watchNum].operation) {
			case DEBUG_WATCH_CHANGED:
				oper = "(when changed)";
				break;
			case DEBUG_WATCH_EQUAL:
				oper = "=";
				break;
			case DEBUG_WATCH_DIFF:
				oper = "!=";
				break;
			case DEBUG_WATCH_LESS:
				oper = "<";
				break;
			case DEBUG_WATCH_GREAT:
				oper = ">";
				break;
			case DEBUG_WATCH_LESS_EQ:
				oper = "<=";
				break;
			case DEBUG_WATCH_GREAT_EQ:
				oper = ">=";
				break;
		}

		// Global

		debugGlobal_t* global = &_debugGlobals[_debugWatches[watchNum].globalNum];

		// Value

		String value = "";
		String type = "";

		if (_debugWatches[watchNum].operation != DEBUG_WATCH_CHANGED) {

			getStrValue(global->type, _debugWatches[watchNum].pointer, global->showLength, value, type);

			PRINTF(F("} %s %s\r\n"), oper.c_str(), value.c_str());

		} else {

			PRINTF(F("} %s \r\n"), oper.c_str());
		}

		return true;
	}

	return false;

}

// Change watches of global variables

static void changeWatches(Fields& fields) {

//	// Extract options
//
//	String globalId = fields.getString(1);
//	String value = fields.getString(3);
//
//	String type = "";
//	String tolower = "";
//	// Clean the value (spaces, '\"' and '\')
//
//	value.replace('\"', ' ');
//	value.replace('\'', ' ');
//	value.trim(); // TODO ver isto
//
//	if (value.length() == 0) {
//
//		Serial.println(F("* SerialDebug: not value informed (right of '=' in command)"));
//		return;
//	}
//
//	tolower = value;
//	tolower.toLowerCase();
//
//	// Show the global and get the index
//
//	int8_t num = showGlobals(globalId, true);
//
//	if (num == -1) {
//
//		// Invalid or not found
//
//		return;
//
//	}
	return;
}

// Verify global

static boolean verifyGlobal(uint8_t globalNum, uint8_t type) {

	return true;
}

// Find a global variable added by name

static boolean debugFindGlobal (const char* globalName, uint8_t* globalNum) {

	String find = globalName;
	String name = "";

	for (uint8_t i=0; i < _debugGlobalsAdded; i++) {

		// Get name

		if (_debugFunctions[i].name) { // Memory

			name = _debugFunctions[i].name;

		} else if (_debugFunctions[i].nameF) { // For Flash F

			name = String(_debugFunctions[i].nameF);
		}

		if (name == find) {
			*globalNum = i;
			return true;
		}
	}

	// Not find

	PRINTF(F("* SerialDebub: debugFindGlobal name not found: %s\r\n"), globalName);

	return false;
}

// Get a string value from a void pointer

static void getStrValue(uint8_t type, void* pointer, uint8_t showLength, String& response, String& responseType) {

	response = "";

	if (responseType) responseType = "";

	switch (type) {

		// Basic types

		case DEBUG_TYPE_BOOLEAN:
			response = ((*(boolean*)pointer) ? "true" : "false");
			if (responseType) responseType = "boolean";
			break;
		case DEBUG_TYPE_CHAR:
			response = String(*(char*)pointer);
			if (responseType) responseType = "char";
			break;
		case DEBUG_TYPE_BYTE:
			response = String(*(byte*)pointer);
			if (responseType) responseType = "byte";
			break;
		case DEBUG_TYPE_INT:
			response = String(*(int*)pointer);
			if (responseType) responseType = "int";
			break;
		case DEBUG_TYPE_U_INT:
			response = String(*(unsigned long*)pointer);
			if (responseType) responseType = "unsigned int";
			break;
		case DEBUG_TYPE_LONG:
			response = String(*(long*)pointer);
			if (responseType) responseType = "long";
			break;
		case DEBUG_TYPE_U_LONG:
			response = String(*(unsigned long*)pointer);
			if (responseType) responseType = "unsigned long";
			break;
		case DEBUG_TYPE_FLOAT:
			response = String(*(float*)pointer);
			if (responseType) responseType = "float";
			break;
		case DEBUG_TYPE_DOUBLE:
			response = String(*(double*)pointer);
			if (responseType) responseType = "double";
			break;

		// Integer C size _t

		case DEBUG_TYPE_INT8_T:
			response = String(*(int8_t*)pointer);
			if (responseType) responseType = "int8_t";
			break;
		case DEBUG_TYPE_INT16_T:
			response = String(*(int16_t*)pointer);
			if (responseType) responseType = "int16_t";
			break;
		case DEBUG_TYPE_INT32_T:
			response = String(*(int32_t*)pointer);
			if (responseType) responseType = "int32_t";
			break;
//#ifdef ESP32
//		case DEBUG_TYPE_INT64_T:
//			response = String(*(int64_t*)pointer);
//			if (responseType) responseType = "int64_t";
//			break;
//#endif
		// Unsigned integer C size _t

		case DEBUG_TYPE_UINT8_T:
			response = String(*(uint8_t*)pointer);
			if (responseType) responseType = "uint8_t";
			break;
		case DEBUG_TYPE_UINT16_T:
			response = String(*(uint16_t*)pointer);
			if (responseType) responseType = "uint16_t";
			break;
		case DEBUG_TYPE_UINT32_T:
			response = String(*(uint32_t*)pointer);
			if (responseType) responseType = "uint32_t";
			break;
//#ifdef ESP32
//			case DEBUG_TYPE_UINT64_T:
//					response = String(*(uint64_t*)pointer);
//					if (responseType) responseType = "uint64_t";
//					break;
//#endif
		// Strings

		case DEBUG_TYPE_CHAR_ARRAY:
			{
				String show = String((char*)pointer);
				size_t size = show.length();

				if (showLength > 0 &&
					size > showLength) {
					show = show.substring(0, showLength);
					show.concat("...");
				}
				response = "\"";
				response.concat(show);
				response.concat("\" (size:");
				response.concat(size);
				response.concat(")");
				if (responseType) responseType = "char array";
			}
			break;

		case DEBUG_TYPE_STRING:
			{
				String show = *(String*)pointer;
				size_t size = show.length();
				if (showLength > 0 &&
						size > showLength) {
					show = show.substring(0, showLength);
					show.concat("...");
				}
				response = "\"";
				response.concat(show);
				response.concat("\" (size:");
				response.concat(size);
				response.concat(")");
				if (responseType) responseType = "String";
			}
			break;
	}
}

// Aplly the operation between two *void pointers values

boolean apllyOperation(uint8_t type, void* pointer1, uint8_t operation, void* pointer2) {
	return true;
}

// Process command receipt by serial (Arduino monitor, etc.)

static void processCommand() {

	// Is in silence mode ? -> Returns to normal

	if (_debugSilence) {
		debugSilence(false, false);
	}

	// Extract options

	String options = "";
	int16_t pos = _debugLastCommand.indexOf(' ');
	if (pos > 0) {
		options = _debugLastCommand.substring(pos + 1);
		_debugLastCommand = _debugLastCommand.substring(0, pos);
	}

	// Invalid

	if (_debugLastCommand.length() > DEBUG_MAX_SIZE_COMMANDS) {

		Serial.println(F("* SerialDebug: Command received is too large, ignoring it\r\r"));
		return;
	}

	if (options.length() > DEBUG_MAX_SIZE_CMD_OPTIONS) {

		Serial.println(F("* SerialDebug: Command options received is too large, ignoring it\r\r"));
		return;
	}

	PRINTF(F("* SerialDebug: Command received: %s options: %s\r\n"), _debugLastCommand.c_str(), options.c_str());

	// Process the command

	if (_debugLastCommand == "h" || _debugLastCommand == "?") {

		// Show help

		showHelp();

		// Enter in silence mode

		debugSilence(true);

	} else if (_debugLastCommand == "v") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_VERBOSE);

	} else if (_debugLastCommand == "d") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_DEBUG);

	} else if (_debugLastCommand == "i") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_INFO);

	} else if (_debugLastCommand == "w") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_WARN);

	} else if (_debugLastCommand == "e") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_ERROR);

	} else if (_debugLastCommand == "n") {

		// Debug level

		debugSetLevel(DEBUG_LEVEL_NONE);

	} else if (_debugLastCommand == "s") {

		// Silence

		debugSilence (!_debugSilence, true);

	} else if (_debugLastCommand == "p") {

		// Show profiler with minimal time

		if (options.length() == 0) {

			// Show profiler

			debugShowProfiler(!_debugShowProfiler, 0, true);

		} else {

			// With minimal time

			if (strIsNum(options)) {

				int32_t aux = options.toInt();

				if (aux > 0) { // Valid number

					// Show profiler

					debugShowProfiler(true, aux, true);

				}

			} else {

				Serial.println(F("* SerialDebug: invalid number in argument"));
			}
		}

	} else if (_debugLastCommand == "filter" && options.length() > 0) {

		debugSetFilter(options);

	} else if (_debugLastCommand == "nofilter") {

		debugSetNoFilter();

	} else if (_debugLastCommand == "f") {

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

		// Process

		processFunctions(options);

#else
		Serial.println(F("* SerialDebug: Debug functions is not enabled in your project"));

#endif

	} else if (_debugLastCommand == "g") {

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

		// Process

		processGlobals(options);
#else
		Serial.println(F("* SerialDebug: Debug functions is not enabled in your project"));

#endif

	} else if (_debugLastCommand == "wa") {

#ifndef DEBUG_DISABLE_DEBUGGER // Only if debugger is enabled

		// Process

		processWatches(options);
#else
		Serial.println(F("* SerialDebug: Debug functions is not enabled in your project"));

#endif

	} else if (_debugLastCommand == "m") {

#ifdef ARDUINO_ARCH_AVR

		PRINTF(F("* Free Heap RAM: %d\r\n"), freeMemory());

#elif defined ESP8266 || defined ESP32

		PRINTF(F("* Free Heap RAM: %d\r\n"), ESP.getFreeHeap());
#else

		Serial.println(F("* SerialDebug: this option is not implemented for this board"));

#endif // ARDUINO_ARCH_AVR

	} else if (_debugLastCommand == "reset" ) {

#ifdef ARDUINO_ARCH_AVR

		Serial.println("* SerialDebug: Resetting the Arduino ...");

		delay(1000);

		// Reset

		resetArduino();

#elif defined ESP8266 || defined ESP32 // Only for Espressif boards

	#if defined ESP8266
		Serial.println("* SerialDebug: Resetting the ESP8266 ...");
	#elif defined ESP32
		Serial.println("* SerialDebug: Resetting the ESP32 ...");
	#endif

		delay(1000);

		// Reset

		ESP.restart();

#else

		Serial.println(F("* SerialDebug: this option is not implemented for this board"));

#endif // ARDUINO_ARCH_AVR

#if defined ESP8266 // Only to ESP8266

	} else if (_debugLastCommand == "cpu80") {

		// Change ESP8266 CPU para 80 MHz

		system_update_cpu_freq(80);
		Serial.println(F("* SerialDebug: CPU ESP8266 changed to: 80 MHz"));

	} else if (_debugLastCommand == "cpu160") {

		// Change ESP8266 CPU para 160 MHz

		system_update_cpu_freq(160);
		Serial.println(F("* SerialDebug: CPU ESP8266 changed to: 160 MHz"));

#endif

	} else {

		Serial.println(F("* SerialDebug: command invalid"));
	}

}

#ifndef DEBUG_DISABLE_FEATURES

// Process functions commands

static void processFunctions(String& options) {

	// Get fields of command options

	Fields fields(options, ' ', true);

	if (fields.size() > 1) {
		Serial.println(F("* SerialDebug: Invalid sintax for functions. use f ? to show help"));
		return;
	}

	String option = (fields.size() == 1)? fields.getString(1) : "";

	if (_debugFunctionsAdded > 0) {

		if (option.length() == 0 || option == "?") {

			// Just show functions and help

			showFunctions(option,false);

		} else if (option.indexOf('*') >= 0) {

			// Search by name (case insensitive)

			showFunctions(options, false);

		} else {

			// Call a function

			callFunction(options);
		}

	} else {

		Serial.println(F("* SerialDebug: Functions not added to SerialDebug in your project"));
		Serial.println(F("* SerialDebug: See how do it in advanced example"));

	}
}

// Show list of functions available
// Used to search and show all (return total showed)
// Or to search and show one (return number of item showed)

static int8_t showFunctions(String& options, boolean one) {

	// Show all ?

	boolean showAll = false;
	boolean byStartName = false;
	int8_t byNumber = -1;

	_debugLastNameF = "";

	// Searching ?

	if (options.length() == 0) {

		if (one) {
			Serial.println(F("* SerialDebug: option not informed"));
			return -1;
		}

		showAll = true;

	} else {

		// Search by start name (case insensitive)

		int8_t pos = options.indexOf('*');

//		PRINTF("pos %d\r\n", pos);

		if (pos > 0) {

			if (one) {

				Serial.println(F("* SerialDebug: * not allowed, use name or number instead"));
				return -1;
			}

			options = options.substring(0, pos);
			options.toLowerCase(); // Case insensitive

			byStartName = true;
//			PRINTF("byName %s\r\n", options.c_str());

		} else {

			// Is integer ?

			if (strIsNum(options)) {

				uint8_t num = options.toInt();

//				PRINTF("byNumber %s = %d\r\n", options.c_str(), num);

				if (num > 0) { // Find by number, else by exact name

					if (num > _debugFunctionsAdded) {

						PRINTF(F("* SerialDebug: Function number must between 1 and %u\r\n"), _debugFunctionsAdded);
						return -1;
					}

					byNumber = num;
				}
			}
		}

		showAll = false;
	}

	// Show function(s)

	if (showAll) {
		PRINTF(F("\r\n* SerialDebug: Showing all functions (%u):\r\n\r\n"), _debugFunctionsAdded);
	} else if (!one) {
		Serial.println(F("\r\n* SerialDebug: Searching and showing functions:\r\n"));
	}

	int8_t showed = (!one)? 0 : -1;

	for (uint8_t i=0; i < _debugFunctionsAdded; i++) {

		String type = "";
		String name = "";
		boolean show = showAll;

		// Get name

		if (_debugFunctions[i].name) { // Memory

			name = _debugFunctions[i].name;

		} else if (_debugFunctions[i].nameF) { // For Flash F

			name = String(_debugFunctions[i].nameF);
		}

		// Show ?

		if (!showAll) {

			if (byStartName) {
				String tolower = name;
				tolower.toLowerCase(); // Case insensitive
				show = (tolower.startsWith(options));
			} else if (byNumber > 0) {
				show = ((i + 1) == byNumber);
			} else {
				show = (name == options);
			}
		}

//		PRINTF("showFunction options = %s name: %s showAll=%d show=%d\r\n", options.c_str(), name.c_str(), showAll, show);

		if (show) {

			switch (_debugFunctions[i].argType) {
				case DEBUG_TYPE_FUNCTION_VOID:
					type = "";
					break;
				case DEBUG_TYPE_STRING:
					type = "String";
					break;
				case DEBUG_TYPE_CHAR:
					type = "char";
					break;
				case DEBUG_TYPE_INT:
					type = "int";
					break;
				default:
					break;
			}

			if (one) { // One search ?

				PRINTF("\r\n* SerialDebug: Function found: %02u %s(%s)\r\n", (i + 1), name.c_str(), type.c_str());

				if (_debugFunctions[i].nameF) { // For Flash F - save the content in variable in memory, to not extract Flash again
					_debugLastNameF = name;
				}
				showed = i; // Return the index

				break;

			} else { // Description, not for one search

				PRINTF("* %02u %s(%s)", (i + 1), name.c_str(), type.c_str());

				if (_debugFunctions[i].description) { // Memory
					PRINTF(" // %s\r\n", _debugFunctions[i].description);
				} else if (_debugFunctions[i].descriptionF) { // For Flash F, multiples print
					Serial.print(" // ");
					Serial.println(_debugFunctions[i].descriptionF);
				} else {
					Serial.println();
				}

				showed++;

			}
		}
	}

	// Help

	if (!one && showed > 0) {

		Serial.println(F("\r\n* SerialDebug: To show: f [name|num]"));
		Serial.println(F("* SerialDebug: To search with start of name (case insensitive): f name*"));
		Serial.println(F("* SerialDebug: To call it, just command: f [name|number] [arg]\r\n"));

	} else if (!one || showed == -1) {

		Serial.println(F("\r\n* SerialDebug: function not found."));
	}

	// Enter in silence mode

	debugSilence(true);

	// Return

	return showed;

}

// Call a function

static void callFunction(String& options) {

	// Extract options

	String funcId = "";
	String funcArg = "";

	int8_t pos = options.indexOf(' ');
	if (pos > 0) {
		funcId = options.substring(0, pos);
		funcArg = options.substring(pos + 1);
	} else {
		funcId = options;
	}

	//PRINTF("callFunction: id %s arg %s\r\n", funcId.c_str(), funcArg.c_str());

	// Find and show a function (one)

	int8_t num = showFunctions(funcId, true);

	if (num == -1) { // Not found or error

		return;

	}

	//PRINTF("callFunction: num %u\r\n", num);

	// Call the function

	unsigned long timeBegin = 0;

	if (_debugFunctions[num].name) { // Memory
		PRINTF(F("* SerialDebug: Calling function %u -> %s("), (num + 1), _debugFunctions[num].name);
	} else if (_debugFunctions[num].nameF) { // Use a temporary var to not get flash again
		PRINTF(F("* SerialDebug: Calling function %u -> %s("), (num + 1), _debugLastNameF.c_str());
	}

	if (!_debugFunctions[num].callback) { // Callback not set ?

		Serial.println(F(") - no callback set for function"));
		return;

	}

	if (funcArg.length() == 0 &&
		!(_debugFunctions[num].argType == DEBUG_TYPE_FUNCTION_VOID ||
			_debugFunctions[num].argType == DEBUG_TYPE_STRING ||
			_debugFunctions[num].argType == DEBUG_TYPE_CHAR_ARRAY)) {  // For others can not empty

		Serial.println(F(") - argument not informed, this function needs one"));
		return;
	}

	// Exit from silence mode

	if (_debugSilence) {
		debugSilence(false, false);
	}

	// Process

	bool called = false;

	switch (_debugFunctions[num].argType) {

		case DEBUG_TYPE_FUNCTION_VOID:
			{
				// Void arg

				PRINTF(F(") ...\r\n"));
				delay(1000);
				timeBegin = micros();

				_debugFunctions[num].callback();
				called = true;
			}
			break;

		case DEBUG_TYPE_STRING:
			{
				// String arq

				PRINTF(F("\"%s\") ...\r\n"), funcArg.c_str());
				delay(1000);
				timeBegin = micros();

				void (*callback)(String) = (void(*) (String))_debugFunctions[num].callback;

				callback(funcArg);
				called = true;
			}
			break;

		case DEBUG_TYPE_CHAR:
			{
				// Arg char

				PRINTF(F("\'%c\') ...\r\n"), funcArg[0]);
				delay(1000);
				timeBegin = micros();

				void (*callback)(char) = (void(*) (char))_debugFunctions[num].callback;

				callback(funcArg[0]);

				called = true;
			}
			break;

		case DEBUG_TYPE_INT:
			{
				// Arg Int

				if (strIsNum(funcArg)) { // Is numeric ?

					int val = funcArg.toInt();

					PRINTF(F("%d) ...\r\n"), val);
					delay(1000);
					timeBegin = micros();

					void (*callback)(int) = (void(*) (int))_debugFunctions[num].callback;

					callback(val);

					called = true;

				} else {

					PRINTF(F("%s) - invalid number in argument\r\n"), funcArg.c_str());
				}
			}
			break;
	}

	// Called ?

	if (called) {

		unsigned long elapsed = (micros() - timeBegin);

		PRINTF(F("* SerialDebug: End of execution. Elapsed: %lu ms (%lu us)\r\n"), (elapsed / 1000), elapsed);

	}

	// Enter in silence mode

	debugSilence(true);

}

// Process global variables commands

static void processGlobals(String& options) {

	if (_debugGlobalsAdded > 0) {

		// Get fields of command options

		Fields fields(options, ' ', true);

		if (fields.size() > 3) {
			Serial.println(F("* SerialDebug: Invalid sintax for globals. use g ? to show help"));
			return;
		}

		String firstOption = (fields.size() >= 1)? fields.getString(1) : "";

		if (firstOption.length() == 0 || firstOption == "?") {

			// Just show globals and help

			showGlobals(firstOption, false);

		} else if (fields.size() >= 2) { // Process change command

			if (fields.getChar(2) == '=') {

				if (fields.size() == 2) {

					Serial.println(F("* SerialDebug: Invalid sintax for globals. use g ? to show help"));
					return;

				} else {

					// Change the variable

					changeGlobal (fields);

				}

			} else {

				Serial.println(F("* SerialDebug: Invalid sintax for globals. use g ? to show help"));
				return;

			}

		} else {

			// Search by name/number

			showGlobals(options, false);

		}

	} else {

		Serial.println(F("* SerialDebug: Global variables not added to SerialDebug in your project"));
		Serial.println(F("* SerialDebug: See how do it in advanced example"));

	}


}

// Show list of globals available
// Used to search and show all (return total showed)
// Or to search and show one (return number showed)

static int8_t showGlobals(String& options, boolean one) {

	// Show all ?

	boolean showAll = false;
	boolean byStartName = false;
	int8_t byNumber = -1;

	// Searching ?

	if (options.length() == 0) {

		if (one) {
			Serial.println(F("* SerialDebug: option not informed"));
			return -1;
		}

		showAll = true;

	} else {

		// Search by start name (case insensitive)

		int8_t pos = options.indexOf('*');

//		PRINTF("pos %d\r\n", pos);

		if (pos > 0) {

			if (one) {

				Serial.println(F("* SerialDebug: * not allowed, use name or number instead"));
				return -1;
			}

			options = options.substring(0, pos);
			options.toLowerCase(); // Case insensitive

			byStartName = true;

		} else {

			// Is integer ?

			if (strIsNum(options)) {

				uint8_t num = options.toInt();

//				PRINTF("byNumber %s = %d\r\n", options.c_str(), num);

				if (num > 0) { // Find by number, else by exact name

					byNumber = num;
				}
			}
		}

		showAll = false;
	}

	// Show global(s)

	if (showAll) {
		PRINTF(F("\r\n* SerialDebug: Showing all global variables (%u) and actual values:\r\n\r\n"), _debugGlobalsAdded);
	} else {
		Serial.println(F("\r\n* SerialDebug: Searching and showing global variables and actual values:\r\n"));
	}

	int8_t showed = (!one)? 0 : -1;

	// Process

	for (uint8_t i=0; i < _debugGlobalsAdded; i++) {

		String name = "";
		boolean show = showAll;

		// Get name

		if (_debugGlobals[i].name) { // Memory

			name = _debugGlobals[i].name;

		} else if (_debugGlobals[i].nameF) { // For Flash F

			name = String(_debugGlobals[i].nameF);
		}

		// Show ?

		if (!showAll) {

			if (byStartName) {
				String tolower = name;
				tolower.toLowerCase(); // Case insensitive
				show = (tolower.startsWith(options));
			} else if (byNumber > 0) {
				show = ((i + 1) == byNumber);
			} else {
				show = (name == options);
			}
		}

//		PRINTF("showGlobais options = %s name: %s showAll=%d show=%d byStartName=%d byNumber=%d\r\n",
//				options.c_str(), name.c_str(), showAll, show,
//				byStartName, byNumber);

		if (show) {

			if (_debugGlobals[i].nameF) { // For Flash F - save the content in variable in memory, to not extract Flash again
				_debugLastNameF = name;
			}

			// Show

			if (showGlobal(i, DEBUG_SHOW_GLOBAL, true)) {

				if (one) { // One search ?

					showed = i; // Return the index
					break;

				} else { // Description, not for one search

					if (_debugGlobals[i].description) { // Memory
						PRINTF(" // %s\r\n", _debugGlobals[i].description);
					} else if (_debugGlobals[i].descriptionF) { // For Flash F, multiples print
						Serial.print(" // ");
						Serial.println(_debugGlobals[i].descriptionF);
					} else {
						Serial.println();
					}

					showed++;

				}
			}
		}
	}

	// Help

	if (!one && showed > 0) {

		Serial.println(F("\r\n* SerialDebug: To show: g [name|num]"));
		Serial.println(F("* SerialDebug: To search by start of name (case insensitive): g name*"));
		Serial.println(F("* SerialDebug: To change global variable, just command: g [name|number] = value\r\n"));

	} else if (!one || showed == -1) {

		Serial.println(F("\r\n* SerialDebug: Global variable not found."));
	}

	// Enter in silence mode

	debugSilence(true);

#ifndef DEBUG_NOT_USE_FLASH_F  // For Espressif boards, have a lot of memory, not using flash, to better performance
	// Cleanup

	if(flashSize > bufInitSize){
		delete[] pFlash;
	}
#endif


	// Return

	return showed;

}

// Show one global variable

static boolean showGlobal(uint8_t globalNum, debugEnumShowGlobais_t mode, boolean getLastNameF) {

	// Validate

	if (globalNum >= _debugGlobalsAdded) {

		Serial.println(F("\r\n* SerialDebug: invalid index for global variable\r\n"));
		return false;
	}

	// Global

	debugGlobal_t* global = &_debugGlobals[globalNum];

	// Get name

	String name = "";

	if (getLastNameF) {

		name = _debugLastNameF; // Yet get from flash

	} else {

		if (global->name) { // Memory

			name = global->name;

		} else if (global->nameF) { // For Flash F

			name = String(global->nameF);
			_debugLastNameF = name; // For Flash F - save the content in variable in memory, to not extract Flash again
		}
	}

	// Get value and type

	String value = "";
	String type = "";

	// Get value

	getStrValue(global->type, global->pointer, global->showLength, value, type);

	if (value.length() > 0) {

		// Show

		switch (mode) {
			case DEBUG_SHOW_GLOBAL:
				PRINTF("* %02u %s(%s) = %s", globalNum, name.c_str(), type.c_str(), value.c_str());
				break;
			case DEBUG_SHOW_GLOBAL_WATCH:
				PRINTF("%02u: %s (%s) = %s", globalNum, name.c_str(), type.c_str(), value.c_str());
				break;
		}

	} else {

		Serial.println(F("\r\n* SerialDebug: not possible show global variable\r\n"));
		return false;
	}
	return true;
}

// Change content of global variable

static void changeGlobal(Fields& fields) {

	// Extract options

	String globalId = fields.getString(1);
	String value = fields.getString(3);

	String type = "";
	String tolower = "";
	// Clean the value (spaces, '\"' and '\')

	value.replace('\"', ' ');
	value.replace('\'', ' ');
	value.trim(); // TODO ver isto

	if (value.length() == 0) {

		Serial.println(F("* SerialDebug: not value informed (right of '=' in command)"));
		return;
	}

	tolower = value;
	tolower.toLowerCase();

	// Show the global and get the index

	int8_t num = showGlobals(globalId, true);

	if (num == -1) {

		// Invalid or not found

		return;

	}

	Serial.println(F(" // <- This is a old value"));

//	PRINTF("changeGlobal: id->%s num=%d value = %s (%d)\r\n", globalId.c_str(), num, value.c_str(),value.length());

	// Verify data

	switch (_debugGlobals[num].type) {

		// Basic types

		case DEBUG_TYPE_BOOLEAN:
			{
				if (value == "0" ||
						tolower == "f" ||
						tolower == "false" ||
						value == "1" ||
						tolower == "t" ||
						tolower == "true") {

					// Value ok

					type = "boolean";
				} else {
					Serial.println(F("* SerialDebug: not boolean in value (0|1|false|true|f|t")) ;
					return;
				}
			}
			break;
		case DEBUG_TYPE_CHAR:

			if (value.length() != 1) {
				Serial.println(F("* SerialDebug: Note: string too large, truncated to size of 1"));
				value = value.substring(0,1);
			}
			type = "char";
			break;
		case DEBUG_TYPE_BYTE:
			if (value.length() != 1) {
				Serial.println(F("* SerialDebug: Note: string too large, truncated to size of 1"));
				value = value.substring(0,1);
			}
			type = "byte";
			break;
		case DEBUG_TYPE_INT:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "int";
			break;
		case DEBUG_TYPE_U_INT:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "unsigned int";
			break;
		case DEBUG_TYPE_LONG:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "long";
			break;
		case DEBUG_TYPE_U_LONG:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "unsigned long";
			break;
		case DEBUG_TYPE_FLOAT:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "float";
			break;
		case DEBUG_TYPE_DOUBLE:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "double";
			break;

		// Integer C size _t

		case DEBUG_TYPE_INT8_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "int8_t";
			break;
		case DEBUG_TYPE_INT16_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "int16_t";
			break;
		case DEBUG_TYPE_INT32_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "int32_t";
			break;
//#ifdef ESP32
//			case DEBUG_TYPE_INT64_T:
//					value = String(*(int64_t*)_debugGlobals[i].pointer);
//					type = "int64_t";
//					break;
//#endif
		// Unsigned integer C size _t

		case DEBUG_TYPE_UINT8_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "uint8_t";
			break;
		case DEBUG_TYPE_UINT16_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "uint16_t";
			break;
		case DEBUG_TYPE_UINT32_T:
			if (!strIsNum(value)) {
				Serial.println(F("* SerialDebug: Not numeric value is informed"));
				return;
			}
			type = "uint32_t";
			break;
//#ifdef ESP32
//			case DEBUG_TYPE_UINT64_T:
//					value = String(*(uint64_t*)_debugGlobals[i].pointer);
//					type = "uint64_t";
//					break;
//#endif
		// Strings

		case DEBUG_TYPE_CHAR_ARRAY:
			{

//				size_t size = sizeof((char*)_debugGlobals[num].pointer);
//				if (value.length() >= size) {
//					PRINTF(F("* SerialDebug: Note: string too large, truncated to size of array (%u)\r\n"), size);
//					value = value.substring(0,size-1);
//				}
//				type = "char array";
				Serial.println(F("* SerialDebug: not allowed change char arrays (due memory issues)"));
				return;
			}
			break;

		case DEBUG_TYPE_STRING:
			type = "String";
			break;

	}

	// Show again with new value to confirm

	if (_debugGlobals[num].type == DEBUG_TYPE_CHAR_ARRAY || _debugGlobals[num].type == DEBUG_TYPE_STRING) {
		if (_debugGlobals[num].name) { // RAM Memory
			PRINTF("* %02u %s(%s) = \"%s\"", (num + 1), _debugGlobals[num].name, type.c_str(), value.c_str());
		} else if (_debugGlobals[num].name) { // Flash memory
			PRINTF("* %02u %s(%s) = \"%s\"", (num + 1), _debugLastNameF.c_str(), type.c_str(), value.c_str());
		}
	} else {
		if (_debugGlobals[num].name) { // RAM Memory
			PRINTF("* %02u %s(%s) = %s", (num + 1), _debugGlobals[num].name, type.c_str(), value.c_str());
		} else if (_debugGlobals[num].name) { // Flash memory
			PRINTF("* %02u %s(%s) = %s", (num + 1), _debugLastNameF.c_str(), type.c_str(), value.c_str());
		}
	}
	Serial.println(F(" // <- This is a new value"));


	// Show a confirm message and wait fro response

	debugBreak(F("\r\n* Confirm do change value of this global ? (y-yes/n-not)"));

	if (_debugBreakResponse == "y" || _debugBreakResponse == "yes") {

		// Do change

		switch (_debugGlobals[num].type) {

				// Basic types

				case DEBUG_TYPE_BOOLEAN:
					{
						boolean change = (value == "1" || value[0] == 't');
						*(boolean*)_debugGlobals[num].pointer = change;
						value = (change) ? "true" : "false";
					}
					break;
				case DEBUG_TYPE_CHAR:
					{
						char change = value[0];
						*(char*)_debugGlobals[num].pointer = change;
					}
					break;
				case DEBUG_TYPE_BYTE:
					{
						char change = value[0];
						*(byte*)_debugGlobals[num].pointer = change;
					}
					break;
				case DEBUG_TYPE_INT:
					{
						int change = value.toInt();
						*(int*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_U_INT:
					{
						unsigned int change = value.toInt();
						*(unsigned int*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_LONG:
					{
						long change = value.toInt();
						*(long*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_U_LONG:
					{
						unsigned long change = value.toInt();
						*(unsigned long*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_FLOAT:
					{
						float change = value.toFloat();
						*(float*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_DOUBLE: // TODO no have toDouble in some archs - see
					{
						double change = value.toFloat();
						*(double*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;

				// Integer C size _t

				case DEBUG_TYPE_INT8_T:
					{
						int8_t change = value.toInt();
						*(int8_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_INT16_T:
					{
						int16_t change = value.toInt();
						*(int16_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_INT32_T:
					{
						int32_t change = value.toInt();
						*(int32_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
		//#ifdef ESP32
		//			case DEBUG_TYPE_INT64_T:
		//					value = String(*(int64_t*)_debugGlobals[i].pointer);
		//					type = "int64_t";
		//					break;
		//#endif

				// Unsigned integer C size _t

				case DEBUG_TYPE_UINT8_T:
					{
						uint8_t change = value.toInt();
						*(uint8_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_UINT16_T:
					{
						uint16_t change = value.toInt();
						*(uint16_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
				case DEBUG_TYPE_UINT32_T:
					{
						uint32_t change = value.toInt();
						*(uint32_t*)_debugGlobals[num].pointer = change;
						value = String(change);
					}
					break;
		//#ifdef ESP32
		//			case DEBUG_TYPE_UINT64_T:
		//					value = String(*(uint64_t*)_debugGlobals[i].pointer);
		//					type = "uint64_t";
		//					break;
		//#endif
				// Strings

				case DEBUG_TYPE_CHAR_ARRAY:
					{
						strcpy((char*)(_debugGlobals[num].pointer), (char*)value.c_str());
					}
					break;
				case DEBUG_TYPE_STRING:
					{
						*(String*)_debugGlobals[num].pointer = value;
					}
					break;
			}

			// Show again with new value

			if (_debugGlobals[num].type == DEBUG_TYPE_CHAR_ARRAY || _debugGlobals[num].type == DEBUG_TYPE_STRING) {
				if (_debugGlobals[num].name) { // RAM Memory
					PRINTF("* %02u %s(%s) = \"%s\"", (num + 1), _debugGlobals[num].name, type.c_str(), value.c_str());
				} else if (_debugGlobals[num].name) { // Flash memory
					PRINTF("* %02u %s(%s) = \"%s\"", (num + 1), _debugLastNameF.c_str(), type.c_str(), value.c_str());
				}
			} else {
				if (_debugGlobals[num].name) { // RAM Memory
					PRINTF("* %02u %s(%s) = %s", (num + 1), _debugGlobals[num].name, type.c_str(), value.c_str());
				} else if (_debugGlobals[num].name) { // Flash memory
					PRINTF("* %02u %s(%s) = %s", (num + 1), _debugLastNameF.c_str(), type.c_str(), value.c_str());
				}
			}
			Serial.println(F(" // <- This has changed w/ success"));


	} else {

		Serial.println(F("* SerialDebug: variable global not changed"));

	}
	// Enter in silence mode

	debugSilence(true);

}

#endif // DEBUG_DISABLE_DEBUGGER

// Show debug help

static void showHelp() {

	PRINTF(F("*** SerialDebug Library - version: %s"), DEBUG_VERSION);

#ifdef ARDUINO_ARCH_AVR

	PRINTF(F("* Free Heap RAM: %d\r\n"), freeMemory());

#elif defined ESP8266 || defined ESP32

	PRINTF(F("* Free Heap RAM: %d"), ESP.getFreeHeap());
	PRINTF(F("* ESP SDK version: %s"), ESP.getSdkVersion());
#endif

	// Using PROGMEM in large strings (even for Espressif boards)

	Serial.println(FPSTR(debugHelp));

#ifdef ESP8266 // Esp8266 only (ESP32 not to easy to change it)

	Serial.println(F("cpu80  -> ESP8266 CPU a 80MHz"));
	Serial.println(F("cpu160 -> ESP8266 CPU a 160MHz"));

#endif

}

// Only for AVR boards

#ifdef ARDUINO_ARCH_AVR

// Reset

// Free memory - based on MemoryFree library: https://github.com/maniacbug/MemoryFree

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

int freeMemory() {

	int free_memory;

	if((int)__brkval == 0)
		free_memory = ((int)&free_memory) - ((int)&__bss_end);
	else
		free_memory = ((int)&free_memory) - ((int)__brkval);

	return free_memory;
}

#endif // ARDUINO_ARCH_AVR

#endif // DEBUG_DISABLE_DEBUGGER

#endif // DEBUG_DISABLED

/////// End
