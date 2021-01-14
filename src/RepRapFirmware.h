/*
 * RepRapFirmwareASF4.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_REPRAPFIRMWARE_H_
#define SRC_REPRAPFIRMWARE_H_

#include <CoreIO.h>
#include "ecv.h"
#undef array
#undef value			// needed because we include <optional>

#include <cmath>
#include <cinttypes>
#include <climits>		// for CHAR_BIT

#include <General/String.h>
#include <General/StringFunctions.h>
#include <General/Bitmap.h>

// These three are implemented in Tasks.cpp
void delay(uint32_t ms);
uint32_t millis();
uint64_t millis64();

// Debugging support
extern "C" void debugPrintf(const char* fmt, ...) __attribute__ ((format (printf, 1, 2)));
#define DEBUG_HERE do { } while (false)
//#define DEBUG_HERE do { debugPrintf("At " __FILE__ " line %d\n", __LINE__); delay(50); } while (false)

#if defined(__SAME51N19A__)
# define SAME5x		1
# define SAMC21		0
#elif defined(__SAMC21G18A__)
# define SAME5x		0
# define SAMC21		1
#endif

// Classes to facilitate range-based for loops that iterate from 0 up to just below a limit
template<class T> class SimpleRangeIterator
{
public:
	SimpleRangeIterator(T value_) : val(value_) {}
    bool operator != (SimpleRangeIterator<T> const& other) const { return val != other.val;     }
    T const& operator*() const { return val; }
    SimpleRangeIterator& operator++() { ++val; return *this; }

private:
    T val;
};

template<class T> class SimpleRange
{
public:
	SimpleRange(T limit) : _end(limit) {}
	SimpleRangeIterator<T> begin() const { return SimpleRangeIterator<T>(0); }
	SimpleRangeIterator<T> end() const { return SimpleRangeIterator<T>(_end); 	}

private:
	const T _end;
};

// A simple milliseconds timer class
class MillisTimer
{
public:
	MillisTimer() { running = false; }
	void Start();
	void Stop() { running = false; }
	bool Check(uint32_t timeoutMillis) const;
	bool CheckAndStop(uint32_t timeoutMillis);
	bool IsRunning() const { return running; }

private:
	uint32_t whenStarted;
	bool running;
};

constexpr size_t ScratchStringLength = 220;							// standard length of a scratch string, enough to print delta parameters to
constexpr size_t ShortScratchStringLength = 50;

// Common conversion factors
constexpr unsigned int MinutesToSeconds = 60;
constexpr float SecondsToMinutes = 1.0/(float)MinutesToSeconds;
constexpr unsigned int SecondsToMillis = 1000.0;
constexpr float MillisToSeconds = 1.0/(float)SecondsToMillis;
constexpr float InchToMm = 25.4;
constexpr float Pi = 3.141592653589793;
constexpr float TwoPi = 3.141592653589793 * 2;
constexpr float DegreesToRadians = 3.141592653589793/180.0;
constexpr float RadiansToDegrees = 180.0/3.141592653589793;

#include "Config/BoardDef.h"

#endif
