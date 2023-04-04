#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <string>
#include <map>
#include <stack>
#include <chrono>

namespace Ryao {

// To time a function, just put:
//
//  Timer functionTimer(__FUNCTION__);
//
// at the beginning of the function
typedef std::chrono::time_point<std::chrono::high_resolution_clock> timePoint;
typedef std::chrono::microseconds usec;
class Timer {
public:
    // start the timer by default -- if a tick is called later,
    // it will just stomp it
    Timer(std::string blockName);
    ~Timer();

    // stop the timer manually
    void stop();

    static double timing(timePoint& begin = _tick, timePoint& end = _tock) {
        return std::chrono::duration_cast<usec>(end - begin).count() * 1e-6;
    };

    static int hours(int seconds) { return seconds / (60 * 60); };

    static int minutes(int seconds) {
        int mod = seconds % (60 * 60);
        return mod / 60;
    };

    static int seconds(int seconds) {
        int mod = seconds % (60 * 60);
        return mod % 60;
    };

    static void printTimings();
    static void printTimingsPerFrame(const int frames);

private:
    static timePoint _tick;
    static timePoint _tock;

    // hash table of all timings
    static std::map<std::string, double> _timings;

    // call stack
    static std::stack<std::string> _callStack;

    // track whether it was stopped already so we don't
    // stop it twice
    bool _stopped;
};

} // Ryao

#endif // !TIMER_H
