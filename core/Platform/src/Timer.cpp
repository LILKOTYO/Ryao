#include <Timer.h>
#include <cassert>
#include <stdio.h>

using namespace std;

namespace Ryao {

timePoint Timer::_tick;
timePoint Timer::_tock;
map<string, double> Timer::_timings;
stack<string> Timer::_callStack;

Timer::Timer(string blockName) {
    // look at the back of the call stack,
    // if there's something there, then store its timing
    //
    // else, it's the first call, so set the global start
    if (_callStack.size() > 0) {
        string function = _callStack.top();
        //gettimeofday(&_tock, 0);
        _tock = chrono::high_resolution_clock::now();

        _timings[function] += timing();
    }

    _callStack.push(blockName);
    _tick = chrono::high_resolution_clock::now();

    _stopped = false;
}

Timer::~Timer() {
    // don't stop it twice
    if (!_stopped) {
        stop();
    }
}

void Timer::stop() {
    // don't stop it twice
    if (_stopped) return;

    assert(_callStack.size() > 0);

    string function = _callStack.top();
    _callStack.pop();
    _tock = chrono::high_resolution_clock::now();

    _timings[function] += timing();
    _tick = chrono::high_resolution_clock::now();

    _stopped = true;
}

void Timer::printTimings() {
    timePoint now;
    now = chrono::high_resolution_clock::now();

    double currentTimer = timing(_tick, now);
    string currentName;
    if (_callStack.size() > 0)
        currentName = _callStack.top();

    // create an inverse map so that it will sort by time
    map<double, string> inverseMap;
    map<string, double>::iterator forwardIter;
    double totalTime = 0.0;
    for (forwardIter = _timings.begin(); forwardIter != _timings.end(); forwardIter++) {
        string name = forwardIter->first;
        double time = forwardIter->second;

        // if we are dinside this function, add the timing
        if (name.compare(currentName) == 0)
            time += currentTimer;

        inverseMap[time] = name;
        totalTime += time;
    }

    // print the map out backwards since it sorts from least to greatest
    cout << "=====================================================================================" << endl;
    cout << " TIMING BREAKDOWN: " << endl;
    cout << "=====================================================================================" << endl;
    map<double, string>::reverse_iterator backwardIter;
    for (backwardIter = inverseMap.rbegin(); backwardIter != inverseMap.rend(); backwardIter++) {
        string name = (*backwardIter).second;
        double time = (*backwardIter).first;

        name = name + string("                                             ");
        name = name.substr(0, 40);

        cout << "[" << time / totalTime * 100.0 << "%\t]: "
            << name.c_str() << " " << hours(time) << ":" << minutes(time) << ":" << seconds(time) << "s" << endl;
    }
    cout << "=====================================================================================" << endl;
    cout << " total time: " << totalTime << endl;
    cout << "=====================================================================================" << endl;
}

void Timer::printTimingsPerFrame(const int frames) {
    timePoint now;
    now = chrono::high_resolution_clock::now();

    double currentTimer = timing(_tick, now);
    string currentName;
    if (_callStack.size() > 0)
        currentName = _callStack.top();

    // create an inverse map so that it will sort by time
    map<double, string> inverseMap;
    map<string, double>::iterator forwardIter;
    double totalTime = 0.0;
    for (forwardIter = _timings.begin(); forwardIter != _timings.end(); forwardIter++) {
        string name = forwardIter->first;
        double time = forwardIter->second;

        // if we are dinside this function, add the timing
        if (name.compare(currentName) == 0)
            time += currentTimer;

        inverseMap[time] = name;
        totalTime += time;
    }

    // print the map out backwards since it sorts from least to greatest
    cout << "==============================================================================================" << endl;
    cout << " TIMING BREAKDOWN, FRAME " << frames << ": " << endl;
    cout << "==============================================================================================" << endl;
    map<double, string>::reverse_iterator backwardIter;
    char buffer[256];
    string timeString;
    string hoursString;
    string minutesString;
    string secondsString;
    for (backwardIter = inverseMap.rbegin(); backwardIter != inverseMap.rend(); backwardIter++) {
        string name = (*backwardIter).second;
        double time = (*backwardIter).first;

        string padding("                                   ");

        name = name + padding;
        name = name.substr(0, 40);

        sprintf(buffer, "%f", time / totalTime * 100.0);
        timeString = string(buffer);
        timeString = timeString + padding;
        timeString = timeString.substr(0, 10);

        sprintf(buffer, "%02i", hours(time));
        hoursString = string(buffer);

        sprintf(buffer, "%02i", minutes(time));
        minutesString = string(buffer);

        sprintf(buffer, "%02i", seconds(time));
        secondsString = string(buffer);

        cout << "[" << timeString.c_str() << "%]: "
            << name.c_str() << " " << hoursString.c_str() << ":"
            << minutesString << ":"
            << secondsString << " s total ";

        double perFrame = time / frames;
        sprintf(buffer, "%02i", hours(perFrame));
        hoursString = string(buffer);

        sprintf(buffer, "%02i", minutes(perFrame));
        minutesString = string(buffer);

        sprintf(buffer, "%02i", seconds(perFrame));
        secondsString = string(buffer);
        cout << " " << hoursString.c_str() << ":" << minutesString.c_str() << ":" << secondsString.c_str() << " s/frame" << endl;
    }
    sprintf(buffer, "%02i", hours(totalTime));
    hoursString = string(buffer);

    sprintf(buffer, "%02i", minutes(totalTime));
    minutesString = string(buffer);

    sprintf(buffer, "%02i", seconds(totalTime));
    secondsString = string(buffer);
    cout << "==============================================================================================" << endl;
    cout << " Total time: " << hoursString.c_str() << ":" << minutesString.c_str() << ":" << secondsString.c_str() << " (" << totalTime << "s) " << endl;
    cout << " Time per frame: " << totalTime / frames << endl;
    cout << "==============================================================================================" << endl;
}

}
