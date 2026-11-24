// Standalone Touch X tracking and force-feedback interface.
// Build: g++ -std=gnu++17 -O2 -Wall -Wextra -Wno-attributes -pthread touch_probe.cpp -lHD -o touch_probe
#include <HD/hd.h>
#include <dlfcn.h>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include "force_safety.hpp"

using Clock = std::chrono::steady_clock;
volatile sig_atomic_t running = 1;
void stop(int) { running = 0; }

struct Sample {
    HDdouble position[3]{};
    std::array<double,3> force{};
    bool forceEnabled=false;
    HDdouble transform[16]{};
    long long monotonicNs{};
    HDint buttons{};
    HDenum calibration{};
    HDErrorInfo error{};
    const char* errorCall = "none";
    unsigned long count{};
    unsigned long calibrationCompatEvents{};
};

struct State {
    HHD device;
    Sample latest;
    std::mutex mutex;
    unsigned long count = 0;
    HDint calibrationStyle = 0;
    bool calibrationCompat = false;
    unsigned long calibrationCompatEvents = 0;

    bool forceAllowed=false;
    bool forceOutputActive=false;
    std::atomic<bool> stopping{false};
    std::mutex forceMutex;
    ForceCommand command, servoCommand;
    ForceSafety safety;
    int64_t (*lastRx)()=nullptr;
};

HDCallbackCode HDCALLBACK sample(void* ptr) {
    auto& state = *static_cast<State*>(ptr);
    Sample next;
    auto captureError = [&](const char* call) {
        auto error = hdGetError();
        if (HD_DEVICE_ERROR(error) && !HD_DEVICE_ERROR(next.error)) {
            next.error = error; next.errorCall = call;
        }
    };
    auto calibrationBefore = hdCheckCalibration();
    captureError("hdCheckCalibration before frame");
    if (state.calibrationStyle && calibrationBefore == HD_CALIBRATION_NEEDS_UPDATE) {
        hdDisable(HD_FORCE_OUTPUT);
        state.forceOutputActive=false;
        state.safety.released=false;
        hdUpdateCalibration(state.calibrationStyle);
        auto error = hdGetError();
        if (state.calibrationCompat && state.calibrationStyle == HD_CALIBRATION_AUTO &&
            error.errorCode == HD_INVALID_OPERATION && error.internalErrorCode == 1 &&
            !hdIsEnabled(HD_FORCE_OUTPUT) && hdCheckCalibration() == HD_CALIBRATION_OK) {
            captureError("calibration compatibility verification");
            if (!HD_DEVICE_ERROR(next.error)) ++state.calibrationCompatEvents;
        } else if (HD_DEVICE_ERROR(error) && !HD_DEVICE_ERROR(next.error)) {
            next.error = error; next.errorCall = "hdUpdateCalibration";
        }
    }
    next.calibrationCompatEvents = state.calibrationCompatEvents;
    hdBeginFrame(state.device);
    captureError("hdBeginFrame");
    hdGetDoublev(HD_CURRENT_POSITION, next.position);
    captureError("HD_CURRENT_POSITION");
    hdGetDoublev(HD_CURRENT_TRANSFORM, next.transform);
    captureError("HD_CURRENT_TRANSFORM");
    hdGetIntegerv(HD_CURRENT_BUTTONS, &next.buttons);
    captureError("HD_CURRENT_BUTTONS");
    next.calibration = hdCheckCalibration();
    captureError("hdCheckCalibration in frame");

    if (state.forceAllowed) {
        { std::unique_lock<std::mutex> lock(state.forceMutex, std::try_to_lock);
          if(lock.owns_lock()) state.servoCommand=state.command; }
        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
        HDdouble velocity[3]{};
        hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
        captureError("HD_CURRENT_VELOCITY");
        const auto rx=state.lastRx ? state.lastRx() : 0;
        bool allowed=!state.stopping && running && !HD_DEVICE_ERROR(next.error)
            && next.calibration==HD_CALIBRATION_OK && rx>0 && now>=rx && now-rx<100000000;
        next.force=state.safety.step(state.servoCommand,now,allowed,next.buttons==1,
                                    {velocity[0]*0.001,velocity[1]*0.001,velocity[2]*0.001});
        bool enable=allowed && state.safety.released && next.buttons==1;
        hdSetDoublev(HD_CURRENT_FORCE,next.force.data());
        captureError("HD_CURRENT_FORCE");
        if(enable && !state.forceOutputActive && !HD_DEVICE_ERROR(next.error)) hdEnable(HD_FORCE_OUTPUT);
        if(!enable || HD_DEVICE_ERROR(next.error)) hdDisable(HD_FORCE_OUTPUT);
        captureError("force output state");
        state.forceOutputActive=enable && !HD_DEVICE_ERROR(next.error);
        next.forceEnabled=state.forceOutputActive;
    }

    hdEndFrame(state.device);
    captureError("hdEndFrame");
    if(HD_DEVICE_ERROR(next.error) && state.forceAllowed) {
        hdDisable(HD_FORCE_OUTPUT); state.forceOutputActive=false;
        next.force={}; next.forceEnabled=false;
    }
    next.count = ++state.count;
    next.monotonicNs = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
    // Never wait for the console thread in the servo callback.
    std::unique_lock<std::mutex> lock(state.mutex, std::try_to_lock);
    if (lock.owns_lock()) state.latest = next;
    return HD_DEVICE_ERROR(next.error) ? HD_CALLBACK_DONE : HD_CALLBACK_CONTINUE;
}

int main(int argc, char** argv) {
    const int rate = argc > 1 ? std::atoi(argv[1]) : 1000;
    const int duration = argc > 2 ? std::atoi(argv[2]) : 30;
    if (rate < 50 || rate > 1000 || duration < 1 || duration > 3600) {
        std::cerr << "Usage: touch_probe [rate:50..1000 Hz] [duration:1..3600 seconds]\n";
        return 2;
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::cout << std::unitbuf;
    // Watchdog for blocked vendor calls.
    std::atomic<long long> deadline{0};
    const auto nowMs = [] { return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count(); };
    deadline = nowMs() + 10000;
    std::atomic<bool> finished{false};
    std::thread watchdog([&] {
        while (!finished.load()) {
            if (nowMs() > deadline.load()) {
                std::cerr << "WATCHDOG: vendor call blocked; exiting diagnostic. Power-cycle Touch X before retrying.\n";
                std::_Exit(3);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    const auto errorCheck = [](const char* where) {
        auto error = hdGetError();
        if (!HD_DEVICE_ERROR(error)) return false;
        std::cerr << where << ": " << hdGetErrorString(error.errorCode)
                  << " code=" << error.errorCode << " internal=" << error.internalErrorCode << '\n';
        return true;
    };
    std::cout << "Initializing Touch X interface; requested rate=" << rate << " Hz\n";
    State state{};
    const char* forceOption=std::getenv("TOUCH_ENABLE_FORCE");
    state.forceAllowed=forceOption && std::string(forceOption)=="1";
    if(state.forceAllowed && (!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="42")) {
        std::cerr << "Force output requires simulation domain 42\n";
        finished=true; watchdog.join(); return 2;
    }
    if(const char* setting=std::getenv("TOUCH_FORCE_LIMIT_N")) {
        char* end=nullptr;
        double limit=std::strtod(setting,&end);
        if(end==setting || *end || !std::isfinite(limit) || limit<0.05 || limit>ForceSafety::hardLimit) {
            std::cerr << "Force limit must be 0.05..1.0 N\n";
            finished=true; watchdog.join(); return 2;
        }
        state.safety.limit=limit;
    }
    state.lastRx=reinterpret_cast<int64_t (*)()>(dlsym(RTLD_DEFAULT,"touch_last_rx_ns"));
    const char* compat = std::getenv("TOUCH_CALIBRATION_COMPAT");
    state.calibrationCompat = compat && std::string(compat) == "1";
    state.device = hdInitDevice(HD_DEFAULT_DEVICE);
    if (errorCheck("initialize")) { finished = true; watchdog.join(); return 1; }
    hdDisable(HD_FORCE_OUTPUT);
    if(state.forceAllowed) {
        hdEnable(HD_FORCE_RAMPING);
        hdEnable(HD_SOFTWARE_FORCE_LIMIT);
        hdEnable(HD_SOFTWARE_VELOCITY_LIMIT);
        hdEnable(HD_SOFTWARE_FORCE_IMPULSE_LIMIT);
    }
    HDint styles = 0;
    hdGetIntegerv(HD_CALIBRATION_STYLE, &styles);
    if (styles & HD_CALIBRATION_INKWELL) state.calibrationStyle = HD_CALIBRATION_INKWELL;
    else if (styles & HD_CALIBRATION_AUTO) state.calibrationStyle = HD_CALIBRATION_AUTO;
    std::cout << "Calibration styles=" << styles << "; selected=" << state.calibrationStyle << '\n';
    for (auto field : {HD_DEVICE_MODEL_TYPE, HD_DEVICE_DRIVER_VERSION}) {
        const char* value = hdGetString(field);
        std::cout << "Device info " << field << ": " << (value ? value : "unavailable") << '\n';
    }
    if (errorCheck("device setup")) {
        hdDisableDevice(state.device); finished = true; watchdog.join(); return 1;
    }
    hdSetSchedulerRate(rate);
    if (errorCheck("requested scheduler rate")) {
        hdDisableDevice(state.device); finished = true; watchdog.join(); return 1;
    }
    auto handle = hdScheduleAsynchronous(sample, &state, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    int result = errorCheck("start scheduler") ? 1 : 0;
    auto start = Clock::now(), lastChange = start;
    unsigned long previous = 0;
    unsigned long reportedCompatEvents = 0;
    const bool stream = std::getenv("TOUCH_STREAM_JSON") != nullptr;
    auto lastRx = reinterpret_cast<int64_t (*)()>(dlsym(RTLD_DEFAULT, "touch_last_rx_ns"));
    unsigned long sent = 0;

    // Nonblocking bounded input, no detached thread and no blocking servo mutex.
    if(state.forceAllowed) fcntl(STDIN_FILENO,F_SETFL,fcntl(STDIN_FILENO,F_GETFL)|O_NONBLOCK);
    std::string input;
    uint64_t lastSequence=0;
    while (!result && running && Clock::now() - start < std::chrono::seconds(duration)) {
        deadline = nowMs() + 5000;
        Sample current;
        { std::lock_guard<std::mutex> lock(state.mutex); current = state.latest; }
        if (current.calibrationCompatEvents != reportedCompatEvents) {
            reportedCompatEvents = current.calibrationCompatEvents;
            std::cout << "Calibration updated and verified OK; handled driver/SDK return-code mismatch.\n";
        }

        if(state.forceAllowed) {
            char buffer[512];
            auto size=read(STDIN_FILENO,buffer,sizeof(buffer));
            if(size==0) state.stopping=true;
            if(size>0) input.append(buffer,size);
            if(input.size()>4096) { state.stopping=true; input.clear(); }
            size_t end;
            while((end=input.find('\n'))!=std::string::npos) {
                auto line=input.substr(0,end); input.erase(0,end+1);
                ForceCommand c; std::string extra; std::istringstream parser(line);
                bool valid=bool(parser>>c.sequence>>c.stamp>>c.force[0]>>c.force[1]>>c.force[2]) && !(parser>>extra);
                for(auto f:c.force) valid &= std::isfinite(f);
                if(valid && c.sequence>lastSequence) {
                    lastSequence=c.sequence;
                    std::lock_guard<std::mutex> lock(state.forceMutex); state.command=c;
                } else { std::lock_guard<std::mutex> lock(state.forceMutex); state.command={}; }
            }
        }

        if (current.count != previous) { previous = current.count; lastChange = Clock::now(); }
        if (Clock::now() - lastChange > std::chrono::seconds(2)) {
            std::cerr << "STALL: no completed frames for 2 seconds\n";
            result = 3;
            break;
        }
        if (stream && current.count != sent && !HD_DEVICE_ERROR(current.error)) {
            std::cout << std::setprecision(17) << "{\"seq\":" << current.count
                      << ",\"monotonic_ns\":" << current.monotonicNs
                      << ",\"transport_ns\":" << (lastRx ? lastRx() : 0)
                      << ",\"force_enabled\":" << (current.forceEnabled ? "true" : "false")
                      << ",\"force_n\":[" << current.force[0] << ',' << current.force[1] << ',' << current.force[2] << "]"
                      << ",\"xyz_mm\":[" << current.position[0] << ',' << current.position[1] << ',' << current.position[2]
                      << "],\"buttons\":" << current.buttons << ",\"calibration\":" << current.calibration
                      << ",\"calibration_epoch\":" << current.calibrationCompatEvents << ",\"transform\":[";
            for (int i = 0; i < 16; ++i) { if (i) std::cout << ','; std::cout << current.transform[i]; }
            std::cout << "]}\n";
            sent = current.count;
        } else if (!stream) std::cout << "frames=" << current.count << " xyz=" << current.position[0] << ','
                  << current.position[1] << ',' << current.position[2] << " buttons=" << current.buttons
                  << " calibration=" << current.calibration << '\n';
        if (HD_DEVICE_ERROR(current.error)) {
            std::cerr << "servo at " << current.errorCall << ": " << hdGetErrorString(current.error.errorCode) << " code="
              << current.error.errorCode << " internal=" << current.error.internalErrorCode << '\n';
            result = 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(stream ? 10 : 200));
    }
    std::cout << "Stopping; frames=" << previous << '\n';
    deadline = nowMs() + 3000;
    state.stopping=true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    hdStopScheduler();
    hdUnschedule(handle);
    hdDisableDevice(state.device);
    finished = true;
    watchdog.join();

    std::cout << "Closed; result=" << result << '\n';
    return result;
}
