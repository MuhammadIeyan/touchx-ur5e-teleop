#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>
struct ForceCommand { uint64_t sequence=0; int64_t stamp=0; std::array<double,3> force{}; };
// All quantities SI. Executed only on the servo thread; no allocation or blocking.
class ForceSafety {
 public:
  static constexpr double hardLimit=1.0, slew=2.0, initialEnergy=0.005;
  double limit=0.5;
  std::array<double,3> output{};
  double energy=initialEnergy;
  bool released=false;
  int64_t previous=0;
  std::array<double,3> step(const ForceCommand& c, int64_t now, bool permitted,
                          bool button, const std::array<double,3>& velocity) {
    double dt=previous ? (now-previous)*1e-9 : 0;
    previous=now;
    bool fresh=c.stamp>0 && now>=c.stamp && now-c.stamp<100000000;
    bool finite=true;
    for(int i=0;i<3;++i) finite &= std::isfinite(c.force[i]) && std::isfinite(velocity[i]);
    if (!permitted || !fresh || !finite || !std::isfinite(limit) || limit<0.05 || limit>hardLimit || dt<=0 || dt>0.02) {
      output={}; released=false; return output;
    }
    if (!button) { output={}; released=true; return output; }
    if (!released) { output={}; return output; }
    auto target=c.force;
    double norm=std::hypot(target[0],target[1],target[2]);
    // An explicit zero is a stop request; never delay it with the slew limit.
    if(norm==0) { output={}; return output; }
    if(norm>limit) for(auto& f:target) f*=limit/norm;
    double change=0;
    for(int i=0;i<3;++i) change+=(target[i]-output[i])*(target[i]-output[i]);
    change=std::sqrt(change);
    double fraction=change>0 ? std::min(1.0,slew*dt/change) : 1;
    for(int i=0;i<3;++i) output[i]+=fraction*(target[i]-output[i]);
    // Energy tank bounds net work supplied by delayed feedback; never refilled on clutch.
    double work=0;
    for(int i=0;i<3;++i) work+=output[i]*velocity[i]*dt;
    if(work>energy && work>0) for(auto& f:output) f*=energy/work;
    energy=std::clamp(energy-work,0.0,initialEnergy);
    return output;
  }
};
