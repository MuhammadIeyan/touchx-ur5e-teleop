#include "../../drivers/touch_x/force_safety.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  ForceSafety s; ForceCommand c{1,1000000000,{10,0,0}};
  auto now=c.stamp;
  auto tick=[&](bool permission,bool button,std::array<double,3> velocity=std::array<double,3>{}) {
    now+=1000000; c.stamp=now;
    return s.step(c,now,permission,button,velocity);
  };
  assert(tick(true,true)[0]==0); // Held at start cannot arm.
  tick(true,false);
  assert(tick(true,true)[0]<=0.002001);
  for(int i=0;i<400;++i) tick(true,true);
  assert(std::abs(s.output[0]-0.5)<1e-9);
  c.force={}; assert(tick(true,true)[0]==0); // Explicit stop bypasses slew.
  c.force={10,0,0}; tick(true,true);
  assert(tick(true,false)[0]==0); // Immediate zero, not slew-limited.
  tick(true,true);
  assert(tick(false,true)[0]==0); // Calibration/transport/shutdown permission lost.
  assert(tick(true,true)[0]==0);
  tick(true,false);tick(true,true);
  c.stamp=now-100000001;
  assert(s.step(c,now+1000000,true,true,{})[0]==0);
  tick(true,false);c.force[0]=std::numeric_limits<double>::quiet_NaN();
  assert(tick(true,true)[0]==0);
  c.force={0.5,0,0};tick(true,false);
  double work=0;
  for(int i=0;i<2000;++i) work+=tick(true,true,{1,0,0})[0]*0.001;
  assert(work<=ForceSafety::initialEnergy+1e-10);
  assert(s.energy==0);
  ForceSafety responsive; responsive.limit=.8;
  c.force={0,10,0};
  for(int i=0;i<600;++i) {
    now+=1000000; c.stamp=now;
    responsive.step(c,now,true,i>1,{});
  }
  assert(std::abs(responsive.output[1]-.8)<1e-9);
  responsive.limit=1.01;now+=1000000;c.stamp=now;
  assert(responsive.step(c,now,true,true,{})[1]==0);
  auto before=now; assert(s.step(c,before-1,true,true,{})[0]==0);
  std::cout<<"Force safety: startup, cap, slew, release, fault, stale, NaN, work budget, clock checks passed\n";
}
