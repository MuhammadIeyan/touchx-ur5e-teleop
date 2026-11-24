#pragma once
#include <mujoco/mujoco.h>
#include <array>
namespace sim_contact_feedback {
inline std::array<double,3> toolContactForce(const mjModel* m, const mjData* d, int tip, int environment) {
  std::array<double,3> total{};
  for(int i=0;i<d->ncon;++i) {
    const auto& c=d->contact[i];
    bool first=c.geom[0]==tip, second=c.geom[1]==tip;
    if(!first && !second) continue;
    int other=c.geom[first ? 1 : 0];
    if(m->geom_bodyid[other]!=environment) continue;
    mjtNum local[6]{};
    mj_contactForce(m,d,i,local);
    // Contact axes are rows; force acts on geom[1].
    for(int j=0;j<3;++j)
      for(int k=0;k<3;++k) total[j]+=(second ? 1 : -1)*c.frame[3*k+j]*local[k];
  }
  return total;
}
}
