#include "contact_force.hpp"
#include <cassert>
#include <iostream>
int main() {
  const char* xml=R"(<mujoco><worldbody><body name="surgical_environment"><geom name="floor" type="plane" size="1 1 .1"/></body><body pos="0 0 .09"><freejoint/><geom name="instrument_tip_collision" type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>)";
  char error[1024];
  auto spec=mj_parseXMLString(xml,nullptr,error,sizeof(error));
  assert(spec);auto m=mj_compile(spec,nullptr);assert(m);auto d=mj_makeData(m);
  int tip=mj_name2id(m,mjOBJ_GEOM,"instrument_tip_collision");
  int env=mj_name2id(m,mjOBJ_BODY,"surgical_environment");
  mj_forward(m,d);
  auto f=sim_contact_feedback::toolContactForce(m,d,tip,env);
  assert(f[2]>0 && std::abs(f[0])<1e-10 && std::abs(f[1])<1e-10);
  // Identical contact represented with reversed geom order and frame normal.
  assert(d->ncon==1);
  std::swap(d->contact[0].geom[0],d->contact[0].geom[1]);
  for(auto& x:d->contact[0].frame) x=-x;
  auto reversed=sim_contact_feedback::toolContactForce(m,d,tip,env);
  for(int j=0;j<3;++j) assert(std::abs(reversed[j]-f[j])<1e-10);
  d->qpos[2]=.2;mj_forward(m,d);
  auto free=sim_contact_feedback::toolContactForce(m,d,tip,env);
  assert(free[0]==0 && free[1]==0 && free[2]==0);
  std::cout<<"MuJoCo contact extraction: upward reaction, reversed geom order, free-space zero passed\n";
  mj_deleteData(d);mj_deleteModel(m);mj_deleteSpec(spec);
}
