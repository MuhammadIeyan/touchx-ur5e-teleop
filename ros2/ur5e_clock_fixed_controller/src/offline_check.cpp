// Exercise the real controller lifecycle against memory-backed interfaces only.
#ifdef CHECK_VENDOR
#include "ur_controllers/scaled_joint_trajectory_controller.hpp"
using Base = ur_controllers::ScaledJointTrajectoryController;
#else
#include "ur5e_clock_fixed_controller/scaled_joint_trajectory_controller.hpp"
using Base = ur5e_clock_fixed_controller::ScaledJointTrajectoryController;
#endif
#include <iostream>
#include <cmath>
#include <stdexcept>

class CheckController : public Base {
public:
  void submit(const std::vector<std::string>& names, const std::vector<double>& q) {
    auto msg=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    msg->joint_names=names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions=q; point.positions[0]+=.001;
    point.velocities.assign(6,0.); point.accelerations.assign(6,0.);
    point.time_from_start.nanosec=300000000;
    msg->points.push_back(point);
    rt_is_holding_=false;
    traj_msg_external_point_ptr_.writeFromNonRT(msg);
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try {
    CheckController controller;
    std::vector<std::string> names={"j0","j1","j2","j3","j4","j5"};
    auto options=rclcpp::NodeOptions().allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({rclcpp::Parameter("joints",names),
      rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"}),
      rclcpp::Parameter("open_loop_control",true),
      rclcpp::Parameter("update_rate",int64_t(500)),
      rclcpp::Parameter("speed_scaling_interface_name",std::string("speed_scaling/factor"))});
    if(controller.init("clock_check","",options)!=controller_interface::return_type::OK)
      throw std::runtime_error("init failed");
    if(controller.configure().id()!=2) throw std::runtime_error("configure failed");
    std::vector<double> command(6,.1),position(6,.1),velocity(6,0.);
    double scale=1.;
    std::vector<hardware_interface::CommandInterface> commands;
    std::vector<hardware_interface::StateInterface> states;
    commands.reserve(6); states.reserve(13);
    for(size_t j=0;j<6;++j) {
      commands.emplace_back(names[j],"position",&command[j]);
      states.emplace_back(names[j],"position",&position[j]);
      states.emplace_back(names[j],"velocity",&velocity[j]);
    }
    states.emplace_back("speed_scaling","factor",&scale);
    std::vector<hardware_interface::LoanedCommandInterface> loan_commands;
    std::vector<hardware_interface::LoanedStateInterface> loan_states;
    for(auto& c:commands)loan_commands.emplace_back(c);
    for(auto& s:states)loan_states.emplace_back(s);
    controller.assign_interfaces(std::move(loan_commands),std::move(loan_states));
    if(controller.get_node()->activate().id()!=3)throw std::runtime_error("activate failed");
    auto now=controller.get_node()->get_clock()->now();
    const auto period=rclcpp::Duration::from_seconds(.002);
    double max_step=0.;
    for(int i=0;i<2000;++i) {
      if(i==600)scale=.2;
      if(i>=100 && i<1000 && i%25==0)controller.submit(names,command);
      controller.update(now+period*i,period);
      for(size_t j=0;j<6;++j) {
        double step=std::abs(command[j]-position[j]);
        if(!std::isfinite(command[j]) || step>.0001)throw std::runtime_error("command discontinuity");
        max_step=std::max(max_step,step);
        velocity[j]=(command[j]-position[j])/.002;position[j]=command[j];
      }
    }
    if(position[0]<=.101)throw std::runtime_error("no tracking progression");
    for(double v:velocity)if(std::abs(v)>1e-6)throw std::runtime_error("finite endpoint did not stop");
    controller.get_node()->deactivate();
    controller.release_interfaces();
    std::cout<<"PASS: activation, 36 replacements, speed scaling, finite hold; max joint step="<<max_step<<" rad\n";
  } catch(const std::exception& e) {
    std::cerr<<"CONTROLLER EXCEPTION: "<<e.what()<<"\n"; result=1;
  }
  rclcpp::shutdown();return result;
}
