#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cstdlib>
#include <string>
#include "contact_force.hpp"
namespace sim_contact_feedback {
class ToolContact : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  int tip_=-1, environment_=-1;
  double lastTime_=-1;
 public:
  bool init(rclcpp::Node::SharedPtr node,const mjModel* m,mjData*) override {
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    if(!domain || std::string(domain)!="42") return false;
    tip_=mj_name2id(m,mjOBJ_GEOM,"instrument_tip_collision");
    environment_=mj_name2id(m,mjOBJ_BODY,"surgical_environment");
    // The common simulation launch also supports models without a grasper.
    if(tip_<0 || environment_<0) {
      RCLCPP_WARN(node->get_logger(),"No instrument contact geometry; force feedback unavailable");
      return true;
    }
    publisher_=node->create_publisher<geometry_msgs::msg::WrenchStamped>("/simulation/tool_contact",rclcpp::QoS(1));
    return true;
  }
  void update(const mjModel* m,mjData* d) override {
    if(!publisher_ || d->time==lastTime_) return;
    lastTime_=d->time; // Never freshen a paused simulation snapshot.
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.frame_id="world";
    msg.header.stamp=rclcpp::Clock(RCL_SYSTEM_TIME).now();
    auto total=toolContactForce(m,d,tip_,environment_);
    msg.wrench.force.x=total[0]; msg.wrench.force.y=total[1]; msg.wrench.force.z=total[2];
    publisher_->publish(msg);
  }
  void cleanup() override { publisher_.reset(); }
};
}
PLUGINLIB_EXPORT_CLASS(sim_contact_feedback::ToolContact,mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
