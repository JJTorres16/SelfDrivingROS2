#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"
#include <cmath>

using std::placeholders::_1;

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &name) : Node(name){

    turtle1_pose_sub_ = create_subscription<turtlesim::msg::Pose> ("/turtle1/pose", 10, 
                                                                   std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));
    turtle2_pose_sub_ = create_subscription<turtlesim::msg::Pose> ("/turtle2/pose", 10, 
                                                                   std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));

    RCLCPP_INFO_STREAM(get_logger(), "using the rclcpp!");
 
}


void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose & pose){
    last_turtle1_pose_ = pose;
    
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose & pose){
    last_turtle2_pose_ = pose;

    float Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    float Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;

    // Computation of the rotation matrix
    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = 180*theta_rad / 3.1416;

    RCLCPP_INFO_STREAM(get_logger(), "\nTranslation Vector trutle2 -> turtle1 \n" <<
                                     "Tx: " << Tx << "\n" <<
                                     "Ty: " << Ty << "\n" <<
                                     "Rotation Matrix turtle2 -> turtle1 \n" <<
                                     "theta_rad: " << theta_rad << "\n" <<
                                     "theta_deg: " << theta_deg << "\n" <<
                                     "|" << cos(theta_rad) << "      " << -sin(theta_rad) << "|\n" <<
                                     "|" << sin(theta_rad) << "      " << cos(theta_rad) << "|" );
} 

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}