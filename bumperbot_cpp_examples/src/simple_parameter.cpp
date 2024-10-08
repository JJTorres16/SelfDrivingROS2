#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <string>
#include <vector>
#include <memory>

using std::placeholders::_1;

class SimpleParameterNode : public rclcpp::Node {
    public:
        SimpleParameterNode() : Node("simple_parameter"){
            
            declare_parameter<int>("simple_int_param", 16);
            declare_parameter<std::string>("simple_string_param", "Julian");

            param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameterNode::paramChangeCallback, this, _1));
        }

    private:
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & parameters){

            rcl_interfaces::msg::SetParametersResult result;

            for(const auto& param: parameters){
                
                if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER){
                    RCLCPP_INFO_STREAM(get_logger(), "Parameter simple_int_param changed! New Value is: " <<param.as_int());
                    result.successful = true;
                }

                if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
                    RCLCPP_INFO_STREAM(get_logger(), "Parameter simple_string_param changed! New Value is: " <<param.as_string());
                    result.successful = true;
                }
            }

            return result;
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
