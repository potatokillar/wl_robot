#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "std_srvs/srv/trigger.hpp"


namespace navigation_lifecycle_manager{

using Transition = lifecycle_msgs::msg::Transition;
using State = lifecycle_msgs::msg::State;
using namespace std::chrono_literals;
using namespace std::placeholders;

class LifecycleManager : public rclcpp::Node {
public:
    LifecycleManager(const rclcpp::NodeOptions & options);
    void start_all_node();
    void pause_all_node();
    void clean_all_node();
    void shutdown_all_node();
    bool change_states_for_node(const std::string & node_name, const std::uint8_t & transition_state);
    bool change_states_one_step(const std::string & node_name, const std::uint8_t & transition);
    bool change_states_for_all_node(const std::uint8_t & transition_state);

    /*wrap process function*/
    void init_transition_table();
    
    ~LifecycleManager();

private:
    std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> change_states_client_map_;
    std::map<std::string, std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> get_states_client_map_;
    std::map<std::pair<std::uint8_t, std::uint8_t>, std::vector<std::uint8_t>> transition_table_;
    std::map<std::uint8_t, std::uint8_t> transition_state_map_;
    std::map<std::uint8_t, std::string> states_label_;
    std::vector<std::string> node_names_;
    bool auto_start_;
    bool system_active_;

    rclcpp::CallbackGroup::SharedPtr transition_callback_group_;
    rclcpp::TimerBase::SharedPtr auto_start_timer_;
    
};

}
