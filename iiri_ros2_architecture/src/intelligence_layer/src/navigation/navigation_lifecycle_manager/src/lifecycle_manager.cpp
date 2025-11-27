#include "navigation_lifecycle_manager/lifecycle_manager.hpp"


using namespace std::placeholders;
using namespace std::chrono_literals;


namespace navigation_lifecycle_manager {

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options) : rclcpp::Node("lifecycle_manager_node", options) {
    RCLCPP_INFO(this->get_logger(), "Hello lifecycle_manager_node !");
    
    system_active_ = false;
    
    this->declare_parameter("auto_start", true);
    this->declare_parameter("node_names", node_names_);

    node_names_ = this->get_parameter("node_names").as_string_array();

    transition_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    for (size_t i = 0; i < node_names_.size(); i++) {
        change_states_client_map_[node_names_[i]] = this->create_client<lifecycle_msgs::srv::ChangeState>(node_names_[i]+"/change_state",
            rmw_qos_profile_services_default, transition_callback_group_);
        get_states_client_map_[node_names_[i]] = this->create_client<lifecycle_msgs::srv::GetState>(node_names_[i]+"/get_state",
            rmw_qos_profile_services_default, transition_callback_group_);
    }

    transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
    transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
    transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] = State::PRIMARY_STATE_FINALIZED;

    states_label_[State::PRIMARY_STATE_UNCONFIGURED] = "Unconfigure";
    states_label_[State::PRIMARY_STATE_INACTIVE] = "Inactive";
    states_label_[State::PRIMARY_STATE_ACTIVE] = "Active";
    states_label_[State::PRIMARY_STATE_FINALIZED] = "Finalized";

    init_transition_table();

    auto_start_ = this->get_parameter("auto_start").as_bool();
    if (auto_start_) {
        RCLCPP_WARN(this->get_logger(), "Lifecycle node auto start !");
        /* The reason use timer here is to wait for all node construction complete and composable container start to spin then to call transition service,
            If not operate like this, the composable node start operation will block in this construct function at waiting get_state response 
            and won't spin node to communicate with each other.
        */ 
        auto_start_timer_ = this->create_wall_timer(0s, [this](){ 
            auto_start_timer_->cancel(),
            start_all_node();
        });
    } 
}

void LifecycleManager::start_all_node() {
    RCLCPP_INFO(this->get_logger(), "start_all_node");
    if (!change_states_for_all_node(State::PRIMARY_STATE_ACTIVE)) {
        RCLCPP_ERROR(this->get_logger(), "Active state transition for all node failed !");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Active state transition for all node success !");
    system_active_ = true;
}

void LifecycleManager::pause_all_node() {
    RCLCPP_INFO(this->get_logger(), "pause_all_node");
    if (!change_states_for_all_node(State::PRIMARY_STATE_INACTIVE)) {
        RCLCPP_ERROR(this->get_logger(), "Inactivate state transition for all node failed !");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Inactivate state transition for all node success !");
    system_active_ = false;
}

void LifecycleManager::clean_all_node() {
    RCLCPP_INFO(this->get_logger(), "clean_all_node");
    if (!change_states_for_all_node(State::PRIMARY_STATE_UNCONFIGURED)) {
        RCLCPP_ERROR(this->get_logger(), "Unconfigure state transition for all node failed !");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Unconfigure state transition for all node success !");
    system_active_ = false;
}

void LifecycleManager::shutdown_all_node() {
    RCLCPP_INFO(this->get_logger(), "shutdown_all_node");
    if (!change_states_for_all_node(State::PRIMARY_STATE_FINALIZED)) {
        RCLCPP_ERROR(this->get_logger(), "Finalized transition for all node failed !");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Finalized transition for all node success !");
    system_active_ = false;
}

bool LifecycleManager::change_states_for_node(const std::string & node_name, const std::uint8_t & transition_state) {
    if (!get_states_client_map_[node_name]->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Timeout for lifecycle %s/get_state service online !", node_name.c_str());
        return false;
    }

    RCLCPP_WARN(this->get_logger(), "debug: 转换节点名称为： %s !", node_name.c_str());
    
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = get_states_client_map_[node_name]->async_send_request(request);
    if (future.wait_for(1s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout for lifecycle %s/get_state service response !", node_name.c_str());
        return false;
    }
    auto response = future.get();

    RCLCPP_WARN(this->get_logger(), "debug: 当前 %s节点状态为: %s !", node_name.c_str(), this->states_label_[response->current_state.id].c_str());
    
    std::pair start_end_state = std::make_pair(response->current_state.id, transition_state);
    if (transition_table_.find(start_end_state) == transition_table_.end()) {
        RCLCPP_ERROR(this->get_logger(), "State transition from %s to %s doesn't exist !", states_label_[start_end_state.first].c_str(), states_label_[start_end_state.second].c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Start %s state transition from %s to %s", node_name.c_str(), states_label_[start_end_state.first].c_str(), states_label_[start_end_state.second].c_str());
    std::vector<uint8_t> transitions = transition_table_[start_end_state];
    for (auto it = transitions.begin(); it != transitions.end(); it++) {
        if (!change_states_one_step(node_name, *it)) {
            RCLCPP_ERROR(this->get_logger(), "%s State transition failed ! ", node_name.c_str());
            return false;
        }
    }

    RCLCPP_INFO(this->get_logger(), "%s State transition from %s to %s success !", node_name.c_str(), states_label_[start_end_state.first].c_str(), states_label_[start_end_state.second].c_str());
    return true;
}

bool LifecycleManager::change_states_one_step(const std::string & node_name, const std::uint8_t & transition) {

    if (!change_states_client_map_[node_name]->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Timeout for %s/change_state service online !", node_name.c_str());
        return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition; 
    auto future_result = change_states_client_map_[node_name]->async_send_request(request);
    if (future_result.wait_for(2s) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout for %s/change_state service response !", node_name.c_str());
        return false;
    }

    std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response> response = future_result.get();
    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "%s State transition failed !", node_name.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "%s State transition success !", node_name.c_str());
    return true;
}

bool LifecycleManager::change_states_for_all_node(const std::uint8_t & transition_state) {
    RCLCPP_INFO(this->get_logger(), "Start state transition for all node to %s!", states_label_[transition_state].c_str());

    RCLCPP_WARN(this->get_logger(), "debug: 节点数量为： %ld !", node_names_.size());
    RCLCPP_WARN(this->get_logger(), "debug: 想要转换的状态为： %s !", states_label_[transition_state].c_str());
    
    for (size_t i = 0; i < node_names_.size(); i++) {
        if (!change_states_for_node(node_names_[i], transition_state)) {
            RCLCPP_ERROR(this->get_logger(), "State transition for %s to state %s failed !", node_names_[i].c_str(), states_label_[transition_state].c_str());
            return false;
        };
    };
    RCLCPP_INFO(this->get_logger(), "State transition for all node to state %s success !", states_label_[transition_state].c_str());
    return true;
}

void LifecycleManager::init_transition_table() {

    /* start from unconfigure */
    std::vector<std::uint8_t> in_place_conversion = {};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_UNCONFIGURED)] = 
        in_place_conversion;
    
    std::vector<std::uint8_t> unconfigure_to_inactive = {Transition::TRANSITION_CONFIGURE};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_INACTIVE)] = 
        unconfigure_to_inactive;

    std::vector<std::uint8_t> unconfigure_to_active = {Transition::TRANSITION_CONFIGURE, Transition::TRANSITION_ACTIVATE};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_ACTIVE)] = 
        unconfigure_to_active;

    std::vector<std::uint8_t> unconfigure_to_shutdown = {Transition::TRANSITION_UNCONFIGURED_SHUTDOWN};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_UNCONFIGURED, State::PRIMARY_STATE_FINALIZED)] = 
        unconfigure_to_shutdown;
    
    /* start from inactive */
    std::vector<std::uint8_t> inactive_to_unconfigure = {Transition::TRANSITION_CLEANUP};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_UNCONFIGURED)] =
        inactive_to_unconfigure;

    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_INACTIVE)] =
        in_place_conversion;
    
    std::vector<std::uint8_t> inactive_to_active = {Transition::TRANSITION_ACTIVATE};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_ACTIVE)] = 
        inactive_to_active;

    std::vector<std::uint8_t> inactive_to_shutdown = {Transition::TRANSITION_CLEANUP, Transition::TRANSITION_UNCONFIGURED_SHUTDOWN};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_INACTIVE, State::PRIMARY_STATE_FINALIZED)] = 
        inactive_to_shutdown;

    /* start from active */
    std::vector<std::uint8_t> active_to_unconfigure = {Transition::TRANSITION_DEACTIVATE, Transition::TRANSITION_CLEANUP};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_UNCONFIGURED)] =
        active_to_unconfigure;
    
    std::vector<std::uint8_t> active_to_inactive = {Transition::TRANSITION_DEACTIVATE};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_INACTIVE)] = 
        active_to_inactive;

    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_ACTIVE)] = 
        in_place_conversion;

    std::vector<std::uint8_t> active_to_shutdown = 
        {Transition::TRANSITION_DEACTIVATE, Transition::TRANSITION_CLEANUP, Transition::TRANSITION_UNCONFIGURED_SHUTDOWN};
    transition_table_[std::pair<std::uint8_t, std::uint8_t>(State::PRIMARY_STATE_ACTIVE, State::PRIMARY_STATE_FINALIZED)] = 
        active_to_shutdown;
}

LifecycleManager::~LifecycleManager() {

    RCLCPP_WARN(this->get_logger(), "Deconstruction...");
    
    for (auto it = change_states_client_map_.begin(); it != change_states_client_map_.end(); it++) {
        it->second.reset();
    }
    for (auto it = get_states_client_map_.begin(); it != get_states_client_map_.end(); it++) {
        it->second.reset();
    }
    change_states_client_map_.clear();
    get_states_client_map_.clear();
    auto_start_timer_.reset();
    transition_callback_group_.reset();
    RCLCPP_WARN(this->get_logger(), "Deconstruction done !");
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation_lifecycle_manager::LifecycleManager);