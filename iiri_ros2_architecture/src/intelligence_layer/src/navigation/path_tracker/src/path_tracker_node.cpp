#include "path_tracker/path_tracker_node.hpp"

using namespace std::placeholders;

namespace path_tracker {

PathTrackerNode::PathTrackerNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("path_tracker_node", options) {
    RCLCPP_INFO(this->get_logger(), "Hello path_tracker_node !");
    
    this->declare_parameter<std::vector<std::string>>("path_tracker_plugins", {"path_tracker/SimplePathTracker"});
    this->declare_parameter<std::vector<std::string>>("goal_checker_plugins", {"path_tracker/SimpleGoalChecker"});
    this->declare_parameter<std::string>("default_path_tracker", "path_tracker/SimplePathTracker");
    this->declare_parameter<std::string>("default_goal_checker", "path_tracker/SimpleGoalChecker");
    this->declare_parameter<double>("control_frequency", 20.0);
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("robot_frame_id", "laser_link");
    this->declare_parameter<std::string>("sensor_frame_id", "laser_link");
    this->declare_parameter<double>("transform_tolerance", 0.1);
    this->declare_parameter<std::string>("progress_checker_plugin", "path_tracker/SimpleProgressChecker");
    path_tracker_loader_ = std::make_shared<pluginlib::ClassLoader<navigation_core::BasePathTracker>>("navigation_core", "navigation_core::BasePathTracker");
    progress_checker_loader_ = std::make_shared<pluginlib::ClassLoader<navigation_core::BaseProgressChecker>>("navigation_core", "navigation_core::BaseProgressChecker");
    goal_checker_loader_ = std::make_shared<pluginlib::ClassLoader<navigation_core::BaseGoalChecker>>("navigation_core", "navigation_core::BaseGoalChecker"); 
    return;
}

CallbackReturn PathTrackerNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "on_configure !");

    flag_active_ = false;
    flag_stop_execute_ = true;

    last_cmd_vel_.linear.x = 0;
    last_cmd_vel_.linear.y = 0;
    last_cmd_vel_.linear.z = 0;
    last_cmd_vel_.angular.x = 0;
    last_cmd_vel_.angular.y = 0;
    last_cmd_vel_.angular.z = 0;

    control_frequency_ = this->get_parameter("control_frequency").as_double();
    double transform_tolerance_d = this->get_parameter("transform_tolerance").as_double();
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance_d);
    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
    sensor_frame_id_ = this->get_parameter("sensor_frame_id").as_string();

    std::vector<std::string> path_tracker_plugins = this->get_parameter("path_tracker_plugins").as_string_array();
    std::vector<std::string> goal_checker_plugins = this->get_parameter("goal_checker_plugins").as_string_array();
    std::string progress_checker_plugin = this->get_parameter("progress_checker_plugin").as_string();
    default_path_tracker_ = this->get_parameter("default_path_tracker").as_string();
    default_goal_checker_ = this->get_parameter("default_goal_checker").as_string();
    
    auto node = shared_from_this();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 5);

    /* initialize path tracker */
    for (size_t i = 0; i < path_tracker_plugins.size(); i++) {
        try {
            std::shared_ptr<navigation_core::BasePathTracker> path_tracker = path_tracker_loader_->createSharedInstance(path_tracker_plugins[i]);
            path_tracker->configure(node);
            path_trackers_.insert({path_tracker_plugins[i], path_tracker});
        } catch (pluginlib::LibraryLoadException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::LibraryLoadException when config %s", path_tracker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        } catch (pluginlib::CreateClassException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::CreateClassException when config %s", path_tracker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        } catch (pluginlib::PluginlibException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::PluginlibException when config %s", path_tracker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        }
    }
    if (path_trackers_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "There is no path tracker ! Make sure path_tracker_plugins parameters is not empty");
        RCLCPP_ERROR(this->get_logger(), "Configure failed !");
        return CallbackReturn::FAILURE;
    }

    /* initialize goal checker */
    for (size_t i = 0; i < goal_checker_plugins.size(); i++) {
        try {
            std::shared_ptr<navigation_core::BaseGoalChecker> goal_checker = goal_checker_loader_->createSharedInstance(goal_checker_plugins[i]);
            goal_checker->configure(node);
            goal_checkers_.insert({goal_checker_plugins[i], goal_checker});
        } catch (pluginlib::LibraryLoadException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::LibraryLoadException when config %s", goal_checker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        } catch (pluginlib::CreateClassException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::CreateClassException when config %s", goal_checker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        } catch (pluginlib::PluginlibException &e) {
            RCLCPP_ERROR(this->get_logger(), "pluginlib::PluginlibException when config %s", goal_checker_plugins[i].c_str());
            RCLCPP_ERROR(this->get_logger(), "Configure failed !");
            return CallbackReturn::FAILURE;
        }
    }
    if (goal_checkers_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "There is no path tracker ! Make sure goal_checker_plugins parameters is not empty");
        RCLCPP_ERROR(this->get_logger(), "Configure failed !");
        return CallbackReturn::FAILURE;
    }

    /* initialize progress checker */
    progress_checker_ = progress_checker_loader_->createSharedInstance(progress_checker_plugin);
    progress_checker_->configure(node);


    path_track_action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    path_track_action_server_ = rclcpp_action::create_server<PathTrackAction>(
        this, 
        "track_path", 
        std::bind(&PathTrackerNode::handle_goal,this,_1,_2), 
        std::bind(&PathTrackerNode::handle_cancel,this,_1),
        std::bind(&PathTrackerNode::handle_accept,this,_1)
        // ,
        // rcl_action_server_get_default_options(),
        // // todo: 外部使用rclcpp_components 启动 composable_node时要指定为多线程启动该参数才会起效
        // path_track_action_callback_group_
    );
    RCLCPP_INFO(this->get_logger(), "Configure done !");
    return CallbackReturn::SUCCESS;
    
}

PathTrackerNode::CallbackReturn PathTrackerNode::on_activate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "on_activate !");
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        flag_active_ = true;
        flag_stop_execute_ = false;
        current_handle_.reset();
        pending_handle_.reset();
    }
    cmd_vel_pub_->on_activate();

    RCLCPP_INFO(get_logger(), "Activate done !");
    
    return CallbackReturn::SUCCESS;
}

PathTrackerNode::CallbackReturn PathTrackerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "on_deactivate !");
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        flag_active_ = false;
        flag_stop_execute_ = true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Wait for the execute thread to complete !");
    if (execute_future_.valid() && execute_future_.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Shutdown execute thread timeout !");
    };

    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        publish_zero_velocity();
        current_handle_.reset();
        pending_handle_.reset();
        cmd_vel_pub_->on_deactivate();
    }
    RCLCPP_INFO(get_logger(), "Deactivate done !");
    return CallbackReturn::SUCCESS;
}

PathTrackerNode::CallbackReturn PathTrackerNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "on_shutdown !");
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        flag_active_ = false;
        flag_stop_execute_ = true;
    }
    
    if (execute_future_.valid() && execute_future_.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Shutdown execute thread timeout !");
    };

    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        current_handle_.reset();
        pending_handle_.reset();
        cmd_vel_pub_.reset();
        tf_buffer_.reset();
        tf_listener_.reset();
        for (auto it = path_trackers_.begin(); it != path_trackers_.end(); it++) {
            it->second.reset();
        }
        for (auto it = goal_checkers_.begin(); it != goal_checkers_.end(); it++) {
            it->second.reset();
        }
        path_trackers_.clear();
        goal_checkers_.clear();
        path_tracker_loader_.reset();
        goal_checker_loader_.reset();
        path_track_action_server_.reset();
        path_track_action_callback_group_.reset();
    }
    RCLCPP_INFO(get_logger(), "Shutdown done !");
    return CallbackReturn::SUCCESS;
}

PathTrackerNode::CallbackReturn PathTrackerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(get_logger(), "on_cleanup !");
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        flag_active_ = false;
        flag_stop_execute_ = true;
    }
    
    if (execute_future_.valid() && execute_future_.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Shutdown execute thread timeout !");
    };

    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        current_handle_.reset();
        pending_handle_.reset();
        cmd_vel_pub_.reset();
        tf_buffer_.reset();
        tf_listener_.reset();
        for (auto it = path_trackers_.begin(); it != path_trackers_.end(); it++) {
            it->second.reset();
        }
        for (auto it = goal_checkers_.begin(); it != goal_checkers_.end(); it++) {
            it->second.reset();
        }
        path_trackers_.clear();
        goal_checkers_.clear();
        path_tracker_loader_.reset();
        goal_checker_loader_.reset();
        path_track_action_server_.reset();
        path_track_action_callback_group_.reset();
    }
    RCLCPP_INFO(get_logger(), "Cleanup done !");
    return CallbackReturn::SUCCESS;
}

void PathTrackerNode::publish_zero_velocity() {
    geometry_msgs::msg::Twist zero_vel;
    zero_vel.linear.x = 0.0;
    zero_vel.linear.y = 0.0;
    zero_vel.linear.z = 0.0;
    zero_vel.angular.x = 0.0;
    zero_vel.angular.y = 0.0;
    zero_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(zero_vel);
    last_cmd_vel_ = zero_vel;
}

rclcpp_action::GoalResponse PathTrackerNode::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PathTrackAction::Goal> goal) {
    (void)uuid; (void)goal;
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    RCLCPP_INFO(this->get_logger(), "Recieve a request to track path !");

    if (!flag_active_) {
        RCLCPP_WARN(this->get_logger(), "Lifecycle node haven't activated ! Rejected track path request!");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accept track path request!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathTrackerNode::handle_cancel(const std::shared_ptr<PathTrackGoalHandle> goal_handle) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    RCLCPP_INFO(this->get_logger(), "Recieve a request to cancel path track!");

    if (!is_active_handle(goal_handle)) {
        RCLCPP_WARN(this->get_logger(), "Current path track handle is not active, do not need cancel.");
        return rclcpp_action::CancelResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accept request to cancel path track");
    return rclcpp_action::CancelResponse::ACCEPT;
    
}

void PathTrackerNode::handle_accept(const std::shared_ptr<PathTrackGoalHandle> goal_handle) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    RCLCPP_INFO(this->get_logger(), "Process the track path request !");

    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Accept goal_handle is empty !");
        flag_stop_execute_ = true; // 置该标志位为true来正确的关闭正在执行的future函数中的进程程序，防止直接关闭导致机器人计算速度异常
        return;
    } 

    terminate_handle(pending_handle_);
    pending_handle_ = goal_handle;

    if (!execute_future_.valid() || execute_future_.wait_for(std::chrono::microseconds(1)) != std::future_status::timeout) {
        execute_future_ = std::async(std::launch::async, [this](){work();});
    }
    RCLCPP_INFO(this->get_logger(), "Process track path request success!");
}

void PathTrackerNode::work() {
    while (rclcpp::ok())  {
        rclcpp::WallRate control_frequency(control_frequency_);
        while (rclcpp::ok()) {
            
            if (!update_handle()) {
                break;
            }

            auto current_pose_t = get_transform(global_frame_id_, robot_frame_id_, transform_tolerance_);

            if (!current_pose_t.has_value()) {
                publish_zero_velocity();
                terminate_handle(current_handle_);
                RCLCPP_ERROR(this->get_logger(), "Current pose get failed ! Stop execute !");
                break;
            };
            geometry_msgs::msg::Pose current_pose = pose_utils_.get_pose(current_pose_t.value());
            
            if (!progress_checker_->check(current_pose)) { // Check if is moving forward.
                RCLCPP_WARN(this->get_logger(), "Robot move failed !");
                publish_zero_velocity();
                terminate_handle(current_handle_);
                break;
            };

            if (goal_checkers_[current_goal_checker_]->is_goal_reached(current_pose)) { // Check if reached the destination.
                publish_zero_velocity();
                current_handle_->succeed(std::make_shared<PathTrackAction::Result>());
                RCLCPP_INFO(this->get_logger(), "Current goal has been successfully completed.");
                current_handle_.reset();
                break;
            }
            
            auto cmd_vel = path_trackers_[current_path_tracker_]->compute_cmd_vel(current_pose, last_cmd_vel_); // Compute the command velocity.
            if (!cmd_vel.has_value()) {
                RCLCPP_ERROR(this->get_logger(), "Command velocity compute failed !");
                publish_zero_velocity();
                terminate_handle(current_handle_);
                break;
            }

            cmd_vel_pub_->publish(cmd_vel.value());
            
            last_cmd_vel_ = cmd_vel.value();

            if (!control_frequency.sleep()) {
                RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz", control_frequency_);
            }
        }
        if (!is_active_handle(pending_handle_)) { // Make sure there is not new track path request come in when fianl cycle execute.
            break;
        }
    }
}

bool PathTrackerNode::update_handle() {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (flag_stop_execute_ || !flag_active_) {
        publish_zero_velocity();
        terminate_handle(current_handle_);
        terminate_handle(pending_handle_);
        RCLCPP_WARN(this->get_logger(), "Force quit execute !");
        return false;
    }
    if (is_active_handle(pending_handle_)) {
        auto goal = pending_handle_->get_goal();

        if (path_trackers_.find(goal->controller_id) == path_trackers_.end()) {
            if (!goal->controller_id.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received path_tracker specified doesn't exist ! stop execute !");
                publish_zero_velocity();
                terminate_handle(pending_handle_);
                terminate_handle(current_handle_);
                return false;
            } else {
                if (path_trackers_.find(default_path_tracker_) == path_trackers_.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Default path_tracker doesn't exist ! stop execute !");
                    publish_zero_velocity();
                    terminate_handle(pending_handle_);
                    terminate_handle(current_handle_);
                    return false;
                }
                current_path_tracker_ = default_path_tracker_;
                RCLCPP_INFO(this->get_logger(), "Select default path tracker: %s", current_path_tracker_.c_str());
            }
        } else {
            current_path_tracker_ = goal->controller_id;
            RCLCPP_INFO(this->get_logger(), "Select path tracker: %s", current_path_tracker_.c_str());
        }

        if (goal_checkers_.find(goal->goal_checker_id) == goal_checkers_.end()) {
            if (!goal->goal_checker_id.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received goal checker specified doesn't exist ! stop execute !");
                publish_zero_velocity();
                terminate_handle(pending_handle_);
                terminate_handle(current_handle_);
                return false;
            } else {
                if (goal_checkers_.find(default_goal_checker_) == goal_checkers_.end()) {
                    RCLCPP_ERROR(this->get_logger(), "Default goal checker doesn't exist ! stop execute !");
                    publish_zero_velocity();
                    terminate_handle(pending_handle_);
                    terminate_handle(current_handle_);
                    return false;
                }
                current_goal_checker_ = default_goal_checker_;
                RCLCPP_INFO(this->get_logger(), "Select default goal checker: %s", current_goal_checker_.c_str());
            }
        } else {
            current_goal_checker_ = goal->goal_checker_id;
            RCLCPP_INFO(this->get_logger(), "Select goal checker: %s", current_goal_checker_.c_str());
        }

        if (goal->path.poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received path is empty ! stop execute !");
            publish_zero_velocity();
            terminate_handle(pending_handle_);
            terminate_handle(current_handle_);
            return false;
        } 
        current_path_ = goal->path;
        path_trackers_[current_path_tracker_]->set_path(current_path_);
        goal_checkers_[current_goal_checker_]->set_goal(current_path_);
        current_handle_ = pending_handle_;
        progress_checker_->reset();
        pending_handle_.reset();
        RCLCPP_INFO(this->get_logger(), "Successfully accept new goal from pending handle");
    }
    if (!is_active_handle(current_handle_)) {
        publish_zero_velocity();
        current_handle_.reset();
        RCLCPP_WARN(this->get_logger(), "Current handle is not in active ! Stop execute !");
        return false;
    }
    if (current_handle_->is_canceling()) {
        publish_zero_velocity();
        current_handle_->canceled(std::make_shared<PathTrackAction::Result>());
        current_handle_.reset();
        RCLCPP_WARN(this->get_logger(), "The current handle is canceling ! Stop execute !");
        return false;
    }
    return true;
}

bool PathTrackerNode::is_active_handle(const std::shared_ptr<PathTrackGoalHandle> & handle) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return (handle != nullptr && handle->is_active());
}

void PathTrackerNode::terminate_handle(std::shared_ptr<PathTrackGoalHandle> & handle) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (handle == nullptr) {
        return;
    }
    if (handle->is_active()) {
        if (handle->is_canceling()) {
            handle->canceled(std::make_shared<PathTrackAction::Result>());
        } else {
            handle->abort(std::make_shared<PathTrackAction::Result>());
        }
    }
    handle.reset();
}

std::optional<geometry_msgs::msg::TransformStamped> PathTrackerNode::get_transform(const std::string & target_frame, const std::string & source_frame, const rclcpp::Duration & timeout_tolerance) {
    // note：这里的tolerance暂时没有用上.
    geometry_msgs::msg::TransformStamped transform;
    try {
        // transform = tf_buffer_->lookupTransform(target_frame, source_frame, this->get_clock()->now(), timeout_tolerance);
        transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    } catch (tf2::LookupException & ex) {
        RCLCPP_ERROR(
        this->get_logger(),
        "No Transform available Error looking up target frame: %s\n", ex.what());
        return std::nullopt;
    } catch (tf2::ConnectivityException & ex) {
        RCLCPP_ERROR(
        this->get_logger(),
        "Connectivity Error looking up target frame: %s\n", ex.what());
        return std::nullopt;
    } catch (tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(
        this->get_logger(),
        "Extrapolation Error looking up target frame: %s\n", ex.what());
        return std::nullopt;
    } catch (tf2::TimeoutException & ex) {
        RCLCPP_ERROR(
        this->get_logger(),
        "Transform timeout with tolerance: %.4f", timeout_tolerance.seconds());
        return std::nullopt;
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
        this->get_logger(), "Failed to transform from %s to %s",
        source_frame.c_str(), target_frame.c_str());
        return std::nullopt;
    }
    return transform;
}


PathTrackerNode::~PathTrackerNode() {
    RCLCPP_WARN(this->get_logger(), "PathTrackerNode deconstruction... ");

    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        flag_active_ = false;
        flag_stop_execute_ = true;
    }

    RCLCPP_INFO(this->get_logger(), "Wait for the execute function to complete !");
    if (execute_future_.valid() && execute_future_.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
        RCLCPP_ERROR(this->get_logger(), "Shutdown Execute function timeout !");
    };
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        cmd_vel_pub_.reset();
        tf_buffer_.reset();
        tf_listener_.reset();
        for (auto it = path_trackers_.begin(); it != path_trackers_.end(); it++) {
            it->second.reset();
        }
        for (auto it = goal_checkers_.begin(); it != goal_checkers_.end(); it++) {
            it->second.reset();
        }
        path_trackers_.clear();
        goal_checkers_.clear();
        current_handle_.reset();
        pending_handle_.reset();
        path_tracker_loader_.reset();
        goal_checker_loader_.reset();
        path_track_action_server_.reset();
        path_track_action_callback_group_.reset();
    }
    RCLCPP_WARN(this->get_logger(), "PathTrackerNode deconstruction done ! ");
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_tracker::PathTrackerNode)