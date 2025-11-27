#include "path_tracker/plugins/simple_path_tracker.hpp"


namespace path_tracker {

void SimplePathTracker::configure(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent) {
    node_ = parent;
    flag_collision_occur_ = false;

    RCLCPP_INFO(node_->get_logger(), "SimplePathTracker configuration...");

    node_->declare_parameter<double>("path_tracker/SimplePathTracker.track_forward_distance", 0.5);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.fixed_velocity", 0.7);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.fixed_max_velocity", 1.0);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.yaw_vel_fixed", 1.57);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.tolerance_distance", 0.6);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.tolerance_back_distance", 0.4);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.weight_position", 0.5);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.weight_theta", 0.5);
    node_->declare_parameter<int>("path_tracker/SimplePathTracker.time_init_ms", 400);
    node_->declare_parameter<int>("path_tracker/SimplePathTracker.time_brake_ms", 200);
    node_->declare_parameter<double>("path_tracker/SimplePathTracker.brake_vel", -1.5);

    track_forward_distance_ = node_->get_parameter("path_tracker/SimplePathTracker.track_forward_distance").as_double();
    fixed_velocity_ = node_->get_parameter("path_tracker/SimplePathTracker.fixed_velocity").as_double();
    fixed_max_velocity_ = node_->get_parameter("path_tracker/SimplePathTracker.fixed_max_velocity").as_double();
    yaw_vel_fixed_ = node_->get_parameter("path_tracker/SimplePathTracker.yaw_vel_fixed").as_double();
    tolerance_distance_ = node_->get_parameter("path_tracker/SimplePathTracker.tolerance_distance").as_double();
    tolerance_back_distance_ = node_->get_parameter("path_tracker/SimplePathTracker.tolerance_back_distance").as_double();
    weight_position_ = node_->get_parameter("path_tracker/SimplePathTracker.weight_position").as_double();
    weight_theta_ = node_->get_parameter("path_tracker/SimplePathTracker.weight_theta").as_double();
    time_init_ms_ = node_->get_parameter("path_tracker/SimplePathTracker.time_init_ms").as_int();
    time_brake_ms_ = node_->get_parameter("path_tracker/SimplePathTracker.time_brake_ms").as_int();
    brake_vel_ = node_->get_parameter("path_tracker/SimplePathTracker.brake_vel").as_double();

    collision_detect_sub_ = node_->create_subscription<std_msgs::msg::Bool>("collision_occur", 10,
        std::bind(&SimplePathTracker::recv_closest_point, this, std::placeholders::_1)
    );
}

void SimplePathTracker::recv_closest_point(const std::shared_ptr<std_msgs::msg::Bool> collision_occur_msg) {
    std::lock_guard<std::mutex> lock(collision_detect_mutex_);
    last_collision_occur_ = flag_collision_occur_;
    flag_collision_occur_ = collision_occur_msg->data;
    if (!last_collision_occur_ && flag_collision_occur_) {
            brake_clock_ = std::chrono::steady_clock::now();
    }
}

std::optional<geometry_msgs::msg::Twist> SimplePathTracker::compute_cmd_vel(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Twist & current_vel) {

    geometry_msgs::msg::Twist cmd_vel_result;
    cmd_vel_result.linear.x = 0;
    cmd_vel_result.linear.y = 0;
    cmd_vel_result.linear.z = 0;
    cmd_vel_result.angular.x = 0;
    cmd_vel_result.angular.y = 0;
    cmd_vel_result.angular.z = 0;

    {
        // For emergency stop
        std::lock_guard<std::mutex> lock(collision_detect_mutex_);
        if (flag_collision_occur_) {
            auto t_now_brake = std::chrono::steady_clock::now(); 
            int time_brake = std::chrono::duration_cast<std::chrono::milliseconds>(t_now_brake-brake_clock_).count();
            if (time_brake <= time_brake_ms_) {
                cmd_vel_result.linear.x = brake_vel_;
                return cmd_vel_result;
            }
            return cmd_vel_result;
        }
    }
    
    if (path_.cols() == 0) {
        RCLCPP_ERROR(node_->get_logger(), "Path is empty ! Stop compute cmd_vel !");
        return std::nullopt;
    }
    Eigen::Vector3d current_position(
        current_pose.position.x, 
        current_pose.position.y, 
        current_pose.position.z);
    Eigen::Quaterniond current_orientation(
        current_pose.orientation.w, 
        current_pose.orientation.x, 
        current_pose.orientation.y, 
        current_pose.orientation.z);
    std::optional<std::pair<int, int>> seg_idx_result = get_segment_idx(current_position);
    if (!seg_idx_result.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "Track error! Can't get the segment idx in the specified range, robot is not near the path!");
        return std::nullopt;
    };
    std::pair<int, int> seg_idx = seg_idx_result.value();
    int closest_idx = get_closest_idx(current_position, seg_idx);
    
    last_segment_idx_ = seg_idx;
    
    int target_idx = get_target_idx(closest_idx, track_forward_distance_);

    // Transform the pose to matrix, then transform the the target path point to current pose to track
    Eigen::MatrixXd path_segment = path_.middleCols(closest_idx, target_idx - closest_idx + 1);
    Eigen::MatrixXd path_segment_4d(4, path_segment.cols());
    path_segment_4d.fill(1.0);
    path_segment_4d.topRows(3) = path_segment;
    Eigen::Matrix4d world_to_current = pose_util_.pose_to_matrix(current_pose).inverse();
    
    Eigen::MatrixXd path_segment_current = (world_to_current * path_segment_4d).topRows(3);
    Eigen::Vector3d target_point_current = path_segment_current.rightCols(1);

    if (target_idx == closest_idx) {
        RCLCPP_WARN(node_->get_logger(), "Robot is close to the goal or track forward distance may too short !");
        Eigen::Vector2d direction_2d = target_point_current.segment(0, 2).normalized();
        cmd_vel_result.linear.x = direction_2d.x() * fixed_velocity_;
        cmd_vel_result.linear.y = direction_2d.y() * fixed_velocity_;
        return cmd_vel_result;
    }

    if (target_point_current(0) <= 0 && target_point_current(1) > 0) {  // If the target point 
        cmd_vel_result.angular.z = yaw_vel_fixed_;
        return cmd_vel_result;
    } else if (target_point_current(0) <= 0 && target_point_current(1) <= 0) {
        cmd_vel_result.angular.z = -yaw_vel_fixed_;
        return cmd_vel_result;
    }
    
#ifdef USE_CERES_OPTIMIZATION
    // Use Ceres optimization for angular velocity calculation (consistent with original wl_ros)
    double angular_velocity = 0.0;
    
    // Create Ceres problem
    ceres::Problem problem;
    
    // Add residual blocks for path fitting
    for (int i = 0; i < path_segment_current.cols(); ++i) {
        double x = path_segment_current(0, i);
        double y = path_segment_current(1, i);
        double theta = std::atan2(y, x);
        
        ceres::CostFunction* cost_function = PathFitResidual::Create(x, y, theta, weight_position_, weight_theta_);
        problem.AddResidualBlock(cost_function, nullptr, &angular_velocity);
    }
    
    // Set solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 50;
    
    // Solve the problem
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // Limit angular velocity
    double max_angular_vel = M_PI / 2.0;
    if (angular_velocity > max_angular_vel) {
        angular_velocity = max_angular_vel;
    } else if (angular_velocity < -max_angular_vel) {
        angular_velocity = -max_angular_vel;
    }
#else
    // Simple geometric calculation without Ceres optimization
    double angular_velocity = std::atan2(target_point_current(1), target_point_current(0));
    
    // Limit angular velocity
    double max_angular_vel = M_PI / 2.0;
    if (angular_velocity > max_angular_vel) {
        angular_velocity = max_angular_vel;
    } else if (angular_velocity < -max_angular_vel) {
        angular_velocity = -max_angular_vel;
    }
#endif

    if (current_vel.linear.x <= 1e-4) {
        init_clock_ = std::chrono::steady_clock::now();
    }
    double fit_vel = fixed_velocity_;
    std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
    int time_to_init = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - init_clock_).count();
    if (time_to_init < time_init_ms_) {
        fit_vel = fixed_velocity_ * time_to_init / time_init_ms_ + fixed_max_velocity_ * (1 - time_to_init / time_init_ms_);
    } else {
        fit_vel = fixed_velocity_;
    }
    
    cmd_vel_result.angular.z = angular_velocity;
    cmd_vel_result.linear.x = fit_vel;
    
    return cmd_vel_result;
}

double SimplePathTracker::get_closest_distance(const int & closest_idx, const geometry_msgs::msg::Pose & current_pose) {
    Eigen::Vector3d current_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
    return (current_position - path_.col(closest_idx)).norm();
}

double SimplePathTracker::get_target_theta(const int & target_idx, const Eigen::Matrix4d & current_frame) {
    if (target_idx == path_.cols() - 1) {
        Eigen::Vector3d dir_vec = path_.col(target_idx) - path_.col(target_idx - 1);
        dir_vec = current_frame.block(0, 0, 3, 3) * dir_vec;
        return std::atan2(dir_vec.y(), dir_vec.x());
    } else {
        Eigen::Vector3d dir_vec = path_.col(target_idx + 1) - path_.col(target_idx - 1);
        dir_vec = current_frame.block(0, 0, 3, 3) * dir_vec;
        return std::atan2(dir_vec.y(), dir_vec.x());
    }
}

void SimplePathTracker::set_path(const nav_msgs::msg::Path & path) {
    path_.resize(3, path.poses.size());
    for (size_t i = 0; i < path.poses.size(); i++) {
        path_.col(i) << path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z;
    };
    last_segment_idx_ = std::make_pair(0, 0);
}

int SimplePathTracker::get_target_idx(const int & closest_idx, const double & track_forward_distance) {
    Eigen::Vector3d closest_point = path_.col(closest_idx);
    for (int i = closest_idx; i < path_.cols(); i++) {
        Eigen::Vector3d point = path_.col(i);
        if ((closest_point - point).norm() > track_forward_distance) 
        {
            return i;
        }
    }
    return path_.cols() - 1;
}

double SimplePathTracker::get_fit_length(const Eigen::MatrixXd & path_segment) {
    double fit_length = 0;
    for (int i = 0; i < path_segment.cols()-1; i++) {
        double interval = (path_segment.col(i).segment(0, 2) - path_segment.col(i+1).segment(0, 2)).norm();
        fit_length += interval;
    }
    return fit_length;
}

std::optional<std::pair<int, int>> SimplePathTracker::get_segment_idx(const Eigen::Vector3d & current_position) {
    double distance = (current_position - path_.col(last_segment_idx_.first)).norm();
    std::pair<int, int> segment_idx(0, 0);

    if (distance < tolerance_distance_) {
        int idx = last_segment_idx_.first - 1;
        for (; idx >= 0; idx--) {
            distance = (current_position - path_.col(idx)).norm();
            if (distance > tolerance_distance_) {
                segment_idx.first = idx;
                break;
            };
        }
        if (idx == -1) {
            segment_idx.first = 0;
        }
        idx = last_segment_idx_.first + 1;
        for (; idx < path_.cols(); idx++) {
            distance = (path_.col(idx) - current_position).norm();
            if (distance > tolerance_distance_) {
                segment_idx.second = idx;
                break;
            }
        }
        if (idx == path_.cols()) {
            segment_idx.second = path_.cols() - 1;
        }
        return segment_idx;
    } else {
        int idx = last_segment_idx_.first + 1;
        for (; idx < path_.cols(); idx++) {
            distance = (path_.col(idx) - current_position).norm();
            if (distance < tolerance_distance_) {
                segment_idx.first = idx - 1; 
                idx = idx + 1;
                for (; idx < path_.cols(); idx++) {
                    distance = (path_.col(idx) - current_position).norm();
                    if (distance > tolerance_distance_) {
                        segment_idx.second = idx;
                        break;
                    }
                }
                if (idx == path_.cols()) {
                    segment_idx.second = path_.cols() - 1;
                }
                return segment_idx;
            };
        }
        if (idx == path_.cols()) {
            idx = last_segment_idx_.first-1;
            for (; idx >=0 && (path_.col(last_segment_idx_.first) - path_.col(idx)).norm() < tolerance_back_distance_; idx--) {
                distance = (path_.col(idx) - current_position).norm();
                if (distance < tolerance_distance_) {
                    segment_idx.second = idx + 1;
                    idx = idx - 1;
                    for (; idx >= 0; idx--) {
                        distance = (path_.col(idx) - current_position).norm();
                        if (distance > tolerance_distance_) {
                            segment_idx.first = idx;
                            break;
                        }
                    }
                    if (idx == -1) {
                        segment_idx.first = 0;
                    }
                    return segment_idx;
                }
            }
        }
    }
    return std::nullopt;
}

int SimplePathTracker::get_closest_idx(const Eigen::Vector3d & current_position, std::pair<int, int> & segment_idx) {
    int closest_idx = segment_idx.first;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = segment_idx.first; i <= segment_idx.second; i++) {
        Eigen::Vector3d point = path_.col(i);
        double dist = (point - current_position).norm();
        if (dist < min_dist) {
            closest_idx = i;
            min_dist = dist;
        }
    }
    return closest_idx;
}

SimplePathTracker::~SimplePathTracker() {
    RCLCPP_WARN(node_->get_logger(), "SimplePathTracker deconstructing...");
    node_.reset();
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_tracker::SimplePathTracker, navigation_core::BasePathTracker)