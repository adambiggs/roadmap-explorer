/**
    Copyright 2025 Suchetan Saravanan.

    Licensed to the Apache Software Foundation (ASF) under one
    or more contributor license agreements.  See the NOTICE file
    distributed with this work for additional information
    regarding copyright ownership.  The ASF licenses this file
    to you under the Apache License, Version 2.0 (the
    "License"); you may not use this file except in compliance
    with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing,
    software distributed under the License is distributed on an
    "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
    KIND, either express or implied.  See the License for the
    specific language governing permissions and limitations
    under the License.
*/

#include "roadmap_explorer/bt_plugins/BootstrapRandomWalkPlugin.hpp"
#include "roadmap_explorer/Parameters.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <random>

namespace roadmap_explorer
{

/**
 * Bootstrap Random Walk BT Node
 * 
 * SAFETY-FIRST implementation:
 * - Checks map for sufficient known cells
 * - If map too empty, performs safe random walk
 * - Uses LiDAR scan to detect obstacles
 * - Publishes to /cmd_vel_explore (goes through twist_mux with e-stop)
 * - Conservative velocities
 * - Stops and turns when obstacles detected
 */
class BootstrapRandomWalkBT : public BT::StatefulActionNode
{
public:
    BootstrapRandomWalkBT(
        const std::string & name,
        const BT::NodeConfiguration & config,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros,
        std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr)
    : BT::StatefulActionNode(name, config),
      explore_costmap_ros_(explore_costmap_ros),
      ros_node_ptr_(ros_node_ptr)
    {
        // Create velocity publisher (goes through twist_mux)
        cmd_vel_pub_ = ros_node_ptr_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_explore", 10);
        cmd_vel_pub_->on_activate();
        
        // Subscribe to laser scan for obstacle detection
        scan_sub_ = ros_node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&BootstrapRandomWalkBT::laserScanCallback, this, std::placeholders::_1));
        
        // Get parameters with defaults
        ros_node_ptr_->declare_parameter("bootstrap.enabled", true);
        ros_node_ptr_->declare_parameter("bootstrap.min_known_cells", 100);
        ros_node_ptr_->declare_parameter("bootstrap.unknown_map_threshold", 0.95);
        ros_node_ptr_->declare_parameter("bootstrap.max_duration", 60.0);
        ros_node_ptr_->declare_parameter("bootstrap.linear_velocity", 0.2);
        ros_node_ptr_->declare_parameter("bootstrap.angular_velocity", 0.5);
        ros_node_ptr_->declare_parameter("bootstrap.min_obstacle_distance", 0.5);
        ros_node_ptr_->declare_parameter("bootstrap.scan_front_arc", 45.0);
        ros_node_ptr_->declare_parameter("bootstrap.turn_duration", 2.0);
        ros_node_ptr_->declare_parameter("bootstrap.forward_duration", 3.0);
        
        enabled_ = ros_node_ptr_->get_parameter("bootstrap.enabled").as_bool();
        min_known_cells_ = ros_node_ptr_->get_parameter("bootstrap.min_known_cells").as_int();
        unknown_threshold_ = ros_node_ptr_->get_parameter("bootstrap.unknown_map_threshold").as_double();
        max_duration_ = ros_node_ptr_->get_parameter("bootstrap.max_duration").as_double();
        linear_vel_ = ros_node_ptr_->get_parameter("bootstrap.linear_velocity").as_double();
        angular_vel_ = ros_node_ptr_->get_parameter("bootstrap.angular_velocity").as_double();
        min_obstacle_dist_ = ros_node_ptr_->get_parameter("bootstrap.min_obstacle_distance").as_double();
        scan_front_arc_ = ros_node_ptr_->get_parameter("bootstrap.scan_front_arc").as_double();
        turn_duration_ = ros_node_ptr_->get_parameter("bootstrap.turn_duration").as_double();
        forward_duration_ = ros_node_ptr_->get_parameter("bootstrap.forward_duration").as_double();
        
        // Initialize state
        last_scan_ = nullptr;
        bootstrap_start_time_ = rclcpp::Time(0);
        state_ = State::IDLE;
        state_start_time_ = rclcpp::Time(0);
        
        // Random number generator
        std::random_device rd;
        rng_ = std::mt19937(rd());
        
        LOG_INFO("BootstrapRandomWalkBT created - enabled: " << enabled_);
    }
    
    ~BootstrapRandomWalkBT()
    {
        stopRobot();
    }
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("bootstrap_completed")
        };
    }
    
    BT::NodeStatus onStart() override
    {
        if (!enabled_) {
            LOG_DEBUG("Bootstrap mode disabled, skipping");
            return BT::NodeStatus::SUCCESS;
        }
        
        // Check if map is too empty
        auto map_status = checkMapStatus();
        
        if (!map_status.needs_bootstrap) {
            LOG_DEBUG("Map has sufficient data (" << map_status.known_cells 
                     << " known cells, " << map_status.unknown_pct << "% unknown), skipping bootstrap");
            setOutput("bootstrap_completed", true);
            return BT::NodeStatus::SUCCESS;
        }
        
        // Enter bootstrap mode
        LOG_INFO("Bootstrap mode: Map too empty (" << map_status.known_cells 
                << " known cells, " << map_status.unknown_pct << "% unknown), starting safe random walk");
        
        bootstrap_start_time_ = ros_node_ptr_->now();
        state_ = State::CHECKING;
        state_start_time_ = bootstrap_start_time_;
        
        return BT::NodeStatus::RUNNING;
    }
    
    BT::NodeStatus onRunning() override
    {
        auto now = ros_node_ptr_->now();
        
        // Check timeout
        if ((now - bootstrap_start_time_).seconds() > max_duration_) {
            LOG_INFO("Bootstrap mode: Timeout reached, checking map status");
            auto map_status = checkMapStatus();
            LOG_INFO("Bootstrap mode: Exit after timeout - accumulated " 
                    << map_status.known_cells << " known cells (" 
                    << map_status.unknown_pct << "% unknown)");
            stopRobot();
            setOutput("bootstrap_completed", true);
            return BT::NodeStatus::SUCCESS;
        }
        
        // Check if we've accumulated enough map data
        auto map_status = checkMapStatus();
        if (!map_status.needs_bootstrap) {
            LOG_INFO("Bootstrap mode: Exit - sufficient map data accumulated (" 
                    << map_status.known_cells << " known cells, " 
                    << map_status.unknown_pct << "% unknown)");
            stopRobot();
            setOutput("bootstrap_completed", true);
            return BT::NodeStatus::SUCCESS;
        }
        
        // Execute state machine
        executeStateMachine(now);
        
        return BT::NodeStatus::RUNNING;
    }
    
    void onHalted() override
    {
        LOG_INFO("Bootstrap mode: Halted");
        stopRobot();
        state_ = State::IDLE;
    }

private:
    enum class State {
        IDLE,
        CHECKING,
        MOVING_FORWARD,
        TURNING,
        STOPPED
    };
    
    struct MapStatus {
        int total_cells;
        int known_cells;
        int unknown_cells;
        double unknown_pct;
        bool needs_bootstrap;
    };
    
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
    }
    
    MapStatus checkMapStatus()
    {
        MapStatus status;
        
        auto costmap = explore_costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        
        status.total_cells = costmap->getSizeInCellsX() * costmap->getSizeInCellsY();
        status.known_cells = 0;
        status.unknown_cells = 0;
        
        // Count known vs unknown cells
        for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x) {
            for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y) {
                unsigned char cost = costmap->getCost(x, y);
                if (cost == nav2_costmap_2d::NO_INFORMATION) {
                    status.unknown_cells++;
                } else {
                    status.known_cells++;
                }
            }
        }
        
        status.unknown_pct = (status.unknown_cells * 100.0) / status.total_cells;
        
        // Needs bootstrap if map is mostly unknown AND we don't have minimum known cells
        status.needs_bootstrap = (status.unknown_pct > (unknown_threshold_ * 100.0)) && 
                                  (status.known_cells < min_known_cells_);
        
        return status;
    }
    
    bool isSafeToMoveForward()
    {
        if (!last_scan_) {
            LOG_WARN("Bootstrap mode: No laser scan data available, stopping");
            return false;
        }
        
        // Check front arc for obstacles
        double front_arc_rad = scan_front_arc_ * M_PI / 180.0;
        float min_distance = std::numeric_limits<float>::max();
        int obstacles_detected = 0;
        
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            float angle = last_scan_->angle_min + i * last_scan_->angle_increment;
            
            // Check if within front arc
            if (std::abs(angle) <= front_arc_rad) {
                float range = last_scan_->ranges[i];
                
                // Skip invalid readings
                if (std::isnan(range) || std::isinf(range)) {
                    continue;
                }
                
                // Check if within min/max range
                if (range < last_scan_->range_min || range > last_scan_->range_max) {
                    continue;
                }
                
                // Track minimum distance in front arc
                if (range < min_distance) {
                    min_distance = range;
                }
                
                // Count obstacles within danger zone
                if (range < min_obstacle_dist_) {
                    obstacles_detected++;
                }
            }
        }
        
        bool safe = obstacles_detected == 0;
        
        if (!safe) {
            LOG_INFO("Bootstrap mode: Obstacle detected at " 
                    << std::fixed << std::setprecision(2) << min_distance 
                    << "m (threshold: " << min_obstacle_dist_ << "m), stopping");
        }
        
        return safe;
    }
    
    void executeStateMachine(const rclcpp::Time& now)
    {
        double state_elapsed = (now - state_start_time_).seconds();
        
        switch (state_) {
            case State::IDLE:
            case State::CHECKING:
                // Start moving forward
                changeState(State::MOVING_FORWARD, now);
                break;
                
            case State::MOVING_FORWARD:
                if (!isSafeToMoveForward()) {
                    // Obstacle detected, stop and turn
                    stopRobot();
                    changeState(State::TURNING, now);
                } else if (state_elapsed > forward_duration_) {
                    // Forward duration complete, turn to new direction
                    stopRobot();
                    changeState(State::TURNING, now);
                } else {
                    // Continue moving forward
                    publishVelocity(linear_vel_, 0.0);
                }
                break;
                
            case State::TURNING:
                if (state_elapsed > turn_duration_) {
                    // Turn complete, move forward again
                    stopRobot();
                    changeState(State::MOVING_FORWARD, now);
                } else {
                    // Continue turning (random direction)
                    double angular = angular_vel_ * (turn_direction_ > 0 ? 1.0 : -1.0);
                    publishVelocity(0.0, angular);
                }
                break;
                
            case State::STOPPED:
                stopRobot();
                break;
        }
    }
    
    void changeState(State new_state, const rclcpp::Time& now)
    {
        if (new_state == State::TURNING) {
            // Randomize turn direction
            std::uniform_int_distribution<> dir_dist(0, 1);
            turn_direction_ = dir_dist(rng_) == 0 ? -1 : 1;
            LOG_DEBUG("Bootstrap mode: Turning " << (turn_direction_ > 0 ? "left" : "right"));
        } else if (new_state == State::MOVING_FORWARD) {
            LOG_DEBUG("Bootstrap mode: Moving forward");
        }
        
        state_ = new_state;
        state_start_time_ = now;
    }
    
    void publishVelocity(double linear, double angular)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;
        
        if (cmd_vel_pub_->is_activated()) {
            cmd_vel_pub_->publish(cmd);
        }
    }
    
    void stopRobot()
    {
        publishVelocity(0.0, 0.0);
    }
    
    // Members
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> explore_costmap_ros_;
    std::shared_ptr<nav2_util::LifecycleNode> ros_node_ptr_;
    
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
    // Parameters
    bool enabled_;
    int min_known_cells_;
    double unknown_threshold_;
    double max_duration_;
    double linear_vel_;
    double angular_vel_;
    double min_obstacle_dist_;
    double scan_front_arc_;
    double turn_duration_;
    double forward_duration_;
    
    // State
    State state_;
    rclcpp::Time bootstrap_start_time_;
    rclcpp::Time state_start_time_;
    int turn_direction_;
    std::mt19937 rng_;
};

BootstrapRandomWalkPlugin::BootstrapRandomWalkPlugin()
{
}

BootstrapRandomWalkPlugin::~BootstrapRandomWalkPlugin()
{
}

void BootstrapRandomWalkPlugin::registerNodes(BT::BehaviorTreeFactory &factory, std::shared_ptr<BTContext> context)
{
    BT::NodeBuilder builder =
        [context](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<BootstrapRandomWalkBT>(
            name,
            config,
            context->explore_costmap_ros,
            context->node);
    };
    factory.registerBuilder<BootstrapRandomWalkBT>("BootstrapRandomWalk", builder);
}

} // namespace roadmap_explorer

PLUGINLIB_EXPORT_CLASS(
    roadmap_explorer::BootstrapRandomWalkPlugin,
    roadmap_explorer::BTPlugin)
