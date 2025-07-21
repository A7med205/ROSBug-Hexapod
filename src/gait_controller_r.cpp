#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <map>

// Constants
const double DISCRETIZATION_TIME_STEP = 0.01;       // 10ms update rate
const double DEG_RAD = M_PI / 180.0;
const double MIN_ANGLE_CHANGE = 1.0 * M_PI / 180.0; // 1 degree in radians

struct LegConfig
{
    int leg_id;
    std::vector<std::string> joint_names;
    double direction;     // Direction of coordinate rotation
    bool is_tripod_a; // True for tripod A (1,3,5), false for tripod B (2,4,6)
    double H_X;       // Leg position in base X
    double W_Y;       // Leg position in base Y
};

class GaitController : public rclcpp::Node
{
public:
    GaitController() : Node("gait_controller_node")
    {
        // 1. Initialize leg configurations
        init_leg_configurations();

        // 2. Declare and get parameters
        this->declare_parameter<double>("step_angle", 15.0);
        this->declare_parameter<double>("step_height", 0.03);
        this->declare_parameter<double>("step_duration", 1.5);
        this->declare_parameter<int>("total_full_steps", 5);

        step_angle_ = this->get_parameter("step_angle").as_double();
        step_angle_ = step_angle_ * DEG_RAD;
        step_height_ = this->get_parameter("step_height").as_double();
        step_duration_ = this->get_parameter("step_duration").as_double();
        total_full_steps_ = this->get_parameter("total_full_steps").as_int();

        RCLCPP_INFO(this->get_logger(), "Gait Parameters:");
        RCLCPP_INFO(this->get_logger(), " - Step Angle: %.3f rad", step_angle_);
        RCLCPP_INFO(this->get_logger(), " - Step Height: %.3f m", step_height_);
        RCLCPP_INFO(this->get_logger(), " - Step Duration: %.3f s", step_duration_);
        RCLCPP_INFO(this->get_logger(), " - Total Full Steps: %d", total_full_steps_);

        // 3. Create Action Client
        action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // 4. Run the main gait sequence
        run_hexapod_gait_sequence();

        RCLCPP_INFO(this->get_logger(), "Hexapod gait sequence completed. Shutting down.");
        rclcpp::shutdown();
    }

private:
    // ROS Parameters
    double step_angle_;
    double step_height_;
    double step_duration_;
    int total_full_steps_;

    // Robot Constants
    const double LEG_L1 = 0.0385; // Coxa link
    const double LEG_L2 = 0.0700; // Femur link
    const double LEG_L3 = 0.1020; // Tibia link
    const double Z_HOME = -0.050; // Home Z
    const double Y_HOME = 0.100;  // Home Y 

    // Leg configurations
    std::vector<LegConfig> leg_configs_;
    std::vector<std::string> all_joint_names_;

    // ROS Action Client
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;

    // --- Initialization ---
    void init_leg_configurations()
    {
        leg_configs_ = {
            {1, {"jl11", "jl12", "jl13"}, 1.0, true, 0.090, 0.0535}, // Leg 1: 135° CCW, Tripod A
            {2, {"jl21", "jl22", "jl23"}, 0.0, false, 0.0, 0.070}, // Leg 2: 180°, Tripod B
            {3, {"jl31", "jl32", "jl33"}, -1.0, true, 0.090, 0.0535},  // Leg 3: 135° CW, Tripod A
            {4, {"jl41", "jl42", "jl43"}, -1.0, false, 0.090, 0.0535}, // Leg 4: 45° CCW, Tripod B
            {5, {"jl51", "jl52", "jl53"}, 0.0, true, 0.0, 0.070},  // Leg 5: 0°, Tripod A
            {6, {"jl61", "jl62", "jl63"}, 1.0, false, 0.090, 0.0535} // Leg 6: 45° CW, Tripod B
        };

        // Build complete joint names vector
        for (const auto &leg : leg_configs_)
        {
            for (const auto &joint : leg.joint_names)
            {
                all_joint_names_.push_back(joint);
            }
        }
    }

    // --- Core Hexapod Gait Logic ---
    void run_hexapod_gait_sequence()
    {
        const double x_home = 0.0;
        const double x_forward = x_home + step_angle_ / 2.0;
        const double x_backward = x_home - step_angle_ / 2.0;

        int step_counter = 0;

        // 1. Initial half-step: Tripod A swings forward, Tripod B pulls backward
        RCLCPP_INFO(this->get_logger(), "Executing initial half-step...");
        auto initial_points = generate_coordinated_trajectory(
            x_home, x_forward,  // Tripod A: swing
            x_home, x_backward, // Tripod B: pull
            0.75 * step_duration_, true, false);
        send_and_wait(initial_points);

        // 2. Loop for full coordinated steps
        for (int i = 0; i < total_full_steps_; ++i)
        {
            step_counter = i + 1;
            RCLCPP_INFO(this->get_logger(), "Executing coordinated step #%d / %d", step_counter, total_full_steps_);

            // Phase 1: Tripod A pulls backward, Tripod B swings forward
            auto phase1_points = generate_coordinated_trajectory(
                x_forward, x_backward, // Tripod A: pull
                x_backward, x_forward, // Tripod B: swing
                step_duration_, false, true);
            send_and_wait(phase1_points);

            // Phase 2: Tripod A swings forward, Tripod B pulls backward
            auto phase2_points = generate_coordinated_trajectory(
                x_backward, x_forward, // Tripod A: swing
                x_forward, x_backward, // Tripod B: pull
                step_duration_, true, false);
            send_and_wait(phase2_points);
        }
        RCLCPP_INFO(this->get_logger(), "Completed %d full coordinated cycles.", step_counter);

        // 3. Final half-step: Tripod A pulls to home, Tripod B swings to home
        RCLCPP_INFO(this->get_logger(), "Executing final half-step to home...");
        auto final_points = generate_coordinated_trajectory(
            x_forward, x_home,  // Tripod A: pull
            x_backward, x_home, // Tripod B: swing
            0.75 * step_duration_, false, true);
        send_and_wait(final_points);
    }

    // --- Coordinated Trajectory Generation ---
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> generate_coordinated_trajectory(
        double tripod_a_x_start, double tripod_a_x_end,
        double tripod_b_x_start, double tripod_b_x_end,
        double duration, bool tripod_a_swing, bool tripod_b_swing)
    {
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
        std::map<int, std::vector<double>> last_angles; // leg_id -> last angles for j1, j2 and j3

        // Initialize last angles
        for (const auto &leg : leg_configs_)
        {
            last_angles[leg.leg_id] = {INFINITY, INFINITY, INFINITY};
        }

        for (double t = 0; t <= duration; t += DISCRETIZATION_TIME_STEP)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(t);
            point.positions.resize(all_joint_names_.size());

            bool add_points = false;

            for (const auto &leg : leg_configs_)
            {
                double x_start, x_end;
                bool is_swing;

                if (leg.is_tripod_a)
                {
                    x_start = tripod_a_x_start;
                    x_end = tripod_a_x_end;
                    is_swing = tripod_a_swing;
                }
                else
                {
                    x_start = tripod_b_x_start;
                    x_end = tripod_b_x_end;
                    is_swing = tripod_b_swing;
                }

                double x_global, y_global, z_global;
                xyz_t(x_start, x_end, t, duration, is_swing, leg.H_X, leg.W_Y, x_global, y_global, z_global);

                // Rotate coordinates for this leg
                transform_coordinates(0.0, Y_HOME, leg.H_X, leg.W_Y, leg.direction, x_global, y_global);

                // Calculate inverse kinematics
                double j1, j2, j3;
                IK(LEG_L1, LEG_L2, LEG_L3, x_global, y_global, z_global, j1, j2, j3);

                // Check if significant change occurred for this leg
                if (points.empty()) {
                    add_points = true;
                    last_angles[leg.leg_id] = {j1, j2, j3};

                } else {
                    std::array<double, 3> new_angles = {j1, j2, j3};
                
                    for (int i = 0; i < 3; ++i) {
                        if (std::abs(new_angles[i] - last_angles[leg.leg_id][i]) > MIN_ANGLE_CHANGE) {
                            last_angles[leg.leg_id][i] = new_angles[i];
                            add_points = true;
                        }
                    }
                }

                // Set joint positions in the correct order
                int base_idx = (leg.leg_id - 1) * 3;
                point.positions[base_idx] = j1;
                point.positions[base_idx + 1] = j2;
                point.positions[base_idx + 2] = j3;
            }

            if (add_points)
            {
                points.push_back(point);
            }
        }
        return points;
    }

    void xyz_t(double x_start, double x_end, double t, double duration,
                              bool is_swing, double H_X, double W_Y, double &x_global, double &y_global, double &z_global)
    {
        if (is_swing)
        {
            // Rotation radius at tip
            double H = (H_X != 0.0) ? Y_HOME * std::sin(M_PI / 4) + H_X : 0;
            double W = (H_X != 0.0) ? Y_HOME * std::cos(M_PI / 4) + W_Y : Y_HOME + W_Y;
            double R_C = std::sqrt(H * H + W * W);
            
            // Angle -> distance
            x_start = x_start * R_C;
            x_end = x_end * R_C;
            
            // Current distance along swing arc
            double C = std::abs(x_end - x_start);
            double h = step_height_;
            double R = (h * h + (C / 2.0) * (C / 2.0)) / (2.0 * h);
            double theta = 2.0 * std::asin(C / (2.0 * R));
            double x_center = (x_start + x_end) / 2.0;
            double Lt = R * theta;                                  
            double L = trapezoidal_L_t(t, duration, Lt); 
            
            // Distance along rotation circle from home position
            double phi = -theta / 2.0 + L / R; // |-| --> |+| 
            double c_dis_local = R * std::sin(phi);
            double c_dis_global = x_center + c_dis_local; 
            
            // Corresponding X and Y 
            x_global = R_C * std::sin(c_dis_global / R_C);
            double P = (H_X != 0.0) ? std::sqrt(H_X * H_X + W_Y * W_Y) * std::cos(atan2(W, H) - atan2(W_Y, H_X)) : W_Y;
            y_global = R_C * std::cos(c_dis_global / R_C) - std::abs(P);

            // Z(X):
            double z_local = std::sqrt(R * R - c_dis_local * c_dis_local) - std::sqrt(R * R - (C / 2.0) * (C / 2.0));
            z_global = Z_HOME + z_local;
        }
        else
        {
            // Pull trajectory (no change in Z)
            // Rotation radius at tip
            double H = (H_X != 0.0) ? Y_HOME * std::sin(M_PI / 4) + H_X : 0;
            double W = (H_X != 0.0) ? Y_HOME * std::cos(M_PI / 4) + W_Y : Y_HOME + W_Y;
            double R_C = std::sqrt(H * H + W * W);
            
            // Angle -> distance
            x_start = x_start * R_C;
            x_end = x_end * R_C;

            // Distance from positive end along circle
            double L_total = std::abs(x_end - x_start);
            double L = trapezoidal_L_t(t, duration, L_total);
            double c_dis_global = x_start + (x_end - x_start) * (L / L_total); // |start| --> |end| 

            // Corresponding X and Y 
            x_global = R_C * std::sin(c_dis_global / R_C);                           
            double P = (H_X != 0.0) ? std::sqrt(H_X * H_X + W_Y * W_Y) * std::cos(atan2(W, H) - atan2(W_Y, H_X)) : W_Y;
            y_global = R_C * std::cos(c_dis_global / R_C) - std::abs(P);
            
            // Constant Z
            z_global = Z_HOME;                                                      
        }
    }

    // --- Coordinate Transformation ---
    void transform_coordinates(double x_c, double y_c, double H_X, double W_Y, double direction, double &X, double &Y)
    {
        // Translate from tip to origin
        double x_translated = X - x_c;
        double y_translated = Y - y_c;

        // Remaining base center to tip angle
        double H = Y_HOME * std::sin(M_PI / 4) + H_X;
        double W = Y_HOME * std::cos(M_PI / 4) + W_Y;
        double angle = direction * std::abs((std::atan2(W, H) - M_PI / 4));

        // Rotate around origin
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        double x_rotated_ = x_translated * cos_a - y_translated * sin_a;
        double y_rotated_ = x_translated * sin_a + y_translated * cos_a;

        // Translate back to tip
        X = x_rotated_ + x_c;
        Y = y_rotated_ + y_c;
    }

    // --- L(t): Path length from time using a 1/3-1/3-1/3 trapezoidal velocity profile ---
    double trapezoidal_L_t(double t, double T, double L_total)
    {
        if (T <= 0)
            return (t > 0) ? L_total : 0.0;

        double ta = T / 3.0;             // Accel, Cruise, and Decel phases are each 1/3 of total time T
        double vmax = 1.5 * L_total / T; // vmax = L_total / (T - ta)
        double a = vmax / ta;

        if (t <= 0.0)
            return 0.0;

        if (t < ta)
        { // Acceleration phase
            return 0.5 * a * t * t;
        }
        else if (t < 2.0 * ta)
        { // Cruise phase
            return 0.5 * a * ta * ta + vmax * (t - ta);
        }
        else if (t <= T)
        { // Deceleration phase
            return L_total - 0.5 * a * (T - t) * (T - t);
        }
        else
        { // After T
            return L_total;
        }
    }

    // --- IK: Calculates joint angles from tip position ---
    void IK(double L1, double L2, double L3, double X, double Y, double Z,
            double &J1, double &J2, double &J3)
    {
        // Base joint angle
        J1 = std::atan2(Y, X) - M_PI / 2;

        // Solving for J2/J3 in the 2D plane defined by the leg links
        double x_prime = std::sqrt(X * X + (Y - L1) * (Y - L1)); // Horizontal distance from J2 axis
        double D = std::sqrt(x_prime * x_prime + Z * Z);         // Straight line distance from J2 axis to tip

        if (D > (L2 + L3) || D < std::abs(L2 - L3))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "IK Warning: Position (%.3f, %.3f) is unreachable. D=%.3f", x_prime, Z, D);
            // Clamp to the boundary to avoid acos domain errors
            D = std::min(D, L2 + L3);
            D = std::max(D, std::abs(L2 - L3));
        }

        // Using Law of Cosines
        double alpha1 = std::atan2(-Z, x_prime);
        double alpha2 = std::acos((L2 * L2 + D * D - L3 * L3) / (2.0 * L2 * D));

        J2 = alpha1 - alpha2;                                                 // Second joint angle
        J3 = M_PI - std::acos((L2 * L2 + L3 * L3 - D * D) / (2.0 * L2 * L3)); // Third joint angle
    }

    // --- Action Client Communication ---
    void send_and_wait(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &points)
    {
        if (points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "send_and_wait called with an empty trajectory. Skipping.");
            return;
        }

        using namespace std::placeholders;

        control_msgs::action::FollowJointTrajectory::Goal goal_msg;
        goal_msg.trajectory.joint_names = all_joint_names_;
        goal_msg.trajectory.points = points;
        // Add a small tolerance
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.1);

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](auto future)
        {
            auto goal_handle = future.get();
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

        // Wait for the goal to be sent and acknowledged
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }

        // Wait for the result
        auto result_future = action_client_->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result");
            return;
        }

        auto result_wrapper = result_future.get();
        if (result_wrapper.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory failed with error code: %d", result_wrapper.result->error_code);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitController>();
    return 0;
}