#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <fl/Headers.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

// MISC FUNCS
// Distance between (x1,y1) and (x2,y2)
double pointDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// Angle from (x1,y1) to (x2,y2) in degrees [-180, 180]
double pointAngle(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::atan2(dy, dx) * (180.0 / M_PI);
}

void polarToCartesian(double mag, double angle_deg, double& x, double& y) {
    double angle_rad = angle_deg * (M_PI / 180.0); // Convert to radians
    x = mag * std::cos(angle_rad);
    y = mag * std::sin(angle_rad);
}

class FuzzyNode : public rclcpp::Node
{
public:
    FuzzyNode()
    : Node("fuzzy_node")
    {
        // Load the FuzzyLite engine from file
        try {

            std::string package_share_dir = ament_index_cpp::get_package_share_directory("robotino_fuzzy");
            std::string fll_path = package_share_dir + "/resources/ObstacleAvoidance.fll";
            engine_ = fl::FllImporter().fromFile(fll_path);
            
            std::string status;
            if (!engine_->isReady(&status)) {
                throw fl::Exception("[engine error] engine is not ready:\n" + status, FL_AT);
            }
            
            RCLCPP_INFO(this->get_logger(), "FuzzyLite engine loaded successfully");
        } catch (const fl::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading FuzzyLite engine: %s", e.what());
            throw;
        }

        // Create Twist publisher (velocity command)
        twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robotino4/cmd_vel", 10);

        // Debug publishers FIXME:
        vel_angle_pub = this->create_publisher<std_msgs::msg::Float64>("/debug/vel_angle", 10);
        vel_mag_pub = this->create_publisher<std_msgs::msg::Float64>("/debug/vel_mag", 10);
        target_angle_pub = this->create_publisher<std_msgs::msg::Float64>("/debug/target_angle", 10);
        target_dist_pub = this->create_publisher<std_msgs::msg::Float64>("/debug/target_distance", 10);

        // Odometry subscriber
        odo_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robotino4/odometry",
            rclcpp::QoS(10),
            std::bind(&FuzzyNode::odometry_callback, this, std::placeholders::_1));
        
        // Target point subscriber
        target_sub = this->create_subscription<geometry_msgs::msg::Point>(
            "/robotino4/target_coordinate",
            rclcpp::QoS(10),
            std::bind(&FuzzyNode::target_point_callback, this, std::placeholders::_1));

        // Proximity sensors subscriber
        sensors_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/robotino4/proximity_sensors",
            rclcpp::QoS(10),
            std::bind(&FuzzyNode::sensors_callback, this,std::placeholders::_1));

        // Initialize variables
        robot_x = 0.0;
        robot_y = 0.0;
        target_x = 0.0;
        target_y = 0.0;
        target_distance = 0.0;
        target_angle = 0.0;
        for (int i = 0; i < 9; i++)
        {
          sensors[i] = 0.0;
        }

        // Get input and output variables pointers of a fuzzy engine
        sens0 = engine_->getInputVariable("sens0");
        sens1 = engine_->getInputVariable("sens1");
        sens2 = engine_->getInputVariable("sens2");
        sens3 = engine_->getInputVariable("sens3");
        sens4 = engine_->getInputVariable("sens4");
        sens5 = engine_->getInputVariable("sens5");
        sens6 = engine_->getInputVariable("sens6");
        sens7 = engine_->getInputVariable("sens7");
        sens8 = engine_->getInputVariable("sens8");

        target_angle_inp = engine_->getInputVariable("target_angle");
        target_distance_inp = engine_->getInputVariable("target_distance");

        vel_angle_out = engine_->getOutputVariable("vel_angle");
        vel_mag_out = engine_->getOutputVariable("vel_mag");
        
        // Create timer for periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // Update every 10ms
            std::bind(&FuzzyNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Compute time measurement
        auto start = std::chrono::high_resolution_clock::now();
        
        // Set input, process, and get output
        sens0->setValue(sensors[0]);
        sens1->setValue(sensors[1]);
        sens2->setValue(sensors[2]);
        sens3->setValue(sensors[3]);
        sens4->setValue(sensors[4]);
        sens5->setValue(sensors[5]);
        sens6->setValue(sensors[6]);
        sens7->setValue(sensors[7]);
        sens8->setValue(sensors[8]);

        target_angle = pointAngle(robot_x, robot_y, target_x, target_y);
        target_distance = pointDistance(robot_x, robot_y, target_x, target_y);

        target_angle_inp->setValue(target_angle);
        target_distance_inp->setValue(target_distance);

        engine_->process();
        double out_vel_ang = vel_angle_out->getValue();
        double out_vel_mag = vel_mag_out->getValue();

        double vx;
        double vy;
        polarToCartesian(out_vel_mag, out_vel_ang, vx, vy);

        // Publish message
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vx;
        twist_msg.linear.y = vy;
        twist_msg.linear.z = 0.0;

        twist_pub->publish(twist_msg);

        // Stop timer
        auto end = std::chrono::high_resolution_clock::now();
    
        // Calculate duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // Log results
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "Posted: vx = %.4f   vy = %.4f, Compute time = %ld µs", vx, vy, duration.count());
        RCLCPP_INFO(this->get_logger(), "Posted: vx = %.4f   vy = %.4f, Compute time = %ld µs", vx, vy, duration.count());

        // DEBUG PUBLISHERS FIXME:

        // Velocity angle (output)
        auto angle_msg = std_msgs::msg::Float64();
        angle_msg.data = out_vel_ang;
        vel_angle_pub->publish(angle_msg);

        // Velocity magnitude (output)
        auto mag_msg = std_msgs::msg::Float64();
        mag_msg.data = out_vel_mag;
        vel_mag_pub->publish(mag_msg);

        // Target angle (input)
        auto target_ang_msg = std_msgs::msg::Float64();
        target_ang_msg.data = target_angle;
        target_angle_pub->publish(target_ang_msg);

        // Target distance (input)
        auto target_dist_msg = std_msgs::msg::Float64();
        target_dist_msg.data = target_distance;
        target_dist_pub->publish(target_dist_msg);

    }

    void odometry_callback(const nav_msgs::msg::Odometry msg){
      robot_x = msg.pose.pose.position.x;
      robot_y = msg.pose.pose.position.y;
    }

    void target_point_callback(const geometry_msgs::msg::Point msg){
      target_x = msg.x;
      target_y = msg.y;
    }

    void sensors_callback(const std_msgs::msg::Float64MultiArray msg){
      for (int i = 0; i < 9; i++)
      {
        sensors[i] = msg.data[i];
      }
    }

    double robot_x;
    double robot_y;
    double target_x;
    double target_y;

    double sensors[9];

    double target_distance;
    double target_angle;

    // Input and output variables of fuzzy engine
    fl::InputVariable* sens0;
    fl::InputVariable* sens1;
    fl::InputVariable* sens2;
    fl::InputVariable* sens3;
    fl::InputVariable* sens4;
    fl::InputVariable* sens5;
    fl::InputVariable* sens6;
    fl::InputVariable* sens7;
    fl::InputVariable* sens8;

    fl::InputVariable* target_angle_inp;
    fl::InputVariable* target_distance_inp;

    fl::OutputVariable* vel_angle_out;
    fl::OutputVariable* vel_mag_out;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

    // DEBUG PUBLISHERS FIXME:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_mag_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_angle_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_dist_pub;
    // Timer for periodic stuff
    rclcpp::TimerBase::SharedPtr timer_;

    // Fuzzylite engine
    fl::Engine* engine_;

    // Debug
    int counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyNode>());
    rclcpp::shutdown();
    return 0;
}