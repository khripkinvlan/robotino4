#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <fl/Headers.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;

// MISC FUNCS
// Distance between (x1,y1) and (x2,y2)
// double pointDistance(double x1, double y1, double x2, double y2) {
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     return std::sqrt(dx*dx + dy*dy);
// }

// Angle from (x1,y1) to (x2,y2) in degrees [-180, 180]
// double pointAngle(double x1, double y1, double x2, double y2) {
//     double dx = x2 - x1;
//     double dy = y2 - y1;
//     return std::atan2(dy, dx) * (180.0 / M_PI);
// }

// void polarToCartesian(double mag, double angle_deg, double& x, double& y) {
//     double angle_rad = angle_deg * (M_PI / 180.0); // Convert to radians
//     x = mag * std::cos(angle_rad);
//     y = mag * std::sin(angle_rad);
// }

const double PI = 3.14159265358979323846;

// Get projection of robot velocity on sensor's line
double projectVelocityOnSensor(int sensorIndex, double vx, double vy) {
    // Угол в радианах для данного датчика
    double angle = sensorIndex * 40.0 * (PI / 180.0); // Преобразуем градусы в радианы
    double sensorX = cos(angle);
    double sensorY = sin(angle);
    double projection = (vx * sensorX + vy * sensorY);
    return projection;
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
        robot_X = 0.0;
        robot_Y = 0.0;
        target_X = 0.0;
        target_Y = 0.0;
        target_errX = 0.0;
        target_errY = 0.0;
        for (int i = 0; i < 9; i++)
        {
          sensors[i] = 0.0;
          previousReadings[i] = 0.0;
          sensDerivatives[i] = 0.0;
        }

        
        // Get input and output variables pointers of a fuzzy engine
        
        for (int i = 0; i < 9; i++)
        {
            sensors_inp[i] = engine_->getInputVariable("sens" + std::to_string(i));
        }
        for (int i = 0; i < 9; i++)
        {
            sensDer_inp[i] = engine_->getInputVariable("sensDer" + std::to_string(i));
        }

        target_errX_inp = engine_->getInputVariable("target_errX");
        target_errY_inp = engine_->getInputVariable("target_errY");

        vel_X_out = engine_->getOutputVariable("vel_X");
        vel_Y_out = engine_->getOutputVariable("vel_Y");
        
        // Create timer for periodic processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // Update every 10ms
            std::bind(&FuzzyNode::timer_callback, this));
    }

private:

    // Функция для вычисления производной показаний датчика
    double update(double currentReading, double deltaTime, int i) {
        double derivative = (currentReading - this->previousReadings[i]) / deltaTime;
        this->previousReadings[i]= currentReading;
        return derivative;
    }
    void timer_callback()
    {
        // Compute time measurement
        auto start = std::chrono::high_resolution_clock::now();
        
        // Set input, process, and get output
        for (int i = 0; i < 9; i++)
        {
          sensors_inp[i]->setValue(sensors[i]);
        }

        for(int i = 0; i < 9; i++)
        {
            sensDer_inp[i]->setValue(this->sensDerivatives[0]);
        }

        target_errX = target_X - robot_X;
        target_errY = target_Y - robot_Y;

        target_errX_inp->setValue(target_errX);
        target_errY_inp->setValue(target_errY);

        engine_->process();
        double vel_X_cmd = vel_X_out->getValue();
        double vel_Y_cmd = vel_Y_out->getValue();

        // Publish message
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vel_X_cmd;
        twist_msg.linear.y = vel_Y_cmd;
        twist_msg.linear.z = 0.0;

        twist_pub->publish(twist_msg);

        // Stop timer
        auto end = std::chrono::high_resolution_clock::now();
    
        // Calculate duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // Log results
        RCLCPP_INFO(this->get_logger(), "target_errX = %.4f   target_errY = %.4f",  target_errX, target_errY);
        RCLCPP_INFO(this->get_logger(), "Posted: vx = %.4f   vy = %.4f, Compute time = %ld µs", twist_msg.linear.x, vel_Y_cmd, duration.count());

    }

    void odometry_callback(const nav_msgs::msg::Odometry msg){
      robot_X = msg.pose.pose.position.x;
      robot_Y = msg.pose.pose.position.y;
    }

    void target_point_callback(const geometry_msgs::msg::Point msg){
      target_X = msg.x;
      target_Y = msg.y;
    }

    void sensors_callback(const std_msgs::msg::Float64MultiArray msg){
      for (int i = 0; i < 9; i++)
      {
        sensors[i] = msg.data[i];
      }
    }

    double robot_X;
    double robot_Y;
    double target_X;
    double target_Y;

    double target_errX;
    double target_errY;

    double sensors[9];
    double previousReadings[9];
    double sensDerivatives[9];

    // Input and output variables of fuzzy engine
    
    

    fl::InputVariable* sensors_inp[9];
    fl::InputVariable* sensDer_inp[9];

    fl::InputVariable* target_errX_inp;
    fl::InputVariable* target_errY_inp;

    fl::OutputVariable* vel_X_out;
    fl::OutputVariable* vel_Y_out;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    
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