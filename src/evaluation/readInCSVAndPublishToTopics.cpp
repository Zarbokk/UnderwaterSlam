//
// Created by tim-external on 18.12.23.
//
//
// Created by jurobotics on 13.09.21.
//
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <fstream>




#define NAME_OF_PATH "/home/tim-external/dataFolder/journalPaperDatasets/ConsecutiveScanTests/ValentinBunkerKeller.csv"


class rosClassSlam : public rclcpp::Node {
public:
    rosClassSlam() : Node("publishingValentinData") {
        //we have to make sure, to get ALLL the data. Therefor we have to change that in the future.
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_system_default);
        qos.history(rmw_qos_history_policy_e::RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        qos.reliability(rmw_qos_reliability_policy_e::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(rmw_qos_durability_policy_e::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        qos.liveliness(rmw_qos_liveliness_policy_e::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT);
        qos.deadline(rmw_time_t(RMW_DURATION_INFINITE));
        qos.lifespan(rmw_time_t(RMW_DURATION_INFINITE));
        qos.liveliness_lease_duration(rmw_time_t(RMW_DURATION_INFINITE));
        qos.avoid_ros_namespace_conventions(false);


        this->publisherSonarEcho = this->create_publisher<ping360_sonar_msgs::msg::SonarEcho>(
                "positionOverTime", qos);


        this->publisherEKF = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "positionOverTimeGT", qos);


        std::chrono::duration<double> my_timer_duration = std::chrono::duration<double>(1.0/2.0);
        this->timer_ = this->create_wall_timer(
                my_timer_duration, std::bind(&rosClassSlam::readNextLine, this));


        infile.open(NAME_OF_PATH);
    }


private:

    //GraphSlam things
    rclcpp::Publisher<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr publisherSonarEcho;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherEKF;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ifstream infile;

public:

    void readNextLine() {
        std::cout << "test" << std::endl;
        std::string line;
        std::getline(infile, line);
//        std::istringstream iss(line);
        std::cout << line << std::endl;
    }


};


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosClassSlam>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return (0);
}
