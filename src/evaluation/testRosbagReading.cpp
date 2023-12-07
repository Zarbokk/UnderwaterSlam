#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <fstream>
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/serialization.hpp"
#include "ping360_sonar_msgs/msg/sonar_echo.hpp"
#include "px4_msgs/msg/sensor_combined.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "micron_driver_ros/msg/scan_line.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"
#include "px4_msgs/msg/actuator_outputs.hpp"

std::string pathToFolder = "/home/tim-external/dataFolder/DFKITests/publishingFiles/ros2Bags/";
std::string nameOfRosbag = "dynamic2_filtered";

int main(int argc, char** argv)
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);

    rclcpp::Node node("test");

    rosbag2_storage::StorageOptions storage_options{};

    auto file_path = pathToFolder+nameOfRosbag+"/" + nameOfRosbag + "_0.db3";
    storage_options.uri = file_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    const auto topics = reader.get_all_topics_and_types();
//    sensor_msgs::msg::Imu msg;

    for (const auto topic : topics)
        RCLCPP_INFO(node.get_logger(), topic.name.c_str());


    std::ofstream generalAllSensors(pathToFolder+nameOfRosbag+"/allMeasurementData.yaml");
    std::ofstream imuData(pathToFolder+nameOfRosbag+"/xsensImuData.yaml");
    std::ofstream sonarIntensity(pathToFolder+nameOfRosbag+"/ping360SonarData.yaml");
    std::ofstream px4SensorCombined(pathToFolder+nameOfRosbag+"/px4IMUData.yaml");
    std::ofstream pressure(pathToFolder+nameOfRosbag+"/pressureData.yaml");
    std::ofstream velocityEstimate(pathToFolder+nameOfRosbag+"/waterlinkedA50Data.yaml");
    std::ofstream markers(pathToFolder+nameOfRosbag+"/markersTrackingSystemData.yaml");
    std::ofstream rigidBodys(pathToFolder+nameOfRosbag+"/rigidBodiesTrackingSystemData.yaml");
    std::ofstream tritechScanLines(pathToFolder+nameOfRosbag+"/tritechSonarData.yaml");
    std::ofstream actuatorOutput(pathToFolder+nameOfRosbag+"/actuatorOutputData.yaml");

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name == "/imu/data") {
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            sensor_msgs::msg::Imu::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Imu>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
            imuData<< sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }

        if(msg->topic_name == "/sonar/intensity"){
            rclcpp::Serialization<ping360_sonar_msgs::msg::SonarEcho> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            ping360_sonar_msgs::msg::SonarEcho::SharedPtr ros_msg = std::make_shared<ping360_sonar_msgs::msg::SonarEcho>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << ping360_sonar_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << ping360_sonar_msgs::msg::to_yaml(*ros_msg) << std::endl;
            sonarIntensity<< ping360_sonar_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/fmu/out/sensor_combined"){
            rclcpp::Serialization<px4_msgs::msg::SensorCombined> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            px4_msgs::msg::SensorCombined::SharedPtr ros_msg = std::make_shared<px4_msgs::msg::SensorCombined>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << px4_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << px4_msgs::msg::to_yaml(*ros_msg) << std::endl;
            px4SensorCombined<< px4_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/pressure"){
            rclcpp::Serialization<sensor_msgs::msg::FluidPressure> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            sensor_msgs::msg::FluidPressure::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::FluidPressure>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
            pressure<< sensor_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/velocity_estimate"){
            rclcpp::Serialization<waterlinked_a50::msg::TransducerReportStamped> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            waterlinked_a50::msg::TransducerReportStamped::SharedPtr ros_msg = std::make_shared<waterlinked_a50::msg::TransducerReportStamped>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << waterlinked_a50::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << waterlinked_a50::msg::to_yaml(*ros_msg) << std::endl;
            velocityEstimate<< waterlinked_a50::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/markers"){
            rclcpp::Serialization<mocap_msgs::msg::Markers> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            mocap_msgs::msg::Markers::SharedPtr ros_msg = std::make_shared<mocap_msgs::msg::Markers>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
            markers<< mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/rigid_bodies"){
            rclcpp::Serialization<mocap_msgs::msg::RigidBodies> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            mocap_msgs::msg::RigidBodies::SharedPtr ros_msg = std::make_shared<mocap_msgs::msg::RigidBodies>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
            rigidBodys<< mocap_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/tritech_sonar/scan_lines"){
            rclcpp::Serialization<micron_driver_ros::msg::ScanLine> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            micron_driver_ros::msg::ScanLine::SharedPtr ros_msg = std::make_shared<micron_driver_ros::msg::ScanLine>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << micron_driver_ros::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << micron_driver_ros::msg::to_yaml(*ros_msg) << std::endl;
            tritechScanLines<< micron_driver_ros::msg::to_yaml(*ros_msg) << std::endl;
        }
        if(msg->topic_name == "/fmu/out/actuator_outputs"){
            rclcpp::Serialization<px4_msgs::msg::ActuatorOutputs> serialization_;
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            px4_msgs::msg::ActuatorOutputs::SharedPtr ros_msg = std::make_shared<px4_msgs::msg::ActuatorOutputs>();
            serialization_.deserialize_message(&serialized_msg, ros_msg.get());

//            std::cout << micron_driver_ros::msg::to_yaml(*ros_msg) << std::endl;
            generalAllSensors << px4_msgs::msg::to_yaml(*ros_msg) << std::endl;
            actuatorOutput<< px4_msgs::msg::to_yaml(*ros_msg) << std::endl;
        }
//        std::cout << "next:"<< std::endl;
    }

    return 0;
}