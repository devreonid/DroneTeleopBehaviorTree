#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> command_pub;

BT::NodeStatus Arm(BT::TreeNode& node)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    double altitude = 3.0;
    node.getInput("altitude", altitude);  // get param from XML file if exist

    auto msg = std_msgs::msg::String();
    msg.data = "ARM" + std::to_string(altitude);
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending ARM command and TAKEOFF with alt %.2f", altitude);

    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Land(BT::TreeNode& self)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    auto msg = std_msgs::msg::String();
    msg.data = "LAND";
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending LAND command");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Forward(BT::TreeNode& self)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    auto msg = std_msgs::msg::String();
    msg.data = "FORWARD";
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending FORWARD command");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Backward(BT::TreeNode& self)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    auto msg = std_msgs::msg::String();
    msg.data = "BACKWARD";
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending BACKWARD command");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Right(BT::TreeNode& self)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    auto msg = std_msgs::msg::String();
    msg.data = "RIGHT";
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending RIGHT command");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Left(BT::TreeNode& self)
{
    rclcpp::sleep_for(std::chrono::seconds(10));
    auto msg = std_msgs::msg::String();
    msg.data = "LEFT";
    command_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("bt_tester"), "BT: Sending LEFT command");
    return BT::NodeStatus::SUCCESS;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_node");

    command_pub = node->create_publisher<std_msgs::msg::String>("/bt_command", 10);

    BT::BehaviorTreeFactory factory;

    factory.registerSimpleAction("Arm", Arm, { BT::InputPort<double>("altitude") });
    factory.registerSimpleAction("Land", Land);
    factory.registerSimpleAction("Forward", Forward);
    factory.registerSimpleAction("Backward", Backward);
    factory.registerSimpleAction("Right", Right);
    factory.registerSimpleAction("Left", Left);

    std::string package_share_dir = ament_index_cpp::get_package_share_directory("teleop_key");
    std::string xml_file = package_share_dir + "/bt_trees/tree.xml";

    auto tree = factory.createTreeFromFile(xml_file);

    rclcpp::Rate loop_rate(1);

    while (rclcpp::ok()) {
        tree.tickOnce();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}