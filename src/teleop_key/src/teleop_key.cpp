#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>

class TeleopKey : public rclcpp::Node
{
public:
    TeleopKey() : Node("teleop_key")
    {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        command_sub_ = this->create_subscription<std_msgs::msg::String>("/bt_command", 10, std::bind(&TeleopKey::cmdCallback, this, std::placeholders::_1));

        // MAVROS CLient
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        land_client_  = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

        RCLCPP_INFO(this->get_logger(), "=== Drone Teleop Keyboard ===");
        print_help();
        //control_loop();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

    double lin_speed = 1.0;
    double ang_speed = 1.0;
    double vert_speed = 0.5;

    int getch()
    {
        struct termios oldt, newt;
        int ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void print_help()
    {
        std::cout << "\n=== Drone Teleop Keyboard ===\n";
        std::cout << "t : ARM + Takeoff (GUIDED)\n";
        std::cout << "w : maju\n";
        std::cout << "s : mundur\n";
        std::cout << "a : kiri\n";
        std::cout << "d : kanan\n";
        std::cout << "r : naik\n";
        std::cout << "f : turun\n";
        std::cout << "q : yaw kiri\n";
        std::cout << "e : yaw kanan\n";
        std::cout << "x : LAND\n";
        std::cout << "h : help\n";
        std::cout << "CTRL+C : quit\n";
        std::cout << "=============================\n";
    }

    void set_mode_guided()
    {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "GUIDED";

        if (!set_mode_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "SetMode service not available");
            return;
        }

        auto future = set_mode_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "SetMode GUIDED command sent");
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    void arm_drone(bool arm)
    {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = arm;

        if (!arming_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Arming service not available");
            return;
        }

        arming_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), arm ? "Arming command sent" : "Disarming command sent");
        rclcpp::sleep_for(std::chrono::seconds(2));
    }

    void takeoff_drone(float altitude)
    {
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = altitude;
        req->latitude = 0.0;
        req->longitude = 0.0;
        req->min_pitch = 0.0;
        req->yaw = 0.0;

        if (!takeoff_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(this->get_logger(), "Takeoff service not available");
            return;
        }

        takeoff_client_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Takeoff (CommandTOL) command sent");
    }

    void takeoff_sequence(float altitude)
    {
        RCLCPP_INFO(this->get_logger(), "Takeoff Sequence Running...");
        set_mode_guided();
        arm_drone(true);
        takeoff_drone(altitude);
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist twist;
        char c;
        while (rclcpp::ok())
        {
            c = getch();
            twist = geometry_msgs::msg::Twist();

            if (c == 't') takeoff_sequence(3.0);
            else if (c == 'w') twist.linear.x = lin_speed;
            else if (c == 's') twist.linear.x = -lin_speed;
            else if (c == 'a') twist.linear.y = lin_speed;
            else if (c == 'd') twist.linear.y = -lin_speed;
            else if (c == 'r') twist.linear.z = vert_speed;
            else if (c == 'f') twist.linear.z = -vert_speed;
            else if (c == 'q') twist.angular.z = ang_speed;
            else if (c == 'e') twist.angular.z = -ang_speed;
            else if (c == 'x') land_drone();
            else if (c == 'h') print_help();
            else continue;

            vel_pub_->publish(twist);
        }
    }

    void land_drone()
    {
        auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        req->altitude = 0.0;      // target ketinggian landing
        req->latitude = 0.0;      // gunakan 0.0 untuk posisi saat ini
        req->longitude = 0.0;
        req->min_pitch = 0.0;
        req->yaw = 0.0;

        if (!land_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(this->get_logger(), "Landing service not available");
            return;
        }

        auto result_future = land_client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Landing command sent successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send landing command");
        }
    }

    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) 
    {
        geometry_msgs::msg::Twist twist;
        twist = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "BT Command received: %s", msg->data.c_str());

        if (msg->data.rfind("ARM", 0) == 0) {
            double altitude = 3.0;

            size_t pos = msg->data.find(":");
            if (pos != std::string::npos) 
            {
                std::string alt_str = msg->data.substr(pos + 1);
                try 
                {
                    altitude = std::stod(alt_str);
                } 
                catch (const std::exception &e) 
                {
                    RCLCPP_WARN(this->get_logger(), "Invalid altitude arg, using default = 3.0");
                }
            }

            takeoff_sequence(altitude);
        }
        else if (msg->data == "LAND") {
            land_drone();
        }
        else if (msg->data == "FORWARD") {
            twist.linear.x = lin_speed;
        }
        else if (msg->data == "BACKWARD") {
            twist.linear.x = -lin_speed;
        }
        else if (msg->data == "RIGHT") {
            twist.linear.y = -lin_speed;
        }
        else if (msg->data == "LEFT") {
            twist.linear.y = lin_speed;
        }
        vel_pub_->publish(twist);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<TeleopKey>(); // Use this for teleop key
    rclcpp::spin(std::make_shared<TeleopKey>()); // Use this for BehaviorTree
    rclcpp::shutdown();
    return 0;
}
