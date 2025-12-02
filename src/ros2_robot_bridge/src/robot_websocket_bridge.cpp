#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>
#include <csignal>

// Mongoose WebSocket library (C library)
extern "C" {
#include "mongoose.h"
}

// Protocol Buffer headers (C library)
extern "C" {
#include "pb_encode.h"
#include "pb_decode.h"
#include "generated/inc/public_api_up.pb.h"
#include "generated/inc/public_api_down.pb.h"
#include "generated/inc/public_api_types.pb.h"
}

#define EXPECTED_PROTOCOL_MAJOR_VERSION 1

class RobotWebSocketBridge : public rclcpp::Node
{
public:
    RobotWebSocketBridge()
        : Node("robot_websocket_bridge"),
          websocket_connected_(false),
          robot_initialized_(false),
          last_cmd_time_(std::chrono::steady_clock::now())
    {
        // Declare parameters
        this->declare_parameter<std::string>("websocket_url", "ws://localhost:8439");
        this->declare_parameter<double>("command_rate", 50.0);
        this->declare_parameter<int>("report_frequency", 50);

        // Get parameters
        websocket_url_ = this->get_parameter("websocket_url").as_string();
        command_rate_ = this->get_parameter("command_rate").as_double();
        report_frequency_ = this->get_parameter("report_frequency").as_int();

        RCLCPP_INFO(this->get_logger(), "WebSocket URL: %s", websocket_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Command rate: %.1f Hz", command_rate_);
        RCLCPP_INFO(this->get_logger(), "Report frequency: %d Hz", report_frequency_);

        // Initialize mongoose
        mg_mgr_init(&mgr_);
        mg_log_set(MG_LL_INFO);

        // Create cmd_vel subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobotWebSocketBridge::cmd_vel_callback, this, std::placeholders::_1));

        // Create odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Create timer for periodic command sending
        double timer_period = 1.0 / command_rate_;
        command_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period * 1000)),
            std::bind(&RobotWebSocketBridge::command_timer_callback, this));

        // Create timer for WebSocket polling
        ws_poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RobotWebSocketBridge::websocket_poll_callback, this));

        // Connect to WebSocket after node is fully constructed
        connect_websocket();

        // Initialize current command to zero
        current_cmd_.linear.x = 0.0;
        current_cmd_.linear.y = 0.0;
        current_cmd_.angular.z = 0.0;
    }

    ~RobotWebSocketBridge()
    {
        shutdown_gracefully();
    }

public:
    void shutdown_gracefully()
    {
        // Prevent double shutdown
        if (is_shutting_down_.exchange(true))
        {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Shutting down gracefully...");
        
        if (!websocket_connected_ || !websocket_connection_)
        {
            if (websocket_connection_ == nullptr)
            {
                mg_mgr_free(&mgr_);
            }
            return;
        }

        // Step 1: Send zero velocity commands to stop the robot
        if (robot_initialized_)
        {
            RCLCPP_INFO(this->get_logger(), "Sending zero velocity commands...");
            for (int i = 0; i < 3; i++)
            {
                send_move_command(0.0, 0.0, 0.0);
                // Poll to send the message
                for (int j = 0; j < 10; j++)
                {
                    mg_mgr_poll(&mgr_, 1);
                    if (websocket_connection_ && websocket_connection_->send.len == 0)
                    {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }

        // Step 2: Send deinit message
        if (robot_initialized_)
        {
            RCLCPP_INFO(this->get_logger(), "Sending deinitialization message...");
            send_deinit_message();
            
            // Step 3: Wait for deinit message to be sent (check send buffer)
            int poll_count = 0;
            while (websocket_connection_ && websocket_connection_->send.len > 0 && poll_count < 100)
            {
                mg_mgr_poll(&mgr_, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                poll_count++;
            }
            
            // Step 4: Give robot time to process the deinit message
            RCLCPP_INFO(this->get_logger(), "Waiting for robot to process deinitialization...");
            for (int i = 0; i < 20; i++)  // 200ms total
            {
                mg_mgr_poll(&mgr_, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Step 5: Close WebSocket connection gracefully
        if (websocket_connection_)
        {
            websocket_connection_->is_closing = 1;
            // Poll a few times to allow graceful close
            for (int i = 0; i < 10; i++)
            {
                mg_mgr_poll(&mgr_, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        mg_mgr_free(&mgr_);
        RCLCPP_INFO(this->get_logger(), "Shutdown complete");
    }

private:
    void connect_websocket()
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to WebSocket: %s", websocket_url_.c_str());
        websocket_connection_ = mg_ws_connect(&mgr_, websocket_url_.c_str(), 
                                             websocket_handler, this, NULL);
        if (websocket_connection_ == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start WebSocket connection");
        }
    }

    static void websocket_handler(struct mg_connection *c, int ev, void *ev_data)
    {
        RobotWebSocketBridge *node = static_cast<RobotWebSocketBridge *>(c->fn_data);
        if (node == nullptr) return;

        switch (ev)
        {
        case MG_EV_WS_OPEN:
        {
            RCLCPP_INFO(node->get_logger(), "WebSocket connection opened");
            node->websocket_connected_ = true;
            
            // Set report frequency
            node->send_report_frequency_message();
            
            // Wait a bit then initialize
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            node->send_init_message();
            break;
        }
        case MG_EV_WS_MSG:
        {
            struct mg_ws_message *wm = (struct mg_ws_message *)ev_data;
            if (wm->flags & WEBSOCKET_OP_BINARY)
            {
                node->handle_websocket_message(wm->data.buf, wm->data.len);
            }
            break;
        }
        case MG_EV_WS_CTL:
        {
            // WebSocket control message
            break;
        }
        case MG_EV_ERROR:
        {
            RCLCPP_ERROR(node->get_logger(), "WebSocket error occurred");
            node->websocket_connected_ = false;
            node->robot_initialized_ = false;
            c->is_closing = 1;
            break;
        }
        case MG_EV_CLOSE:
        {
            RCLCPP_WARN(node->get_logger(), "WebSocket connection closed");
            node->websocket_connected_ = false;
            node->robot_initialized_ = false;
            break;
        }
        default:
            break;
        }
    }

    void handle_websocket_message(const void *data, size_t len)
    {
        pb_istream_t stream = pb_istream_from_buffer((const uint8_t *)data, len);
        APIUp rx_msg = APIUp_init_zero;
        bool status = pb_decode(&stream, APIUp_fields, &rx_msg);
        
        if (!status)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to decode APIUp message");
            return;
        }

        // Check protocol version
        if (rx_msg.protocol_major_version != EXPECTED_PROTOCOL_MAJOR_VERSION)
        {
            RCLCPP_WARN(this->get_logger(), 
                       "Protocol major version mismatch: expected %d, got %d",
                       EXPECTED_PROTOCOL_MAJOR_VERSION, rx_msg.protocol_major_version);
        }

        // Handle log messages (log is a pb_callback_t in nanopb, requires callback setup)
        // For now, we skip log handling - if needed, set up a callback function

        // Handle base status and odometry
        if (rx_msg.which_status == APIUp_base_status_tag)
        {
            const BaseStatus &base_status = rx_msg.status.base_status;
            
            // Check if robot is initialized
            if (base_status.api_control_initialized && !robot_initialized_)
            {
                robot_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Robot initialized successfully");
            }

            // Publish odometry if available
            if (base_status.has_estimated_odometry)
            {
                publish_odometry(base_status.estimated_odometry);
            }
        }
    }

    void publish_odometry(const BaseEstimatedOdometry &odom)
    {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set position
        odom_msg.pose.pose.position.x = odom.pos_x;
        odom_msg.pose.pose.position.y = odom.pos_y;
        odom_msg.pose.pose.position.z = 0.0;

        // Set orientation (yaw from pos_z)
        double yaw = odom.pos_z;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sin(yaw / 2.0);
        odom_msg.pose.pose.orientation.w = cos(yaw / 2.0);

        // Set velocity
        odom_msg.twist.twist.linear.x = odom.speed_x;
        odom_msg.twist.twist.linear.y = odom.speed_y;
        odom_msg.twist.twist.angular.z = odom.speed_z;

        // Publish
        odom_pub_->publish(odom_msg);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        current_cmd_ = *msg;
        last_cmd_time_ = std::chrono::steady_clock::now();
    }

    void command_timer_callback()
    {
        if (!websocket_connected_ || !robot_initialized_)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        
        // Check if command is recent (within 1 second)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_cmd_time_).count();
        
        if (elapsed > 1000)
        {
            // No recent command, send zero velocity
            send_move_command(0.0, 0.0, 0.0);
        }
        else
        {
            // Send current command
            send_move_command(
                current_cmd_.linear.x,
                current_cmd_.linear.y,
                current_cmd_.angular.z
            );
        }
    }

    void websocket_poll_callback()
    {
        mg_mgr_poll(&mgr_, 0);
    }

    void send_report_frequency_message()
    {
        APIDown msg = APIDown_init_default;
        msg.which_down = APIDown_set_report_frequency_tag;
        
        // Map report frequency
        switch (report_frequency_)
        {
        case 1000:
            msg.down.set_report_frequency = ReportFrequency_Rf1000Hz;
            break;
        case 500:
            msg.down.set_report_frequency = ReportFrequency_Rf500Hz;
            break;
        case 250:
            msg.down.set_report_frequency = ReportFrequency_Rf250Hz;
            break;
        case 100:
            msg.down.set_report_frequency = ReportFrequency_Rf100Hz;
            break;
        case 50:
            msg.down.set_report_frequency = ReportFrequency_Rf50Hz;
            break;
        case 1:
            // Note: Rf1Hz (value 5) exists in proto but not in generated C headers
            // Using Rf50Hz as fallback. To use 1Hz, regenerate protobuf files.
            msg.down.set_report_frequency = ReportFrequency_Rf50Hz;
            RCLCPP_WARN(this->get_logger(), "1Hz report frequency not available in generated headers, using 50Hz");
            break;
        default:
            msg.down.set_report_frequency = ReportFrequency_Rf50Hz;
            break;
        }

        send_protobuf_message(&msg);
    }

    void send_init_message()
    {
        APIDown msg = APIDown_init_default;
        msg.which_down = APIDown_base_command_tag;
        msg.down.base_command.which_command = BaseCommand_api_control_initialize_tag;
        msg.down.base_command.command.api_control_initialize = true;

        send_protobuf_message(&msg);
        RCLCPP_INFO(this->get_logger(), "Sent initialization message");
    }

    void send_deinit_message()
    {
        APIDown msg = APIDown_init_default;
        msg.which_down = APIDown_base_command_tag;
        msg.down.base_command.which_command = BaseCommand_api_control_initialize_tag;
        msg.down.base_command.command.api_control_initialize = false;

        send_protobuf_message(&msg);
        RCLCPP_INFO(this->get_logger(), "Sent deinitialization message");
    }

    void send_move_command(double speed_x, double speed_y, double speed_z)
    {
        APIDown msg = APIDown_init_default;
        msg.which_down = APIDown_base_command_tag;
        msg.down.base_command.which_command = BaseCommand_simple_move_command_tag;
        msg.down.base_command.command.simple_move_command.which_command = 
            SimpleBaseMoveCommand_xyz_speed_tag;
        msg.down.base_command.command.simple_move_command.command.xyz_speed.speed_x = 
            static_cast<float>(speed_x);
        msg.down.base_command.command.simple_move_command.command.xyz_speed.speed_y = 
            static_cast<float>(speed_y);
        msg.down.base_command.command.simple_move_command.command.xyz_speed.speed_z = 
            static_cast<float>(speed_z);

        send_protobuf_message(&msg);
    }

    void send_protobuf_message(APIDown *msg)
    {
        if (!websocket_connection_ || !websocket_connected_)
        {
            return;
        }

        uint8_t buffer[1024];
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        bool status = pb_encode(&stream, APIDown_fields, msg);
        
        if (!status)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode APIDown message");
            return;
        }

        mg_ws_send(websocket_connection_, buffer, stream.bytes_written, WEBSOCKET_OP_BINARY);
    }

    // ROS2 members
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    rclcpp::TimerBase::SharedPtr ws_poll_timer_;

    // WebSocket members
    struct mg_mgr mgr_;
    struct mg_connection *websocket_connection_ = nullptr;
    bool websocket_connected_;
    bool robot_initialized_;

    // Command state
    geometry_msgs::msg::Twist current_cmd_;
    std::mutex cmd_mutex_;
    std::chrono::steady_clock::time_point last_cmd_time_;

    // Parameters
    std::string websocket_url_;
    double command_rate_;
    int report_frequency_;
    
    // Shutdown flag
    std::atomic<bool> is_shutting_down_{false};
};

// Global node pointer for signal handler
static std::shared_ptr<RobotWebSocketBridge> g_node = nullptr;

void signal_handler(int signal)
{
    if (g_node)
    {
        RCLCPP_INFO(rclcpp::get_logger("robot_websocket_bridge"), 
                   "Received signal %d, shutting down gracefully...", signal);
        g_node->shutdown_gracefully();
    }
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<RobotWebSocketBridge>();
    
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    // Spin the node
    rclcpp::spin(g_node);
    
    // Explicitly call shutdown if not already called by signal handler
    if (g_node)
    {
        g_node->shutdown_gracefully();
        g_node.reset();
    }
    
    rclcpp::shutdown();
    return 0;
}

