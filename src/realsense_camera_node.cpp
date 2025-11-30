#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>

class RealSenseCameraNode : public rclcpp::Node
{
public:
    RealSenseCameraNode() : Node("realsense_camera_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("camera_name", "camera");
        this->declare_parameter<std::string>("camera_serial", "");
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        
        camera_name_ = this->get_parameter("camera_name").as_string();
        camera_serial_ = this->get_parameter("camera_serial").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_int();
        
        // Create topic prefix
        std::string topic_prefix = "/camera_" + camera_name_;
        
        // Create publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix + "/image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix + "/camera_info", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix + "/depth/image_raw", 10);
        
        // Initialize RealSense
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize RealSense camera");
            return;
        }
        
        // Start capture thread
        running_ = true;
        capture_thread_ = std::thread(&RealSenseCameraNode::captureLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "RealSense camera node started (name: %s)", camera_name_.c_str());
    }
    
    ~RealSenseCameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (pipeline_) {
            pipeline_->stop();
        }
    }

private:
    bool initCamera()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing RealSense camera: %s...", camera_name_.c_str());
        
        try {
            // Create context
            ctx_ = std::make_shared<rs2::context>();
            
            // Query devices
            auto devices = ctx_->query_devices();
            size_t device_count = devices.size();
            RCLCPP_INFO(this->get_logger(), "Detected %zu RealSense devices", device_count);
            
            if (device_count == 0) {
                RCLCPP_ERROR(this->get_logger(), "No RealSense devices found");
                return false;
            }
            
            // Collect device serials
            std::vector<std::string> device_serials;
            std::vector<std::string> device_names;
            
            for (size_t i = 0; i < device_count; ++i) {
                try {
                    auto dev = devices[i];
                    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                    std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                    device_serials.push_back(serial);
                    device_names.push_back(name);
                    RCLCPP_INFO(this->get_logger(), "  Device %zu: Serial=%s, Name=%s", i, serial.c_str(), name.c_str());
                } catch (const rs2::error& e) {
                    RCLCPP_WARN(this->get_logger(), "Cannot access device %zu: %s (skipping)", i, e.what());
                    continue;
                }
            }
            
            if (device_serials.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No accessible RealSense devices");
                return false;
            }
            
            // Select device
            std::string selected_serial;
            if (!camera_serial_.empty()) {
                // Use specified serial
                auto it = std::find(device_serials.begin(), device_serials.end(), camera_serial_);
                if (it != device_serials.end()) {
                    selected_serial = camera_serial_;
                    RCLCPP_INFO(this->get_logger(), "Found specified device: %s", selected_serial.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Device with serial %s not found", camera_serial_.c_str());
                    return false;
                }
            } else {
                // Auto-select based on camera_name
                if (camera_name_ == "front" && device_serials.size() > 0) {
                    selected_serial = device_serials[0];
                    RCLCPP_INFO(this->get_logger(), "Auto-selected first device (front): %s", selected_serial.c_str());
                } else if (camera_name_ == "back") {
                    if (device_serials.size() > 1) {
                        selected_serial = device_serials[1];
                        RCLCPP_INFO(this->get_logger(), "Auto-selected second device (back): %s", selected_serial.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Only one device available, back camera will be disabled");
                        return false;
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Cannot auto-select device for camera_name=%s", camera_name_.c_str());
                    return false;
                }
            }
            
            // Create pipeline
            pipeline_ = std::make_shared<rs2::pipeline>(*ctx_);
            rs2::config cfg;
            cfg.enable_device(selected_serial);
            
            // Try different resolutions
            std::vector<std::tuple<int, int, int>> resolutions = {
                {width_, height_, fps_},
                {640, 480, 30},
                {640, 480, 15},
                {640, 360, 30},
                {424, 240, 30}
            };
            
            bool pipeline_started = false;
            for (const auto& res : resolutions) {
                int w = std::get<0>(res);
                int h = std::get<1>(res);
                int f = std::get<2>(res);
                
                try {
                    RCLCPP_INFO(this->get_logger(), "Trying configuration: RGB=%dx%d@%dfps, Depth=%dx%d@%dfps", 
                               w, h, f, w, h, f);
                    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, f);
                    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, f);
                    
                    auto profile = pipeline_->start(cfg);
                    width_ = w;
                    height_ = h;
                    fps_ = f;
                    pipeline_started = true;
                    RCLCPP_INFO(this->get_logger(), "Successfully configured: %dx%d@%dfps", w, h, f);
                    break;
                } catch (const rs2::error& e) {
                    RCLCPP_DEBUG(this->get_logger(), "Configuration %dx%d@%dfps not supported, trying next...", w, h, f);
                    continue;
                }
            }
            
            if (!pipeline_started) {
                RCLCPP_ERROR(this->get_logger(), "All resolution configurations failed");
                return false;
            }
            
            // Get camera intrinsics
            auto profile = pipeline_->get_active_profile();
            auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            intrinsics_ = color_stream.get_intrinsics();
            
            // Create align object
            align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
            
            // Add delay for back camera
            if (camera_name_ == "back") {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            RCLCPP_INFO(this->get_logger(), "RealSense camera initialized successfully");
            return true;
            
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            return false;
        }
    }
    
    void captureLoop()
    {
        while (running_ && rclcpp::ok()) {
            try {
                if (!pipeline_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                
                // Wait for frames
                rs2::frameset frames = pipeline_->wait_for_frames();
                
                // Align depth to color
                auto aligned_frames = align_to_color_->process(frames);
                
                rs2::video_frame color_frame = aligned_frames.get_color_frame();
                rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
                
                if (!color_frame || !depth_frame) {
                    continue;
                }
                
                // Convert to OpenCV
                cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()),
                                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                                    CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
                
                // Apply custom colormap to depth
                cv::Mat depth_colormap = applyDepthColormap(depth_image);
                
                // Publish images
                publishImage(color_image, image_pub_, "bgr8");
                publishImage(depth_colormap, depth_pub_, "bgr8");
                publishCameraInfo();
                
            } catch (const rs2::error& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "RealSense error in capture loop: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Exception in capture loop: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    cv::Mat applyDepthColormap(const cv::Mat& depth_image)
    {
        // Custom colormap: <0.3m red, 0.3-1m yellow, >1m blue
        const int NEAR_THRESHOLD = 300;  // 0.3m in mm
        const int FAR_THRESHOLD = 1000;   // 1m in mm
        const int MAX_DISPLAY_DEPTH = 5000; // 5m
        
        cv::Mat colormap = cv::Mat::zeros(depth_image.size(), CV_8UC3);
        
        for (int y = 0; y < depth_image.rows; ++y) {
            for (int x = 0; x < depth_image.cols; ++x) {
                uint16_t depth_mm = depth_image.at<uint16_t>(y, x);
                
                if (depth_mm == 0) {
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Black
                } else if (depth_mm < NEAR_THRESHOLD) {
                    // Red: closer = more red
                    uint8_t intensity = static_cast<uint8_t>(
                        std::max(50, 255 - (depth_mm * 255 / NEAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, intensity); // BGR
                } else if (depth_mm < FAR_THRESHOLD) {
                    // Yellow: scale intensity
                    uint8_t intensity = static_cast<uint8_t>(
                        ((depth_mm - NEAR_THRESHOLD) * 255 / (FAR_THRESHOLD - NEAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, intensity, intensity); // BGR
                } else if (depth_mm < MAX_DISPLAY_DEPTH) {
                    // Blue: farther = more blue
                    uint8_t intensity = static_cast<uint8_t>(
                        ((depth_mm - FAR_THRESHOLD) * 255 / (MAX_DISPLAY_DEPTH - FAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(intensity, 0, 0); // BGR
                }
            }
        }
        
        return colormap;
    }
    
    void publishImage(const cv::Mat& image, 
                     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
                     const std::string& encoding)
    {
        try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_" + camera_name_ + "_link";
            pub->publish(*msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void publishCameraInfo()
    {
        sensor_msgs::msg::CameraInfo info_msg;
        info_msg.header.stamp = this->now();
        info_msg.header.frame_id = "camera_" + camera_name_ + "_link";
        info_msg.width = intrinsics_.width;
        info_msg.height = intrinsics_.height;
        info_msg.distortion_model = "plumb_bob";
        
        // Camera matrix
        info_msg.k[0] = intrinsics_.fx;
        info_msg.k[1] = 0.0;
        info_msg.k[2] = intrinsics_.ppx;
        info_msg.k[3] = 0.0;
        info_msg.k[4] = intrinsics_.fy;
        info_msg.k[5] = intrinsics_.ppy;
        info_msg.k[6] = 0.0;
        info_msg.k[7] = 0.0;
        info_msg.k[8] = 1.0;
        
        // Distortion coefficients
        for (size_t i = 0; i < 5 && i < sizeof(intrinsics_.coeffs)/sizeof(intrinsics_.coeffs[0]); ++i) {
            info_msg.d.push_back(intrinsics_.coeffs[i]);
        }
        
        camera_info_pub_->publish(info_msg);
    }
    
    std::string camera_name_;
    std::string camera_serial_;
    int width_, height_, fps_;
    
    std::shared_ptr<rs2::context> ctx_;
    std::shared_ptr<rs2::pipeline> pipeline_;
    std::shared_ptr<rs2::align> align_to_color_;
    rs2_intrinsics intrinsics_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    std::thread capture_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




