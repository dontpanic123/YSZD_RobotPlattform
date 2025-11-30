#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <atomic>

class CameraThread
{
public:
    CameraThread(const std::string& name, const std::string& serial, 
                 int width, int height, int fps, rclcpp::Node* node)
        : camera_name_(name), camera_serial_(serial), 
          width_(width), height_(height), fps_(fps), node_(node), running_(false)
    {
        // Create topic prefix
        std::string topic_prefix = "/camera_" + camera_name_;
        
        // Create publishers
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix + "/image_raw", 10);
        camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix + "/camera_info", 10);
        depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix + "/depth/image_raw", 10);
    }
    
    ~CameraThread()
    {
        stop();
    }
    
    bool init(std::shared_ptr<rs2::context> ctx)
    {
        try {
            RCLCPP_INFO(node_->get_logger(), "Initializing camera: %s (serial: %s)", 
                       camera_name_.c_str(), camera_serial_.c_str());
            
            // Create pipeline
            pipeline_ = std::make_shared<rs2::pipeline>(*ctx);
            
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
                    // Create fresh config for each attempt
                    rs2::config cfg;
                    cfg.enable_device(camera_serial_);
                    
                    RCLCPP_INFO(node_->get_logger(), "[%s] Trying: %dx%d@%dfps", 
                               camera_name_.c_str(), w, h, f);
                    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, f);
                    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, f);
                    
                    auto profile = pipeline_->start(cfg);
                    width_ = w;
                    height_ = h;
                    fps_ = f;
                    pipeline_started = true;
                    RCLCPP_INFO(node_->get_logger(), "[%s] Successfully configured: %dx%d@%dfps", 
                               camera_name_.c_str(), w, h, f);
                    break;
                } catch (const rs2::error& e) {
                    RCLCPP_DEBUG(node_->get_logger(), "[%s] Configuration %dx%d@%dfps failed: %s", 
                                camera_name_.c_str(), w, h, f, e.what());
                    // Stop pipeline if it was started but failed
                    try {
                        if (pipeline_) {
                            pipeline_->stop();
                        }
                    } catch (...) {
                        // Ignore stop errors
                    }
                    continue;
                }
            }
            
            if (!pipeline_started) {
                RCLCPP_ERROR(node_->get_logger(), "[%s] All configurations failed", camera_name_.c_str());
                return false;
            }
            
            // Get intrinsics
            auto profile = pipeline_->get_active_profile();
            auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            intrinsics_ = color_stream.get_intrinsics();
            
            // Validate intrinsics - only log if invalid
            if (intrinsics_.width == 0 || intrinsics_.height == 0 || 
                intrinsics_.fx <= 0 || intrinsics_.fy <= 0 ||
                std::isnan(intrinsics_.fx) || std::isnan(intrinsics_.fy) ||
                std::isnan(intrinsics_.ppx) || std::isnan(intrinsics_.ppy)) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Intrinsics not available from profile, will get from first frame",
                           camera_name_.c_str());
            } else {
                RCLCPP_INFO(node_->get_logger(), "[%s] Intrinsics initialized: fx=%.2f, fy=%.2f, size=%dx%d",
                           camera_name_.c_str(), intrinsics_.fx, intrinsics_.fy,
                           intrinsics_.width, intrinsics_.height);
            }
            
            // Create align
            align_to_color_ = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
            
            // Wait for streams to stabilize and get first frames
            // This ensures the pipeline is ready before we start polling
            RCLCPP_INFO(node_->get_logger(), "[%s] Waiting for first frames...", camera_name_.c_str());
            for (int i = 0; i < 30; ++i) {  // Wait up to 3 seconds
                rs2::frameset frames;
                if (pipeline_->poll_for_frames(&frames)) {
                    RCLCPP_INFO(node_->get_logger(), "[%s] First frames received", camera_name_.c_str());
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            return true;
            
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] RealSense error: %s", camera_name_.c_str(), e.what());
            return false;
        }
    }
    
    void start()
    {
        if (running_) return;
        running_ = true;
        thread_ = std::thread(&CameraThread::captureLoop, this);
    }
    
    void stop()
    {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        if (pipeline_) {
            pipeline_->stop();
        }
    }
    
private:
    void captureLoop()
    {
        int consecutive_failures = 0;
        const int MAX_CONSECUTIVE_FAILURES = 10;
        int no_frame_count = 0;
        bool first_frame_received = false;
        
        while (running_ && rclcpp::ok()) {
            try {
                if (!pipeline_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                
                // For the first few frames, use wait_for_frames to ensure we get data
                // After that, use poll_for_frames for non-blocking operation
                rs2::frameset frames;
                bool got_frames = false;
                
                if (!first_frame_received) {
                    // Wait for first frames with timeout
                    try {
                        frames = pipeline_->wait_for_frames(1000);  // 1 second timeout
                        got_frames = true;
                        first_frame_received = true;
                        RCLCPP_INFO(node_->get_logger(), "[%s] First frames received in capture loop", camera_name_.c_str());
                    } catch (const rs2::error& e) {
                        // Timeout or error, try polling instead
                        got_frames = pipeline_->poll_for_frames(&frames);
                    }
                } else {
                    // Use non-blocking poll_for_frames
                    got_frames = pipeline_->poll_for_frames(&frames);
                }
                
                if (!got_frames) {
                    // No frames available, sleep briefly and continue
                    no_frame_count++;
                    if (no_frame_count > 100) {  // After 10 seconds of no frames
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                           "[%s] No frames received for extended period", 
                                           camera_name_.c_str());
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                
                no_frame_count = 0;
                
                // Align depth to color
                auto aligned_frames = align_to_color_->process(frames);
                
                rs2::video_frame color_frame = aligned_frames.get_color_frame();
                rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
                
                // Validate frames
                if (!color_frame || !depth_frame) {
                    consecutive_failures++;
                    if (consecutive_failures > MAX_CONSECUTIVE_FAILURES) {
                        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                           "[%s] Too many consecutive frame failures, check camera connection", 
                                           camera_name_.c_str());
                    }
                    continue;
                }
                
                // Reset failure counter on success
                consecutive_failures = 0;
                
                // Validate frame dimensions
                if (color_frame.get_width() == 0 || color_frame.get_height() == 0 ||
                    depth_frame.get_width() == 0 || depth_frame.get_height() == 0) {
                    continue;
                }
                
                // Update intrinsics from frame if not valid (fallback) - only once
                static bool intrinsics_updated_from_frame = false;
                if (!intrinsics_updated_from_frame && 
                    (intrinsics_.width == 0 || intrinsics_.width != color_frame.get_width() ||
                     intrinsics_.height == 0 || intrinsics_.height != color_frame.get_height() ||
                     intrinsics_.fx <= 0 || std::isnan(intrinsics_.fx))) {
                    try {
                        auto profile = color_frame.get_profile().as<rs2::video_stream_profile>();
                        auto new_intrinsics = profile.get_intrinsics();
                        
                        // Only update if the new intrinsics are valid
                        if (new_intrinsics.fx > 0 && new_intrinsics.fy > 0 && 
                            !std::isnan(new_intrinsics.fx) && !std::isnan(new_intrinsics.fy)) {
                            intrinsics_ = new_intrinsics;
                            intrinsics_updated_from_frame = true;
                            RCLCPP_INFO(node_->get_logger(), "[%s] Updated intrinsics from frame: fx=%.2f, fy=%.2f, size=%dx%d",
                                       camera_name_.c_str(), intrinsics_.fx, intrinsics_.fy, 
                                       intrinsics_.width, intrinsics_.height);
                        }
                    } catch (const rs2::error& e) {
                        // Only log once per camera
                        static bool logged_error = false;
                        if (!logged_error) {
                            RCLCPP_WARN(node_->get_logger(), "[%s] Failed to get intrinsics from frame: %s",
                                       camera_name_.c_str(), e.what());
                            logged_error = true;
                        }
                    }
                }
                
                // Convert to OpenCV
                cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()),
                                   CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                                    CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
                
                // Validate images are not empty
                if (color_image.empty() || depth_image.empty()) {
                    continue;
                }
                
                // Apply custom colormap
                cv::Mat depth_colormap = applyDepthColormap(depth_image);
                
                // Publish both RGB and depth images
                publishImage(color_image, image_pub_, "bgr8");
                publishImage(depth_colormap, depth_pub_, "bgr8");
                publishCameraInfo();
                
            } catch (const rs2::error& e) {
                consecutive_failures++;
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "[%s] RealSense error: %s", camera_name_.c_str(), e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (const std::exception& e) {
                consecutive_failures++;
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "[%s] Exception: %s", camera_name_.c_str(), e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    cv::Mat applyDepthColormap(const cv::Mat& depth_image)
    {
        const int NEAR_THRESHOLD = 300;
        const int FAR_THRESHOLD = 1000;
        const int MAX_DISPLAY_DEPTH = 5000;
        
        cv::Mat colormap = cv::Mat::zeros(depth_image.size(), CV_8UC3);
        
        for (int y = 0; y < depth_image.rows; ++y) {
            for (int x = 0; x < depth_image.cols; ++x) {
                uint16_t depth_mm = depth_image.at<uint16_t>(y, x);
                
                if (depth_mm == 0) {
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                } else if (depth_mm < NEAR_THRESHOLD) {
                    uint8_t intensity = static_cast<uint8_t>(
                        std::max(50, 255 - (depth_mm * 255 / NEAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, intensity);
                } else if (depth_mm < FAR_THRESHOLD) {
                    uint8_t intensity = static_cast<uint8_t>(
                        ((depth_mm - NEAR_THRESHOLD) * 255 / (FAR_THRESHOLD - NEAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, intensity, intensity);
                } else if (depth_mm < MAX_DISPLAY_DEPTH) {
                    uint8_t intensity = static_cast<uint8_t>(
                        ((depth_mm - FAR_THRESHOLD) * 255 / (MAX_DISPLAY_DEPTH - FAR_THRESHOLD)));
                    colormap.at<cv::Vec3b>(y, x) = cv::Vec3b(intensity, 0, 0);
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
            msg->header.stamp = node_->now();
            msg->header.frame_id = "camera_" + camera_name_ + "_link";
            pub->publish(*msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] cv_bridge exception: %s", camera_name_.c_str(), e.what());
        }
    }
    
    void publishCameraInfo()
    {
        // Check if intrinsics are valid before publishing
        if (intrinsics_.width == 0 || intrinsics_.height == 0 || 
            intrinsics_.fx <= 0 || intrinsics_.fy <= 0 ||
            std::isnan(intrinsics_.fx) || std::isnan(intrinsics_.fy)) {
            // Only log once, then silently skip
            static bool logged_warning = false;
            if (!logged_warning) {
                RCLCPP_WARN(node_->get_logger(), "[%s] Camera intrinsics not yet initialized, skipping camera_info publish",
                           camera_name_.c_str());
                logged_warning = true;
            }
            return;
        }
        // Reset warning flag once intrinsics are valid
        static bool logged_warning = false;
        if (logged_warning) {
            logged_warning = false;
        }
        
        sensor_msgs::msg::CameraInfo info_msg;
        info_msg.header.stamp = node_->now();
        info_msg.header.frame_id = "camera_" + camera_name_ + "_link";
        info_msg.width = intrinsics_.width;
        info_msg.height = intrinsics_.height;
        info_msg.distortion_model = "plumb_bob";
        
        // Camera matrix K (3x3, row-major)
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
            double coeff = intrinsics_.coeffs[i];
            // Check for NaN or invalid values
            if (std::isnan(coeff) || std::isinf(coeff)) {
                info_msg.d.push_back(0.0);
            } else {
                info_msg.d.push_back(coeff);
            }
        }
        
        // Ensure we have at least 5 distortion coefficients
        while (info_msg.d.size() < 5) {
            info_msg.d.push_back(0.0);
        }
        
        camera_info_pub_->publish(info_msg);
    }
    
    std::string camera_name_;
    std::string camera_serial_;
    int width_, height_, fps_;
    rclcpp::Node* node_;
    
    std::shared_ptr<rs2::pipeline> pipeline_;
    std::shared_ptr<rs2::align> align_to_color_;
    rs2_intrinsics intrinsics_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    std::thread thread_;
    std::atomic<bool> running_;
};

class RealSenseDualCameraNode : public rclcpp::Node
{
public:
    RealSenseDualCameraNode() : Node("realsense_dual_camera_node")
    {
        // Don't declare parameters with type - let ROS2 accept any type
        // We'll handle type conversion when reading
        // Use declare_parameter without type to allow both string and integer
        try {
            this->declare_parameter("front_camera_serial", rclcpp::ParameterValue(""));
            this->declare_parameter("back_camera_serial", rclcpp::ParameterValue(""));
        } catch (...) {
            // If already declared, ignore
        }
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("fps", 30);
        
        // Get serial numbers - handle both string and integer types
        std::string front_serial;
        std::string back_serial;
        
        // Get front camera serial - convert to string if needed
        try {
            if (this->has_parameter("front_camera_serial")) {
                auto front_param = this->get_parameter("front_camera_serial");
                if (front_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    front_serial = front_param.as_string();
                } else if (front_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    front_serial = std::to_string(front_param.as_int());
                    RCLCPP_WARN(this->get_logger(), "front_camera_serial was passed as integer, converting to string: %s", front_serial.c_str());
                } else {
                    front_serial = "";
                    RCLCPP_ERROR(this->get_logger(), "front_camera_serial has invalid type: %d", static_cast<int>(front_param.get_type()));
                }
            } else {
                front_serial = "";
                RCLCPP_WARN(this->get_logger(), "front_camera_serial parameter not set, using empty string");
            }
        } catch (const std::exception& e) {
            front_serial = "";
            RCLCPP_ERROR(this->get_logger(), "Error getting front_camera_serial: %s", e.what());
        }
        
        // Get back camera serial - convert to string if needed
        try {
            if (this->has_parameter("back_camera_serial")) {
                auto back_param = this->get_parameter("back_camera_serial");
                if (back_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    back_serial = back_param.as_string();
                } else if (back_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                    back_serial = std::to_string(back_param.as_int());
                    RCLCPP_WARN(this->get_logger(), "back_camera_serial was passed as integer, converting to string: %s", back_serial.c_str());
                } else {
                    back_serial = "";
                    RCLCPP_ERROR(this->get_logger(), "back_camera_serial has invalid type: %d", static_cast<int>(back_param.get_type()));
                }
            } else {
                back_serial = "";
                RCLCPP_WARN(this->get_logger(), "back_camera_serial parameter not set, using empty string");
            }
        } catch (const std::exception& e) {
            back_serial = "";
            RCLCPP_ERROR(this->get_logger(), "Error getting back_camera_serial: %s", e.what());
        }
        int width = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();
        int fps = this->get_parameter("fps").as_int();
        
        // Create shared context
        ctx_ = std::make_shared<rs2::context>();
        
        // Discover devices
        auto devices = ctx_->query_devices();
        std::vector<std::string> device_serials;
        
        for (size_t i = 0; i < devices.size(); ++i) {
            try {
                auto dev = devices[i];
                std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                device_serials.push_back(serial);
                RCLCPP_INFO(this->get_logger(), "Device %zu: Serial=%s, Name=%s", i, serial.c_str(), name.c_str());
            } catch (const rs2::error& e) {
                RCLCPP_WARN(this->get_logger(), "Cannot access device %zu: %s", i, e.what());
            }
        }
        
        if (device_serials.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No accessible RealSense devices found");
            return;
        }
        
        // Determine serials
        std::string front_serial_final = front_serial.empty() ? 
            (device_serials.size() > 0 ? device_serials[0] : "") : front_serial;
        std::string back_serial_final = back_serial.empty() ? 
            (device_serials.size() > 1 ? device_serials[1] : "") : back_serial;
        
        // Create camera threads
        if (!front_serial_final.empty()) {
            front_camera_ = std::make_unique<CameraThread>("front", front_serial_final, 
                                                          width, height, fps, this);
            if (front_camera_->init(ctx_)) {
                RCLCPP_INFO(this->get_logger(), "Front camera initialized: %s", front_serial_final.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize front camera");
                front_camera_.reset();
            }
        }
        
        // Add delay for back camera
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        if (!back_serial_final.empty()) {
            back_camera_ = std::make_unique<CameraThread>("back", back_serial_final, 
                                                          width, height, fps, this);
            if (back_camera_->init(ctx_)) {
                RCLCPP_INFO(this->get_logger(), "Back camera initialized: %s", back_serial_final.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize back camera");
                back_camera_.reset();
            }
        }
        
        // Start camera threads
        if (front_camera_) {
            front_camera_->start();
        }
        if (back_camera_) {
            back_camera_->start();
        }
        
        RCLCPP_INFO(this->get_logger(), "RealSense dual camera node started");
    }
    
    ~RealSenseDualCameraNode()
    {
        if (front_camera_) {
            front_camera_->stop();
        }
        if (back_camera_) {
            back_camera_->stop();
        }
    }

private:
    std::shared_ptr<rs2::context> ctx_;
    std::unique_ptr<CameraThread> front_camera_;
    std::unique_ptr<CameraThread> back_camera_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseDualCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




