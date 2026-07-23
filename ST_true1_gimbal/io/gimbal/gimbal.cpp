#include "gimbal.hpp"
#include "tools/yaml.hpp"

namespace io{
    Gimbal::Gimbal(const std::string & config_path){
        auto config = tools::load(config_path);
        auto port = tools::read<std::string>(config, "com_port");
        serial_.setPort(port);
        serial_.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
        serial_.setTimeout(timeout);
        serial_.open();
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (serial_.isOpen()) {
                serial_.flushInput();
                serial_.flushOutput();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        rx_thread_ = std::thread(&Gimbal::rx_thread, this);
        tx_thread_ = std::thread(&Gimbal::tx_thread, this);
        queue_.pop();
        tools::logger()->info("[Gimbal] First q received.");
    }
    Gimbal::~Gimbal(){
        quit_ = true;
        if (rx_thread_.joinable()) rx_thread_.join();
        if (tx_thread_.joinable()) tx_thread_.join();
        serial_.close();
    }
    GimbalMode Gimbal::getmode() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }
    GimbalState Gimbal::getstate() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }
    std::string Gimbal::str(GimbalMode mode) const{
        switch (mode) {
            case GimbalMode::IDLE:
            return "IDLE";
            case GimbalMode::AUTO_AIM:
            return "AUTO_AIM";
            case GimbalMode::SMALL_BUFF:
            return "SMALL_BUFF";
            case GimbalMode::BIG_BUFF:
            return "BIG_BUFF";
            default:
            return "INVALID";
        }
    }
    void Gimbal::send(io::VisionToGimbal VisionToGimbal){
        auto gimbal_state = getstate();
        VisionToGimbal.yaw = VisionToGimbal.yaw-gimbal_state.yaw;
        VisionToGimbal.pitch = VisionToGimbal.pitch-gimbal_state.pitch;
        if(VisionToGimbal.mode == 0){
            VisionToGimbal.yaw = VisionToGimbal.pitch = 0.0f;
        }
        tx_queue_.push(VisionToGimbal);
    }
    void Gimbal::tx_thread(){
        while(true){
            if(quit_) break;
            VisionToGimbal pkg = tx_queue_.pop();
            while(!tx_queue_.empty()){
                tx_queue_.pop(pkg);
            }
            auto tx_start = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> serial_lock(serial_mutex_);
            try {
                serial_.write(reinterpret_cast<uint8_t *>(&pkg), sizeof(pkg));
                auto tx_end = std::chrono::steady_clock::now();
                double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();
                if (tx_latency > 5.0) {
                    tools::logger()->warn("[Gimbal] High TX latency: {:.2f}ms", tx_latency);
                }
            } 
            catch (const std::exception & e) {
            tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
            }
        }
    }
    void Gimbal::rx_thread(){
        const int package_size = sizeof(GimbalToVision);
        uint8_t buffer[package_size];
        uint8_t temp[512] = {0};
        size_t temp_len = 0;
        auto temp_room_lambda = [&temp_len](){ return sizeof(temp) - temp_len; };
        auto serial_avail_lambda = [this](){ return serial_.available(); };
        size_t head_index = static_cast<size_t>(-1);
        while(!quit_){
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if(temp_room_lambda() > 0 && serial_avail_lambda() > 0){
                    size_t bytes_to_read = std::min(temp_room_lambda(), serial_avail_lambda());
                    size_t bytes_read = serial_.read(temp + temp_len, bytes_to_read);
                    temp_len += bytes_read;
                }
            }
            if(head_index == static_cast<size_t>(-1) && temp_len >= 2){
                for(size_t i = 0; i < temp_len - 1; ++i){
                    if(temp[i] == 0x5A && temp[i + 1] == 0xA5){
                        head_index = i;
                        break;
                    }
                }
                if(head_index == static_cast<size_t>(-1)){
                    temp[0] = temp[temp_len - 1];
                    temp_len = 1;
                    continue;
                }
                else{
                    std::memmove(temp, temp + head_index, temp_len - head_index);
                    temp_len -= head_index;
                    head_index = static_cast<size_t>(0);
                }
            }
            if(head_index != static_cast<size_t>(-1) && temp_len >= package_size){
                std::memcpy(buffer, temp, package_size);
                std::memmove(temp, temp + package_size, temp_len - package_size);
                temp_len -= package_size;
                head_index = static_cast<size_t>(-1);
                continue;
            }
            if(buffer[package_size - 2] == 0x7F && buffer[package_size - 1] == 0xFE){
                GimbalToVision * pkg = reinterpret_cast<GimbalToVision *>(buffer);
                Eigen::Quaterniond q(pkg->q[0], pkg->q[1], pkg->q[2], pkg->q[3]);
                auto now = std::chrono::steady_clock::now();
                queue_.push(std::make_tuple(q, now));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    mode_ = static_cast<GimbalMode>(pkg->mode);
                    state_.yaw = pkg->yaw;
                    state_.yaw_vel = pkg->yaw_vel;
                    state_.pitch = pkg->pitch;
                    state_.pitch_vel = pkg->pitch_vel;
                    state_.bullet_speed = pkg->bullet_speed;
                    state_.bullet_count = pkg->bullet_count;
                }
            }
            else{
                tools::logger()->warn("[Gimbal] Invalid package tail: {:02X} {:02X}", buffer[package_size - 2], buffer[package_size - 1]);
                continue;
            }
        }
        tools::logger()->info("[Gimbal] RX thread exited.");
    }
}
