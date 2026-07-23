#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
/*==============================================================================
 * GIMBAL COMMUNICATION PROTOCOL v2.0
 * ============================================================================
 * Frame Format: [HEAD(2)] [MODE(1)] [PAYLOAD(~39)] [TAIL(2)]
 * 
 * HEAD: 0x5A 0xA5 (同步标志)
 * TAIL: 0x7F 0xFE (帧结束标志)
 * 
 * Serial Config:
 *   - BaudRate: 115200 bps
 *   - Total Timeout: 200ms (确保完整帧接收)
 *   - Inter-byte Timeout: 20ms (字节间隔超过此值认为帧结束)
 *   - Expected inter-frame delay: ~8.7ms @ 115200 (43字节)
 * 
 * Error Handling:
 *   - Frame sync: 滑动窗口寻找 0x5A 0xA5
 *   - Tail check: 验证 0x7F 0xFE
 *   - Data sanity: 检查角度/速度是否在合理范围（±π, ±10rad/s）
 *   - Reconnect threshold: 连续错误 >100 次
 *   - Buffer flush: 错误 % 20 == 0 时清空积压数据
 * 
 * Data Corruption Mitigation:
 *   - 每个成功帧减少错误计数 (error_count--)
 *   - 超时参数充足余量（200ms >> 3.7ms）
 *   - 长时间无数据（>2000次无读）自动重连
 * ==============================================================================
 */

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {0x5A, 0xA5};  //  uint8_t head[2] = {'S', 'P'};原来
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint8_t tail[2]= {0x7F, 0xFE};
};//43字节【云台到视觉】，yaw、pitch和速度加速度都是弧度制

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {0x5A, 0xA5}; // uint8_t head[2] = {'S', 'P'};原来
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;//原本绝对，现在相对增值
  float yaw_vel = 0.0f;
  float yaw_acc = 0.0f;
  float pitch;//原本绝对，现在相对增值
  float pitch_vel = 0.0f;
  float pitch_acc = 0.0f;
  uint8_t tail[2]= {0x7F, 0xFE};
};//视觉发给云台的是29个字节，yaw、pitch和速度加速度都是弧度制

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode getmode() const;
  GimbalState getstate() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;
  std::thread rx_thread_;
  std::thread tx_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex serial_mutex_;
  GimbalToVision rx_data_;//云台到视觉，视觉是receive
  tools::ThreadSafeQueue<VisionToGimbal, true> tx_queue_{128};
  GimbalMode mode_ = GimbalMode::IDLE;//默认空闲
  GimbalState state_;//云台状态
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>,true> queue_{5000};
  void rx_thread();//读取线程
  void tx_thread();//发送线程
};

}  // namespace io

#endif  // IO__GIMBAL_HPP