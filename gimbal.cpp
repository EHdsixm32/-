#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");


  //auto baud_rate = tools::read<int>(yaml, "baud_rate", 115200);


  try {
    serial_.setPort(com_port);


    serial_.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // 50ms 超时
    serial_.setTimeout(timeout);

    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
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

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}




void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                  float pitch, float pitch_vel, float pitch_acc)
{
  VisionToGimbal tx_data;
  tx_data.head[0] = 0x5A;
  tx_data.head[1] = 0xA5;
  tx_data.mode = control ? (fire ? 2 : 1) : 0;
  tx_data.yaw = yaw;
  tx_data.yaw_vel = yaw_vel;
  tx_data.yaw_acc = yaw_acc;
  tx_data.pitch = pitch;
  tx_data.pitch_vel = pitch_vel;
  tx_data.pitch_acc = pitch_acc;
  tx_data.tail[0] = 0x7F;
  tx_data.tail[1] = 0xFE;
  // 不再计算 CRC
  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data), sizeof(tx_data));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
    tools::logger()->info("[Gimbal] read_thread started.");
    int error_count = 0;

    while (!quit_) {
        if (error_count > 5000) {
            error_count = 0;
            tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
            reconnect();
            continue;
        }

        // 1. 读取帧头 2 字节
        uint8_t head[2];
        if (!read(head, 2)) {
            error_count++;
            continue;
        }

        // 2. 检查帧头
        if (head[0] != 0x5A || head[1] != 0xA5) {
            // 未匹配，跳过，继续读下一个字节（但前面已消耗2字节，这里简单丢弃）
            continue;
        }

        // 3. 匹配到帧头，读取剩余数据（整个结构体减去头部2字节）
        GimbalToVision rx_data;
        rx_data.head[0] = head[0];
        rx_data.head[1] = head[1];
        size_t remaining = sizeof(GimbalToVision) - 2;
        if (!read(reinterpret_cast<uint8_t*>(&rx_data) + 2, remaining)) {
            error_count++;
            continue;
        }

        // 4. 可选：校验尾部（暂时注释）
        // if (rx_data.tail[0] != 0x7F || rx_data.tail[1] != 0xFE) {
        //     tools::logger()->debug("[Gimbal] Tail check failed.");
        //     continue;
        // }

        // 5. 数据有效，更新状态
        auto t = std::chrono::steady_clock::now();
        error_count = 0;
        Eigen::Quaterniond q(rx_data.q[0], rx_data.q[1], rx_data.q[2], rx_data.q[3]);
        queue_.push({q, t});

        std::lock_guard<std::mutex> lock(mutex_);
        state_.yaw = rx_data.yaw;
        state_.yaw_vel = rx_data.yaw_vel;
        state_.pitch = rx_data.pitch;
        state_.pitch_vel = rx_data.pitch_vel;
        state_.bullet_speed = rx_data.bullet_speed;
        state_.bullet_count = rx_data.bullet_count;

        switch (rx_data.mode) {
            case 0: mode_ = GimbalMode::IDLE; break;
            case 1: mode_ = GimbalMode::AUTO_AIM; break;
            case 2: mode_ = GimbalMode::SMALL_BUFF; break;
            case 3: mode_ = GimbalMode::BIG_BUFF; break;
            default:
                mode_ = GimbalMode::IDLE;
                tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data.mode);
                break;
        }
    }
    tools::logger()->info("[Gimbal] read_thread stopped.");
}


void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io