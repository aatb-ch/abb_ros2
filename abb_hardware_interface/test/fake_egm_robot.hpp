// Copyright 2026 aatb-ch
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// A minimal fake ABB robot controller for tests: it plays the robot side of an
// EGM session over UDP (EgmRobot out every period, EgmSensor replies in),
// exactly like the real controller does towards abb_libegm's UDP server.
// Optionally the "arm" follows the commanded position, so stale commands and
// jumps show up as motion. Everything is thread-safe.

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <abb_libegm/egm.pb.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace abb_test
{
using Joints = std::array<double, 6>;  // degrees

struct EgmReply
{
  uint32_t robot_seqno;  // seqno of the last robot message sent before this reply arrived
  Joints position;       // planned joints [deg]
  Joints speed;          // speedRef joints [deg/s]
  std::chrono::steady_clock::time_point stamp;
};

class FakeEgmRobot
{
public:
  FakeEgmRobot(uint16_t egm_port, const Joints& start)
    : egm_port_(egm_port), position_(start), reference_(start)
  {
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    local.sin_port = 0;
    ::bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local));
  }

  ~FakeEgmRobot()
  {
    stop();
    ::close(sock_);
  }

  void start()
  {
    if (thread_.joinable())
    {
      return;
    }
    run_ = true;
    thread_ = std::thread([this] { loop(); });
  }

  void stop()
  {
    run_ = false;
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  // Robot-side state, as reported in every message.
  void setStates(bool motors_on, bool egm_running, bool rapid_running)
  {
    std::lock_guard<std::mutex> l(m_);
    motors_on_ = motors_on;
    egm_running_ = egm_running;
    rapid_running_ = rapid_running;
  }
  // false = the robot stops sending (an inbound gap / lost link).
  void setSending(bool sending)
  {
    sending_ = sending;
  }
  // Every datagram is sent twice (a duplicating link).
  void setDuplicate(bool dup)
  {
    duplicate_ = dup;
  }
  // Next cycle, also send one old datagram (seqno - back): a reordering link.
  void sendStaleOnce(uint32_t back)
  {
    stale_back_ = back;
  }
  // The next message starts a new EGM session (seqno 0).
  void newSession()
  {
    std::lock_guard<std::mutex> l(m_);
    seqno_ = 0;
  }
  // true = the arm moves to each commanded position (at most max_speed_dps).
  void setFollow(bool follow, double max_speed_dps = 1e9)
  {
    std::lock_guard<std::mutex> l(m_);
    follow_ = follow;
    max_speed_dps_ = max_speed_dps;
  }
  // Move the arm by other means (RAPID, a jog): EGM only reports it.
  void setPosition(const Joints& p)
  {
    std::lock_guard<std::mutex> l(m_);
    position_ = p;
    reference_ = p;
  }
  // Measured = reference + sag (servo error under gravity) + noise.
  void setSag(const Joints& sag)
  {
    std::lock_guard<std::mutex> l(m_);
    sag_ = sag;
  }
  void setNoise(double amplitude_deg)
  {
    std::lock_guard<std::mutex> l(m_);
    noise_ = amplitude_deg;
  }
  Joints reference() const
  {
    std::lock_guard<std::mutex> l(m_);
    return reference_;
  }
  Joints position() const
  {
    std::lock_guard<std::mutex> l(m_);
    return position_;
  }
  void setUtilization(double u)
  {
    std::lock_guard<std::mutex> l(m_);
    utilization_ = u;
  }
  uint32_t sent() const
  {
    std::lock_guard<std::mutex> l(m_);
    return sent_;
  }
  std::vector<EgmReply> replies() const
  {
    std::lock_guard<std::mutex> l(m_);
    return replies_;
  }
  void clearReplies()
  {
    std::lock_guard<std::mutex> l(m_);
    replies_.clear();
  }
  // Waits until at least n replies were received in total since the last clear.
  bool waitReplies(size_t n, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> l(m_);
    return cv_.wait_for(l, timeout, [&] { return replies_.size() >= n; });
  }

  std::chrono::milliseconds period{ 4 };

private:
  void loop()
  {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port = htons(egm_port_);
    auto next = std::chrono::steady_clock::now();
    const auto t0 = next;
    while (run_)
    {
      if (sending_)
      {
        std::string out;
        std::string stale;
        {
          std::lock_guard<std::mutex> l(m_);
          buildMessage(std::chrono::steady_clock::now() - t0).SerializeToString(&out);
          const uint32_t back = stale_back_.exchange(0);
          if (back > 0 && back <= seqno_)
          {
            const uint32_t keep = seqno_;
            seqno_ -= back;
            buildMessage(std::chrono::steady_clock::now() - t0).SerializeToString(&stale);
            seqno_ = keep;
          }
          last_seqno_ = seqno_++;
          ++sent_;
        }
        ::sendto(sock_, out.data(), out.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        if (duplicate_)
        {
          ::sendto(sock_, out.data(), out.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        }
        if (!stale.empty())
        {
          ::sendto(sock_, stale.data(), stale.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        }
      }
      next += period;
      // Collect replies until the next send is due.
      while (run_)
      {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next)
        {
          break;
        }
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(next - now).count();
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock_, &fds);
        timeval tv{ static_cast<time_t>(us / 1000000), static_cast<suseconds_t>(us % 1000000) };
        if (::select(sock_ + 1, &fds, nullptr, nullptr, &tv) > 0)
        {
          char buf[2048];
          const auto n = ::recv(sock_, buf, sizeof(buf), 0);
          if (n > 0)
          {
            onReply(buf, static_cast<size_t>(n));
          }
        }
      }
    }
  }

  abb::egm::EgmRobot buildMessage(std::chrono::steady_clock::duration since_start)
  {
    using namespace abb::egm;
    EgmRobot msg;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(since_start).count();
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(since_start).count();
    msg.mutable_header()->set_seqno(seqno_);
    msg.mutable_header()->set_tm(static_cast<uint32_t>(ms));
    msg.mutable_header()->set_mtype(EgmHeader_MessageType_MSGTYPE_DATA);
    auto fill = [&](auto* joints, bool measured) {
      for (int i = 0; i < 6; ++i)
      {
        double v = measured ? position_[i] + sag_[i] : reference_[i];
        if (measured && noise_ > 0.0)
        {
          rng_ = rng_ * 6364136223846793005ULL + 1442695040888963407ULL;
          v += noise_ * (static_cast<double>(rng_ >> 11) / 9007199254740992.0 * 2.0 - 1.0);
        }
        joints->add_joints(v);
      }
    };
    auto pose = [](EgmPose* p) {
      p->mutable_pos()->set_x(1000.0);
      p->mutable_pos()->set_y(0.0);
      p->mutable_pos()->set_z(1000.0);
      p->mutable_orient()->set_u0(1.0);
      p->mutable_orient()->set_u1(0.0);
      p->mutable_orient()->set_u2(0.0);
      p->mutable_orient()->set_u3(0.0);
      p->mutable_euler()->set_x(0.0);
      p->mutable_euler()->set_y(0.0);
      p->mutable_euler()->set_z(0.0);
    };
    auto clock = [&](EgmClock* c) {
      c->set_sec(static_cast<uint64_t>(us / 1000000) + 1000);
      c->set_usec(static_cast<uint64_t>(us % 1000000));
    };
    fill(msg.mutable_feedback()->mutable_joints(), true);
    pose(msg.mutable_feedback()->mutable_cartesian());
    clock(msg.mutable_feedback()->mutable_time());
    fill(msg.mutable_planned()->mutable_joints(), false);
    pose(msg.mutable_planned()->mutable_cartesian());
    clock(msg.mutable_planned()->mutable_time());
    msg.mutable_motorstate()->set_state(motors_on_ ? EgmMotorState_MotorStateType_MOTORS_ON :
                                                     EgmMotorState_MotorStateType_MOTORS_OFF);
    msg.mutable_mcistate()->set_state(egm_running_ ? EgmMCIState_MCIStateType_MCI_RUNNING :
                                                     EgmMCIState_MCIStateType_MCI_STOPPED);
    msg.set_mciconvergencemet(true);
    msg.mutable_rapidexecstate()->set_state(rapid_running_ ? EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_RUNNING :
                                                             EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_STOPPED);
    msg.set_utilizationrate(utilization_);
    return msg;
  }

  void onReply(const char* data, size_t n)
  {
    abb::egm::EgmSensor s;
    if (!s.ParseFromArray(data, static_cast<int>(n)))
    {
      return;
    }
    EgmReply r{};
    r.stamp = std::chrono::steady_clock::now();
    for (int i = 0; i < 6 && i < s.planned().joints().joints_size(); ++i)
    {
      r.position[i] = s.planned().joints().joints(i);
    }
    for (int i = 0; i < 6 && i < s.speedref().joints().joints_size(); ++i)
    {
      r.speed[i] = s.speedref().joints().joints(i);
    }
    std::lock_guard<std::mutex> l(m_);
    r.robot_seqno = last_seqno_;
    const bool moving = s.planned().joints().joints_size() >= 6 && motors_on_ && egm_running_ && rapid_running_;
    if (moving)
    {
      reference_ = r.position;  // the robot's reference follows the EGM command
    }
    if (follow_ && moving)
    {
      const double max_step = max_speed_dps_ * std::chrono::duration<double>(period).count();
      for (int i = 0; i < 6; ++i)
      {
        const double d = r.position[i] - position_[i];
        position_[i] += std::max(-max_step, std::min(max_step, d));
      }
    }
    replies_.push_back(r);
    cv_.notify_all();
  }

  uint16_t egm_port_;
  int sock_{ -1 };
  std::thread thread_;
  std::atomic<bool> run_{ false };
  std::atomic<bool> sending_{ true };
  std::atomic<bool> duplicate_{ false };
  std::atomic<uint32_t> stale_back_{ 0 };
  mutable std::mutex m_;
  std::condition_variable cv_;
  Joints position_;
  Joints reference_;
  Joints sag_{};
  double noise_{ 0.0 };
  uint64_t rng_{ 42 };
  bool motors_on_{ true };
  bool egm_running_{ true };
  bool rapid_running_{ true };
  bool follow_{ false };
  double max_speed_dps_{ 1e9 };
  double utilization_{ 50.0 };
  uint32_t seqno_{ 0 };
  uint32_t last_seqno_{ 0 };
  uint32_t sent_{ 0 };
  std::vector<EgmReply> replies_;
};

}  // namespace abb_test
