#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

struct SensorArrayStatus {
  bool leftLineDetected;
  bool centerLineDetected;
  bool rightLineDetected;
};

struct RawSensorArrayStatus {
  uint16_t left;
  uint16_t center;
  uint16_t right;
};

class SerialLineSensor {
 public:
  SerialLineSensor(const std::string& serialDeviceName, const std::chrono::milliseconds timeout);
  ~SerialLineSensor();

  [[nodiscard]] std::optional<int16_t> GetRawLeft() const;
  [[nodiscard]] std::optional<int16_t> GetRawCenter() const;
  [[nodiscard]] std::optional<int16_t> GetRawRight() const;

  [[nodiscard]] std::optional<SensorArrayStatus> GetArrayStatus() const;
  [[nodiscard]] std::optional<RawSensorArrayStatus> GetRawArrayStatus() const;

 private:
  std::string m_serialDeviceName;
  int m_serialPort;
  uint16_t m_calibrationThreshold;
  std::optional<uint16_t> m_currentLeft{std::nullopt};
  std::optional<uint16_t> m_currentCenter{std::nullopt};
  std::optional<uint16_t> m_currentRight{std::nullopt};
  std::chrono::time_point<std::chrono::steady_clock> m_lastUpdateTime;
  std::chrono::milliseconds m_timeout{std::chrono::milliseconds{100}};
  std::thread m_receiveThread;
  std::atomic<bool> m_runThread{false};
  bool m_connected{false};
  mutable std::mutex m_dataMutex;

  void ReceiverThread();
  [[nodiscard]] static std::optional<RawSensorArrayStatus> ParseMessage(std::string_view message);
};