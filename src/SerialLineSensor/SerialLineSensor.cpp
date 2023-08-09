#include "SerialLineSensor.h"

#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

SerialLineSensor::SerialLineSensor(const std::string& serialDeviceName, const std::chrono::milliseconds timeout)
    : m_serialDeviceName{serialDeviceName}, m_timeout{timeout} {
  m_runThread.store(true);
  m_receiveThread = std::thread(&SerialLineSensor::ReceiverThread, this);
}

SerialLineSensor::~SerialLineSensor() {
  m_runThread.store(false);
  m_receiveThread.join();
}

[[nodiscard]] std::optional<int16_t> SerialLineSensor::GetRawLeft() const {
  auto rawValues = GetRawArrayStatus();
  if (!rawValues) {
    return std::nullopt;
  }
  return rawValues.value().left;
}

[[nodiscard]] std::optional<int16_t> SerialLineSensor::GetRawCenter() const {
  auto rawValues = GetRawArrayStatus();
  if (!rawValues) {
    return std::nullopt;
  }
  return rawValues.value().center;
}

[[nodiscard]] std::optional<int16_t> SerialLineSensor::GetRawRight() const {
  auto rawValues = GetRawArrayStatus();
  if (!rawValues) {
    return std::nullopt;
  }
  return rawValues.value().right;
}

[[nodiscard]] std::optional<SensorArrayStatus> SerialLineSensor::GetArrayStatus() const {
  auto rawValues = GetRawArrayStatus();
  if (!rawValues) {
    return std::nullopt;
  }
  return SensorArrayStatus{.leftLineDetected = rawValues.value().left < m_calibrationThreshold,
                           .centerLineDetected = rawValues.value().center < m_calibrationThreshold,
                           .rightLineDetected = rawValues.value().right < m_calibrationThreshold};
}

[[nodiscard]] std::optional<RawSensorArrayStatus> SerialLineSensor::GetRawArrayStatus() const {
  std::scoped_lock lock(m_dataMutex);
  auto timeout = (std::chrono::steady_clock::now() - m_lastUpdateTime) > m_timeout;
  if (timeout || !m_currentLeft || !m_currentCenter || !m_currentRight) {
    return std::nullopt;
  }
  return RawSensorArrayStatus{
      .left = m_currentLeft.value(), .center = m_currentCenter.value(), .right = m_currentRight.value()};
}

void SerialLineSensor::ReceiverThread() {
  while (m_runThread.load()) {
    std::cout << "Receive thread loop\n";
    // Connect
    while (m_runThread.load() && !m_connected) {
      std::cout << "Connect loop\n";
      m_serialPort = open(m_serialDeviceName.c_str(), O_RDONLY);
      if (m_serialPort >= 0) {
        struct termios tty;
        if (tcgetattr(m_serialPort, &tty) != 0) {
          std::cout << "Could not get attributes\n";
          close(m_serialPort);
          continue;
        }

        tty.c_cflag &= ~PARENB;  // No parity bit
        tty.c_cflag &= ~CSTOPB;  // One stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;                      // 8 data bits
        tty.c_cflag &= ~CRTSCTS;                 // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;           // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        tty.c_lflag &= ~ICANON;                  // Read lines
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                         ICRNL);  // Disable any special handling of received bytes
        tty.c_cc[VTIME] = 1;      // 100ms
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B1152000);

        if (tcsetattr(m_serialPort, TCSANOW, &tty) != 0) {
          std::cout << "Could not set attributes\n";
          close(m_serialPort);
        }

        m_connected = true;

      } else {
        std::cout << "Could not connect\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    // Receive data
    while (m_runThread.load() && m_connected) {
      std::cout << "Receive loop\n";
      char buf[256];

      int nBytes = read(m_serialPort, &buf, sizeof(buf));

      if (nBytes < 0) {
        std::cout << "Bad data received\n";
        close(m_serialPort);
        m_connected = false;
      } else if (nBytes > 0) {
        std::cout << "Raw: \"" << std::string_view(buf, nBytes) << "\"\n";
        std::cout << "Raw2: \"";
        for (int i = 0; i < nBytes; ++i) {
          std::cout << (int)buf[i];
        }
        std::cout << "\"\n";
        auto rawStates = ParseMessage(std::string_view(buf, nBytes));
        if (rawStates) {
          std::scoped_lock lock(m_dataMutex);
          m_currentLeft = rawStates.value().left;
          m_currentCenter = rawStates.value().center;
          m_currentRight = rawStates.value().right;
          m_lastUpdateTime = std::chrono::steady_clock::now();
        }
      }
    }
  }

  if (m_connected) {
    close(m_serialPort);
    m_connected = false;
  }
}

[[nodiscard]] std::optional<RawSensorArrayStatus> SerialLineSensor::ParseMessage(std::string_view message) {
  if (message.size() < 25) {
    std::cout << "Not enough data (" << message.size() << ")\n";
    return std::nullopt;
  }
  if (message.substr(0, 3) != "l: " || message.substr(7, 5) != ", c: " || message.substr(16, 5) != ", r: ") {
    std::cout << "Didn't find stuff\n";
    return std::nullopt;
  }
  try {
    return RawSensorArrayStatus{
        .left = static_cast<uint16_t>(std::stoul(std::string(message.substr(3, 4)), nullptr, 10)),
        .center = static_cast<uint16_t>(std::stoul(std::string(message.substr(12, 4)), nullptr, 10)),
        .right = static_cast<uint16_t>(std::stoul(std::string(message.substr(21, 4)), nullptr, 10))};
  } catch (std::invalid_argument&) {
    std::cout << "Invalid argument\n";
    return std::nullopt;
  } catch (std::out_of_range&) {
    std::cout << "Out of range\n";
    return std::nullopt;
  }
}
