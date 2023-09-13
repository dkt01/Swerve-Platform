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

SerialLineSensor::SerialLineSensor(const std::chrono::milliseconds timeout)
    : m_serialDeviceName{""}, m_timeout{timeout} {
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
  return SensorArrayStatus{.leftLineDetected = rawValues.value().left < m_calibrationActivateThreshold,
                           .centerLineDetected = rawValues.value().center < m_calibrationActivateThreshold,
                           .rightLineDetected = rawValues.value().right < m_calibrationActivateThreshold};
}

[[nodiscard]] std::optional<ProportionalArrayStatus> SerialLineSensor::GetProportionalArrayStatus() const {
  auto rawValues = GetRawArrayStatus();
  if (!rawValues) {
    return std::nullopt;
  }
  double range = m_calibrationDeactivateThreshold - m_calibrationActivateThreshold;
  return ProportionalArrayStatus{
      .left = std::clamp((m_calibrationDeactivateThreshold - rawValues.value().left) / range, 0.0, 1.0),
      .center = std::clamp((m_calibrationDeactivateThreshold - rawValues.value().center) / range, 0.0, 1.0),
      .right = std::clamp((m_calibrationDeactivateThreshold - rawValues.value().right) / range, 0.0, 1.0)};
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

[[nodiscard]] SerialLineSensor::RecoveryDirection SerialLineSensor::GetRecoveryDirection() {
  std::scoped_lock lock(m_dataMutex);
  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_recoveryStartTime) >
      m_recoveryTime) {
    m_activeRecoveryDirection = RecoveryDirection::Timeout;
  }
  return m_activeRecoveryDirection;
}

[[nodiscard]] bool SerialLineSensor::GetRecoveryActive() {
  switch (GetRecoveryDirection()) {
    case RecoveryDirection::Left:
    case RecoveryDirection::Right:
      return true;
    default:
      return false;
  }
}

void SerialLineSensor::ReceiverThread() {
  while (m_runThread.load()) {
    // Connect
    while (m_runThread.load() && !m_connected) {
      std::string portFName = m_serialDeviceName;
      if (portFName.empty()) {
        auto discoveredPort = DiscoverSerialDevice();
        if (discoveredPort) {
          portFName = discoveredPort.value().string();
        }
      }

      std::cout << "Port name: " << portFName << '\n';

      if (portFName.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      m_serialPort = open(portFName.c_str(), O_RDWR | O_NOCTTY);
      if (m_serialPort >= 0) {
        struct termios tty;

        if (tcgetattr(m_serialPort, &tty) != 0) {
          std::cerr << "Could not get attributes\n";
          close(m_serialPort);
          continue;
        }

        // Set baudrate
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Disable hardware based flow control
        tty.c_cflag &= ~CRTSCTS;

        // Enable receiver
        tty.c_cflag |= CREAD | CLOCAL;

        // Disable software based flow control
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Termois Non Canonical Mode
        tty.c_lflag |= ICANON;

        // Timeout in deciseconds for read
        tty.c_cc[VTIME] = 1;

        // Save tty
        if (tcsetattr(m_serialPort, TCSANOW, &tty) < 0) {
          close(m_serialPort);
          std::cerr << "Failed to configure port!\n";
          continue;
        }

        // Flush RX Buffer
        if (tcflush(m_serialPort, TCIFLUSH) < 0) {
          close(m_serialPort);
          std::cerr << "Failed to flush buffer!\n";
          continue;
        }

        m_connected = true;

      } else {
        std::cerr << "Could not connect\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    // Receive data
    while (m_runThread.load() && m_connected) {
      char buf[256];

      int nBytes = read(m_serialPort, &buf, sizeof(buf));

      if (nBytes < 0) {
        std::cerr << "Bad data received\n";
        close(m_serialPort);
        m_connected = false;
      } else if (nBytes > 0) {
        auto rawStates = ParseMessage(std::string_view(buf, nBytes));
        if (rawStates) {
          std::scoped_lock lock(m_dataMutex);
          if (m_currentLeft <= m_calibrationDeactivateThreshold && m_currentRight > m_calibrationDeactivateThreshold) {
            m_activeRecoveryDirection = RecoveryDirection::Left;
          } else if (m_currentRight <= m_calibrationDeactivateThreshold &&
                     m_currentLeft > m_calibrationDeactivateThreshold) {
            m_activeRecoveryDirection = RecoveryDirection::Right;
          }
          /// @todo fix left/right in arduino code or something...
          m_currentLeft = rawStates.value().right;
          m_currentCenter = rawStates.value().center;
          m_currentRight = rawStates.value().left;
          if (m_currentLeft < m_calibrationDeactivateThreshold || m_currentCenter < m_calibrationDeactivateThreshold ||
              m_currentRight < m_calibrationDeactivateThreshold) {
            m_activeRecoveryDirection = RecoveryDirection::LineDetected;
            m_recoveryStartTime = std::chrono::steady_clock::now();
          }
          m_lastUpdateTime = std::chrono::steady_clock::now();
          std::cout << m_currentLeft.value() << ' ' << m_currentCenter.value() << ' ' << m_currentRight.value() << '\n';
        }
      } else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       m_lastUpdateTime) > m_timeout) {
        std::cerr << "Lost connection\n";
        close(m_serialPort);
        m_connected = false;
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
    std::cerr << "Not enough data (" << message.size() << ")\n";
    return std::nullopt;
  }
  if (message.substr(0, 3) != "l: " || message.substr(7, 5) != ", c: " || message.substr(16, 5) != ", r: ") {
    std::cerr << "Didn't find stuff\n";
    return std::nullopt;
  }
  try {
    return RawSensorArrayStatus{
        .left = static_cast<uint16_t>(std::stoul(std::string(message.substr(3, 4)), nullptr, 10)),
        .center = static_cast<uint16_t>(std::stoul(std::string(message.substr(12, 4)), nullptr, 10)),
        .right = static_cast<uint16_t>(std::stoul(std::string(message.substr(21, 4)), nullptr, 10))};
  } catch (std::invalid_argument&) {
    std::cerr << "Invalid argument\n";
    return std::nullopt;
  } catch (std::out_of_range&) {
    std::cerr << "Out of range\n";
    return std::nullopt;
  }
}

[[nodiscard]] std::optional<std::filesystem::path> SerialLineSensor::DiscoverSerialDevice() {
  try {
    for (const auto& candidatePort : std::filesystem::directory_iterator("/dev/serial/by-id")) {
      auto candidatePortFname = candidatePort.path().filename().string();
      std::transform(
          candidatePortFname.begin(), candidatePortFname.end(), candidatePortFname.begin(), [](unsigned char c) {
            return std::tolower(c);
          });
      if (candidatePortFname.find("arduino") != std::string::npos) {
        return std::filesystem::canonical(candidatePort.path().parent_path() /
                                          std::filesystem::read_symlink(candidatePort.path()));
      }
    }
    return std::nullopt;
  } catch (std::filesystem::filesystem_error const&) {
    return std::nullopt;
  }
}
