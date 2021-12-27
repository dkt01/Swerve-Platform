////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <chrono>
#include <iostream>
#include <optional>
#include <SDL.h>
#include <argosLib/controller/Vibration.h>

class XBoxController {
  public:
    enum Button {
      kA = 0,
      kB = 1,
      kX = 3,
      kY = 4,
      kBumperLeft = 6,
      kBumperRight = 7,
      kBack = 10,
      kStart = 11,
      kXBox = 12,
      kStickLeft = 13,
      kStickRight = 14
    };

    enum Axis {
      kLeftX = 0,
      kLeftY = 1,
      kRightX = 2,
      kRightY = 3,
      kRightTrigger = 4,
      kLeftTrigger = 5
    };

    struct ButtonStates {
      bool A;
      bool B;
      bool X;
      bool Y;
      bool LB;
      bool RB;
      bool Back;
      bool Start;
      bool StickLeft;
      bool StickRight;
      bool LT;
      bool RT;
      bool DUp;
      bool DRight;
      bool DDown;
      bool DLeft;
    };

    struct AxisStates {
      double LeftX;
      double LeftY;
      double LT;
      double RT;
      double RightX;
      double RightY;
    };

    struct ControllerState {
      ButtonStates Buttons;
      AxisStates   Axes;
    };

    XBoxController(int index);
    ~XBoxController();
    XBoxController(const XBoxController&) = delete;
    XBoxController(XBoxController&&);
    XBoxController& operator=(const XBoxController&) = delete;
    XBoxController& operator=(XBoxController&&);

    [[nodiscard]] bool operator()();
    [[nodiscard]] std::optional<ControllerState> CurrentState();

    void SetVibration(double leftPercent,
                      double rightPercent,
                      std::optional<std::chrono::milliseconds> duration = std::nullopt);

    void SetVibration(ArgosLib::VibrationModel newModel);

  private:
    bool Initialize();
    void Deinitialize();

    void UpdateVibration();

    const int m_index;
    SDL_Joystick* m_pJoystick;

    ArgosLib::VibrationModel m_vibrationModel;

    constexpr static auto JsIntToPct = [](int jsVal){ return static_cast<double>(jsVal) / 32767.0; };
};

std::ostream& operator<<(std::ostream& os, const XBoxController::ButtonStates buttons);
std::ostream& operator<<(std::ostream& os, const XBoxController::AxisStates axes);
std::ostream& operator<<(std::ostream& os, const XBoxController::ControllerState state);
