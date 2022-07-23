////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "XBoxController.h"
#include <algorithm>
#include <stdio.h>

XBoxController::XBoxController(int index)
    : m_index{index}, m_pJoystick{nullptr}, m_latestState{}, m_vibrationModel{ArgosLib::VibrationOff()} {
  Initialize();
}

XBoxController::~XBoxController() {
  Deinitialize();
  SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
  SDL_Quit();
}

XBoxController::XBoxController(XBoxController&& other)
    : m_index(other.m_index), m_pJoystick(other.m_pJoystick), m_vibrationModel{other.m_vibrationModel} {
  other.m_pJoystick = nullptr;
}

XBoxController& XBoxController::operator=(XBoxController&& other) {
  Deinitialize();
  new (this) XBoxController(std::move(other));
  return *this;
}

bool XBoxController::Initialize() {
  // Close joystick if it was open already
  if (m_pJoystick) {
    SDL_GameControllerClose(m_pJoystick);
  }

  // Restart SDL
  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");  //so Ctrl-C still works
  if (SDL_WasInit(SDL_INIT_GAMECONTROLLER) != 0) {
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
  }
  SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER);

  // Fail if desired index is unavailable
  const int numJoysticks = SDL_NumJoysticks();
  if (numJoysticks <= m_index) {
    std::cout << "Joystick index " << m_index << " unavailable\n";
    return false;
  }

  if (!SDL_IsGameController(m_index)) {
    std::cout << "Joystick index " << m_index << "is not a game controller\n";
    return false;
  }

  // Check if joystick at requested index matches XBox controller
  SDL_GameController* candidateJoystick = SDL_GameControllerOpen(m_index);
  if (!SDL_GameControllerGetAttached(candidateJoystick)) {
    std::cout << "Game controller open failed\n";
    return false;
  }

  const char* name = SDL_GameControllerName(candidateJoystick);
  const int num_axes = SDL_JoystickNumAxes(SDL_GameControllerGetJoystick(candidateJoystick));
  const int num_buttons = SDL_JoystickNumButtons(SDL_GameControllerGetJoystick(candidateJoystick));
  const int num_hats = SDL_JoystickNumHats(SDL_GameControllerGetJoystick(candidateJoystick));

  printf("Found joystick joystick '%s' with:\n"
         "\t%d axes\n"
         "\t%d buttons\n"
         "\t%d hats\n\n",
         name,
         num_axes,
         num_buttons,
         num_hats);

  /// @todo actually check values...
  if (num_axes == 7 && num_buttons == 10 && num_hats == 1) {
    std::cout << "Connected to new XBox One controller\n";
    SDL_GameControllerEventState(SDL_ENABLE);
    m_pJoystick = candidateJoystick;
    return true;
  } else {
    std::cout << "VendorID: " << SDL_GameControllerGetVendor(m_pJoystick)
              << ", ProductID: " << SDL_GameControllerGetProduct(m_pJoystick) << '\n';
    std::cout << "Joystick type: " << SDL_GameControllerGetType(m_pJoystick) << '\n';
  }

  printf("[ERROR] Joystick does not match an XBox controller\n");
  return false;
}

void XBoxController::Deinitialize() {
  m_latestState = ControllerState();
  if (m_pJoystick) {
    SDL_GameControllerEventState(SDL_DISABLE);
    SDL_GameControllerClose(m_pJoystick);
    m_pJoystick = nullptr;
  }
}

std::optional<XBoxController::ControllerState> XBoxController::CurrentState() {
  // Try getting controller if it was lost
  if (m_pJoystick == nullptr) {
    Initialize();
  }

  if (m_pJoystick != nullptr) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      m_lastHeartbeat = std::chrono::steady_clock::now();
      switch (event.type) {
        case SDL_QUIT:
          Deinitialize();
          return std::nullopt;
          break;

        // Handle new controller attaching
        case SDL_CONTROLLERDEVICEADDED:
          std::cout << "DEVICEADDED cdevice.which = " << event.cdevice.which << std::endl;
          break;

        // If a controller button is pressed
        case SDL_CONTROLLERBUTTONDOWN:
          // Looking for the button that was pressed
          if (event.cbutton.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(m_pJoystick))) {
            // So the relevant state can be updated
            switch (event.cbutton.button) {
              case SDL_CONTROLLER_BUTTON_A:
                m_latestState.Buttons.A = true;
                break;
              case SDL_CONTROLLER_BUTTON_B:
                m_latestState.Buttons.B = true;
                break;
              case SDL_CONTROLLER_BUTTON_X:
                m_latestState.Buttons.X = true;
                break;
              case SDL_CONTROLLER_BUTTON_Y:
                m_latestState.Buttons.Y = true;
                break;
              case SDL_CONTROLLER_BUTTON_BACK:
                m_latestState.Buttons.Back = true;
                break;
              case SDL_CONTROLLER_BUTTON_GUIDE:
                m_latestState.Buttons.XBox = true;
                break;
              case SDL_CONTROLLER_BUTTON_START:
                m_latestState.Buttons.Start = true;
                break;
              case SDL_CONTROLLER_BUTTON_LEFTSTICK:
                m_latestState.Buttons.StickLeft = true;
                break;
              case SDL_CONTROLLER_BUTTON_RIGHTSTICK:
                m_latestState.Buttons.StickRight = true;
                break;
              case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
                m_latestState.Buttons.LB = true;
                break;
              case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
                m_latestState.Buttons.RB = true;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_UP:
                m_latestState.Buttons.DUp = true;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                m_latestState.Buttons.DDown = true;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                m_latestState.Buttons.DLeft = true;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                m_latestState.Buttons.DRight = true;
                break;
            }
          }
          break;

        // Do the same for releasing a button
        case SDL_CONTROLLERBUTTONUP:
          if (event.cbutton.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(m_pJoystick))) {
            switch (event.cbutton.button) {
              case SDL_CONTROLLER_BUTTON_A:
                m_latestState.Buttons.A = false;
                break;
              case SDL_CONTROLLER_BUTTON_B:
                m_latestState.Buttons.B = false;
                break;
              case SDL_CONTROLLER_BUTTON_X:
                m_latestState.Buttons.X = false;
                break;
              case SDL_CONTROLLER_BUTTON_Y:
                m_latestState.Buttons.Y = false;
                break;
              case SDL_CONTROLLER_BUTTON_BACK:
                m_latestState.Buttons.Back = false;
                break;
              case SDL_CONTROLLER_BUTTON_GUIDE:
                m_latestState.Buttons.XBox = false;
                break;
              case SDL_CONTROLLER_BUTTON_START:
                m_latestState.Buttons.Start = false;
                break;
              case SDL_CONTROLLER_BUTTON_LEFTSTICK:
                m_latestState.Buttons.StickLeft = false;
                break;
              case SDL_CONTROLLER_BUTTON_RIGHTSTICK:
                m_latestState.Buttons.StickRight = false;
                break;
              case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
                m_latestState.Buttons.LB = false;
                break;
              case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
                m_latestState.Buttons.RB = false;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_UP:
                m_latestState.Buttons.DUp = false;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
                m_latestState.Buttons.DDown = false;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
                m_latestState.Buttons.DLeft = false;
                break;
              case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
                m_latestState.Buttons.DRight = false;
                break;
            }
          }
          break;

        // And something similar for axis motion
        case SDL_CONTROLLERAXISMOTION:
          if (event.cbutton.which == SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(m_pJoystick))) {
            switch (event.caxis.axis) {
              case SDL_CONTROLLER_AXIS_LEFTX:
                m_latestState.Axes.LeftX = JsIntToPct(event.caxis.value);
                break;
              case SDL_CONTROLLER_AXIS_LEFTY:
                m_latestState.Axes.LeftY = -1.0 * JsIntToPct(event.caxis.value);
                break;
              case SDL_CONTROLLER_AXIS_RIGHTX:
                m_latestState.Axes.RightX = JsIntToPct(event.caxis.value);
                break;
              case SDL_CONTROLLER_AXIS_RIGHTY:
                m_latestState.Axes.RightY = -1.0 * JsIntToPct(event.caxis.value);
                break;
              case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
                m_latestState.Axes.LT = (1.0 + JsIntToPct(event.caxis.value)) * 0.5;
                m_latestState.Buttons.LT = m_latestState.Axes.LT > 0.5;
                break;
              case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
                m_latestState.Axes.RT = (1.0 + JsIntToPct(event.caxis.value)) * 0.5;
                m_latestState.Buttons.RT = m_latestState.Axes.RT > 0.5;
                break;
            }
          }
          break;
      }
    }

    UpdateVibration();
    if (std::chrono::steady_clock::now() - m_lastHeartbeat > std::chrono::milliseconds(2000)) {
      printf("Timeout!\n");
      return std::nullopt;
    }

    return m_latestState;
  }
  return std::nullopt;
}

void XBoxController::UpdateVibration() {
  if (m_pJoystick != nullptr) {
    const auto vibrationIntensity = m_vibrationModel();
    SDL_GameControllerRumble(m_pJoystick,
                             std::numeric_limits<uint16_t>::max() * vibrationIntensity.intensityLeft,
                             std::numeric_limits<uint16_t>::max() * vibrationIntensity.intensityRight,
                             std::numeric_limits<uint32_t>::max());
  }
}

void XBoxController::SetVibration(double leftPercent,
                                  double rightPercent,
                                  std::optional<std::chrono::milliseconds> duration) {
  if (!duration) {
    m_vibrationModel = ArgosLib::VibrationConstant(leftPercent, rightPercent);
  } else {
    auto startTime = std::chrono::steady_clock::now();
    m_vibrationModel = [startTime, leftPercent, rightPercent, duration]() {
      auto expired = (std::chrono::steady_clock::now() - startTime) >= duration;
      return ArgosLib::VibrationStatus{.intensityLeft{expired ? 0.0 : leftPercent},
                                       .intensityRight{expired ? 0.0 : rightPercent}};
    };
  }
  UpdateVibration();
}

void XBoxController::SetVibration(ArgosLib::VibrationModel newModel) {
  m_vibrationModel = newModel;
  UpdateVibration();
}

std::ostream& operator<<(std::ostream& os, const XBoxController::ButtonStates buttons) {
  os << "{";
  os << " A: " << buttons.A;
  os << " B: " << buttons.B;
  os << " X: " << buttons.X;
  os << " Y: " << buttons.Y;
  os << " LB: " << buttons.LB;
  os << " RB: " << buttons.RB;
  os << " Back: " << buttons.Back;
  os << " Start: " << buttons.Start;
  os << " StickLeft: " << buttons.StickLeft;
  os << " StickRight: " << buttons.StickRight;
  os << " LT: " << buttons.LT;
  os << " RT: " << buttons.RT;
  os << " DUp: " << buttons.DUp;
  os << " DRight: " << buttons.DRight;
  os << " DDown: " << buttons.DDown;
  os << " DLeft: " << buttons.DLeft;
  os << " }";
  return os;
}

std::ostream& operator<<(std::ostream& os, const XBoxController::AxisStates axes) {
  os << "{";
  os << " LeftX:" << axes.LeftX;
  os << " LeftY:" << axes.LeftY;
  os << " LT:" << axes.LT;
  os << " RT:" << axes.RT;
  os << " RightX:" << axes.RightX;
  os << " RightY:" << axes.RightY;
  os << " }";
  return os;
}

std::ostream& operator<<(std::ostream& os, const XBoxController::ControllerState state) {
  os << "{ Buttons: " << state.Buttons << ", Axes: " << state.Axes << " }";
  return os;
}
