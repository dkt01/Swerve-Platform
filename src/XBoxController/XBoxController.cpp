////////////////////////////////////////////////////////////////////////////////////////////////////
/// @copyright Copyright (c) 2021, David K Turner. All rights reserved.
/// @license This project is released under the BSD 3-Clause License
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "XBoxController.h"
#include <stdio.h>

XBoxController::XBoxController(int index)
    : m_index{index}
    , m_pJoystick{nullptr} {
  Initialize();
}

XBoxController::~XBoxController() {
  Deinitialize();
  SDL_Quit();
}

XBoxController::XBoxController(XBoxController&& other)
   : m_index(other.m_index)
   , m_pJoystick(other.m_pJoystick) {
  other.m_pJoystick = nullptr;
}

XBoxController& XBoxController::operator=(XBoxController&& other) {
  Deinitialize();
  new(this) XBoxController(std::move(other));
  return *this;
}

bool XBoxController::Initialize() {
  // Close joystick if it was open already
  if(m_pJoystick) {
    SDL_JoystickClose(m_pJoystick);
  }

  // Restart SDL
  SDL_Quit();
  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1"); //so Ctrl-C still works
  SDL_Init(SDL_INIT_JOYSTICK);

  // Fail if desired index is unavailable
  const int numJoysticks = SDL_NumJoysticks();
  if (numJoysticks <= m_index) { return false; }

  // Check if joystick at requested index matches XBox controller
  SDL_Joystick *candidateJoystick = SDL_JoystickOpen(m_index);
  if(candidateJoystick == nullptr) { return false; } // Failed to open

  const char *name = SDL_JoystickName(candidateJoystick);
  const int num_axes = SDL_JoystickNumAxes(candidateJoystick);
  const int num_buttons = SDL_JoystickNumButtons(candidateJoystick);
  const int num_hats = SDL_JoystickNumHats(candidateJoystick);

  printf("Found joystick joystick '%s' with:\n"
			"%d axes\n"
			"%d buttons\n"
			"%d hats\n\n",
			name,
			num_axes,
			num_buttons,
			num_hats);

  /// @todo actually check values...
  if( num_axes == 6 && num_buttons == 16 && num_hats == 1 ) {
    m_pJoystick = candidateJoystick;
    return true;
  }

  printf("[ERROR] Joystick does not match an XBox controller\n");
  return false;
}

void XBoxController::Deinitialize() {
  if(m_pJoystick) {
    SDL_JoystickClose(m_pJoystick);
    m_pJoystick = nullptr;
  }
}

std::optional<XBoxController::ControllerState> XBoxController::CurrentState() {
  // Try getting controller if it was lost
  if(m_pJoystick == nullptr) {
    Initialize();
  }

  if(m_pJoystick != nullptr) {
    SDL_Event event;
    if (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT ||
          event.jdevice.type == SDL_JOYDEVICEREMOVED) {
            Deinitialize();
            return std::nullopt;
      }
    }

    ControllerState currentState {
      .Buttons {
        .A{0 != SDL_JoystickGetButton(m_pJoystick, Button::kA)},
        .B{0 != SDL_JoystickGetButton(m_pJoystick, Button::kB)},
        .X{0 != SDL_JoystickGetButton(m_pJoystick, Button::kX)},
        .Y{0 != SDL_JoystickGetButton(m_pJoystick, Button::kY)},
        .LB{0 != SDL_JoystickGetButton(m_pJoystick, Button::kBumperLeft)},
        .RB{0 != SDL_JoystickGetButton(m_pJoystick, Button::kBumperRight)},
        .Back{0 != SDL_JoystickGetButton(m_pJoystick, Button::kBack)},
        .Start{0 != SDL_JoystickGetButton(m_pJoystick, Button::kStart)},
        .StickLeft{0 != SDL_JoystickGetButton(m_pJoystick, Button::kStickLeft)},
        .StickRight{0 != SDL_JoystickGetButton(m_pJoystick, Button::kStickRight)},
        .LT{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kLeftTrigger)) > 0},
        .RT{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kRightTrigger)) > 0},
        .DUp{0 != (SDL_JoystickGetHat(m_pJoystick, 0) & SDL_HAT_UP)},
        .DRight{0 != (SDL_JoystickGetHat(m_pJoystick, 0) & SDL_HAT_RIGHT)},
        .DDown{0 != (SDL_JoystickGetHat(m_pJoystick, 0) & SDL_HAT_DOWN)},
        .DLeft{0 != (SDL_JoystickGetHat(m_pJoystick, 0) & SDL_HAT_LEFT)}
      },
      .Axes { // Invert Y axes so positive direction is forward
        .LeftX{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kLeftX))},
        .LeftY{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kLeftY) * -1)},
        .LT{(1.0 + JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kLeftTrigger))) * 0.5}, // Triggers range from -1 to 1; want 0 to 1
        .RT{(1.0 + JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kRightTrigger))) * 0.5},
        .RightX{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kRightX))},
        .RightY{JsIntToPct(SDL_JoystickGetAxis(m_pJoystick, Axis::kRightY) * -1)}
      }
    };

    return currentState;
  }
  return std::nullopt;
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
