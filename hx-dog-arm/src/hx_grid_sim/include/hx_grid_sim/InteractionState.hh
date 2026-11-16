#pragma once
#include <algorithm>
#include <cmath>

namespace hx_grid {
// Simulator-independent transition rules. All durations are simulation seconds.
struct CabinetState {
  bool fault{false};
  bool rotary{false};
  bool armed{false};
  double held{0};
  unsigned services{0};
  void InjectFault() { fault = true; held = 0; }
  bool Step(double dt, double travel, double angle) {
    armed = !rotary || (std::isfinite(angle) && angle >= 1.5707963267948966 - .04);
    if (!std::isfinite(dt) || dt <= 0) return false;
    if (!fault || !armed || !std::isfinite(travel) || travel < .006) {
      held = 0; return false;
    }
    held += dt;
    if (held + 1e-9 < .12) return false;
    fault = false; held = 0; ++services;
    return true;
  }
};
struct DoorState {
  double swipeHeld{0}, authorizedUntil{-1};
  bool latched{true}, opened{false};
  bool Authorized(double now) const { return now < authorizedUntil; }
  void Step(double now, double dt, bool validCardNear, double handle, double hinge) {
    if (!std::isfinite(now) || !std::isfinite(dt) || dt <= 0) return;
    swipeHeld = validCardNear ? swipeHeld + dt : 0;
    if (swipeHeld >= .25) authorizedUntil = now + 15.;
    if (Authorized(now) && std::isfinite(handle) && handle <= -.65) latched = false;
    // Once physically opened, authorization expiry must not snap the door shut.
    opened = std::isfinite(hinge) && hinge <= -1.25;
    if (!Authorized(now) && std::abs(hinge) < .025 && std::abs(handle) < .1)
      latched = true;
  }
};
}  // namespace hx_grid
