#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "hx_grid_sim/InteractionState.hh"
using namespace hx_grid;
int main() {
  CabinetState normal; normal.fault=true;
  for (int i=0; i<100; ++i) assert(!normal.Step(.001,.005,0));
  for (int i=0; i<119; ++i) assert(!normal.Step(.001,.008,0));
  assert(normal.Step(.001,.008,0)); assert(!normal.fault && normal.services==1);
  for (int i=0; i<300; ++i) assert(!normal.Step(.001,.008,0));
  assert(normal.services==1);
  normal.InjectFault();
  for (int i=0; i<80; ++i) normal.Step(.001,.008,0);
  normal.Step(.001,0,0);  // Interrupted press does not accumulate dwell.
  for (int i=0; i<80; ++i) normal.Step(.001,.008,0);
  assert(normal.fault);
  assert(!normal.Step(0,.012,0)); assert(!normal.Step(-1,.012,0));
  normal.Step(.001,std::numeric_limits<double>::quiet_NaN(),0);
  assert(normal.held==0);
  CabinetState rotary; rotary.fault=true; rotary.rotary=true;
  for (int i=0; i<300; ++i) assert(!rotary.Step(.001,.012,1.2));
  assert(rotary.fault && !rotary.armed);
  for (int i=0; i<300; ++i) rotary.Step(.001,0,1.57079632679);
  assert(rotary.fault && rotary.armed);  // Rotation alone never clears fault.
  for (int i=0; i<120; ++i) rotary.Step(.001,.009,1.57079632679);
  assert(!rotary.fault && rotary.services==1);
  rotary.InjectFault();
  for (int i=0; i<70; ++i) rotary.Step(.001,.01,1.57);
  rotary.Step(.001,.01,0);  // Re-lock before dwell completes.
  for (int i=0; i<70; ++i) rotary.Step(.001,.01,1.57);
  assert(rotary.fault);
  DoorState door;
  door.Step(1,.1,false,-1,0); assert(door.latched); // No valid swipe.
  door.Step(2,.1,true,0,0); door.Step(2.1,.1,true,0,0);
  assert(!door.Authorized(2.1));
  door.Step(2.2,.1,false,0,0); // Moving card away resets swipe dwell.
  door.Step(3,.25,true,0,0); assert(door.Authorized(3) && door.latched);
  door.Step(3.1,.1,false,-.3,0); assert(door.latched);
  door.Step(3.2,.1,false,-.8,0); assert(!door.latched && !door.opened);
  door.Step(3.3,.1,false,-.8,-1.3); assert(door.opened);
  door.Step(30,.1,false,0,-1.3); assert(!door.latched); // No forced shut on expiry.
  door.Step(31,.1,false,0,0); assert(door.latched && !door.opened);
  DoorState expired;
  expired.Step(0,.25,true,0,0);
  expired.Step(20,.1,false,-1,0); assert(expired.latched);
  std::cout << "Interaction state tests passed (press, rotation, swipe, latch, expiry).\n";
}
