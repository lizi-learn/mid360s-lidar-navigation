#include "../navigation/velocity_contract.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main(){
  using namespace p2_nav;const std::uint64_t now=1000000000ULL;
  Packet p{0x504e4156,1,1,7,1,now,.20f,0,.30f,0};Authority a;
  Consumer c;assert(c.accept(p,now,a).vx==0); // A packet cannot grant itself authority.
  a.healthy=a.calibrated=a.policy_active=a.navigation_armed=true;
  assert(c.accept(p,now,a).vx>.19);assert(c.accept(p,now,a).vx==0); // Replay rejected.
  ++p.sequence;a.estop=true;assert(c.accept(p,now,a).vx==0);a.estop=false;
  a.manual_override=true;assert(c.accept(p,now,a).vx==0);a.manual_override=false;
  p.vy=.01f;assert(c.accept(p,now,a).vx==0);p.vy=0;
  p.vx=-.1f;assert(c.accept(p,now,a).vx==0);p.vx=.2f;
  p.vx=std::numeric_limits<float>::quiet_NaN();assert(c.accept(p,now,a).vx==0);p.vx=.2f;
  p.wz=.5f;assert(c.accept(p,now,a).vx==0);p.wz=.3f;
  assert(c.accept(p,now+400000000ULL,a).vx==0); // Expired lease.
  assert(c.accept(p,now-10000000ULL,a).vx==0); // Future timestamp.
  p.instance=8;assert(c.accept(p,now,a).vx==0); // Replacing a source needs disarmed rearm.
  c.reset_disarmed();assert(c.accept(p,now,a).vx>.19);
  ++p.sequence;p.reserved=1;assert(c.accept(p,now,a).vx==0);
  std::cout<<"Navigation-to-gait lease contract checks passed; no hardware I/O.\n";
}
