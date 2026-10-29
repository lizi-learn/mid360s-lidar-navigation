#pragma once
// Independent navigation-to-gait lease. No motor, joystick or shared-memory writer.
#include <cstdint>
#include <cmath>
#include <cstring>
namespace p2_nav {
struct Packet {
  std::uint32_t magic; std::uint16_t version, flags;
  std::uint64_t instance, sequence, monotonic_ns;
  float vx, vy, wz; std::uint32_t reserved;
};
static_assert(sizeof(Packet)==48,"Navigation wire layout must be 48 bytes");
struct Authority {
  bool estop=false, healthy=false, calibrated=false, policy_active=false;
  bool navigation_armed=false, manual_override=false;
};
struct Decision { double vx=0., vy=0., wz=0.; const char* reason="disabled"; };
class Consumer {
  std::uint64_t instance_=0,sequence_=0;
 public:
  void reset_disarmed(){instance_=sequence_=0;}
  Decision accept(const Packet& p,std::uint64_t now,const Authority& a){
    if(a.estop)return {0,0,0,"estop"};
    if(a.manual_override)return {0,0,0,"manual_has_authority"};
    if(!a.healthy||!a.calibrated||!a.policy_active||!a.navigation_armed)return {0,0,0,"not_ready"};
    if(p.magic!=0x504e4156||p.version!=1||p.flags!=1||p.reserved||!p.instance)return {0,0,0,"invalid_packet"};
    if(!std::isfinite(p.vx)||!std::isfinite(p.vy)||!std::isfinite(p.wz)||p.vx<0||p.vx>.300001f||std::abs(p.vy)>1e-6f||std::abs(p.wz)>.350001f)return {0,0,0,"motion_contract"};
    if(p.monotonic_ns>now+5000000ULL||now>p.monotonic_ns+300000000ULL)return {0,0,0,"expired"};
    if(instance_&&p.instance!=instance_)return {0,0,0,"source_changed_rearm_required"};
    if(p.sequence<=sequence_)return {0,0,0,"replayed_packet"};
    instance_=p.instance;sequence_=p.sequence;return {p.vx,0,p.wz,"accepted"};
  }
};
} // namespace p2_nav
