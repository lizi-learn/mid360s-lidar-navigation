#pragma once
// Fixed-size local transport contract. No joint/joystick publisher.
#include "physical_speed_input.hpp"
#include "native_operator.hpp"
#include <algorithm>
#include <array>
#include <mutex>
#include <type_traits>

namespace p2_nav {
constexpr std::uint32_t frame_magic=0x504e4731;
enum class Operation : std::uint32_t { authority=1, arm=2, velocity=3, stop=4, cancel_ack=5 };
struct LocalFrame {
  std::uint32_t magic=frame_magic, version=1;
  Operation operation=Operation::stop;
  std::uint32_t flags=0;
  std::uint64_t client=0, sequence=0, monotonic_ns=0;
  std::uint64_t controller=0, generation=0, cancel_generation=0;
  std::uint64_t transport_epoch=0;
  std::uint64_t receiver_instance=0, receiver_generation=0;
  std::uint64_t operator_instance=0, operator_stamp=0, health_stamp=0;
  std::uint32_t operator_mode=0, reserved=0;
  Packet velocity{};
};
enum FrameFlag : std::uint32_t {
  receiver_connected=1, switch_verified=2, stick_active=4,
  health_healthy=8, health_calibrated=16, health_ground=32, health_localized=64
};
constexpr std::uint32_t frame_flags=127;
struct MailboxSnapshot {
  LocalFrame authority{}, arm{}, velocity{}, acknowledgement{};
  std::uint64_t stop_serial=0;
  std::uint64_t transport_epoch=0;
  NativeOperatorPacket native{};
};
struct NativeGaitState {
  std::uint64_t monotonic_ns=0, controller=0, policy_generation=0;
  std::uint64_t receiver_instance=0, receiver_generation=0, receiver_stamp=0;
  bool receiver_connected=false, manual_enabled=false, estop=false;
  bool d13_selected=false, walking_ready=false, hardware_healthy=false;
  // Must be established independently from the RPC client/GUI.
  bool commissioning_verified=false, driver_protection_verified=false;
};
struct LoopReply {
  std::uint32_t magic=frame_magic, version=1;
  std::uint64_t tick=0, monotonic_ns=0, controller=0, generation=0;
  std::uint64_t receiver_instance=0, receiver_generation=0;
  std::uint64_t seen_command=0, cancel_generation=0;
  std::uint64_t transport_epoch=0;
  std::uint64_t native_instance=0,native_stamp=0,native_sequence=0;
  std::uint32_t armed=0, navigation_owned=0, force_zero=1, cancel_pending=0;
  std::uint32_t manual_owned=0,operator_mode=0,operator_verified=0,manual_activity=0;
  float vx=0, vy=0, wz=0;std::uint32_t readiness=0;
  std::array<char,96> reason{};
};
static_assert(std::is_trivially_copyable<LocalFrame>::value);
static_assert(sizeof(LocalFrame)==168);
static_assert(sizeof(LoopReply)==248);

// Only the worker locks normally. The 100Hz gait loop tries once, then retains
// the previous snapshot with its ORIGINAL timestamps. Contention cannot renew it.
class LocalMailbox {
  std::mutex input_mutex_, reply_mutex_;
  MailboxSnapshot input_{};
  LoopReply reply_{};
 public:
  void post(const LocalFrame& f){
    std::lock_guard<std::mutex> g(input_mutex_);
    switch(f.operation){
      case Operation::authority:input_.authority=f;break;
      case Operation::arm:input_.arm=f;break;
      case Operation::velocity:input_.velocity=f;break;
      case Operation::cancel_ack:input_.acknowledgement=f;break;
      case Operation::stop:++input_.stop_serial;break;
    }
  }
  void connect(){
    std::lock_guard<std::mutex> g(input_mutex_);
    const auto serial=input_.stop_serial+1,epoch=input_.transport_epoch+1;
    const auto native=input_.native;input_={};input_.native=native;input_.stop_serial=serial;input_.transport_epoch=epoch;
  }
  void disconnect(){
    std::lock_guard<std::mutex> g(input_mutex_);
    const auto serial=input_.stop_serial+1,epoch=input_.transport_epoch;
    const auto native=input_.native;input_={};input_.native=native;input_.stop_serial=serial;input_.transport_epoch=epoch;
  }
  void post_native(const NativeOperatorPacket& p){std::lock_guard<std::mutex> g(input_mutex_);input_.native=p;}
  void native_disconnect(){std::lock_guard<std::mutex> g(input_mutex_);input_.native={};}
  bool try_input(MailboxSnapshot& out){
    std::unique_lock<std::mutex> g(input_mutex_,std::try_to_lock);
    if(!g.owns_lock())return false;
    out=input_;return true;
  }
  void publish(const LoopReply& r){
    std::unique_lock<std::mutex> g(reply_mutex_,std::try_to_lock);
    if(g.owns_lock())reply_=r;
  }
  std::uint64_t epoch(){std::lock_guard<std::mutex> g(input_mutex_);return input_.transport_epoch;}
  LoopReply reply(){std::lock_guard<std::mutex> g(reply_mutex_);return reply_;}
};

class GaitNavigationInput {
  PhysicalSpeedInput lease_;
  MailboxSnapshot snapshot_{};
  std::uint64_t stop_=0, arm_=0, packet_=0, ack_=0, tick_=0;
  std::uint64_t operator_instance_=0, receiver_instance_=0, receiver_generation_=0,client_=0;
  bool manual_override_=false;
  static bool fresh(std::uint64_t t,std::uint64_t now,std::uint64_t limit){
    return t && (t>now?t-now<=5000000ULL:now-t<=limit);
  }
 public:
  LoopReply step(LocalMailbox& box,const NativeGaitState& n){
    box.try_input(snapshot_);
    const auto& a=snapshot_.authority;const auto& native=snapshot_.native;
    const auto now=n.monotonic_ns;
    const bool native_fresh=fresh(native.monotonic_ns,now,100000000ULL)&&native.instance&&
      native.sequence&&(native.flags&1U)&&(native.flags&2U)&&native.mode>=1&&native.mode<=3;
    const bool authority_fresh=a.operation==Operation::authority&&a.client&&
      a.operator_instance==native.instance&&fresh(a.operator_stamp,now,100000000ULL)&&
      fresh(a.monotonic_ns,now,100000000ULL)&&a.receiver_instance==n.receiver_instance&&
      a.receiver_generation==n.receiver_generation;
    const bool source_changed=operator_instance_&&
      (operator_instance_!=native.instance||receiver_instance_!=n.receiver_instance||
       receiver_generation_!=n.receiver_generation||client_!=a.client);
    if(source_changed){lease_.disarm();manual_override_=false;}
    if(native_fresh){operator_instance_=native.instance;receiver_instance_=n.receiver_instance;
      receiver_generation_=n.receiver_generation;client_=a.client;}
    const bool stopped=snapshot_.stop_serial!=stop_;
    if(stopped){stop_=snapshot_.stop_serial;lease_.disarm();}
    const bool activity=native_fresh&&(native.flags&4U);
    if(native_fresh&&native.mode!=1)manual_override_=false;
    if(native_fresh&&native.mode==1&&activity)manual_override_=true;
    GaitFeedback f;f.monotonic_ns=now;f.controller_instance=n.controller;f.policy_generation=n.policy_generation;
    f.supported_policy=n.d13_selected;f.walking_ready=n.walking_ready;
    f.operator_connected=n.receiver_connected&&native_fresh;
    f.estop=n.estop||(native.flags&8U);f.manual_activity=activity;
    f.operator_mode=!native_fresh?OperatorMode::unknown:native.mode==3?OperatorMode::manual:
      native.mode==2?OperatorMode::paused:OperatorMode::navigation_allowed;
    f.operator_mode_verified=native_fresh;
    const bool health_fresh=authority_fresh&&fresh(a.health_stamp,now,100000000ULL);
    f.healthy=n.hardware_healthy&&n.driver_protection_verified&&health_fresh&&(a.flags&health_healthy);
    f.calibrated=n.commissioning_verified&&health_fresh&&(a.flags&health_calibrated);
    f.ground_ok=health_fresh&&(a.flags&health_ground);f.localized=health_fresh&&(a.flags&health_localized);
    auto selected=lease_.poll(f,now);
    auto belongs=[&](const LocalFrame& p){return native_fresh&&authority_fresh&&p.client==a.client&&
      p.controller==n.controller&&p.generation==n.policy_generation&&p.transport_epoch==snapshot_.transport_epoch&&
      snapshot_.transport_epoch&&fresh(p.monotonic_ns,now,100000000ULL);};
    if(snapshot_.acknowledgement.sequence!=ack_){ack_=snapshot_.acknowledgement.sequence;
      if(belongs(snapshot_.acknowledgement))lease_.acknowledge_cancel(snapshot_.acknowledgement.cancel_generation);}
    if(snapshot_.arm.sequence!=arm_){arm_=snapshot_.arm.sequence;
      if(!stopped&&!source_changed&&belongs(snapshot_.arm)&&lease_.arm(f,now))manual_override_=false;}
    if(snapshot_.velocity.sequence!=packet_){packet_=snapshot_.velocity.sequence;
      if(belongs(snapshot_.velocity))lease_.accept(snapshot_.velocity.velocity,sizeof(Packet),f,now);}
    selected=lease_.poll(f,now);
    LoopReply r;r.tick=++tick_;r.monotonic_ns=now;r.controller=n.controller;r.generation=n.policy_generation;
    r.receiver_instance=n.receiver_instance;r.receiver_generation=n.receiver_generation;
    r.transport_epoch=snapshot_.transport_epoch;r.native_instance=native.instance;
    r.native_stamp=native.monotonic_ns;r.native_sequence=native.sequence;
    r.operator_mode=native_fresh?native.mode:0;r.operator_verified=native_fresh;r.manual_activity=activity;
    r.seen_command=std::max({arm_,packet_,ack_});r.cancel_generation=selected.cancel_generation;
    r.readiness=(n.d13_selected?1U:0U)|(n.walking_ready?2U:0U)|(n.hardware_healthy?4U:0U)|
      (n.driver_protection_verified?8U:0U)|(n.commissioning_verified?16U:0U)|
      (f.ground_ok?32U:0U)|(f.localized?64U:0U)|(native_fresh?128U:0U);
    r.armed=lease_.armed();r.navigation_owned=selected.navigation_owns_velocity;
    r.cancel_pending=selected.cancel_navigation_goal;r.vx=selected.vx;r.wz=selected.wz;
    r.manual_owned=manual_override_&&native_fresh&&native.mode==1&&!f.estop&&n.walking_ready&&n.hardware_healthy;
    if(r.manual_owned){r.vx=native.x;r.vy=native.y;r.wz=native.yaw;}
    // With no own native sample, retain the ORIGINAL factory manual path. Its
    // original receiver masks axes in E-left/middle. No nav command is retained.
    const bool factory_manual=!native_fresh||native.mode==3;
    r.force_zero=!r.navigation_owned&&!r.manual_owned&&!factory_manual;
    const char* reason=r.manual_owned?"native_stick_manual_takeover":
      native_fresh&&native.mode==3?"native_manual_priority":selected.reason;
    std::strncpy(r.reason.data(),reason,r.reason.size()-1);box.publish(r);return r;
  }
};
} // namespace p2_nav
