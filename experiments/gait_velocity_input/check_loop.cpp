#include "gait_navigation_server.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
using namespace p2_nav;
constexpr std::uint64_t base=10000000000ULL;
NativeGaitState native(std::uint64_t t=base){
 NativeGaitState n;n.monotonic_ns=n.receiver_stamp=t;n.controller=10;n.policy_generation=1;
 n.receiver_instance=20;n.receiver_generation=1;n.receiver_connected=n.d13_selected=n.walking_ready=true;
 n.hardware_healthy=n.commissioning_verified=n.driver_protection_verified=true;return n;
}
struct Rig {
 NativeOperatorPacket raw{};
 LocalMailbox box;GaitNavigationInput gate;std::uint64_t seq=0;LoopReply reply{};
 Rig(){raw.instance=40;raw.sequence=1;raw.monotonic_ns=base;raw.mode=1;raw.flags=3;box.post_native(raw);box.connect();reply=gate.step(box,native());}
 void physical(std::uint64_t now=base){raw.monotonic_ns=now;++raw.sequence;box.post_native(raw);}
 LocalFrame frame(Operation op,std::uint64_t now=base){
  LocalFrame f;f.operation=op;f.client=30;f.sequence=++seq;f.monotonic_ns=now;
  f.controller=10;f.generation=1;f.transport_epoch=reply.transport_epoch;
  f.receiver_instance=20;f.receiver_generation=1;f.operator_instance=40;
  f.operator_stamp=f.health_stamp=now;f.operator_mode=1;f.flags=frame_flags&~stick_active;
  f.velocity={0x504e4156,1,1,30,f.sequence,now,0,0,0,0};return f;
 }
 LoopReply tick(NativeGaitState n=native()){reply=gate.step(box,n);return reply;}
 void authority(std::uint64_t now=base){physical(now);box.post(frame(Operation::authority,now));tick(native(now));}
 void active(){authority();box.post(frame(Operation::arm));tick();assert(reply.armed);
  box.post(frame(Operation::velocity));tick();assert(reply.navigation_owned);
  auto f=frame(Operation::velocity);f.velocity.vx=.1f;box.post(f);tick();assert(reply.vx>.099f);}
};
int main(){
 {Rig r;r.active();auto n=native();r.raw.mode=3;r.physical();r.tick(n);
  assert(!r.reply.navigation_owned&&!r.reply.force_zero&&r.reply.cancel_pending);
  r.raw.mode=1;r.physical();r.tick();assert(!r.reply.navigation_owned&&r.reply.force_zero);}
 for(int i=0;i<16;++i){
  Rig r;r.active();auto n=native(base+1);auto a=r.frame(Operation::authority,base+1);
  if(i==0)n.receiver_instance++;
  if(i==1)n.receiver_generation++;
  if(i==2)n.policy_generation++;
  if(i==3)n.d13_selected=false;
  if(i==4)n.walking_ready=false;
  if(i==5)n.hardware_healthy=false;
  if(i==6)n.commissioning_verified=false;
  if(i==7)n.driver_protection_verified=false;
  if(i==8)n.estop=true;
  if(i==9){r.raw.mode=2;r.physical(base+1);}
  if(i==10){r.raw.flags|=4;r.raw.x=.5f;r.physical(base+1);}
  if(i==11)a.operator_instance++;
  if(i==12)a.health_stamp=base-100000001;
  if(i==13)a.operator_stamp=base-100000001;
  if(i==14)a.flags&=~health_ground;
  if(i==15)a.flags&=~health_localized;
  r.box.post(a);r.tick(n);assert(!r.reply.navigation_owned&&r.reply.cancel_pending);if(i!=10)assert(r.reply.force_zero);else assert(r.reply.manual_owned&&r.reply.vx==.5f);
  r.authority(base+2);assert(!r.reply.navigation_owned); // no automatic resume
 }
 {Rig r;r.active();r.tick(native(base+100000001));assert(!r.reply.navigation_owned&&r.reply.cancel_pending);}
 {Rig r;r.active();r.authority(base+300000001);assert(!r.reply.navigation_owned&&r.reply.cancel_pending);}
 {Rig r;r.authority();auto arm=r.frame(Operation::arm);r.box.post(arm);r.box.post(r.frame(Operation::stop));r.tick();assert(!r.reply.armed);r.tick();assert(!r.reply.armed);}
 {Rig r;r.active();const auto old=r.frame(Operation::arm);r.box.disconnect();r.tick();assert(!r.reply.navigation_owned);
  r.box.connect();r.tick();r.authority();auto ack=r.frame(Operation::cancel_ack);ack.cancel_generation=r.reply.cancel_generation;
  r.box.post(ack);r.tick();r.box.post(old);r.tick();assert(!r.reply.armed); // old connection cannot ARM
 }
 {Rig r;r.active();auto a=r.frame(Operation::authority);r.raw.flags|=4;r.physical();r.box.post(a);r.tick();r.raw.flags&=~4;r.physical();
  auto ack=r.frame(Operation::cancel_ack);ack.cancel_generation=r.reply.cancel_generation;r.box.post(ack);
  r.authority();r.box.post(r.frame(Operation::arm));r.tick();assert(r.reply.armed&&!r.reply.navigation_owned);
  auto bad=r.frame(Operation::velocity);bad.velocity.vx=.1f;r.box.post(bad);r.tick();assert(!r.reply.armed);}
 // Missing navigation authority/protection must NEVER prevent original manual.
 {Rig r;auto n=native();r.raw.mode=3;r.physical();n.commissioning_verified=n.driver_protection_verified=false;
  r.tick(n);assert(!r.reply.force_zero&&!r.reply.navigation_owned);}
 // Physical middle overrides a malicious/faulty client still reporting left.
 {Rig r;r.active();r.raw.mode=2;r.physical();r.box.post(r.frame(Operation::authority));r.tick();
  assert(!r.reply.navigation_owned&&r.reply.force_zero&&r.reply.operator_mode==2);}
 // The navigation RPC cannot forge a raw physical stick or E value.
 {Rig r;r.active();auto f=r.frame(Operation::authority);f.operator_mode=3;f.flags|=stick_active;
  r.box.post(f);r.tick();assert(r.reply.navigation_owned&&!r.reply.manual_owned);}
 // Raw input expiry revokes the lease even if RPC restamps every other field.
 {Rig r;r.active();auto t=base+100000001;auto f=r.frame(Operation::authority,t);
  r.box.post(f);r.tick(native(t));assert(!r.reply.navigation_owned&&r.reply.cancel_pending);}
 std::cout<<"native manual priority,16 authority faults,expiry,no-auto-resume,zero handshake,connection replay passed\n";
}
