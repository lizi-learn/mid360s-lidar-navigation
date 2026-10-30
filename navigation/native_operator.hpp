#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
namespace p2_nav {
constexpr std::uint32_t native_magic=0x4e4f5031;
struct NativeOperatorPacket {
 std::uint32_t magic=native_magic,version=1;
 std::uint64_t instance=0,sequence=0,monotonic_ns=0,unix_us=0;
 std::uint32_t mode=0,flags=0;
 float x=0,y=0,yaw=0;std::uint32_t reserved=0;
};
static_assert(sizeof(NativeOperatorPacket)==64);
// Original commissioned37-value source is decoded by the existing read-only
// observer; this fixed-size path is separate from navigation/GUI RPC.
class NativeOperatorGuard {
 std::uint64_t instance_=0,sequence_=0,stamp_=0;
 public:
 bool accept(const NativeOperatorPacket& p,std::uint64_t now){
  if(p.magic!=native_magic||p.version!=1||p.reserved||!p.instance||!p.sequence||!p.monotonic_ns||
     p.mode>3||(p.flags&~15U)||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)||
     std::abs(p.x)>1||std::abs(p.y)>1||std::abs(p.yaw)>1)return false;
  if(p.monotonic_ns>now? p.monotonic_ns-now>5000000ULL:now-p.monotonic_ns>100000000ULL)return false;
  if(p.instance==instance_&&(p.sequence<=sequence_||p.monotonic_ns<stamp_))return false;
  instance_=p.instance;sequence_=p.sequence;stamp_=p.monotonic_ns;return true;
 }
 void reset(){instance_=sequence_=stamp_=0;}
};
} // namespace p2_nav
