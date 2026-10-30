#include "gait_navigation_server.hpp"
#include <iostream>
using namespace p2_nav;
int main(int argc,char**argv){
 if(argc!=3)return 2;
 const bool synthetic=std::string(argv[2])=="--synthetic-commissioned";
 if(!synthetic&&std::string(argv[2])!="--uncommissioned")return 2;
 GaitNavigationServer server;server.start(argv[1]);
 std::uint64_t nav=0,manual=0;auto next=std::chrono::steady_clock::now();
 for(int i=0;i<1600;++i){
  NativeGaitState n;n.monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  n.controller=10;n.policy_generation=1;n.receiver_instance=20;n.receiver_generation=1;
  n.receiver_connected=n.d13_selected=n.walking_ready=n.hardware_healthy=true;
  n.driver_protection_verified=n.commissioning_verified=synthetic;
  auto r=server.step(n);nav+=r.navigation_owned;manual+=r.manual_owned;
  next+=std::chrono::milliseconds(10);std::this_thread::sleep_until(next);
 }
 std::cout<<"{\"navigation_ticks\":"<<nav<<",\"manual_ticks\":"<<manual<<",\"hardware_io\":false,\"synthetic_fixture\":true}\n";
}
