#include "gait_navigation_server.hpp"
#include <iostream>
int main(int argc,char**argv){
 if(argc!=3)return 2;
 try{
  p2_nav::GaitNavigationServer server;server.start(argv[1],static_cast<uid_t>(std::stoul(argv[2])));
  for(int i=0;i<400;++i){p2_nav::NativeGaitState n;
   n.monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
   n.controller=1;n.policy_generation=1;server.step(n);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 3;}
}
