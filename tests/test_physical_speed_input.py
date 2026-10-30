"""Compile and exercise actual C++ lease state transitions; no robot interfaces."""
import pathlib,shutil,subprocess,tempfile,unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "physical_speed_input.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace p2_nav;
constexpr std::uint64_t base=10000000000ULL;
GaitFeedback mode(std::uint64_t now=base){
 GaitFeedback f;f.monotonic_ns=now;f.controller_instance=10;f.policy_generation=7;
 f.operator_mode=OperatorMode::navigation_allowed;f.operator_mode_verified=true;
 f.supported_policy=f.walking_ready=f.operator_connected=f.healthy=f.calibrated=f.ground_ok=f.localized=true;return f;
}
Packet packet(std::uint64_t seq,float vx=0.f,float vy=0.f,float wz=0.f,std::uint64_t now=base){return {0x504e4156,1,1,11,seq,now,vx,vy,wz,0};}
void active(PhysicalSpeedInput& g){assert(g.arm(mode(),base));assert(g.accept(packet(1),48,mode(),base));assert(g.accept(packet(2,.15f,0,.24f),48,mode(),base));assert(g.poll(mode(),base).navigation_owns_velocity);}
int main(){
 PhysicalSpeedInput g;assert(!g.poll(mode(),base).navigation_owns_velocity);
 active(g);auto d=g.poll(mode(),base);assert(d.vx>0.149&&d.vy==0&&d.wz>0.239);
 // Manual input wins on the very next tick without waiting for another packet.
 auto f=mode(base+1000000);f.manual_activity=true;d=g.poll(f,base+1000000);
 assert(!d.navigation_owns_velocity&&d.cancel_navigation_goal&&!g.armed());
 assert(!g.poll(mode(base+2000000),base+2000000).navigation_owns_velocity);
 assert(!g.accept(packet(3,.1f,0,0,base+2000000),48,mode(base+2000000),base+2000000));
 assert(g.poll(mode(base+2000000),base+2000000).cancel_navigation_goal);
 assert(!g.acknowledge_cancel(d.cancel_generation+1));assert(g.acknowledge_cancel(d.cancel_generation));
 // Explicit restart requires a new zero handshake, not the old moving command.
 assert(g.arm(mode(base+3000000),base+3000000));
 assert(!g.accept(packet(4,.1f,0,0,base+3000000),48,mode(base+3000000),base+3000000));
 PhysicalSpeedInput timeout;active(timeout);
 // Packet silence expires at300ms with fresh gait feedback, then stays revoked.
 d=timeout.poll(mode(base+300000001),base+300000001);assert(!d.navigation_owns_velocity&&d.cancel_navigation_goal);
 assert(!timeout.accept(packet(3,.1f,0,0,base+300000002),48,mode(base+300000002),base+300000002));
 // Arrival before poll cannot conceal a gap in the command lease.
 PhysicalSpeedInput gap;active(gap);assert(!gap.accept(packet(3,.1f,0,0,base+300000001),48,mode(base+300000001),base+300000001));
 // Do not retain accepted motion after stale controller feedback.
 PhysicalSpeedInput stale;active(stale);assert(!stale.poll(mode(),base+100000001).navigation_owns_velocity);
 // A fresh navigation packet never refreshes stale gait telemetry.
 PhysicalSpeedInput fresh_packet;active(fresh_packet);assert(!fresh_packet.accept(packet(3,.1f,0,0,base+110000000),48,mode(),base+110000000));
 for(int i=0;i<9;++i){
  PhysicalSpeedInput q;active(q);auto m=mode(base+1000);
  if(i==0)m.estop=true;if(i==1)m.operator_connected=false;if(i==2)m.supported_policy=false;
  if(i==3)m.walking_ready=false;if(i==4)m.healthy=false;if(i==5)m.calibrated=false;
  if(i==6)m.ground_ok=false;if(i==7)m.localized=false;if(i==8)++m.policy_generation;
  assert(!q.poll(m,base+1000).navigation_owns_velocity&&!q.armed());
 }
 for(int i=0;i<12;++i){
  PhysicalSpeedInput q;active(q);auto p=packet(3,.1f);
  if(i==0)p.vy=.001f;if(i==1)p.vx=-.01f;if(i==2)p.vx=.150001f;if(i==3)p.wz=.240001f;
  if(i==4)p.vx=std::numeric_limits<float>::quiet_NaN();if(i==5)p.monotonic_ns=base-300000001;
  if(i==6)p.monotonic_ns=base+5000001;if(i==7)++p.instance;if(i==8)p.sequence=2;
  if(i==9)p.reserved=1;
  if(i==11)p.monotonic_ns=base-1;
  assert(!q.accept(p,i==10?47:48,mode(),base)&&!q.armed());
 }
 PhysicalSpeedInput restart;active(restart);auto m=mode(base+1000);m.controller_instance++;
 assert(!restart.poll(m,base+1000).navigation_owns_velocity);
 PhysicalSpeedInput rollback;active(rollback);assert(!rollback.poll(mode(base-1),base-1).navigation_owns_velocity);
 PhysicalSpeedInput pending;active(pending);pending.disarm();assert(!pending.arm(mode(),base));
 auto cancel=pending.poll(mode(),base);assert(cancel.cancel_navigation_goal);assert(!pending.arm(mode(),base));
 assert(pending.acknowledge_cancel(cancel.cancel_generation));assert(pending.arm(mode(),base));
 PhysicalSpeedInput sw;active(sw);auto sf=mode(base+1000);sf.operator_mode=OperatorMode::manual;
 auto manual=sw.poll(sf,base+1000);assert(manual.manual_velocity_allowed&&!manual.force_zero_velocity&&!manual.navigation_owns_velocity&&manual.cancel_navigation_goal);
 sf.operator_mode=OperatorMode::paused;auto paused=sw.poll(sf,base+2000);assert(paused.force_zero_velocity&&!paused.manual_velocity_allowed);
 sf.operator_mode=OperatorMode::navigation_allowed;assert(!sw.poll(sf,base+3000).navigation_owns_velocity);
 PhysicalSpeedInput unknown;auto uf=mode();uf.operator_mode_verified=false;assert(!unknown.arm(uf,base));
 std::cout<<"manual handoff, rearm, speed bounds, authority loss, source identity, telemetry and command expiry passed\n";
}
'''

class PhysicalSpeedInputTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
    def test_cpp_authority_transitions(self):
        with tempfile.TemporaryDirectory() as folder:
            p=pathlib.Path(folder);(p/'test.cpp').write_text(SOURCE)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation'),str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True,text=True)
            result=subprocess.run([str(p/'test')],check=True,capture_output=True,text=True)
            self.assertIn('manual handoff',result.stdout)

if __name__=='__main__':unittest.main()
