#include <cassert>
#include <cmath>
#include <limits>
#include "../navigation/p2_nav_controller/include/p2_nav_controller/terminal_heading.hpp"
#include "../navigation/p2_nav_controller/include/p2_nav_controller/terminal_path.hpp"
#include "../navigation/p2_nav_controller/include/p2_nav_controller/terminal_velocity.hpp"
int main() {
  for(int k=-60;k<=60;++k) {
    const std::array<double,3> start{1.,2.,k*.1},goal{1.1,2.2,-k*.1};
    auto path=p2_nav_controller::terminal_path(start,goal);assert(!path.empty());
    assert(std::hypot(path.back()[0]-goal[0],path.back()[1]-goal[1])<1e-10);
    assert(std::abs(std::remainder(path.back()[2]-goal[2],2*std::acos(-1.)))<1e-10);
    for(size_t i=1;i<path.size();++i) {
      const auto &a=path[i-1];const auto &b=path[i];
      const double dx=b[0]-a[0],dy=b[1]-a[1];
      assert(std::hypot(dx,dy)<=.02000001);
      assert(std::abs(std::remainder(b[2]-a[2],2*std::acos(-1.)))<=.02000001);
      assert(dx*std::cos(a[2])+dy*std::sin(a[2])>=-1e-10);
      assert(std::abs(-dx*std::sin(a[2])+dy*std::cos(a[2]))<1e-10);
    }
  }
  assert(p2_nav_controller::terminal_path({0,0,0},{.31,0,0}).empty());
  assert(!p2_nav_controller::terminal_path({0,0,0},{0,0,1}).empty());
  p2_nav_controller::TerminalHeading p;
  p.set_goal(1,2,0,"map"); assert(!p.step(.2,.15,0)); assert(p.step(.14,.15,.05));
  p.set_goal(1,2,0,"map"); assert(p.step(.18,.15,.10)); // Same-goal replan.
  assert(!p.step(.31,.15,.15)); // Reacquire after substantial drift.
  assert(p.step(.1,.15,.20)); assert(!p.step(.2,.15,.8)); // No latch after a control gap.
  assert(p.step(.1,.15,.85)); p.set_goal(1,2,1,"map"); assert(!p.step(.2,.15,.9));
  assert(p.step(.1,.15,.95)); p.set_goal(1,2,1,"odom"); assert(!p.step(.2,.15,1.));
  assert(p.step(.1,.15,1.05)); assert(!p.step(.2,.15,.5)); // Clock rollback.
  bool rejected=false;
  try { p.step(std::numeric_limits<double>::quiet_NaN(),.15,1.1); } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected); assert(!p.step(.2,.15,1.15));
  assert(p.step(.1,.15,1.20)); assert(!p.step(.18,.15,1.25,true)); // Correct residual XY after alignment.
  const auto forward=p2_nav_controller::terminal_velocity(.2,0,2,false,.24);
  assert(forward[0]>.0 && forward[0]<=.1 && forward[1]==0.);
  const auto pivot=p2_nav_controller::terminal_velocity(.2,3.,0,false,.24);
  assert(pivot[0]==0. && std::abs(pivot[1])<=.24);
  const auto final=p2_nav_controller::terminal_velocity(.1,0,.1,true,.24);
  assert(final[0]==0. && std::abs(final[1]-.08)<1e-10);
  for (int i=-400;i<=400;++i) {
    const auto c=p2_nav_controller::terminal_velocity(.2,i*.05,0,false,.24);
    assert(c[0]>=0. && c[0]<=.1 && std::abs(c[1])<=.24);
    if (std::abs(std::remainder(i*.05,2.*std::acos(-1.)))>=.15)assert(c[0]==0.);
  }
}
