#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace p2_nav_controller {
// Experimental pure-pivot response; no hardware I/O and no fake measurement.
// Feedback belongs to the caller's actual measured yaw and angular velocity.
class SettledPivot {
  bool active_=false,coasting_=false,near_pulse_=false;
  double last_=-1.,start_=0.,end_=0.,quiet_=0.,from_=0.,peak_=0.;
  double command_=0.,duration_=1.1,near_level_=.5,near_duration_=1.1,near_delta_=0.,near_peak_=0.;
  double far_duration_=2.,far_delta_=0.,far_peak_=0.;
 public:
  void reset(){*this=SettledPivot();}
  bool pending()const{return active_||coasting_;}
  // A tracking correction can become clear before its bounded pulse ends.
  // Withdraw the request, then observe the same measured coast-down before
  // releasing translation. Do not erase the response still queued downstream.
  void settle(double now) {
    if(!std::isfinite(now))throw std::invalid_argument("Invalid pivot stop time");
    if(active_){active_=false;coasting_=true;end_=now;quiet_=0.;}
  }
  double step(double error,double yaw,double rate,double now,double tolerance=.15) {
    if(!std::isfinite(error)||!std::isfinite(yaw)||!std::isfinite(rate)||!std::isfinite(now)||!std::isfinite(tolerance)||tolerance<=0.)
      throw std::invalid_argument("Invalid measured pivot state");
    if(last_>=0.&&now<last_)reset();
    else if(last_>=0.&&now-last_>.3) {
      // Drop the old pulse/pose/rate evidence after a callback gap, while
      // retaining the bounded input already found necessary for THIS goal.
      // Otherwise repeated gaps continually retry an ineffective .5 pulse.
      const double learned_level=near_level_;reset();near_level_=learned_level;
      if(near_level_>=1.)near_duration_=2.;
    }
    const double dt=last_<0.?0.:now-last_;last_=now;
    if(coasting_) {
      peak_=std::max(peak_,std::abs(rate));
      quiet_=std::abs(rate)<.03?quiet_+dt:0.;
      // Once an actual yaw response has been observed, measured quiet is the
      // completion evidence. Keep the original two-second wait for a pulse that
      // has not yet produced measurable motion (including delayed/deadzone cases).
      if(quiet_<.3||(peak_<.03&&now-end_<2.))return 0.;
      coasting_=false;
      if(near_pulse_) {
        near_delta_=std::max(0.,std::remainder(yaw-from_,2.*std::acos(-1.))*std::copysign(1.,command_));
        near_peak_=peak_;near_duration_=duration_;
        // A fresh but motionless response may still be below the selected
        // multiplier/deadzone. Retry at the existing1.0 raw-input maximum;
        // the operator multiplier and native ramp remain unchanged.
        // Brief rate spikes from a standing biped do not establish useful
        // rotation. Once the pulse has settled, use the observed heading
        // displacement to detect a motionless response. The existing full
        // input retry remains bounded by the normal pulse, ramp and checks.
        if(near_delta_<.02||(near_peak_<.06&&near_delta_<.06)){near_duration_=near_level_<1.?2.:std::clamp(duration_+.2,1.1,2.5);near_level_=1.;near_peak_=0.;}
      }else {far_delta_=std::abs(std::remainder(yaw-from_,2.*std::acos(-1.)));far_peak_=peak_;far_duration_=duration_;}
    }
    if(!active_&&std::abs(error)<=tolerance)return 0.;
    if(!active_) {
      from_=yaw;start_=now;peak_=std::abs(rate);active_=true;
      const bool far=std::abs(error)>1.2;
      near_pulse_=!far;command_=std::copysign(far?1.:near_level_,error);
      const double far_desired=std::min(.9,std::max(.3,std::abs(error)-.7));
      // A first large turn has not learned this plant's ramp/delay yet. Use
      // the three-and-a-half-second pulse bound so it need not stop and restart
      // the input ramp while the heading is still far away. Measured-rate
      // braking, crossing detection and the near-turn behavior stay unchanged.
      duration_=far_peak_>.03?std::clamp(far_duration_+(far_desired-far_delta_)/far_peak_,1.4,3.5):3.5;
      if(!far) {
        const double desired=std::min(.5,std::max(.04,std::abs(error)-.07));
        duration_=near_peak_>.03?std::clamp(near_duration_+(desired-near_delta_)/near_peak_,.95,2.5):near_duration_;
      }
    }
    peak_=std::max(peak_,std::abs(rate));
    const bool crossed=error*command_<=0.;
    const bool far_brake=!near_pulse_&&std::abs(error)<std::max(.8,1.8*std::abs(rate));
    if(crossed||far_brake||now-start_>=duration_) {
      active_=false;coasting_=true;end_=now;quiet_=0.;return 0.;
    }
    return command_;
  }
};
}
