#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_BalanceStand : public FSM_State<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter() override;

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

 private:
  // Keep track of the control iterations
  int _iter = 0;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();

  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  T last_height_command = 0;

  Vec3<T> _ini_body_pos;
  Vec3<T> balancetest_rpy;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;

  // Blance  Limitation
  T const rollLimit = 0.523; // 30degrees
  T const pitchLimit = 0.349;
  T const yawLimit = 0.249; //20degrees
  T const maxLegLength = 0.644;
  T const heightLimitUpper = 0.85 * maxLegLength;
  T const heightLimitLower = 0.3 * maxLegLength;
  float zstart = 0.0;
  float zMin = -0.5;
  float zMax = 0.5;
  bool zmove = false;
  bool rollmove = false;
  bool pitchmove = false;
  bool yawmove = false;
  float heightchange1 = 0.0;
  float heiightchange2 = 0.0;
  bool up1 = true;
  bool down = false;
  bool up2 = false;
  float lastheight;
  float lastroll;
  float lastpitch;
  float lastyaw;

};

#endif  // FSM_STATE_BALANCESTAND_H
