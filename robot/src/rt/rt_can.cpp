#include "rt/rt_can.h"
#include <chrono>
#include <thread>


CANCommand* can_cmd = new CANCommand;
CANData* can_data = new CANData;


// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

// only used for actual robot

// ABAD Q = [-0.313001, -0.289807, -1.046502, -0.634813]
// HIP  Q = [-1.037804, 0.214656, -1.327726, 0.110285]
// KNEE Q = [3.380598, -4.334211, 3.818379, -4.786488]

// ABAD Q = [-0.350690, -0.318799, 0.017510, -0.655108]
// HIP  Q = [-2.125010, 0.307431, -1.408904, 0.472686]
// KNEE Q = [3.882162, -4.171854, 3.890860, -4.241436]

// const float abad_offset[4] = {-0.310101f, -0.292706f, 0.0175f, -0.6348f};
// const float hip_offset[4]  = {-1.0378f, 0.2103f, -1.3277f,  0.3726f};
// const float knee_offset[4] = {3.9169f, -4.2085f, 3.8908f, -4.6212f};
/////

// ABAD Q = [-0.310101, -0.292706, -0.025978, -0.626116]
// HIP  Q = [-1.075494, 0.261044, -2.333753, 0.371214]
// KNEE Q = [3.876364, -4.226940, 3.856069, -4.580644]

// ABAD Q = [-0.339094, -0.289807, -0.014382, -0.666705]
// HIP  Q = [-1.205959, 0.243648, -1.362516, 0.434997]
// KNEE Q = [4.085107, -4.482070, 3.574846, -4.638628]


// const fl55oat abad_offset[4] = {-0.339094f, -0.289807f, -0.014382f, -0.666705f};
// const float hip_offset[4]  = {-1.105959f, 0.243648f, -2.362516f,  0.404997f};
// const float knee_offset[4] = {3.956364f, -4.532070f, 3.534846f, -4.638628f};

/////////// After robot collapsed
// ABAD Q = [-0.310101, -0.301404, -0.040475, -0.643511]
// HIP  Q = [-1.104486, 0.275540, -2.391737, 0.147974]
// KNEE Q = [3.940146, -4.569047, 3.574846, -5.627260]

/// new ofset Nov11
// ABAD Q = [-0.373884, -0.318799, 0.011711, -0.324598]
// HIP  Q = [-1.084192, 0.234951, -2.342451, 0.084192]
// KNEE Q = [3.914053, -4.528458, 3.612535, -5.531586]


const float abad_offset[4] = {-0.343884f, -0.318799f, -0.021711f, -0.350598f};
const float hip_offset[4]  = {-1.14192f, 0.294951f, -2.382451f,  0.15192f};
const float knee_offset[4] = {3.914053f, -4.528458f, 3.612535f, -5.531586f};  


const float max_torque[3] = {17.f, 17.f, 26.f};
const float wimp_torque[3] = {6.f, 6.f, 6.f};
const float disabled_torque[3] = {0.f, 0.f, 0.f};
float torque_out[12];


void CAN::init_can() {
  
  FR.enable(m4);
  FR.enable(m5);
  FR.enable(m6);
  printf("[IUSTHardware] Leg 1 is enabled!\n");

  FL.enable(m1);
  FL.enable(m2);
  FL.enable(m3);
  printf("[IUSTHardware] Leg 2 is enabled!\n");
  
  RR.enable(m10);
  RR.enable(m11);
  RR.enable(m12);
  printf("[IUSTHardware] Leg 3 is enabled!\n");
  
  RL.enable(m7);
  RL.enable(m8);
  RL.enable(m9);
  printf("[IUSTHardware] Leg 4 is enabled!\n");

  // Loop to initialize each motor command
  for (int i = 0; i < 4; i++) {
    // printf("just once???? \n");
    can_cmd->q_des_abad[i] = 0.0;
    can_cmd->q_des_hip[i] = 0.0;
    can_cmd->q_des_knee[i] = 0.0;

    can_cmd->qd_des_abad[i] = 0.0;
    can_cmd->qd_des_hip[i] = 0.0;
    can_cmd->qd_des_knee[i] = 0.0;

    can_cmd->kp_abad[i] = 0.0;
    can_cmd->kp_hip[i] = 0.0;
    can_cmd->kp_knee[i] = 0.0;

    can_cmd->kd_abad[i] = 0.0;
    can_cmd->kd_hip[i] = 0.0;
    can_cmd->kd_knee[i] = 0.0;

    can_cmd->tau_abad_ff[i] = 0.0;
    can_cmd->tau_hip_ff[i] = 0.0;
    can_cmd->tau_knee_ff[i] = 0.0;
  }

  // Loop to initialize each motor command
  for (int i = 0; i < 4; i++) {
    can_data->q_abad[i] = 0.0;
    can_data->q_hip[i] = 0.0;
    can_data->q_knee[i] = 0.0;
    can_data->qd_abad[i] = 0.0;
    can_data->qd_hip[i] = 0.0;
    can_data->qd_knee[i] = 0.0;
  }
}


void CAN::Torque_limit_checker(CANCommand* can_command, CANData* can_response){
  // printf("[RT CAN]: in safety\n");
  for (int i=0; i<4; i++){
   torque_out[3*i] =  can_command->kd_abad[i]*
    (can_command->q_des_abad[i]-can_response->q_abad[i])+can_command->kd_abad[i]*
    (can_command->qd_des_abad[i]-can_response->qd_abad[i])+can_command->tau_abad_ff[i];

    torque_out[3*i+1] =  can_command->kd_hip[i]*
    (can_command->q_des_hip[i]-can_response->q_hip[i])+can_command->kd_hip[i]*
    (can_command->qd_des_hip[i]-can_response->qd_hip[i])+can_command->tau_hip_ff[i];

    torque_out[3*i+2] =  can_command->kd_knee[i]*
    (can_command->q_des_knee[i]-can_response->q_knee[i])+can_command->kd_knee[i]*
    (can_command->qd_des_knee[i]-can_response->qd_knee[i])+can_command->tau_knee_ff[i];
  }
  for (int i = 0; i<4; i++){
    if ((torque_out[3*i]>max_torque[0]) || (torque_out[3*i]<-max_torque[0])){
      printf("[rt_can]: max limit happend at joint %d the torque value: %f!!!!!\n",3*i,torque_out[3*i]);
      // saftey_action_needed(can_command);
      stop_can();
      exit(0);
    }else if ((torque_out[3*i+1]>max_torque[1]) || (torque_out[3*i+1]<-max_torque[1])){
      printf("[rt_can]: max limit happend at joint %d the torque value: %f!!!!!\n",3*i+1,torque_out[3*i+1]);
      // saftey_action_needed(can_command);
      stop_can();
      exit(0);
    }else if((torque_out[3*i+2]>max_torque[2]) || torque_out[3*i+2]<-max_torque[2]){
      printf("[rt_can]: max limit happend at joint %d the torque value: %f!!!!!\n",3*i+2,torque_out[3*i+2]);
      // saftey_action_needed(can_command);
      stop_can();
      exit(0);
    }
  }
}

void CAN::can_send_receive(CANCommand* can_command, CANData* can_response) {

  // Torque_limit_checker(can_command, can_response);


  FR.command(m4, (can_command->q_des_abad[0] / abad_side_sign[0]) + abad_offset[0],can_command->qd_des_abad[0]/abad_side_sign[0],can_command->kp_abad[0],can_command->kd_abad[0],can_command->tau_abad_ff[0]/abad_side_sign[0]);
  FL.command(m1, (can_command->q_des_abad[1] / abad_side_sign[1]) + abad_offset[1],can_command->qd_des_abad[1]/abad_side_sign[1],can_command->kp_abad[1],can_command->kd_abad[1],can_command->tau_abad_ff[1]/abad_side_sign[1]);
  RR.command(m10, (can_command->q_des_abad[2] / abad_side_sign[2]) + abad_offset[2],can_command->qd_des_abad[2]/abad_side_sign[2],can_command->kp_abad[2],can_command->kd_abad[2],can_command->tau_abad_ff[2]/abad_side_sign[2]);
  RL.command(m7, (can_command->q_des_abad[3]/ abad_side_sign[3]) + abad_offset[3],can_command->qd_des_abad[3]/abad_side_sign[3],can_command->kp_abad[3],can_command->kd_abad[3],can_command->tau_abad_ff[3]/abad_side_sign[3]);

  FR.command(m5, (can_command->q_des_hip[0] / hip_side_sign[0]) + hip_offset[0],can_command->qd_des_hip[0]/ hip_side_sign[0],can_command->kp_hip[0],can_command->kd_hip[0],can_command->tau_hip_ff[0]/ hip_side_sign[0]);
  FL.command(m2, (can_command->q_des_hip[1] / hip_side_sign[1]) + hip_offset[1],can_command->qd_des_hip[1]/ hip_side_sign[1],can_command->kp_hip[1],can_command->kd_hip[1],can_command->tau_hip_ff[1]/ hip_side_sign[1]);
  RR.command(m11, (can_command->q_des_hip[2] / hip_side_sign[2]) + hip_offset[2],can_command->qd_des_hip[2]/ hip_side_sign[2],can_command->kp_hip[2],can_command->kd_hip[2],can_command->tau_hip_ff[2]/ hip_side_sign[2]);
  RL.command(m8, (can_command->q_des_hip[3]/ hip_side_sign[3]) + hip_offset[3],can_command->qd_des_hip[3]/ hip_side_sign[3],can_command->kp_hip[3],can_command->kd_hip[3],can_command->tau_hip_ff[3]/ hip_side_sign[3]);

  FR.command(m6, (can_command->q_des_knee[0] / knee_side_sign[0]) + knee_offset[0],can_command->qd_des_knee[0]/ knee_side_sign[0],can_command->kp_knee[0],can_command->kd_knee[0],can_command->tau_knee_ff[0]* knee_side_sign[0]);
  FL.command(m3, (can_command->q_des_knee[1] / knee_side_sign[1]) + knee_offset[1],can_command->qd_des_knee[1]/ knee_side_sign[1],can_command->kp_knee[1],can_command->kd_knee[1],can_command->tau_knee_ff[1]* knee_side_sign[1]);
  RR.command(m12, (can_command->q_des_knee[2] / knee_side_sign[2]) + knee_offset[2],can_command->qd_des_knee[2]/ knee_side_sign[2],can_command->kp_knee[2],can_command->kd_knee[2],can_command->tau_knee_ff[2]* knee_side_sign[2]);
  RL.command(m9, (can_command->q_des_knee[3]/ knee_side_sign[3]) + knee_offset[3],can_command->qd_des_knee[3]/ knee_side_sign[3],can_command->kp_knee[3],can_command->kd_knee[3],can_command->tau_knee_ff[3]* knee_side_sign[3]);

  // printf("Motor response :%f\n", RL.cycle_responses[2][1]);
  

  can_response->q_abad[0] = (FR.cycle_responses[0][1] - abad_offset[0]) * abad_side_sign[0];
  can_response->q_hip[0] = (FR.cycle_responses[1][1] - hip_offset[0]) * hip_side_sign[0];
  can_response->q_knee[0] = (FR.cycle_responses[2][1] - knee_offset[0]) * knee_side_sign[0];

  can_response->q_abad[1]= (FL.cycle_responses[0][1] - abad_offset[1]) * abad_side_sign[1];
  can_response->q_hip[1]= (FL.cycle_responses[1][1] - hip_offset[1]) * hip_side_sign[1];
  can_response->q_knee[1]= (FL.cycle_responses[2][1] - knee_offset[1]) * knee_side_sign[1];

  can_response->q_abad[2]= (RR.cycle_responses[0][1] - abad_offset[2]) * abad_side_sign[2];
  can_response->q_hip[2]= (RR.cycle_responses[1][1] - hip_offset[2]) * hip_side_sign[2];
  can_response->q_knee[2]= (RR.cycle_responses[2][1] - knee_offset[2]) * knee_side_sign[2];


  can_response->q_abad[3] = (RL.cycle_responses[0][1] - abad_offset[3]) * abad_side_sign[3];
  can_response->q_hip[3] = (RL.cycle_responses[1][1] - hip_offset[3]) * hip_side_sign[3];
  can_response->q_knee[3] = (RL.cycle_responses[2][1] - knee_offset[3]) * knee_side_sign[3];

  ////// For offset calculation uncomment below:
  // printf("ABAD Q = [%f, %f, %f, %f]\n", FR.cycle_responses[0][1] , FL.cycle_responses[0][1], RR.cycle_responses[0][1],RL.cycle_responses[0][1]);
  // printf("HIP  Q = [%f, %f, %f, %f]\n", FR.cycle_responses[1][1] , FL.cycle_responses[1][1], RR.cycle_responses[1][1],RL.cycle_responses[1][1]);
  // printf("KNEE Q = [%f, %f, %f, %f]\n", FR.cycle_responses[2][1] , FL.cycle_responses[2][1], RR.cycle_responses[2][1],RL.cycle_responses[2][1]);
  
  
  // std::cout<<"new data:"<<std::endl;
  // // std::cout<<FR.cycle_responses[1][1]<<std::endl;
  // std::cout<<RL.cycle_responses[0][1]<<std::endl;
  // std::cout<<RL.cycle_responses[1][1]<<std::endl;
  // std::cout<<RR.cycle_responses[1][1]<<std::endl;
  // // std::cout<<RL.cycle_responses[1][1]<<std::endl;
  // std::cout<<"end of new data"<<std::endl;

  can_response->qd_abad[0] = FR.cycle_responses[0][2]* abad_side_sign[0];
  can_response->qd_hip[0] = FR.cycle_responses[1][2] * hip_side_sign[0];
  can_response->qd_knee[0] = FR.cycle_responses[2][2]* knee_side_sign[0];

  can_response->qd_abad[1] = FL.cycle_responses[0][2]* abad_side_sign[1];
  can_response->qd_hip[1] = FL.cycle_responses[1][2] * hip_side_sign[1];
  can_response->qd_knee[1] = FL.cycle_responses[2][2]* knee_side_sign[1]; 

  can_response->qd_abad[2] = RR.cycle_responses[0][2]* abad_side_sign[2];
  can_response->qd_hip[2] = RR.cycle_responses[1][2] * hip_side_sign[2];
  can_response->qd_knee[2] = RR.cycle_responses[2][2]* knee_side_sign[2];

  can_response->qd_abad[3] = RL.cycle_responses[0][2]* abad_side_sign[3];
  can_response->qd_hip[3] = RL.cycle_responses[1][2] * hip_side_sign[3];
  can_response->qd_knee[3] = RL.cycle_responses[2][2]* knee_side_sign[3];
  
  // printf("----------------------------------Torques------------------------------------\n");
  // printf("ABAD Q = [%f, %f, %f, %f]\n", FR.cycle_responses[0][3] , FL.cycle_responses[0][3], RR.cycle_responses[0][3],RL.cycle_responses[0][3]);
  // printf("HIP  Q = [%f, %f, %f, %f]\n", FR.cycle_responses[1][3] , FL.cycle_responses[1][3], RR.cycle_responses[1][3],RL.cycle_responses[1][3]);
  // printf("KNEE Q = [%f, %f, %f, %f]\n", FR.cycle_responses[2][3] , FL.cycle_responses[2][3], RR.cycle_responses[2][3],RL.cycle_responses[2][3]);
  // printf("-----------------------------------------------------------------------------\n");



  can_cmd = can_command;
  can_data = can_response;
  // std::cout<<can_data->q_abad[0]<<std::endl;
  // std::cout<<"---------"<<std::endl;
  // std::cout<<can_response->q_abad[0]<<std::endl;

}

void CAN::check_safety() {

    // if()
        // stop_can();
}
void CAN::saftey_action_needed(CANCommand* can_command){
  for (int i=0; i<4; i++){
    can_command->kp_abad[i] = 0.0;
    can_command->kp_hip[i] = 0.0;
    can_command->kp_knee[i] = 0.0;

    can_command->kd_abad[i] = 0.0;
    can_command->kd_hip[i] = 0.0;
    can_command->kd_knee[i] = 0.0;

    can_command->tau_abad_ff[i] = max_torque[0];
    can_command->tau_hip_ff[i] = max_torque[1];
    can_command->tau_knee_ff[i] = max_torque[3];
  }

}
void CAN::stop_can() {

  FR.disable(m4);
  FR.disable(m5);
  FR.disable(m6);
  
  FL.disable(m1);
  FL.disable(m2);
  FL.disable(m3);
  
  RR.disable(m10);
  RR.disable(m11);
  RR.disable(m12);
  
  RL.disable(m7);
  RL.disable(m8);
  RL.disable(m9);

}


CANCommand* CAN::get_can_command() {
  // can_cmd = new actuator_command_t[12];
  


  // int m_ids[12] = {m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12};


  return can_cmd;
}

CANData* CAN::get_can_data() {
  // can_data = new actuator_response_t[12];

  // int m_ids[12] = {m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12};



  // can_data->id = m1;
  return can_data;
}