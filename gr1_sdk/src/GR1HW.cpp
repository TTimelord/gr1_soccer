#include "gr1_hardware/GR1HW.h"
#include <string>
#include <nlohmann/json.hpp>  // Include the JSON library header
#include <fstream>            // For file operations
#include <chrono>
#include <thread>
#include <FsaConfig.h>

// #define ESTIMATION_ONLY
// #define TIMER


bool GR1HW::init() {
  default_joint_pos.setZero(TOTAL_JOINT_NUM+3); // leg + waist
  imu.initIMU();

  // ================== set PID params ======================
  ifstream ifs(motorlistFile);
  // Parse the JSON data
  nlohmann::json j;
  ifs >> j;

  // Populate the vectors according to the order in ip_list
  for (const string& ip : ip_list) {
      if (j.find(ip) != j.end()) {
          ratio.push_back(j[ip]["motor_gear_ratio"]);
          scale.push_back(j[ip]["c_t_scale"]);
          absolute_pos_zero.push_back(j[ip]["absolute_pos_zero"]);
          absolute_pos_dir.push_back(j[ip]["absolute_pos_dir"]);
          absolute_pos_ratio.push_back(j[ip]["absolute_pos_gear_ratio"]);
          motor_dir.push_back(j[ip]["motorDir"]);
          pos_gain.push_back(j[ip]["controlConfig"]["pos_gain"]);
          vel_gain.push_back(j[ip]["controlConfig"]["vel_gain"]);
          vel_integrator_gain.push_back(j[ip]["controlConfig"]["vel_integrator_gain"]);
      } else {
          // If the IP is not found in the JSON
          exit(1);
      }
  }

  std::vector<double> arm_head_pos_gain; 
  std::vector<double> arm_head_vel_gain;
  // get pid for arm and head
  for (const string& ip : arm_and_head_ip_list) {
      if (j.find(ip) != j.end()) {
          arm_head_pos_gain.push_back(j[ip]["controlConfig"]["pos_gain"]);
          arm_head_vel_gain.push_back(j[ip]["controlConfig"]["vel_gain"]);
      } else {
          // If the IP is not found in the JSON
          exit(1);
      }
  }

  FSA_CONNECT::FSAConfig::FSAPIDParams pid_params;
  FSA_CONNECT::FSAConfig::FSAPIDParams get_pid_params;

  int ret = 0;
  for (int i = 0; i < TOTAL_JOINT_NUM + 3; i++) { //consider legs + waist
      fsa_list[i].init(ip_list[i]);
      pid_params.control_position_kp = pos_gain[i];
      pid_params.control_velocity_kp = vel_gain[i];
      std::cout<<"write kp: "<<pid_params.control_position_kp << "kd:" << pid_params.control_velocity_kp << std::endl;
      fsa_list[i].SetPIDParams(pid_params);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); // read PVC for the first time to get rid of errors
      fsa_list[i].GetPIDParams(get_pid_params);
      std::cout<<"read kp: "<<get_pid_params.control_position_kp << "kd:" << get_pid_params.control_velocity_kp << std::endl;
      last_cmd_pos[i] = read_joint_pos[i];
      last_cmd_cur[i] = 0;
      last_cmd_vel[i] = 0;
  }

  for (int i = 0; i < arm_and_head_ip_list.size(); i++) { // arm and head
      arm_and_head_fsa_list[i].init(arm_and_head_ip_list[i]);
      pid_params.control_position_kp = arm_head_pos_gain[i];
      pid_params.control_velocity_kp = arm_head_vel_gain[i];
      std::cout<<"write kp: "<<pid_params.control_position_kp << "kd:" << pid_params.control_velocity_kp << std::endl;
      arm_and_head_fsa_list[i].SetPIDParams(pid_params);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      arm_and_head_fsa_list[i].GetPIDParams(get_pid_params);
      std::cout<<"read kp: "<<get_pid_params.control_position_kp << "kd:" << get_pid_params.control_velocity_kp << std::endl;
  }
  // ================== END set PID params ======================


  if(!calculate_offset()){
    exit(1);
  }

  #ifndef ESTIMATION_ONLY
  for (int i = 0; i < TOTAL_JOINT_NUM+3; i++) {
      ret = fsa_list[i].Enable();
      if (ret < 0) {
          std::cout << "wrong" << std::endl;
          exit(0);
      }
  }

  for (int i = 0; i < TOTAL_JOINT_NUM+3; i++) {
      fsa_list[i].EnablePosControl();
  }

  for (int i = 0; i<arm_and_head_ip_list.size(); i++) {
    arm_and_head_fsa_list[i].Enable();
    arm_and_head_fsa_list[i].EnablePosControl();
  }
  #endif

  return true;
}    

bool GR1HW::calculate_offset(){
  rapidjson::Document msg_json;
  char ser_msg[1024] = {0};
  for (int i = 0; i < TOTAL_JOINT_NUM + 3; i++) {   // consider waist joints;
    fse.demo_get_measured(ae_ip_list[i], NULL, ser_msg);
    fsa_list[i].GetPVC(read_joint_pos[i], read_joint_vel[i], read_joint_torq[i]); 
    if (msg_json.Parse(ser_msg).HasParseError())
    {
        std::cout<<"fi_decode()"<<i<<"failed\n";
        return 0;
    }
    double ae_current = msg_json["radian"].GetDouble();
    pos_offset[i] = ae_current - absolute_pos_zero[i];
    // std::cout<<i<<"======="<<std::endl;
    // std::cout<<ae_current<<std::endl;
    // std::cout<<absolute_pos_zero[i]<<std::endl;

    while(pos_offset[i]<-PI){
      pos_offset[i] += 2*PI;
    }
    while(pos_offset[i]>PI){
      pos_offset[i] -= 2*PI;
    }
    // std::cout<<pos_offset[i]<<std::endl;
    pos_offset[i] = absolute_pos_dir[i]*pos_offset[i]/absolute_pos_ratio[i] - motor_dir[i]*read_joint_pos[i]*PI/180;
  }
  return true;
}

std::vector<double> GR1HW::read(){
  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    fsa_list[i].GetPVC(current_motor_pos[i], current_motor_vel[i], current_motor_cur[i]); // TODO: current to tau conversion required
    read_joint_pos[i] = motor_dir[i] * (PI / 180 * current_motor_pos[i]) + pos_offset[i];
    read_joint_vel[i] = motor_dir[i] * PI / 180 * current_motor_vel[i];
    std::cerr<<"read motor "<<i<<" pos: "<< current_motor_pos[i] << "vel: "<< current_motor_vel[i] << "torq:" << read_joint_torq[i] <<"\n";
  }

  parallel_to_serial();

  Eigen::Quaterniond quaternion = Eigen::AngleAxisd(imu.imudata(0), Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(imu.imudata(1), Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(imu.imudata(2), Eigen::Vector3d::UnitX());
  
  sensor_data[0] = quaternion.coeffs()(0);
  sensor_data[1] = quaternion.coeffs()(1);
  sensor_data[2] = quaternion.coeffs()(2);
  sensor_data[3] = quaternion.coeffs()(3);
  sensor_data[4] = imu.imudata(3);
  sensor_data[5] = imu.imudata(4);
  sensor_data[6] = imu.imudata(5);
  sensor_data[7] = imu.imudata(6);
  sensor_data[8] = imu.imudata(7);
  sensor_data[9] = imu.imudata(8);

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    sensor_data[i + 10] = read_joint_pos[i];
    sensor_data[i + 10 + 12] = read_joint_vel[i];
    sensor_data[i + 10 + 24] = read_joint_torq[i];
  }
  return sensor_data;
}

void GR1HW::write_tor(std::vector<double> target_torque) {

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_torq[i] = target_torque[i];
  }

  serial_to_parallel();

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_current[i] = torque_to_current(i, write_joint_torq[i]);
    double filtered_current = (1 - cur_lpf_ratio) * last_cmd_cur[i] + cur_lpf_ratio * write_joint_current[i];
    last_cmd_cur[i] = filtered_current;
    fsa_list[i].SetCurrent(filtered_current);
  }
}

void GR1HW::write_pos(std::vector<double> target_position) {

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_pos[i] = target_position[i];
    write_joint_vel[i] = 0;
    write_joint_torq[i] = 0;
  }

  serial_to_parallel();

  for (int i = 0; i < TOTAL_JOINT_NUM; ++i) {
    write_joint_pos[i] = (write_joint_pos[i] - pos_offset[i]) * 180/PI * motor_dir[i];
    double filtered_pos = (1 - pos_lpf_ratio) * last_cmd_pos[i] + pos_lpf_ratio * write_joint_pos[i];
    last_cmd_pos[i] = filtered_pos;
    if (filtered_pos - current_motor_pos[i] > 30 || filtered_pos - current_motor_pos[i] < - 30){
      disable_all_motors();
      exit(1);
    }
    fsa_list[i].SetPosition(filtered_pos, 0.0, 0.0);
  }
}

bool GR1HW::disable_all_motors(){

  for(int i=0; i<fsa_list.size(); i++){
    fsa_list[i].Disable();
  }

  for(int i=0; i<arm_and_head_fsa_list.size(); i++){
    arm_and_head_fsa_list[i].Disable();
  }

  return true;
}

bool GR1HW::parallel_to_serial(){
  Eigen::VectorXd motorPos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorTorq = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd anklePose = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleTorq = Eigen::VectorXd::Zero(4);

  motorPos << read_joint_pos[4], read_joint_pos[5], read_joint_pos[10], read_joint_pos[11];
  motorVel << read_joint_vel[4], read_joint_vel[5], read_joint_vel[10], read_joint_vel[11];
  motorTorq << read_joint_torq[4], read_joint_torq[5], read_joint_torq[10], read_joint_torq[11];

  funS2P.setEst(motorPos, motorVel, motorTorq);
  funS2P.calcFK();
  funS2P.getAnkleState(anklePose, ankleVel, ankleTorq);

  read_joint_pos[4] = anklePose[0];
  read_joint_pos[5] = anklePose[1];
  read_joint_pos[10] = anklePose[2];
  read_joint_pos[11] = anklePose[3];

  read_joint_vel[4] = ankleVel[0];
  read_joint_vel[5] = ankleVel[1];
  read_joint_vel[10] = ankleVel[2];
  read_joint_vel[11] = ankleVel[3];

  read_joint_torq[4] = ankleTorq[0];
  read_joint_torq[5] = ankleTorq[1];
  read_joint_torq[10] = ankleTorq[2];
  read_joint_torq[11] = ankleTorq[3];

  return true;
}

bool GR1HW::serial_to_parallel(){
  Eigen::VectorXd motorPos = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd motorTorq = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd anklePose = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleVel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd ankleTorq = Eigen::VectorXd::Zero(4);

  anklePose << write_joint_pos[4], write_joint_pos[5], write_joint_pos[10], write_joint_pos[11];
  ankleVel << write_joint_vel[4], write_joint_vel[5], write_joint_vel[10], write_joint_vel[11];
  ankleTorq << write_joint_torq[4], write_joint_torq[5], write_joint_torq[10], write_joint_torq[11];

  funS2P.setRef(anklePose, ankleVel, ankleTorq);
  funS2P.calcIK();
  funS2P.getMotorCmd(motorPos, motorVel, motorTorq);

  write_joint_pos[4] = motorPos[0];
  write_joint_pos[5] = motorPos[1];
  write_joint_pos[10] = motorPos[2];
  write_joint_pos[11] = motorPos[3];

  write_joint_vel[4] = motorVel[0];
  write_joint_vel[5] = motorVel[1];
  write_joint_vel[10] = motorVel[2];
  write_joint_vel[11] = motorVel[3];

  write_joint_torq[4] = motorTorq[0];
  write_joint_torq[5] = motorTorq[1];
  write_joint_torq[10] = motorTorq[2];
  write_joint_torq[11] = motorTorq[3]; 
  return true;
}

double GR1HW::torque_to_current(int index, double torque){
  double current = torque * motor_dir[index]/(ratio[index]*scale[index]);
  return current;
}

double GR1HW::current_to_torque(int index, double current){
  double torque = current * (ratio[index]*scale[index]) * motor_dir[index];
  return torque;
}


GR1HW:: ~GR1HW(){
}


