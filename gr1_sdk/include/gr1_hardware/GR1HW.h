#pragma once
#include <iostream>
#include <ch108IMU/ch108IMU.h>
#include <parallel_ankle/parallel_ankle.h>
#include <Fsa.h>
#include <main.h>
#include <Eigen/Dense>
#include <pybind11/pybind11.h>
namespace py = pybind11;

#define TOTAL_JOINT_NUM 12  // total number of joints for legs
#define JOINT_NUM 6 // number of joints per leg
#define PI 3.141592

// "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" 
// fr fl rr rl


struct MotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct ImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};


class GR1HW 
{
  public:
  bool init() ;
  void read();
  void write();
  ~GR1HW();

private:

  bool setupImu();

  bool calculate_offset();
  bool go_to_default_pos();

  bool serial_to_parallel();
  bool parallel_to_serial();

  double torque_to_current(int index, double torque);
  double current_to_torque(int index, double torque);

  bool disable_all_motors();


  MotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  ImuData imuData_{};
  
  std::string referenceFile;
  std::string motorlistFile;

  ch108::ch108IMU imu;
  parallel_ankle funS2P;

  std::vector<FSA_CONNECT::FSA> fsa_list = std::vector<FSA_CONNECT::FSA>(TOTAL_JOINT_NUM+3);
  std::vector<FSA_CONNECT::FSA> arm_and_head_fsa_list = std::vector<FSA_CONNECT::FSA>(3+7+7);

  const std::vector<std::string> ip_list = {"192.168.137.70", "192.168.137.71","192.168.137.72","192.168.137.73","192.168.137.74","192.168.137.75",
                                    "192.168.137.50", "192.168.137.51","192.168.137.52","192.168.137.53","192.168.137.54","192.168.137.55",
                                    "192.168.137.90", "192.168.137.91", "192.168.137.92"};  // L_leg, R_leg, waist
  
  const std::vector<std::string> arm_and_head_ip_list = {
        // "192.168.137.93", "192.168.137.94","192.168.137.95",
        "192.168.137.10", "192.168.137.11","192.168.137.12","192.168.137.13","192.168.137.14","192.168.137.15","192.168.137.16",
        "192.168.137.30", "192.168.137.31","192.168.137.32","192.168.137.33","192.168.137.34","192.168.137.35","192.168.137.36",
    };

  Eigen::VectorXd default_joint_pos;

  Sensor::FSE fse;
  const std::vector<std::string> ae_ip_list = {"192.168.137.170", "192.168.137.171","192.168.137.172","192.168.137.173","192.168.137.174","192.168.137.175",
                                    "192.168.137.150", "192.168.137.151","192.168.137.152","192.168.137.153","192.168.137.154","192.168.137.155",
                                    "192.168.137.190", "192.168.137.191", "192.168.137.192"};  


  std::vector<double> ratio;
  std::vector<double> scale;
  std::vector<double> absolute_pos_zero;                         
  std::vector<double> absolute_pos_dir;
  std::vector<double> absolute_pos_ratio;
  std::vector<double> motor_dir; 
  std::vector<double> pos_gain; 
  std::vector<double> vel_gain; 
  std::vector<double> vel_integrator_gain; 
  std::vector<double> pos_offset = std::vector<double>(TOTAL_JOINT_NUM+3);                         

  std::vector<double> read_joint_pos = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> read_joint_vel = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> read_joint_torq = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> write_joint_pos = std::vector<double>(TOTAL_JOINT_NUM+3); 
  std::vector<double> write_joint_vel = std::vector<double>(TOTAL_JOINT_NUM+3); 
  std::vector<double> write_joint_torq = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> write_joint_current = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> current_motor_pos = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> current_motor_vel = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> current_motor_cur = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> last_cmd_pos = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> last_cmd_vel = std::vector<double>(TOTAL_JOINT_NUM+3);
  std::vector<double> last_cmd_cur = std::vector<double>(TOTAL_JOINT_NUM+3);

  const double cur_lpf_ratio = 0.1;
  const double pos_lpf_ratio = 0.3;
  const double vel_lpf_ratio = 0.05;

  const std::vector<double> current_bound = {7, 9, 45, 45, 2.4, 2.4, 7, 9, 45, 45, 2.4, 2.4};

};


PYBIND11_MODULE(gr1_sdk, m){
    py::class_<GR1HW>(m, "GR1")
        .def(py::init<>())
        //.def("read", &GR1HW::read)
        //.def("write", &GR1HW::read);
}
