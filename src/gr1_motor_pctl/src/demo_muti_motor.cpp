#include "FsaMutiMotor.h"
#include <fstream>
int main() {
    string ip1 = "192.168.137.55";
    string ip2 = "192.168.137.75";
    std::vector<std::string> ip_list;
    ip_list.push_back(ip1);
    ip_list.push_back(ip2);
    FSA_CONNECT::FSAMutiMotor fsa1(2);

    fsa1.init(ip_list);
    fsa1.Enable();
    sleep(2);
    fsa1.Enable();
    sleep(2);

    fsa1.EnablePosControl();

    Eigen::VectorXd pos_read(2);
    Eigen::VectorXd vel_read(2);
    Eigen::VectorXd cur_read(2);
    double max_t = 10;
    double dt = 0.002;
    Eigen::VectorXd pos_list(2);
    pos_list.setZero();
    Eigen::VectorXd vel_list(2);
    vel_list.setZero();
    Eigen::VectorXd cur_list(2);
    cur_list.setZero();
    
    //init pos
    fsa1.SetPosition(pos_list, vel_list, cur_list);
    sleep(1);
    fsa1.SetPosition(pos_list, vel_list, cur_list);
    
    for (int i = 0; i < max_t / dt; i++) {

        //read data
        fsa1.GetPVC(pos_read, vel_read, cur_read);
        std::cout<<"motor 1"<<std::endl;
        std::cout<<" pos read: "<<pos_read(0) <<" vel read: "<< vel_read(0) <<" cur read: "<< cur_read(0) << std::endl;
        std::cout<<"motor 2"<<std::endl;
        std::cout<<" pos read: "<<pos_read(1) <<" vel read: "<< vel_read(1) <<" cur read: "<< cur_read(1) << std::endl;
        //set pos
        double set_position = 15 * cos(2 * 3.1415926 / max_t * double(i) * dt) - 15;
        pos_list(0) = set_position;
        pos_list(1) = -set_position;
        fsa1.SetPosition(pos_list, vel_list, cur_list);
        //sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return 0;
}
