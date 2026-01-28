#include <ur_rtde/robotiq_gripper.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>


using namespace ur_rtde;
using namespace std;
using namespace cv;




std::vector <double> TCP_initial = {-0.39350, 0.5782,0.5359, 3.14, 0, 0};   // well D
//std::vector <double> TCP_initial = {-0.39350, 0.667,0.5009, 3.14, 0.0, 0.0};      // well C
std::vector <double> TCP_latch_pose(6,0);
const int number_points =200;
const double contact_duration = 500;
 int flag_result = 0;
  vector<vector<double>> force_log;
  vector<vector<double>> tcp_log;

int main() {
  string robot_ip = "192.168.1.102";      // Ur5 ip address
  RTDEControlInterface rtde_control(robot_ip);  // Initializaing to send commands
  RTDEReceiveInterface rtde_receive(robot_ip);   // initializing to receive commands
  RobotiqGripper gripper(robot_ip, 63352, true);  // default Modbus port is 63352
  //this_thread::sleep_for(chrono::seconds(3));
  cout << "The robot and gripper is comnnected " << endl;

  gripper.connect();

   rtde_control.moveL(TCP_initial,0.05,0.05);      // move to home position first

    gripper.move(1.0);          // Open the gripper for full throttle
  this_thread::sleep_for(chrono::seconds(1));     // pause for a moment 
  gripper.move(0.0);          // close the gripper
  this_thread::sleep_for(chrono::seconds(1));    // pause for a moment 

  std::vector<double> int_pose = rtde_receive.getActualTCPPose();

  int_pose[2] +=0.01;     // just move up by 1cm 

  rtde_control.moveL(int_pose,0.05,0.05);  // just for testing 

std::vector<double> direction_move = {0, 0, -0.00700, 0, 0, 0};

  rtde_control.moveUntilContact(direction_move);    // move in the direction untill robot hits an object
  this_thread::sleep_for(chrono::seconds(1));
    int_pose = rtde_receive.getActualTCPPose();
    int_pose[2] +=0.01;
    rtde_control.moveL(int_pose,0.05,0.05);       // Just to retract, a safe mechanism
    this_thread::sleep_for(chrono::seconds(1));   // just a momentary pause 
    gripper.move(0.3);
    int_pose = rtde_receive.getActualTCPPose();
    int_pose[2] -=0.0125;
    rtde_control.moveL(int_pose,0.05,0.05);
    gripper.setForce(1.0);
    gripper.setSpeed(0.2);                          // setting the force, speed to grasp 
    gripper.move(0.0);                              // close the gripper to grasp the vail
    this_thread::sleep_for(chrono::seconds(1));     // just a momentary break before rotating or any action
    double step_size = 0.0;                         // initialze to keep track of rotation
     std::vector<double> average_force(6,0.0);     // variable to store force and pose
        std::vector<double> pose_measure(6,0.0);
        double force_threshold = 0;
    int_pose= rtde_receive.getActualTCPPose();     
    pose_measure = int_pose;                         // nothing but initialization
    step_size = int_pose[4];
    cout << "Starting to probe" << endl;
  
    for(int ii=0;ii<number_points;++ii){
    
    pose_measure = rtde_receive.getActualTCPPose();              // collecting the TCP Pose
       int samples =0;                                         // defining average vector to average the force, samples to keep the count of how many samples, and finallu clock
        std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
        std::vector<int> selection_vector = {0, 0, 1, 0, 0, 1}; // Z compliance
        std::vector<double> wrench = {0, 0, -2, 0.0, 0.0, 2}; // 5N push
        int force_type = 2;
        std::vector<double> limits = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        pose_measure = rtde_receive.getActualTCPPose();
         std::cout << "Applying Z-force control..." << std::endl;
          //this_thread::sleep_for(chrono::seconds(2));
        rtde_control.forceMode(task_frame, selection_vector, wrench, 2, limits);

        cout << "The initial samples:" << samples << endl;
        auto start=chrono::steady_clock::now(); 
            while (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() < contact_duration) {
                vector<double> f = rtde_receive.getActualTCPForce();
                for (int j = 0; j < 6; ++j)
                    average_force[j] += f[j];
                samples++;
                this_thread::sleep_for(chrono::milliseconds(10)); // 100 Hz sampling
            }
                cout << "Stopping force mode" << endl;   
        rtde_control.forceModeStop();
            if (average_force[2]< -30) {
                cout << "The found z force:" << average_force[2] << endl;
                force_threshold= average_force[2];
                TCP_latch_pose = rtde_receive.getActualTCPPose();
                flag_result = 1;
                cout<< "Found the Pose" << endl;
                break;
            }
            for (auto &v :average_force){
                  v /= samples;
            }
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Averaged Force/Torque: [";
            for (size_t j = 0; j < average_force.size(); ++j) {
                std::cout << average_force[j];
                if (j < average_force.size() - 1)
                std::cout << ", ";
                    }
        std::cout << "]  (samples = " << samples << ")" << std::endl;
        force_log.push_back(average_force);
        tcp_log.push_back(pose_measure);
    std::cout << "Iteration number: " << ii + 1 << std::endl;
    }
    cout << "Done with probing" << endl;    // here ends the probing

    if (flag_result==1) {
        cout << "Latching" << endl;
        std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
        std::vector<int> selection_vector = {0, 0, 1, 0, 0, 1}; // Z compliance
        std::vector<double> wrench = {0, 0, -7, 0.0, 0.0, 1}; // 5N push
        int force_type = 2;
        std::vector<double> limits = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
         auto start=chrono::steady_clock::now(); 
          rtde_control.forceMode(task_frame, selection_vector, wrench, 2, limits);
        this_thread::sleep_for(chrono::seconds(5));
        rtde_control.forceModeStop();
    }

    ofstream log_file("/home/lunet/ttpp/Crystallisation_lab_pp/ur_rtde/my_tests/tcp_force_data_CL.csv");
    log_file << std::fixed << std::setprecision(6);

    log_file << "Initial Pose:";

    for (size_t j = 0; j < TCP_initial.size(); ++j) {
                log_file << TCP_initial[j];
                    if (j < TCP_initial.size() - 1)
                        log_file << ", ";
                }
                log_file << "\n";
  cout << "✅ written the target location" << endl;
                    log_file << "iteration,Pos_x,Pos_y,Pos_z,Roll,Pitch,Yaw,Force_x,Force_y,Force_z,Torque_x,Torque_y,Torque_z\n";
                // write the data to the file
                    for (size_t i=0;i<force_log.size();++i){
                        log_file<<i+1<<",";     // Iteration number
                        

                        // TCP Pose
                        for (int j=0;j<6;++j){
                            log_file << tcp_log[i][j] << ",";
                        }
                        // Force 
                        for (int j=0;j<6;++j) {
                            log_file << force_log[i][j];
                            if (j < 5)
                                log_file << ",";
                        }
                        
                         log_file << "\n";

                    }
                    log_file << "THe final_force_threshold,";
                    log_file << force_threshold << ",";
                    log_file.close();
                        cout << "✅ Data saved to tcp_force_data.csv" << endl;
 
  gripper.move(0.3);
  this_thread::sleep_for(chrono::seconds(1));     // just a momentary pause 
  cout<< "Moving to home position" << endl;
  cout << "The z location "<< TCP_initial[2] << endl;
  rtde_control.moveL(TCP_initial,0.05,0.05);       // return to home position
  gripper.move(0);
 
 cout <<"Program execution is over and at home position" << endl;
return 0;            // end of the program

}
