#include<ur_rtde/rtde_control_interface.h>
#include<ur_rtde/rtde_receive_interface.h>
#include<ur_rtde/robotiq_gripper.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <iomanip>


using namespace ur_rtde;
using namespace std;





std::vector <double> TCP_initial = {-0.39350, 0.5782,0.5359, 3.14, 0.0, 0.0};   // well D


const int number_points =50;
const double contact_duration = 500;
const double radius = 0.008;
  vector<vector<double>> force_log;
  vector<vector<double>> tcp_log;

int main() {

    string robot_ip = "192.168.1.102";       // Ur5 ip address
    RTDEControlInterface rtde_control(robot_ip);   // initializing to send commands and recieve
    RTDEReceiveInterface rtde_receive(robot_ip);   
     RobotiqGripper gripper(robot_ip, 63352, true);  // default Modbus port is 63352
  //this_thread::sleep_for(chrono::seconds(3));
  cout << "The robot and gripper is comnnected " << endl;


  gripper.connect();
  rtde_control.moveL(TCP_initial,0.05,0.05);  // move to home position 

    // Just below are the lines to test the griper initialization
   gripper.move(1.0);          // Open the gripper for full throttle
  this_thread::sleep_for(chrono::seconds(1));     // pause for a moment 
  gripper.move(0.0);          // close the gripper
  this_thread::sleep_for(chrono::seconds(1));    // pause for a moment 
  // over//

   // detect and grasp the vail
   std::vector<double> direction_move = {0, 0, -0.00700, 0, 0, 0};
   rtde_control.moveUntilContact(direction_move);          // go down and detect the object
   this_thread::sleep_for(chrono::seconds(1));
   std::vector<double> homre_pose = rtde_receive.getActualTCPPose();  // keep track of home pose
   std::vector<double> int_pose = rtde_receive.getActualTCPPose();    // initialze a vector for intermediate purpose
   int_pose[2] +=0.01;
   rtde_control.moveL(int_pose,0.05,0.05);       // Just to retract, a safe mechanism
   this_thread::sleep_for(chrono::seconds(1));   // just a momentary pause 
     gripper.move(0.3);
    int_pose = rtde_receive.getActualTCPPose();
    int_pose[2] -=0.02;
    rtde_control.moveL(int_pose,0.05,0.05);
    gripper.setForce(1.0);
    gripper.setSpeed(0.2);                          // setting the force, speed to grasp 
    gripper.move(0.0);                              // close the gripper to grasp the vail
    this_thread::sleep_for(chrono::seconds(1)); 
    int_pose = rtde_receive.getActualTCPPose();
    int_pose[2] +=0.1;                             // grasp the object and test if you can manipulate
    rtde_control.moveL(int_pose,0.05,0.05);  
     this_thread::sleep_for(chrono::seconds(2));


    for(int ii=0;ii<number_points;++ii){
     double angle = (2.0*M_PI*ii/number_points);
        std::vector<double> test_pose = TCP_initial; // extracting the guess and it forms a way point
        test_pose[0] = TCP_initial[0]+radius*cos(angle);
        test_pose[1] = TCP_initial[1]+radius*sin(angle);
        rtde_control.moveL(test_pose,0.05,0.05);     // move to the next way point


        this_thread::sleep_for(chrono::seconds(1));
        rtde_control.moveUntilContact(direction_move);
        this_thread::sleep_for(chrono::seconds(2));

          std::vector<double> average_force(6,0.0);    // vectors to store the force and pose
        std::vector<double> pose_measure(6,0.0);

        // definition for force mode
       std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
        std::vector<int> selection_vector = {1, 1, 1, 1, 1, 0}; // Z compliance
        std::vector<double> wrench = {2, 2, -3, 0.0, 0.0, 0}; // 5N push
        int force_type = 2;
        std::vector<double> limits = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};


        pose_measure = rtde_receive.getActualTCPPose();    // obtain the pose
         std::cout << "Applying Z-force control..." << std::endl;
          //this_thread::sleep_for(chrono::seconds(2));
                  rtde_control.zeroFtSensor();
        rtde_control.forceMode(task_frame, selection_vector, wrench, 2, limits);      // applying force mode 
        int samples =0; 
        
        auto start = chrono::steady_clock::now();
        while(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count()<contact_duration){
        vector<double> f = rtde_receive.getActualTCPForce();     // obtain the tcp force/torque
          for(int j=0;j<6;++j)
            average_force[j]+=f[j];          
            samples++;
            this_thread::sleep_for(chrono::milliseconds(10)); // 100 Hz sampling

        }
        cout << "Stopping the force mode" <<endl;
         rtde_control.forceModeStop();     // stop the force mode
          for (auto &v :average_force){
                  v /= samples;
            }
            std::cout << std::fixed << std::setprecision(4);
            std::cout <<"Averaged Force/Torque: [";
            for (size_t j=0;j<average_force.size();++j){
                std::cout << average_force[j];
                if (j < average_force.size() - 1)
                std::cout << ", ";
            }
            std::cout << "]  (samples = " << samples << ")" << std::endl;
        force_log.push_back(average_force);
        tcp_log.push_back(pose_measure);

        this_thread::sleep_for(chrono::seconds(1));

        cout <<"The iteration number is: " << ii+1 << endl;
    }
    this_thread::sleep_for(chrono::seconds(2));
    rtde_control.moveL(TCP_initial,0.05,0.05);
    this_thread::sleep_for(chrono::seconds(2));
    rtde_control.moveUntilContact(direction_move);
    this_thread::sleep_for(chrono::seconds(2));
        gripper.move(0.8);                       // open the gripper to indicate the job done
         this_thread::sleep_for(chrono::seconds(2));
     rtde_control.moveL(TCP_initial,0.05,0.05);   // go to the initial pose where everything started
     this_thread::sleep_for(chrono::seconds(1));  // just a pause
      gripper.move(0);

      return(0);                               // end of the program


}