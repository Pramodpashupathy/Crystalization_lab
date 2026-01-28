

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

std::vector <double> TCP_initial = {0.247, -0.528, 0.320, 3.14, 0.0, 0.0};
std::vector<double> TCP_home_pose = {0.227,-0.570,0.300,3.14,0.0,0.0};

//std::vector <double> TCP_target_pose_guess = {0.261, -0.549, 0.320, 3.14, 0.0, 0.0};  
std::vector <double> TCP_target_pose_guess = {0.227,-0.570, 0.300, 3.14, 0.0, 0.0}; 
const double radius = 0.001;
const int number_points = 15;
const double contact_duration = 1000;   // In ms, 1000= 1s
  vector<vector<double>> force_log;
  vector<vector<double>> tcp_log;
 
// end of initilzation




void operate_gripper (const string& robot_ip){
    try {
        cout << "🔌 Connecting to Robotiq gripper..." << endl;
        RobotiqGripper gripper(robot_ip, 63352, true);  // default Modbus port is 63352
        gripper.connect();

        cout << "✅ Connected! Activating gripper..." << endl;
        gripper.activate();   // Initialize / calibrate gripper
        this_thread::sleep_for(chrono::seconds(2)); // Allow activation to complete

        cout << "🤖 Opening gripper..." << endl;
         gripper.move(255.0);  // 255 = fully closed
        
        this_thread::sleep_for(chrono::seconds(3)); // Wait while open

        cout << "✊ Closing gripper..." << endl;
       gripper.move(0.0);    // 0 = fully open
        this_thread::sleep_for(chrono::seconds(3)); // Wait while closed

        cout << "🧹 Releasing connection..." << endl;
        //gripper.disconnect();

        cout << "🏁 Done! Gripper opened and closed successfully." << endl;
    }
    catch (const exception &e) {
        cerr << "❌ Error: " << e.what() << endl;
    }

}
int main() {
    string robot_ip = "192.168.1.102";   // <-- Replace with your robot's IP
    RTDEControlInterface rtde_control(robot_ip);
    RTDEReceiveInterface rtde_receive(robot_ip);
    RobotiqGripper gripper(robot_ip, 63352, true);  // default Modbus port is 63352
     cout << "testing the bashrc file monday " <<endl;

     gripper.connect();
    //this_thread::sleep_for(chrono::seconds(5));
    //cout << "starting after sleep" <<endl;
//     VideoCapture cap(0);      // try 0, if laptop webcam opens, try 1 or 2 for realsense
//      if (!cap.isOpened()) {
//         cerr << "❌ ERROR: Cannot open camera" << endl;
//         return -1;
//     }

//     cout << "✅ Camera opened successfully!" << endl;


//     // test for camera 

//     cv::Mat frame;
// while (true) {
//     cap >> frame; // Capture a new frame
//     if (frame.empty()) {
//         cerr << "⚠️  Empty frame captured, exiting..." << endl;
//         break;
//     }

//     // Show the frame in a window
//     cv::imshow("Camera Feed", frame);

//     // Exit when 'q' is pressed
//     if (cv::waitKey(1) == 'q') {
//         cout << "👋 Exiting camera view..." << endl;
//         break;
//     }
// }

// cap.release();           // Release the camera
// cv::destroyAllWindows(); // Close any OpenCV windows

     //this_thread::sleep_for(chrono::seconds(10));
    // cout << "Initializing the gripper.."<< endl;
    // operate_gripper(robot_ip);
    // cout << "Finished Initializing.." << endl;
    //  this_thread::sleep_for(chrono::seconds(4)); 
      // Wait while open 
     gripper.move(1.0);    // open the gripper
      this_thread::sleep_for(chrono::seconds(2)); 
      gripper.move(0.0);   // close the gripper
      this_thread::sleep_for(chrono::seconds(2));
    rtde_control.moveL(TCP_home_pose, 0.05, 0.05);
    vector<double> speed = {0, 0, -0.00700, 0, 0, 0};
      gripper.move(0.0);      // close the gripper
    this_thread:: sleep_for(chrono::seconds(2));
    rtde_control.moveUntilContact(speed);
    cout << "Found the object.." <<endl;

    gripper.setSpeed(0.000001);
    gripper.setForce(0.07);

    // gripper.move(0.0);      // close the gripper
    // this_thread:: sleep_for(chrono::seconds(2));
    std::vector<double> current_pose = rtde_receive.getActualTCPPose();
    current_pose[2] +=0.02;
    rtde_control.moveL(current_pose,0.05,0.05);
    gripper.move(0.195);
    this_thread::sleep_for(chrono::seconds(1));
    current_pose = rtde_receive.getActualTCPPose();
    current_pose[2] -=0.026;
    rtde_control.moveL(current_pose, 0.05,0.05);
    gripper.move(0.0);
    this_thread::sleep_for(chrono::seconds(2));
    current_pose = rtde_receive.getActualTCPPose();
    current_pose[2] +=0.02;
    rtde_control.moveL(current_pose,0.05,0.05);
    this_thread::sleep_for(chrono::seconds(2));
    rtde_control.moveL(TCP_target_pose_guess,0.05,0.05);  
    cout << "Moving in grid points"  << endl;
    rtde_control.zeroFtSensor();
    this_thread::sleep_for(chrono::milliseconds(100));

     for (int ii=0;ii<number_points;++ii){
        double angle = (2.0*M_PI*ii/number_points);
        std::vector<double> test_pose = TCP_target_pose_guess; // extracting the guess and it forms a way point
        test_pose[0] = TCP_target_pose_guess[0]+radius*cos(angle);
        test_pose[1] = TCP_target_pose_guess[1]+radius*sin(angle);
        rtde_control.moveL(test_pose,0.05,0.05);
        

        this_thread::sleep_for(chrono::seconds(1));
        rtde_control.moveUntilContact(speed);
        this_thread::sleep_for(chrono::seconds(2));
        // current_pose= rtde_receive.getActualTCPPose();
        //  current_pose[2] +=0.0001;
        //  rtde_control.moveL(current_pose,0.05,0.05);
        
        // average the forces out 
        
        std::vector<double> average_force(6,0.0);
        std::vector<double> pose_measure(6,0.0);
       std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
        std::vector<int> selection_vector = {1, 1, 1, 1, 1, 1}; // Z compliance
        std::vector<double> wrench = {0, 0, -0.5, 0.0, 0.0, 1}; // 5N push
        int force_type = 2;
        std::vector<double> limits = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
        pose_measure = rtde_receive.getActualTCPPose();
         std::cout << "Applying Z-force control..." << std::endl;
          //this_thread::sleep_for(chrono::seconds(2));
        rtde_control.forceMode(task_frame, selection_vector, wrench, 2, limits);
        int samples =0;                                         // defining average vector to average the force, samples to keep the count of how many samples, and finallu clock

        cout << "The initial samples:" << samples << endl;
        auto start=chrono::steady_clock::now();


//         rtde_control.zeroFtSensor();
//    this_thread::sleep_for(chrono::seconds(2));
            while (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() < contact_duration) {
                vector<double> f = rtde_receive.getActualTCPForce();
                for (int j = 0; j < 6; ++j)
                    average_force[j] += f[j];
                samples++;
                this_thread::sleep_for(chrono::milliseconds(10)); // 100 Hz sampling
            }
             cout << "Stopping force mode" << endl;   
        rtde_control.forceModeStop();
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

        this_thread::sleep_for(chrono::seconds(1));
        cout << "The iteration number:" << ii+1 << endl;
     }
        cout << "✅ done with probing" << endl;
            ofstream log_file("/home/lunet/ttpp/Crystallisation_lab_pp/ur_rtde/my_tests/tcp_force_data.csv");
            log_file << std::fixed << std::setprecision(6);

            log_file << "# TCP_target_pose_guess, ";
           for (size_t j = 0; j < TCP_target_pose_guess.size(); ++j) {
                log_file << TCP_target_pose_guess[j];
                    if (j < TCP_target_pose_guess.size() - 1)
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
                    log_file.close();
                        cout << "✅ Data saved to tcp_force_data.csv" << endl;

    this_thread::sleep_for(chrono::seconds(2)); 
    rtde_control.moveL(TCP_target_pose_guess,0.05,0.05);
    rtde_control.moveUntilContact(speed);
    current_pose = rtde_receive.getActualTCPPose();
    current_pose[2] +=0.02;
    rtde_control.moveL(current_pose,0.05,0.05);
    current_pose= rtde_receive.getActualTCPPose();
    current_pose[2] -=0.02;
    rtde_control.moveL(current_pose,0.001,0.001);
    this_thread::sleep_for(chrono::seconds(2));
    gripper.move(1.00);
    this_thread::sleep_for(chrono::seconds(2));
    current_pose=rtde_receive.getActualTCPPose();
    current_pose[2] += 0.05;
    rtde_control.moveL(current_pose,0.05,0.05);
    this_thread::sleep_for(chrono::seconds(2));
    gripper.move(0.0);
    this_thread::sleep_for(chrono::seconds(2));
    // gripper.move(255.0);    // open the gripper
    return 0;
}
