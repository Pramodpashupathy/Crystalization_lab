#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/imgproc.hpp>   // <-- for cv::putText, FONT_HERSHEY_SIMPLEX
#include <iostream>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>

// -----------------------------------------------------------------------------
// 🧩 Forward declaration (needed for OpenCV 4.12, where header doesn’t expose it)
// -----------------------------------------------------------------------------
namespace cv {
namespace aruco {

// --- Implementation of the removed legacy function ---
void estimatePoseSingleMarkers(
    const std::vector<std::vector<cv::Point2f>>& corners,
    float markerLength,
    const cv::InputArray& cameraMatrix,
    const cv::InputArray& distCoeffs,
    std::vector<cv::Vec3d>& rvecs,
    std::vector<cv::Vec3d>& tvecs)
{
    rvecs.clear();
    tvecs.clear();

    for (const auto& corner : corners)
    {
        // 3D object points for the marker corners (assuming square in XY plane)
        std::vector<cv::Point3f> objPoints = {
            {-markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f,  markerLength/2.f, 0},
            { markerLength/2.f, -markerLength/2.f, 0},
            {-markerLength/2.f, -markerLength/2.f, 0}
        };

        cv::Vec3d rvec, tvec;
        cv::solvePnP(objPoints, corner, cameraMatrix, distCoeffs, rvec, tvec);
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }
}

}} // namespace cv::aruco

using namespace cv;
using namespace std;
using namespace ur_rtde;


int main() try
{
    const double MARKER_LENGTH_M = 0.05;  // Marker size in meters (adjust to your marker)
  string robot_ip = "192.168.1.102";       // Ur5 ip address
    RTDEControlInterface rtde_control(robot_ip);   // initializing to send commands and recieve
    RTDEReceiveInterface rtde_receive(robot_ip);   
     RobotiqGripper gripper(robot_ip, 63352, true);  // default Modbus port is 63352
        cout << "The robot and gripper is comnnected " << endl;

    // =========================================================================
    // 🎥 RealSense setup
    // =========================================================================
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    auto profile = pipe.start(cfg);
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_stream.get_intrinsics();

    cv::Mat K = (cv::Mat_<double>(3,3) << intr.fx, 0, intr.ppx,
                                          0, intr.fy, intr.ppy,
                                          0, 0, 1);
    cv::Mat D = (cv::Mat_<double>(1,5) << intr.coeffs[0], intr.coeffs[1],
                                          intr.coeffs[2], intr.coeffs[3],
                                          intr.coeffs[4]);

    // =========================================================================
    // 🧠 ArUco setup
    // =========================================================================
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cout << "🎥 RealSense + ArUco running (press ESC to quit)" << endl;

    // =========================================================================
    // 🔁 Main loop
    // =========================================================================
    cv::Mat frame;

    const int num_points = 5;
      std::vector<double> int_pose = rtde_receive.getActualTCPPose();  
      std::vector<double> test_pose = int_pose;     // just for initializing

 
    while (true)
    {
           for (int ii=0;ii<num_points;++ii){
             const double step_size = 0.02;
    if (ii < 2) {
        test_pose[0] = int_pose[0] + (ii + 1) * step_size;        
    } else if (ii == 2) {
        test_pose[0] = int_pose[0];
        cout << "The second case" << endl;
    } else {
        test_pose[0] = int_pose[0] - (ii - 2) * step_size;
         cout << "The third case" << endl;
    }

    rtde_control.moveL(test_pose, 0.05, 0.05);

    // get actual position after motion
    std::vector<double> new_pose = rtde_receive.getActualTCPPose();
    std::cout << "New actual Z: " << new_pose[2] << std::endl;
            int_pose = rtde_receive.getActualTCPPose();
            cout << "The new z position is:" << int_pose[2]<< endl;
            cout<< "The robot is ready for detection:" << endl;
            this_thread::sleep_for(chrono::seconds(2));    // just a pause
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        cv::Mat(color.get_height(), color.get_width(), CV_8UC3,
                (void*)color.get_data(), color.get_stride_in_bytes()).copyTo(frame);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;

        detector.detectMarkers(frame, corners, ids, rejected);

        if (!ids.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            // ✅ Use the legacy pose estimation function
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH_M, K, D, rvecs, tvecs);

            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                cv::drawFrameAxes(frame, K, D, rvecs[i], tvecs[i], float(MARKER_LENGTH_M * 0.5));

                cv::Vec3d t = tvecs[i];
                double x = t[0];
                double y = t[1];
                double z = t[2];
                double dist = cv::norm(t);

               // Draw on image
                std::string txt = "ID " + std::to_string(ids[i]) +
                  cv::format(" | X: %.3f Y: %.3f Z: %.3f | %.3fm", x, y, z, dist);

                cv::putText(frame, txt, corners[i][0] + cv::Point2f(0, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 2);

                // Print to terminal
                std::cout << "✅ Marker " << ids[i]
                    << " | X: " << x << " m"
                    << " | Y: " << y << " m"
                    << " | Z: " << z << " m"
                    << " | Distance: " << dist << " m"
                    << std::endl;

            }
        }
        else
        {
            cout << "❌ No marker detected." << endl;
        }

        cv::imshow("RealSense + ArUco (4.12)", frame);
        if (cv::waitKey(1) == 27) break; // ESC
    }
    }
    return 0;
}
catch (const rs2::error& e)
{
    cerr << "RealSense error: " << e.what() << endl;
    return -1;
}
catch (const cv::Exception& e)
{
    cerr << "OpenCV error: " << e.what() << endl;
    return -1;
}
