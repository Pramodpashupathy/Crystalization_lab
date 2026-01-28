// ===============================
// OpenCV 4.12 + RealSense
// ArUcoDetector (new API) + ChArUco pose (classic functions)
// ===============================

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

// New ArUco detector API (no Ptr<Dictionary> confusion)
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

// Classic ChArUco functions (interpolate + pose)
#include <opencv2/aruco/charuco.hpp>


#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace ur_rtde;
using namespace std;
using cv::Mat;
using cv::FileStorage;
std::vector<double> tcp_home_position ={-.237,-0.479,0.485,2.192,2.224,0.239};
static cv::Mat poseVecToT(const std::vector<double>& tcp)
{
    // tcp = [x, y, z, rx, ry, rz]
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    cv::Vec3d rvec(tcp[3], tcp[4], tcp[5]);
    cv::Mat R;
    cv::Rodrigues(rvec, R);  // OpenCV Rodrigues accepts axis-angle vector too

    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tcp[0];
    T.at<double>(1, 3) = tcp[1];
    T.at<double>(2, 3) = tcp[2];

    return T;
}

static cv::Mat rvecTvecToT(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec[0];
    T.at<double>(1, 3) = tvec[1];
    T.at<double>(2, 3) = tvec[2];
    return T;
}


int main()
{
    
    try
    {
        string robot_ip ="192.168.1.102";
        RTDEControlInterface rtde_control(robot_ip);
        RTDEReceiveInterface rtde_receive(robot_ip);
        cout << "The robot is moving to home position:" << endl;
        rtde_control.moveL(tcp_home_position,0.01,0.01);
        // ==========================================================
        // 1️⃣ Load hand–eye calibration
        // ==========================================================
        Mat R_cam2gripper, t_cam2gripper;
        FileStorage fs("handeye_result.yaml", FileStorage::READ);
        if (!fs.isOpened()) {
            cerr << "❌ Cannot open handeye_result.yaml\n";
            return -1;
        }
        fs["R_cam2gripper"] >> R_cam2gripper;
        fs["t_cam2gripper"] >> t_cam2gripper;
        fs.release();

        // NOTE: In your system, "X as returned" was correct.
        // So we use it directly as returned (no invert here).
        Mat R_used = R_cam2gripper.clone();
        Mat t_used = t_cam2gripper.clone();

        cout << "✅ Loaded hand–eye calibration.\n";

        // ------------------------------------------------------------
        // 1) Start RealSense color stream
        // ------------------------------------------------------------
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        rs2::pipeline_profile profile = pipe.start(cfg);

        auto vsp  = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto intr = vsp.get_intrinsics();

        cv::Mat K = (cv::Mat_<double>(3,3) << intr.fx, 0, intr.ppx,
                                              0, intr.fy, intr.ppy,
                                              0, 0, 1);

        cv::Mat D = (cv::Mat_<double>(1,5) << intr.coeffs[0], intr.coeffs[1],
                                              intr.coeffs[2], intr.coeffs[3],
                                              intr.coeffs[4]);

        std::cout << "✅ RealSense started (640x480 @30)\n";
        std::cout << "✅ Intrinsics loaded\n";

        // ------------------------------------------------------------
        // 2) Define the SAME ChArUco board you printed
        // ------------------------------------------------------------
        int squaresX = 7;
        int squaresY = 5;
        float squareLength = 0.03f;   // 30mm
        float markerLength = 0.022f;  // 22mm
        double Width = squaresX*squareLength;
        double height = squaresY*squareLength;

        cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

     
cv::Ptr<cv::aruco::CharucoBoard> board =
    cv::makePtr<cv::aruco::CharucoBoard>(
        cv::Size(squaresX, squaresY),
        squareLength,
        markerLength,
        dictionary
    );

        // ------------------------------------------------------------
        // 3) Create ArUco detector (new API)
        // ------------------------------------------------------------
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector arucoDetector(dictionary, detectorParams);

        std::cout << "✅ ArUco/ChArUco objects created\n";

        // ------------------------------------------------------------
        // 4) MAIN LOOP
        // ------------------------------------------------------------
        cv::namedWindow("ChArUco Pose", cv::WINDOW_AUTOSIZE);

        while (true)
        {
            // Get a frame
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color = frames.get_color_frame();
            

            cv::Mat frame(
                cv::Size(color.get_width(), color.get_height()),
                CV_8UC3,
                (void*)color.get_data(),
                color.get_stride_in_bytes()
            );

            // Clone to ensure writable memory for drawing
            cv::Mat img = frame.clone();

            // Detect ArUco markers
            std::vector<std::vector<cv::Point2f>> markerCorners, rejected;
            std::vector<int> markerIds;

            arucoDetector.detectMarkers(img, markerCorners, markerIds, rejected);

            if (!markerIds.empty())
                cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
                

            // Interpolate ChArUco corners
            cv::Mat charucoCorners, charucoIds;

            if (!markerIds.empty()){
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board,
                    charucoCorners, charucoIds, K, D);
                    int Nc = (int)charucoIds.total();
                    cv::putText(img,"ChArUco corners: " + std::to_string(Nc),
                             cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                             0.8, cv::Scalar(0, 255, 0), 2);

                 if (Nc>0){
                    cv::aruco::drawDetectedCornersCharuco(img,charucoCorners,charucoIds);  
                 }
                 // Estimate the board pose (offline measurement model)
                        cv::Vec3d rvec, tvec;
                bool okPose = false;
              if (charucoIds.total() >= 4)
{
    std::vector<cv::Point2f> imgPoints;
    std::vector<cv::Point3f> objPoints;

    // board is Ptr<CharucoBoard> in your case
    board->matchImagePoints(charucoCorners, charucoIds, objPoints, imgPoints);

    if (objPoints.size() >= 4)
    {
        okPose = cv::solvePnP(objPoints, imgPoints, K, D, rvec, tvec);
    }
}
                if(okPose) {
                    //  imshow("Board Centering", frame);
                    cv::putText(img, "Pose: OK",
                                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                                0.8, cv::Scalar(0, 255, 0), 2);
                    Mat T_cb = rvecTvecToT(rvec,tvec);
                      
                       std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                        std::vector<double> int_pose = rtde_receive.getActualTCPPose();
                      Mat T_be = poseVecToT(int_pose);
                      cv::Mat T_Ec = cv::Mat::eye(4, 4, CV_64F);  // ^eeT_c
                    R_used.copyTo(T_Ec(cv::Rect(0, 0, 3, 3)));
                    T_Ec.at<double>(0, 3) = t_used.at<double>(0);
                    T_Ec.at<double>(1, 3) = t_used.at<double>(1);
                    T_Ec.at<double>(2, 3) = t_used.at<double>(2);
                    // 4) Camera pose in base
cv::Mat T_bc = T_be * T_Ec;              // ^bT_c

// 5) Board pose in base
cv::Mat T_bB = T_bc * T_cb;              // ^bT_B


const auto& chesscorners = board->getChessboardCorners();
cv::Point3d cB(0,0,0);
for (const auto&p :chesscorners){
    cB.x+=p.x;cB.y+=p.y; cB.z+=p.z;
}
cB.x /=chesscorners.size();
cB.y /=chesscorners.size();
cB.z /=chesscorners.size();

cv::Mat p_center_B = (cv::Mat_<double>(3,1) << cB.x, cB.y, cB.z);
cv::Mat R_bB = T_bB(cv::Rect(0,0,3,3)).clone();
cv::Mat p_B  = (cv::Mat_<double>(3,1) <<
                T_bB.at<double>(0,3),
                T_bB.at<double>(1,3),
                T_bB.at<double>(2,3));

cv::Mat p_center_b = R_bB * p_center_B + p_B;

std::cout << "Boardcentre  in base (m): "
          << p_center_b.at<double>(0,0) << ", "
          << p_center_b.at<double>(1,0) << ", "
          << p_center_b.at<double>(2,0) << std::endl;
         double bx= p_center_b.at<double>(0,0);
         double by = p_center_b.at<double>(0,1);
         int_pose[0] = bx;
         int_pose[1] = by;
         std::cout << "The centre of the board is:" << int_pose[0] << int_pose[1]<< endl;
         
         
        for (int i = 0; i < 30; ++i) {
    rs2::frameset f = pipe.wait_for_frames();
    rs2::video_frame c = f.get_color_frame();
    cv::Mat fr(cv::Size(c.get_width(), c.get_height()), CV_8UC3,
               (void*)c.get_data(), c.get_stride_in_bytes());
    cv::imshow("ChArUco Pose", fr);
    if (cv::waitKey(1) == 27) break;
}
            rtde_control.moveL(int_pose,0.01,0.01);
            
            for (int i = 0; i < 30; ++i) {
    rs2::frameset f = pipe.wait_for_frames();
    rs2::video_frame c = f.get_color_frame();
    cv::Mat fr(cv::Size(c.get_width(), c.get_height()), CV_8UC3,
               (void*)c.get_data(), c.get_stride_in_bytes());
    cv::imshow("ChArUco Pose", fr);
    if (cv::waitKey(1) == 27) break;
}
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                break;

                 }
                 else{
                       cv::putText(img, "Pose: NOT OK",
                                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                                0.8, cv::Scalar(0, 0, 255), 2);
                 }

            }
            else {
                      cv::putText(img, "No markers detected",
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                            0.8, cv::Scalar(0, 0, 255), 2); 
            }

        

            // Show frame
            cv::imshow("ChArUco Pose", img);

            // ESC to exit
            if (cv::waitKey(1) == 27)
                break;
        }

        pipe.stop();
        return 0;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "❌ RealSense error: " << e.what() << "\n";
        return -1;
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "❌ OpenCV error: " << e.what() << "\n";
        return -1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "❌ Exception: " << e.what() << "\n";
        return -1;
    }
}
