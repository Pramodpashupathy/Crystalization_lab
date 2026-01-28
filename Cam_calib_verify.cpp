#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <librealsense2/rs.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace ur_rtde;

int main() {
   
    try {
        string robot_ip = "192.168.1.102";    
         RTDEControlInterface rtde_control(robot_ip);  
        double markerLength = 0.022;            // ✅ 22mm marker side (m)
        const int FIXED_MARKER_ID = 0;          // ✅ always use marker ID 8

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

        // ==========================================================
        // 2️⃣ Connect to robot (receive-only)
        // ==========================================================
        RTDEReceiveInterface rtde_receive(robot_ip);
        cout << "✅ Connected to robot.\n";

        // ==========================================================
        // 3️⃣ Start RealSense camera
        // ==========================================================
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        auto profile = pipe.start(cfg);

        auto intr = profile.get_stream(RS2_STREAM_COLOR)
                        .as<rs2::video_stream_profile>()
                        .get_intrinsics();

        Mat K = (Mat_<double>(3,3) << intr.fx, 0, intr.ppx,
                                      0, intr.fy, intr.ppy,
                                      0, 0, 1);
        Mat D = (Mat_<double>(1,5) << intr.coeffs[0], intr.coeffs[1],
                                      intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]);

        cout << "✅ RealSense camera started.\n";

        // ==========================================================
        // 4️⃣ Setup ArUco detector
        // ==========================================================
        cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::DetectorParameters detectorParams;
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);

        cout << "\nPress 's' to sample marker pose, 'ESC' to quit.\n";

        // ==========================================================
        // 5️⃣ Data containers
        // ==========================================================
        vector<Mat> samples;
        ofstream logFile("marker_in_base.csv");
        logFile << "X,Y,Z\n";

        // ==========================================================
        // 6️⃣ Main loop
        // ==========================================================
        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color = frames.get_color_frame();

            Mat frame(Size(color.get_width(), color.get_height()), CV_8UC3,
                      (void*)color.get_data(), color.get_stride_in_bytes());

            vector<vector<Point2f>> corners;
            vector<int> ids;
            vector<vector<Point2f>> rejected;

            detector.detectMarkers(frame, corners, ids, rejected);

            if (!ids.empty()) {
                aruco::drawDetectedMarkers(frame, corners, ids);

                // ✅ Find the index of the marker we want (ID=8)
                int idx = -1;
                for (int i = 0; i < (int)ids.size(); i++) {
                    if (ids[i] == FIXED_MARKER_ID) {
                        idx = i;
                        break;
                    }
                }

                if (idx != -1) {
                    // Estimate pose for marker ID=8
                    vector<Point3d> objPoints = {
                        {-markerLength/2,  markerLength/2, 0},
                        { markerLength/2,  markerLength/2, 0},
                        { markerLength/2, -markerLength/2, 0},
                        {-markerLength/2, -markerLength/2, 0}
                    };

                    Vec3d rvec, tvec;
                    solvePnP(objPoints, corners[idx], K, D, rvec, tvec);

                    // double dxIBVS = tvec[0];
                    // double dyIBVS = tvec[1];
                    // double k = 0.4;
                    // double dxcam = -k*dxIBVS;
                    // double dycam = -k*dyIBVS;
                    // double maxStep = 0.01; // 1cm per loop
                    // dxcam = std::max(-maxStep, std::min(maxStep, dxcam));
                    // dycam = std::max(-maxStep, std::min(maxStep, dycam));

                    // // Transforming camera coordinates to robot pose 
                    // cv::Mat dC= (cv::Mat_<double>(3,1)<<dxcam,dycam,0.0);     // defining a vector of camera coordinates
                    // std::vector<double> int_pose = rtde_receive.getActualTCPPose();
                    // cout << "The coordinates in x and y:" << int_pose[0] << int_pose[1] << endl;
                    // cv:: Mat R_BG;
                    // cv:: Rodrigues(cv::Vec3d(int_pose[3],int_pose[4],int_pose[5]),R_BG);   //axis angle vector to rotation matrix 
                    // cv::Mat dB = R_BG*R_used*dC;
                    // int_pose[0] +=dB.at<double>(0);
                    //  int_pose[1] +=dB.at<double>(1);                   // converting the camera coordinates to real robot base frame
                    //  cout << "The coordinates in x and y:" << int_pose[0] << int_pose[1] << endl;
                    //  rtde_control.moveL(int_pose,0.01,0.01);
                    
               

                       // Draw a green dot on the image centre 
            Point center (frame.cols /2.0f,frame.cols /2.0f);
            circle(frame,center,6,Scalar(0,255,0),-1); // green dot

            // draw a red dot on the marker centre 
            Point2f markercenter(0,0);     // I need to mark the marker centre
                    for(const auto&p :corners[idx])
                        markercenter +=p;
                        markercenter *=0.25f;
                         circle(frame,markercenter,6,Scalar(0,0,255),-1);

                         // draw a line from marker centre to image centre 
                         line(frame,center,markercenter,Scalar(255,0,0),1);
                    

                    // shorter axis to avoid warnings
                    drawFrameAxes(frame, K, D, rvec, tvec, 0.02);

                    putText(frame, "Marker ID 8 visible",
                            {20, 30}, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);

                    // SINGLE waitKey
                   
                    int key = waitKey(1);
                    if (key == 27) break;

                    if (key == 's') {
                        cout << "\n🧩 Sampling marker pose (ID=3)...\n";

                        // Robot pose (base<-gripper)
                        vector<double> tcp = rtde_receive.getActualTCPPose();
                        Mat R_base2gripper;
                        Rodrigues(Vec3d(tcp[3], tcp[4], tcp[5]), R_base2gripper);
                        Mat t_base2gripper = (Mat_<double>(3,1) << tcp[0], tcp[1], tcp[2]);

                        // Marker pose (cam<-marker)
                        Mat R_cam2marker;
                        Rodrigues(rvec, R_cam2marker);
                        Mat t_cam2marker = (Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);

                        // Compute marker in base:
                        // base<-marker = (base<-gripper) * (gripper<-cam?) * (cam<-marker)
                        // In your calibration code, "X as returned" worked best,
                        // so we apply it the same way: base<-gripper * X * cam<-marker
                        Mat R_base2marker = R_base2gripper * R_used * R_cam2marker;
                        Mat t_base2marker = R_base2gripper * (R_used * t_cam2marker + t_used) + t_base2gripper;
                            //  Mat R_base2cam = R_base2gripper * R_used.t();
                            // Mat t_cam = (Mat_<double>(3,1) << dxcam, dycam, 0.0);
                            // Mat t_base = R_base2cam * t_cam;
                            // std::vector<double> int_pose = rtde_receive.getActualTCPPose();
                            // int_pose[0]=t_base.at<double>(0);
                            // int_pose[1]=t_base.at<double>(1);
                            // rtde_control.moveL(int_pose,0.02,0.02);

                        double xB = t_base2marker.at<double>(0);  // marker X in base
                        double yB = t_base2marker.at<double>(1);  // marker Y in base
                        double zB = t_base2marker.at<double>(2);  // marker Z in base
                         std::vector<double> int_pose = rtde_receive.getActualTCPPose();
                         int_pose[0] = xB;
                         int_pose[1] = yB;
                        rtde_control.moveL(int_pose,0.02,0.02);

                        cout << fixed << setprecision(5);
                        cout << "Marker in base (m): "
                             << "X=" << t_base2marker.at<double>(0)
                             << " Y=" << t_base2marker.at<double>(1)
                             << " Z=" << t_base2marker.at<double>(2) << endl;

                        logFile << t_base2marker.at<double>(0) << ","
                                << t_base2marker.at<double>(1) << ","
                                << t_base2marker.at<double>(2) << "\n";

                        samples.push_back(t_base2marker);

                        if (samples.size() > 1) {
                            Mat mean = Mat::zeros(3,1,CV_64F);
                            for (auto &t : samples) mean += t;
                            mean /= (double)samples.size();

                            double err = 0;
                            for (auto &t : samples)
                                err += norm(t - mean);
                            err /= samples.size();

                            cout << "↔️ Average deviation: " << err * 1000 << " mm\n";
                        }
                    }
                } else {
                    putText(frame, "Marker ID 8 not visible",
                            {20, 30}, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);

                    int key = waitKey(1);
                    if (key == 27) break;
                }
            } else {
                putText(frame, "No markers detected",
                        {20, 30}, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);
                int key = waitKey(1);
                if (key == 27) break;
            }
        
             imshow("Calibration Verification", frame);
        }

        logFile.close();
        cout << "✅ Results saved to marker_in_base.csv\n";
    }
    catch (const std::exception &e) {
        cerr << "❌ Exception: " << e.what() << endl;
        return -1;
    }

    return 0;
}
