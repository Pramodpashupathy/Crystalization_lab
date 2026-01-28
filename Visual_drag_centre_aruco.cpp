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
std::vector<double> tcp_home_position ={-0.222, -0.479, 0.485, 0, 3.14, 0.0};

int main() {
   bool aligned = false;
    try {
        string robot_ip = "192.168.1.102";    
         RTDEControlInterface rtde_control(robot_ip);  
        double markerLength = 0.022;            // ✅ 22mm marker side (m)
        const int FIXED_MARKER_ID = 10;          // ✅ always use marker ID 8
        cout << "The robot is moving to home position:\n"<< endl;
        rtde_control.moveL(tcp_home_position,0.05,0.05);

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
            // std::cout << CV_VERSION << std::endl;


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
                    
            
                       // Draw a green dot on the image centre 
            Point center (frame.cols /2.0f,frame.rows /2.0f);
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

                  putText(frame,
        "Marker ID " + std::to_string(FIXED_MARKER_ID) + " visible",
        cv::Point(20, 30),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2);

                    // SINGLE waitKey
                   
                    int key = waitKey(1);
                    if (key == 27) break;

                    if (key == 's') {
                        cout << "\n🧩 Starting the IBVS Loop:\n";

                        double tol = 0.002;       // 2 mm
                        int maxIters = 30;        // safety limit
                        double k = 0.4;
                        double maxStep = 0.01;    // max 1cm per iteration
                         for (int iter = 0; iter < maxIters; iter++){
                            rs2::frameset frames2 = pipe.wait_for_frames();
                            rs2::video_frame color2 = frames2.get_color_frame();

                            Mat frame2(Size(color2.get_width(), color2.get_height()), CV_8UC3,
                   (void*)color2.get_data(), color2.get_stride_in_bytes());

        // ---------- 2) Detect marker ----------
                            vector<vector<Point2f>> corners2;
                            vector<int> ids2;
                            vector<vector<Point2f>> rejected2;

                            detector.detectMarkers(frame2, corners2, ids2, rejected2);
                            if (!ids2.empty()){
                                cv::aruco::drawDetectedMarkers(frame2,corners2,ids2);
                            }
                             putText(frame2,
        "Marker ID " + std::to_string(FIXED_MARKER_ID) + " visible",
        cv::Point(20, 30),
        FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2);
                               // find marker index again
        int idx2 = -1;
        for (int i = 0; i < (int)ids2.size(); i++)
        {
            if (ids2[i] == FIXED_MARKER_ID)
            {
                idx2 = i;
                break;
            }
        }

        if (idx2 == -1)
        {
            cout << "❌ Marker lost. Stopping.\n";
            break;
        }
         vector<Point3d> objPoints = {
            {-markerLength/2,  markerLength/2, 0},
            { markerLength/2,  markerLength/2, 0},
            { markerLength/2, -markerLength/2, 0},
            {-markerLength/2, -markerLength/2, 0}
        };

        Vec3d rvec2, tvec2;
        solvePnP(objPoints, corners2[idx2], K, D, rvec2, tvec2);
        drawFrameAxes(frame2,K,D,rvec2,tvec2,0.02);
        K.convertTo(K, CV_64F);
        D.convertTo(D, CV_64F);

        std::vector<cv::Point3d> objPointsD = {
         {-markerLength/2.0,  markerLength/2.0, 0.0},
        { markerLength/2.0,  markerLength/2.0, 0.0},
        { markerLength/2.0, -markerLength/2.0, 0.0},
        {-markerLength/2.0, -markerLength/2.0, 0.0}
        };

        std::vector<cv::Point2d> proj;
        cv::projectPoints(objPointsD, rvec2, tvec2, K, D, proj);
        // std::vector<cv::Point2f> proj;
        // cv::projectPoints(objPoints, rvec2, tvec2, K, D, proj);
        double reprojerr;
        double reprojerrMax;
        for (size_t i =0; i<proj.size();i++){
              cv::Point2d p_img = corners2[idx2][i]; // convert Point2f -> Point2d automatically
    double e = cv::norm(p_img - proj[i]);
            reprojerr +=e;
            reprojerrMax=std::max(reprojerrMax,e);
        }
        reprojerr /=proj.size();
        std::cout<< "\rReproj Err mean="<< std::fixed<<std::setprecision(3)<< reprojerr<< "px"<< "  max=" << reprojerrMax << " px"<<std::flush;
        double dxIBVS = tvec2[0];
        double dyIBVS = tvec2[1];

        double err = sqrt(dxIBVS*dxIBVS + dyIBVS*dyIBVS);
        cout << "Iter " << iter << " error = " << err * 1000 << " mm\n";
        std::ostringstream ss;
        ss<< std::fixed << std::setprecision(2)<< "The offset is ="<< err*1000<< " mm";
        cv::putText(frame2, ss.str(),
            cv::Point(20, 60),              // position
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),         // yellow
            2);

        // ✅ exit if aligned
        if (err < tol)
        {
            cout << "✅ Aligned! stopping loop.\n";
             aligned = true;
             // Let robot settle a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Grab a fresh frame AFTER stopping
    rs2::frameset framesF = pipe.wait_for_frames();
    rs2::video_frame colorF = framesF.get_color_frame();
    Mat finalFrame(Size(colorF.get_width(), colorF.get_height()), CV_8UC3,
                   (void*)colorF.get_data(), colorF.get_stride_in_bytes());

    // Re-detect marker on final frame
    vector<vector<Point2f>> cF;
    vector<int> idF;
    vector<vector<Point2f>> rejF;
    detector.detectMarkers(finalFrame, cF, idF, rejF);

    int idxF = -1;
    for (int i = 0; i < (int)idF.size(); i++)
        if (idF[i] == FIXED_MARKER_ID) { idxF = i; break; }

    if (idxF != -1)
    {
        Point centerF(finalFrame.cols/2, finalFrame.rows/2);
        circle(finalFrame, centerF, 6, Scalar(0,255,0), -1);

        Point2f mF(0,0);
        for (auto &p : cF[idxF]) mF += p;
        mF *= 0.25f;

        circle(finalFrame, mF, 6, Scalar(0,0,255), -1);
        line(finalFrame, centerF, mF, Scalar(255,0,0), 2);
    }

            imshow("Calibration Verification", frame2);
            waitKey(1000);
            break;
        }
          double dxcam = k * dxIBVS;
        double dycam = k * dyIBVS;

        dxcam = std::max(-maxStep, std::min(maxStep, dxcam));
        dycam = std::max(-maxStep, std::min(maxStep, dycam));
          cv::Mat dC = (cv::Mat_<double>(3,1) << dxcam, dycam, 0.0);

          std::vector<double> int_pose = rtde_receive.getActualTCPPose();
            cv::Mat R_BG;
        cv::Rodrigues(cv::Vec3d(int_pose[3], int_pose[4], int_pose[5]), R_BG);
           cv::Mat dB = R_BG * R_used * dC;

        int_pose[0] += dB.at<double>(0);
        int_pose[1] += dB.at<double>(1);
          // ---------- 7) Move robot ----------
        rtde_control.moveL(int_pose, 0.01, 0.01);

        // ---------- 8) Show line ----------
        Point center(frame2.cols / 2, frame2.rows / 2);
        circle(frame2, center, 6, Scalar(0,255,0), -1);

        Point2f markercenter(0,0);
        for (auto &p : corners2[idx2]) markercenter += p;
        markercenter *= 0.25f;

        circle(frame2, markercenter, 6, Scalar(0,0,255), -1);
        line(frame2, center, markercenter, Scalar(255,0,0), 2);

        imshow("Calibration Verification", frame2);
        waitKey(1);

        // optional small settle delay
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                         }
                         if (aligned){
                            cout << "Holding the final frame (press any key):\n";
                            waitKey(0);
                        
                            break;
                         }
                    }
                } else {
                   
                  putText(frame,
        "Marker ID " + std::to_string(FIXED_MARKER_ID) + " not visible",
        cv::Point(20, 30), 
        FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2);

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
        rtde_control.stopScript();
    }
    catch (const std::exception &e) {
        cerr << "❌ Exception: " << e.what() << endl;
        return -1;
    }
    
    return 0;
}
