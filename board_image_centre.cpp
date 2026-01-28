#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <librealsense2/rs.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;
using namespace ur_rtde;

// ------------------------------------------------------------
// Robot home pose
// ------------------------------------------------------------
std::vector<double> tcp_home_position =
{ -0.222, -0.479, 0.481, 0, 3.14, 0.0};


double computeReprojectionError(
    const std::vector<cv::Point3d>& objPts,
    const std::vector<cv::Point2f>& imgPts,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat& K,
    const cv::Mat& D)
{
    if (objPts.size() != imgPts.size() || imgPts.empty())
        return std::numeric_limits<double>::infinity();

    // 🔑 Convert object points to Point3f (THIS FIXES IT)
    std::vector<cv::Point3f> objPtsF;
    objPtsF.reserve(objPts.size());
    for (const auto& p : objPts)
        objPtsF.emplace_back(
            static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z));

    cv::Mat rvecMat = (cv::Mat_<double>(3,1)
        << rvec[0], rvec[1], rvec[2]);
    cv::Mat tvecMat = (cv::Mat_<double>(3,1)
        << tvec[0], tvec[1], tvec[2]);

    std::vector<cv::Point2f> projected;
    cv::projectPoints(
        objPtsF,          // ✅ Point3f, not Point3d
        rvecMat,
        tvecMat,
        K,
        D,
        projected);

    if (projected.size() != imgPts.size())
        return std::numeric_limits<double>::infinity();

    double err2 = 0.0;
    for (size_t i = 0; i < imgPts.size(); ++i)
    {
        cv::Point2f d = imgPts[i] - projected[i];
        err2 += d.dot(d);
    }

    return std::sqrt(err2 / imgPts.size());
}



// ------------------------------------------------------------
// Compute board centre from ALL visible markers
// ------------------------------------------------------------
bool computeBoardCenterCamera(
    const vector<vector<Point2f>>& corners,
    const vector<int>& ids,
    const Mat& K,
    const Mat& D,
    double markerLength,
    Point3d& boardCenterCam,
    Point2f& boardCenterPx,   double& reprojErrPx,     // <-- NEW
    int& numMarkersUsed )
{
    if (ids.empty()) return false;

    vector<Point3d> centersCam;
    vector<Point2f> centersPx;
     vector<double> reprojErrors;

    vector<Point3d> objPts =
    {
        {-markerLength/2,  markerLength/2, 0},
        { markerLength/2,  markerLength/2, 0},
        { markerLength/2, -markerLength/2, 0},
        {-markerLength/2, -markerLength/2, 0}
    };

    for (size_t i = 0; i < ids.size(); ++i)
    {
        Vec3d rvec, tvec;
        if (!solvePnP(objPts, corners[i], K, D, rvec, tvec))
            continue;
           
        double err_px = computeReprojectionError(objPts,corners[i],rvec,tvec,K,D);
     
        reprojErrors.push_back(err_px);

        centersCam.emplace_back(tvec[0], tvec[1], tvec[2]);

        Point2f c(0,0);
        for (auto& p : corners[i]) c += p;
        centersPx.push_back(c * 0.25f);
    }

    if (centersCam.empty()) return false;

    boardCenterCam = Point3d(0,0,0);
    for (auto& c : centersCam) boardCenterCam += c;
    boardCenterCam *= (1.0 / centersCam.size());

    boardCenterPx = Point2f(0,0);
    for (auto& c : centersPx) boardCenterPx += c;
    boardCenterPx *= (1.0f / centersPx.size());
        numMarkersUsed = static_cast<int>(reprojErrors.size());
    if (numMarkersUsed == 0)
        return false;

    std::nth_element(
        reprojErrors.begin(),
        reprojErrors.begin() + reprojErrors.size() / 2,
        reprojErrors.end());

    reprojErrPx = reprojErrors[reprojErrors.size() / 2];

    return true;
}



// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main()
{
    try
    {
        // --------------------------------------------------------
        // Robot
        // --------------------------------------------------------
        string robot_ip = "192.168.1.102";
        RTDEControlInterface rtde_control(robot_ip);
        RTDEReceiveInterface rtde_receive(robot_ip);
       
        std::cout << "Moving to home location:"<< endl;
        rtde_control.moveL(tcp_home_position, 0.01, 0.01);
        

        // --------------------------------------------------------
        // Load hand–eye calibration
        // --------------------------------------------------------
        Mat R_cam2gripper, t_cam2gripper;
        FileStorage fs("handeye_result.yaml", FileStorage::READ);
        fs["R_cam2gripper"] >> R_cam2gripper;
        fs["t_cam2gripper"] >> t_cam2gripper;
        fs.release();

        // --------------------------------------------------------
        // RealSense
        // --------------------------------------------------------
        std::cout<< "Before Vision Loop:" << endl;
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480,
                          RS2_FORMAT_BGR8, 30);
        auto profile = pipe.start(cfg);

        auto intr = profile.get_stream(RS2_STREAM_COLOR)
                        .as<rs2::video_stream_profile>()
                        .get_intrinsics();

        Mat K = (Mat_<double>(3,3) <<
            intr.fx, 0, intr.ppx,
            0, intr.fy, intr.ppy,
            0, 0, 1);

        Mat D = (Mat_<double>(1,5) <<
            intr.coeffs[0], intr.coeffs[1],
            intr.coeffs[2], intr.coeffs[3],
            intr.coeffs[4]);

        // --------------------------------------------------------
        // ArUco detector
        // --------------------------------------------------------
        auto dictionary =
            aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

        aruco::DetectorParameters params;
        aruco::ArucoDetector detector(dictionary, params);

        double markerLength = 0.022; // meters

        // --------------------------------------------------------
        // IBVS parameters
        // --------------------------------------------------------
        double k = 0.2;
        double tol = 0.002;      // 2 mm
        double maxStep = 0.01;   // 1 cm

        namedWindow("Board Centering", WINDOW_AUTOSIZE);

        // --------------------------------------------------------
        // MAIN LOOP
        // --------------------------------------------------------
        while (true)
        {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color = frames.get_color_frame();

            Mat frame(Size(color.get_width(), color.get_height()),
                      CV_8UC3,
                      (void*)color.get_data(),
                      color.get_stride_in_bytes());

            vector<vector<Point2f>> corners, rejected;
            vector<int> ids;

            detector.detectMarkers(frame, corners, ids, rejected);

            if (!ids.empty())
                aruco::drawDetectedMarkers(frame, corners, ids);

            Point3d boardCenterCam;
            Point2f boardCenterPx;
            double reprojErrPx;
int numMarkers;
     
            bool ok = computeBoardCenterCamera(
                corners, ids, K, D, markerLength,
                boardCenterCam, boardCenterPx,reprojErrPx,numMarkers);
                

            // Image centre
            Point imgCenter(frame.cols/2, frame.rows/2);
            circle(frame, imgCenter, 6, Scalar(0,255,0), -1);

            if (ok)
            {
                // Draw board centre
                circle(frame, boardCenterPx, 6, Scalar(0,0,255), -1);
                line(frame, imgCenter, boardCenterPx,
                     Scalar(255,0,0), 2);

                double dx = boardCenterCam.x;
                double dy = boardCenterCam.y;
                double err = sqrt(dx*dx + dy*dy);

                putText(frame,
                    format("Error: %.1f mm", err*1000),
                    {20,30}, FONT_HERSHEY_SIMPLEX,
                    0.7, Scalar(0,255,0), 2);
                    putText(frame,
    format("Reproj err: %.2f px (%d markers)",
           reprojErrPx, numMarkers),
    {20, 90},
    FONT_HERSHEY_SIMPLEX,
    0.6,
    Scalar(255,255,0),
    2);

                if (err < tol)
                {
                    putText(frame, "ALIGNED",
                        {20,60}, FONT_HERSHEY_SIMPLEX,
                        0.9, Scalar(0,255,0), 2);std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    
                }
                else
                {
                    double dxcam = clamp(k*dx, -maxStep, maxStep);
                    double dycam = clamp(k*dy, -maxStep, maxStep);

                    Mat dC = (Mat_<double>(3,1) << dxcam, dycam, 0);

                    auto pose = rtde_receive.getActualTCPPose();
                    Mat R_BG;
                    Rodrigues(Vec3d(pose[3],pose[4],pose[5]), R_BG);

                    Mat dB = R_BG * R_cam2gripper * dC;

                    pose[0] += dB.at<double>(0);
                    pose[1] += dB.at<double>(1);

                    rtde_control.moveL(pose, 0.01, 0.01);
                    imshow("Board Centering", frame);
                        waitKey(1);
                }
            }
            else
            {
                putText(frame, "Board not visible",
                        {20,30}, FONT_HERSHEY_SIMPLEX,
                        0.7, Scalar(0,0,255), 2);
            }

            imshow("Board Centering", frame);
            if (waitKey(1) == 27) break;
        }

        pipe.stop();
        rtde_control.stopScript();
        return 0;
    }
    catch (const std::exception& e)
    {
        cerr << "❌ Exception: " << e.what() << endl;
        return -1;
    }
}
