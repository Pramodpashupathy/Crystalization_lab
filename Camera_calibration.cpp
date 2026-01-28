#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <ur_rtde/rtde_receive_interface.h>

using namespace cv;
using namespace std;
using namespace ur_rtde;

// -----------------------------------------------------------------------------
// 🧩 Compatibility helper for OpenCV ≥ 4.7 (replaces estimatePoseSingleMarkers)
// -----------------------------------------------------------------------------
namespace cv { namespace aruco {
inline void estimatePoseSingleMarkers(
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


int main() try
{
    const double MARKER_LENGTH_M = 0.022; // 5 cm markers
    const int FIXED_MARKER_ID = 8;       // ✅ always use marker ID 8

    string robot_ip = "192.168.1.102";

    // ✅ Only receive data (safe in Local mode)
    RTDEReceiveInterface rtde_receive(robot_ip);
    cout << "✅ Connected to UR5 for pose reading only (Local control allowed).\n";

    // ======== RealSense setup =========
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    auto profile = pipe.start(cfg);

    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = color_stream.get_intrinsics();

    Mat K = (Mat_<double>(3,3) << intr.fx, 0, intr.ppx,
                                  0, intr.fy, intr.ppy,
                                  0, 0, 1);

    Mat D = (Mat_<double>(1,5) << intr.coeffs[0], intr.coeffs[1],
                                  intr.coeffs[2], intr.coeffs[3],
                                  intr.coeffs[4]);

    // ✅ Print intrinsics once (STEP 1 check)
    cout << "\n📌 Camera Intrinsics (K):\n" << K << endl;
    cout << "📌 Distortion (D):\n" << D << "\n" << endl;

    // ======== ArUco setup =========
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    // ======== Data containers =========
    vector<Mat> R_gripper2base, t_gripper2base;
    vector<Mat> R_target2cam,  t_target2cam;

    cout << "🎥 Move robot with pendant, press [s] to record sample, [c] to calibrate, [ESC] to quit.\n";
    cout << "📌 NOTE: Only marker ID " << FIXED_MARKER_ID << " will be used for sampling.\n";

    while (true)
    {
        // --- Get RealSense frame ---
        rs2::frameset fs = pipe.wait_for_frames();
        rs2::video_frame color = fs.get_color_frame();
        Mat frame(Size(color.get_width(), color.get_height()), CV_8UC3,
                  (void*)color.get_data(), color.get_stride_in_bytes());

        // --- Detect ArUco markers ---
        vector<vector<Point2f>> corners;
        vector<int> ids;
        vector<vector<Point2f>> rejected;
        detector.detectMarkers(frame, corners, ids, rejected);

        vector<Vec3d> rvecs, tvecs;

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH_M, K, D, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i)
                cv::drawFrameAxes(frame, K, D, rvecs[i], tvecs[i], 0.025);

            // ✅ Print live marker Z (if any marker exists)
            cout << "\rMarker Z distance (m): " << fixed << setprecision(3) << tvecs[0][2] << "      " << flush;
        }

        putText(frame, "Samples: " + to_string(R_gripper2base.size()),
                {20, 40}, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);

        imshow("Hand–Eye Calibration", frame);

        int key = waitKey(1) & 0xFF;
        if (key == 27) break; // ESC

        // ============================
        // ✅ SAVE SAMPLE USING MARKER ID 8 ONLY
        // ============================
        if (key == 's')
        {
            if (ids.empty())
            {
                cout << "\n⚠️ No markers detected, sample not saved.\n";
                continue;
            }

            // Find marker index with ID = FIXED_MARKER_ID
            int use = -1;
            for (int i = 0; i < (int)ids.size(); i++)
            {
                if (ids[i] == FIXED_MARKER_ID)
                {
                    use = i;
                    break;
                }
            }

            if (use == -1)
            {
                cout << "\n⚠️ Marker ID " << FIXED_MARKER_ID << " not visible, sample not saved.\n";
                continue;
            }

            cout << "\n\n📸 Capturing sample using marker ID " << FIXED_MARKER_ID << " ..." << endl;

            // Marker pose
            Vec3d rvec_marker = rvecs[use];
            Vec3d tvec_marker = tvecs[use];

            Mat R_tc;
            Rodrigues(rvec_marker, R_tc);

            Mat t_tc = (Mat_<double>(3,1) << tvec_marker[0], tvec_marker[1], tvec_marker[2]);

            R_target2cam.push_back(R_tc.clone());
            t_target2cam.push_back(t_tc.clone());

            // Robot TCP pose
            vector<double> tcp = rtde_receive.getActualTCPPose(); // [x,y,z,rx,ry,rz]

            cout << "TCP xyz (m):   " << tcp[0] << ", " << tcp[1] << ", " << tcp[2] << endl;
            cout << "TCP rvec (rad):" << tcp[3] << ", " << tcp[4] << ", " << tcp[5] << endl;

            Mat R_gb;
            Rodrigues(Vec3d(tcp[3], tcp[4], tcp[5]), R_gb);

            Mat t_gb = (Mat_<double>(3,1) << tcp[0], tcp[1], tcp[2]);

            R_gripper2base.push_back(R_gb.clone());
            t_gripper2base.push_back(t_gb.clone());

            cout << "Marker ID used: " << ids[use] << endl;
            cout << "Marker tvec (m): " << tvec_marker << "   (Z should be 0.2 ~ 1.0 m)" << endl;

            cout << "✅ Saved sample #" << R_gripper2base.size() << endl;

            // Print rotation change (deg)
            if (R_gripper2base.size() >= 2)
            {
                Mat R_prev = R_gripper2base[R_gripper2base.size() - 2];
                Mat R_now  = R_gripper2base.back();

                Mat dR = R_now * R_prev.t();
                double angle = acos(std::min(1.0, std::max(-1.0, (trace(dR)[0] - 1) / 2.0)));

                cout << "🌀 Rotation change from previous sample (deg): "
                     << angle * 180.0 / CV_PI << endl;
            }
        }

        // ============================
        // ✅ CALIBRATE + STEP 2 CHECK
        // ============================
        if (key == 'c' && R_gripper2base.size() >= 8)
        {
            cout << "\n🧩 Running Hand–Eye calibration (" << R_gripper2base.size() << " samples)...\n";

            Mat R_cam2gripper, t_cam2gripper;
            calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam,  t_target2cam,
                R_cam2gripper, t_cam2gripper,
                CALIB_HAND_EYE_TSAI
            );

            cout << "\n✅ Result (Eye-in-hand)\n";
            cout << "R_cam2gripper = \n" << R_cam2gripper << endl;
            cout << "t_cam2gripper = " << t_cam2gripper.t() << endl;

            // STEP 2 Consistency check
            auto makeT = [](const Mat& R, const Mat& t) {
                Mat T = Mat::eye(4, 4, CV_64F);
                R.copyTo(T(Rect(0, 0, 3, 3)));
                t.copyTo(T(Rect(3, 0, 1, 3)));
                return T;
            };

            Mat X = makeT(R_cam2gripper, t_cam2gripper);
            Mat Xinv = X.inv();

            auto evalX = [&](const Mat& Xtest, const string& name)
            {
                vector<Vec3d> pts;
                pts.reserve(R_gripper2base.size());

                for (size_t i = 0; i < R_gripper2base.size(); i++)
                {
                    Mat T_bg = makeT(R_gripper2base[i], t_gripper2base[i]);
                    Mat T_ct = makeT(R_target2cam[i],  t_target2cam[i]);

                    Mat T_bt = T_bg * Xtest * T_ct;

                    pts.emplace_back(
                        T_bt.at<double>(0, 3),
                        T_bt.at<double>(1, 3),
                        T_bt.at<double>(2, 3)
                    );
                }

                Vec3d mean(0, 0, 0);
                for (auto& p : pts) mean += p;
                mean *= (1.0 / pts.size());

                Vec3d var(0, 0, 0);
                for (auto& p : pts)
                {
                    Vec3d d = p - mean;
                    var[0] += d[0] * d[0];
                    var[1] += d[1] * d[1];
                    var[2] += d[2] * d[2];
                }
                var *= (1.0 / pts.size());

                Vec3d stdev(sqrt(var[0]), sqrt(var[1]), sqrt(var[2]));

                cout << "\n🔎 STEP 2 Consistency check: " << name << "\n";
                cout << "Mean base-target position (m): " << mean << "\n";
                cout << "Std dev (mm): ["
                     << stdev[0] * 1000 << ", "
                     << stdev[1] * 1000 << ", "
                     << stdev[2] * 1000 << "]\n";
            };

            evalX(X,    "Using X as returned");
            evalX(Xinv, "Using inverse(X)");

            FileStorage fs("handeye_result.yaml", FileStorage::WRITE);
            fs << "R_cam2gripper" << R_cam2gripper;
            fs << "t_cam2gripper" << t_cam2gripper;
            fs.release();
            cout << "💾 Saved to handeye_result.yaml\n";
        }
    }

    return 0;
}
catch (const rs2::error& e)  { cerr << "RealSense error: " << e.what() << endl; return -1; }
catch (const cv::Exception& e){ cerr << "OpenCV error: " << e.what() << endl; return -1; }
catch (const std::exception& e){ cerr << "Error: " << e.what() << endl; return -1; }
