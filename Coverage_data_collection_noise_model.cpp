#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ur_rtde/rtde_receive_interface.h>

#include <fstream>
#include <numeric>

using namespace cv;
using namespace std;
using namespace ur_rtde;

// -------------------- PARAMETERS --------------------
static const int AVG_FRAMES = 30;
static const double MARKER_LENGTH = 0.022;

bool computeBoardCenterCamera(
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<int>& ids,
    const cv::Mat& K,
    const cv::Mat& D,
    double markerLength,
    cv::Point3d& boardCenterCam,
    cv::Point2f& boardCenterPx,
    double& reprojErrPx,
    int& numMarkersUsed)
{
    if (ids.empty()) return false;

    std::vector<cv::Point3d> centersCam;
    std::vector<cv::Point2f> centersPx;
    std::vector<double> reprojErrs;

    // Marker corner model (marker frame)
    std::vector<cv::Point3f> objPts = {
        {-markerLength/2.f,  markerLength/2.f, 0},
        { markerLength/2.f,  markerLength/2.f, 0},
        { markerLength/2.f, -markerLength/2.f, 0},
        {-markerLength/2.f, -markerLength/2.f, 0}
    };

    for (size_t i = 0; i < ids.size(); ++i)
    {
        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(objPts, corners[i], K, D, rvec, tvec))
            continue;

        // ---- board center = marker origin (for now) ----
        centersCam.emplace_back(tvec[0], tvec[1], tvec[2]);

        cv::Point2f c(0,0);
        for (auto& p : corners[i]) c += p;
        centersPx.push_back(c * 0.25f);

        // ---- reprojection error ----
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objPts, rvec, tvec, K, D, projected);

        double err2 = 0.0;
        for (int k = 0; k < 4; k++)
        {
            cv::Point2f d = corners[i][k] - projected[k];
            err2 += d.dot(d);
        }
        reprojErrs.push_back(std::sqrt(err2 / 4.0));
    }

    if (centersCam.empty()) return false;

    // ---- average board center ----
    boardCenterCam = cv::Point3d(0,0,0);
    for (auto& p : centersCam) boardCenterCam += p;
    boardCenterCam *= (1.0 / centersCam.size());

    boardCenterPx = cv::Point2f(0,0);
    for (auto& p : centersPx) boardCenterPx += p;
    boardCenterPx *= (1.0f / centersPx.size());

    reprojErrPx = std::accumulate(
        reprojErrs.begin(), reprojErrs.end(), 0.0) / reprojErrs.size();

    numMarkersUsed = (int)centersCam.size();
    return true;
}


// -------------------- MAIN ---------------------------
int main()
{
    string robot_ip = "192.168.1.102";
    RTDEReceiveInterface rtde_receive(robot_ip);

    // --------- Camera ----------
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    auto profile = pipe.start(cfg);

    auto intr = profile.get_stream(RS2_STREAM_COLOR)
                        .as<rs2::video_stream_profile>()
                        .get_intrinsics();

    Mat K = (Mat_<double>(3,3) <<
        intr.fx, 0, intr.ppx,
        0, intr.fy, intr.ppy,
        0, 0, 1);

    Mat D = Mat::zeros(1,5,CV_64F);

    // --------- ArUco ----------
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    aruco::ArucoDetector detector(dict);

    // --------- CSV ----------
    ofstream csv("board_noise_experiment.csv");
    csv << "type,sample_id,"
        << "bx,by,bz,"
        << "sx,sy,sz,"
        << "frames_used,"
        << "mean_markers,"
        << "mean_reproj_err,"
        << "tcp_x,tcp_y,tcp_z,tcp_rx,tcp_ry,tcp_rz\n";

    int sample_id = 0;

    cout << "Controls:\n"
         << "  s = capture averaged measurement\n"
         << "  g = capture ground truth\n"
         << "  ESC = exit\n";

    while (true)
    {
        rs2::frameset fs = pipe.wait_for_frames();
        auto color = fs.get_color_frame();

        Mat frame(Size(color.get_width(), color.get_height()),
                  CV_8UC3,
                  (void*)color.get_data(),
                  color.get_stride_in_bytes());

        vector<vector<Point2f>> corners;
        vector<int> ids;

        cv::Point3d boardCenterCam;
cv::Point2f boardCenterPx;
double reprojErrPx = 0.0;
int numMarkers = 0;
bool boardOk = false;

        detector.detectMarkers(frame, corners, ids);

        if (!ids.empty())
            aruco::drawDetectedMarkers(frame, corners, ids);

        // imshow("Experiment", frame);
       

        boardOk = computeBoardCenterCamera(
    corners, ids, K, D, MARKER_LENGTH,
    boardCenterCam,
    boardCenterPx,
    reprojErrPx,
    numMarkers);
Point imgCenter(frame.cols / 2, frame.rows / 2);
                    circle(frame, imgCenter, 6, Scalar(0,255,0), -1);
        if (boardOk)
                {
                    vector<Point3f> pts3d = {
                          Point3f(
            (float)boardCenterCam.x,
            (float)boardCenterCam.y,
            (float)boardCenterCam.z)
                    };
                    vector<Point2f> pts2d;
                    projectPoints(pts3d,Vec3d(0,0,0), Vec3d(0,0,0), K, D, pts2d);
                    circle(frame,pts2d[0],6,Scalar(0,0,255), -1);
                       line(frame, imgCenter, pts2d[0], Scalar(255,0,0), 2);
                };
                imshow("Experiment", frame);
int key = waitKey(1);
        // ================= CAPTURE MEASUREMENT =================
        if (key == 's')
        {
            cout << "\n📸 Capturing averaged measurement...\n";

            vector<Point3d> centers;
            vector<int> markerCounts;
            vector<double> reprojErrs;

            for (int i = 0; i < AVG_FRAMES; i++)
            {
                rs2::frameset fs2 = pipe.wait_for_frames();
                auto c2 = fs2.get_color_frame();
                Mat f2(Size(c2.get_width(), c2.get_height()),
                       CV_8UC3,
                       (void*)c2.get_data(),
                       c2.get_stride_in_bytes());

                vector<vector<Point2f>> c;
                vector<int> id;
                detector.detectMarkers(f2, c, id);

                Point3d bc;
                Point2f bpx;
                double reprojErr;
                int nMarkers;

                bool ok = computeBoardCenterCamera(
                    c, id, K, D, MARKER_LENGTH,
                    bc, bpx, reprojErr, nMarkers);
                    

            


                if (!ok) continue;

                centers.push_back(bc);
                markerCounts.push_back(nMarkers);
                reprojErrs.push_back(reprojErr);
            }

            if (centers.size() < 5)
            {
                cout << "⚠️ Too few valid frames, skipping.\n";
                continue;
            }

            // ---- Mean ----
            Point3d mean(0,0,0);
            for (auto& p : centers) mean += p;
            mean *= (1.0 / centers.size());

            // ---- Std dev ----
            Point3d var(0,0,0);
            for (auto& p : centers)
            {
                Point3d d = p - mean;
                var.x += d.x*d.x;
                var.y += d.y*d.y;
                var.z += d.z*d.z;
            }
            var *= (1.0 / centers.size());

            Point3d stddev(
                sqrt(var.x),
                sqrt(var.y),
                sqrt(var.z));

            double meanMarkers =
                accumulate(markerCounts.begin(), markerCounts.end(), 0.0)
                / markerCounts.size();

            double meanReproj =
                accumulate(reprojErrs.begin(), reprojErrs.end(), 0.0)
                / reprojErrs.size();

            auto tcp = rtde_receive.getActualTCPPose();

            csv << "meas," << sample_id++ << ","
                << mean.x << "," << mean.y << "," << mean.z << ","
                << stddev.x << "," << stddev.y << "," << stddev.z << ","
                << centers.size() << ","
                << meanMarkers << ","
                << meanReproj << ","
                << tcp[0] << "," << tcp[1] << "," << tcp[2] << ","
                << tcp[3] << "," << tcp[4] << "," << tcp[5] << "\n";

            csv.flush();

            cout << "✅ Saved measurement sample " << sample_id-1 << endl;
        }

        // ================= CAPTURE GROUND TRUTH =================
        if (key == 'g')
        {
            cout << "\n🎯 CAPTURE GROUND TRUTH\n";
            cout << "→ Manually center the board perfectly\n";
            cout << "→ Press 'g' again when aligned\n";

            waitKey(0);  // wait for second 'g'

            // reuse one averaged capture for GT
            vector<Point3d> centers;
            for (int i = 0; i < AVG_FRAMES; i++)
            {
                rs2::frameset fs2 = pipe.wait_for_frames();
                auto c2 = fs2.get_color_frame();
                Mat f2(Size(c2.get_width(), c2.get_height()),
                       CV_8UC3,
                       (void*)c2.get_data(),
                       c2.get_stride_in_bytes());

                vector<vector<Point2f>> c;
                vector<int> id;
                detector.detectMarkers(f2, c, id);

                Point3d bc;
                Point2f px;
                double e;
                int nm;

                if (computeBoardCenterCamera(
                        c, id, K, D, MARKER_LENGTH,
                        bc, px, e, nm))
                    centers.push_back(bc);
            }

            Point3d gt(0,0,0);
            for (auto& p : centers) gt += p;
            gt *= (1.0 / centers.size());

            auto tcp = rtde_receive.getActualTCPPose();

            csv << "gt,0,"
                << gt.x << "," << gt.y << "," << gt.z << ","
                << "0,0,0,"
                << centers.size() << ","
                << "0,0,"
                << tcp[0] << "," << tcp[1] << "," << tcp[2] << ","
                << tcp[3] << "," << tcp[4] << "," << tcp[5] << "\n";

            csv.flush();
            cout << "🏁 Ground truth captured.\n";
        }

        if (key == 27) break;
    }

    csv.close();
    return 0;
}
