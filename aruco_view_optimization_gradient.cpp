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
#include <string>

using namespace ur_rtde;
using namespace std;
string robot_ip = "192.168.1.102"; 

 RTDEControlInterface rtde_control(robot_ip);   // initializing to send commands and recieve
    RTDEReceiveInterface rtde_receive(robot_ip); 
        cv::Point2f compute_marker_center(
    const std::vector<std::vector<cv::Point2f>>& corners)
        {
    cv::Point2f c(0.f, 0.f);
    if (corners.empty()) return c;

    for (const auto& p : corners[0]) c += p;
    return 0.25f * c;
        }

        struct ProbeResult {
    std::string dir;
    std::vector<double> pose;
    double cost;
};
    double compute_centre_error(const cv::Mat& image,
                            const std::vector<std::vector<cv::Point2f>>& corners){
                                if (corners.empty()) return 1.0;
                             cv::Point2f img_center(image.cols * 0.5f, image.rows * 0.5f);
                               const auto& pts = corners[0];

    cv::Point2f marker_center(0.f, 0.f);
    for (const auto& p : pts) marker_center += p;
    marker_center *= 0.25f;

    double dx = marker_center.x - img_center.x;
    double dy = marker_center.y - img_center.y;

    double pixel_dist = std::sqrt(dx*dx + dy*dy);
    double diag = std::sqrt(image.cols*image.cols + image.rows*image.rows);

    return pixel_dist / diag;

                            }

    double measure_avg_error(rs2::pipeline& pipe,
                         cv::aruco::ArucoDetector& detector,
                         int N = 15){
                               double sum = 0.0;
    int count = 0;

    for (int i = 0; i < N; ++i) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        cv::Mat image(cv::Size(640, 480), CV_8UC3,
                      (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(image, corners, ids);

        if (!corners.empty()) {
            sum += compute_centre_error(image, corners);
            count++;
        }
    }

    if (count == 0) return 1.0; // no detection => bad
    return sum / count;
                         }
//     


std::vector<ProbeResult> probe_all_directions(
    const std::vector<double>& current_pose,
    RTDEControlInterface& rtde_control,
    rs2::pipeline& pipe,
    cv::aruco::ArucoDetector& detector,
    double step,
    int avg_frames)
{
    struct Cand { std::string name; std::vector<double> pose; };
    std::vector<Cand> cands;

    auto make_pose = [&](double dx, double dy) {
        auto p = current_pose;
        p[0] += dx;
        p[1] += dy;
        return p;
    };

    cands.push_back({"E", make_pose(+step, 0)});
    cands.push_back({"W", make_pose(-step, 0)});
    cands.push_back({"N", make_pose(0, +step)});
    cands.push_back({"S", make_pose(0, -step)});

    std::vector<ProbeResult> results;

    for (auto& c : cands) {
        rtde_control.moveL(c.pose, 0.05, 0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        double J = measure_avg_error(pipe, detector, avg_frames);

        rtde_control.moveL(current_pose, 0.05, 0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        results.push_back({c.name, c.pose, J});
        std::cout << c.name << " cost = " << J << std::endl;
    }

    return results;
}

 std::vector<double> ur5_home_position= {0.265,-0.438,0.649,2.7,-2.0,0.127};
 const double CENTER_ERROR_THRESH = 0.07;

int main() {
    try {
        // -------- RealSense setup --------
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);

        std::cout << "RealSense started. Detecting ArUco markers...\n";

        // -------- ArUco setup --------
        cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        cv::aruco::DetectorParameters params;
        cv::aruco::ArucoDetector detector(dictionary, params);
        int frame_counter = 0;
        rtde_control.moveL(ur5_home_position,0.05,0.05);
        while (true) {
            frame_counter ++;
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frame color_frame = frames.get_color_frame();

            cv::Mat image(
                cv::Size(640, 480),
                CV_8UC3,
                (void*)color_frame.get_data(),
                cv::Mat::AUTO_STEP
            );
            cv::Point2f img_center(image.cols * 0.5f, image.rows * 0.5f);


            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            detector.detectMarkers(image, corners, ids);

            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(image, corners, ids);
                cv::circle(image, img_center, 5, cv::Scalar(255, 0, 0), -1);
                cv::putText(image, "IMG_CENTER", img_center + cv::Point2f(10, -10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
            
            
                for (size_t i = 0; i < ids.size(); ++i) {
                    double center_error = compute_centre_error(image,corners);
                       cv::Point2f marker_center = compute_marker_center(corners);  
                    cv ::line(image,img_center,marker_center,cv::Scalar(0,255,255),2);
                    std::vector<double> int_pose = rtde_receive.getActualTCPPose(); 
                    std::cout << "Center error: " << center_error << std::endl;
                    std::cout << "---------------------------\n";
                    cv::circle(image, marker_center, 5, cv::Scalar(0, 255, 0), -1); // green dot
                    cv::putText(image, "MARKER_CENTER",
            marker_center + cv::Point2f(10, -10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0, 255, 0), 1);
   
                }
                if (!ids.empty() && frame_counter % 60 == 0) {
                     double center_error0 = compute_centre_error(image, corners);

        std::vector<double> current_pose =
            rtde_receive.getActualTCPPose();
                    double step_size = 0.05;
        auto probe_results =
        probe_all_directions(
            current_pose,
            rtde_control,
            pipe,
            detector,
            step_size,
            5
        );

            
      auto best = std::min_element(
        probe_results.begin(), probe_results.end(),
        [](const ProbeResult& a, const ProbeResult& b) {
            return a.cost < b.cost;
        });
        

        // Calculating the gradient
        double J_E =0.0; double J_W =0.0; double J_N =0.0; double J_S=0.0;

        for (const auto&r :probe_results){
            if (r.dir=="E") J_E=r.cost;
            else if (r.dir=="W") J_W=r.cost;    // Assigning the variables
            else if (r.dir=="N") J_N=r.cost;
            else if (r.dir=="S") J_S=r.cost;

        }

        double bx = (J_N-J_S)/(2*step_size);
        double by = (J_E-J_W)/(2*step_size);    // Gradient


        double gnorm = std::sqrt(bx*bx+by*by);
        double ux =-bx/gnorm;
        double uy =-by/gnorm;
        std::cout << "Gradient direction (unit vector):" << "ux="<< ux<<"uy="<<uy<< std::endl;
        std::cout << "The gradient is :"<< gnorm<< std::endl;
     if (gnorm<1e-6) {
        std::cout << "✅ gradient too small and found local minimum";
        std::cout << "✅ Found best pose and converged at center_error = "
                  << center_error0 << std::endl;
                   rtde_control.stopScript();

    // Optional: small pause to ensure stop is processed
     std::this_thread::sleep_for(std::chrono::milliseconds(200));
        break;
    }

   double step_move = 0.05;

     std::vector<double> next_pose =
            rtde_receive.getActualTCPPose();
   std::cout << "the next coordinates are" << current_pose[0]<< current_pose[1]<< std::endl;
   next_pose[0] += step_move*ux;
   next_pose[1] += step_move*uy;
   std::cout << "the next coordinates are" << next_pose[0]<< next_pose[1]<< std::endl;
   rtde_control.moveL(next_pose,0.05,0.05);

    }


            } else {
                cv::putText(
                    image,
                    "No markers detected",
                    cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2
                );
            }

            cv::imshow("ArUco Detection", image);

            char key = (char)cv::waitKey(1);
if (key == 's') {
    cv::imwrite("realsense_frame.jpg", image);
    std::cout << "Saved frame!\n";
}
if (key == 'q') break;

        }

        pipe.stop();
        cv::destroyAllWindows();
        std::cout << "Stopped.\n";
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}



