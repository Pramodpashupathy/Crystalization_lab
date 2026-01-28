#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <ur_rtde/rtde_receive_interface.h>
#include <librealsense2/rs.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

using namespace std;
std::vector<std::vector<cv::Point2f>> allCorners;      // 2D image points (from detectMarkers)
std::vector<std::vector<cv::Point3f>> allObjectPoints; 
cv::Size imageSize(640, 480);

// Struct to hold column info
struct ColumnInfo {
    std::string label;
    int markerId;
};

//    
ColumnInfo chooseColumn(double amount_g) {
    if (amount_g >= 0.06 && amount_g <= 1.2)
        return {"12 g silica column", 31};
    else if (amount_g >1.2 && amount_g <= 2.4)
        return {"24 g flash column", 23};
    else if (amount_g > 2.4 && amount_g <= 4.0)
        return {"40 g flash column", 13};
    else if (amount_g >4 && amount_g <= 8.0)
        return {"80 g flash column", 19};
    else
        return {"Unknown", -1};
}


int main (){
    try{
    // create a pipeline 

       double amount;
    std::cout << "Enter solvent amount in grams (e.g., 0.6): ";
    std::cin >> amount;

    ColumnInfo target = chooseColumn(amount);

    if (target.markerId == -1) {
        std::cout << "⚠️  No matching flash column for this amount.\n";
        return 0;
    }
    cout << "Detecting for suitable markers" << endl;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);

    // start streaming 

    pipe.start(cfg);
    cout << "Realsense started and trying to identifying aruco markers" << endl;
      //  --- Get internal calibration (camera intrinsics) --

      auto stream = pipe.get_active_profile()
                  .get_stream(RS2_STREAM_COLOR)
                  .as<rs2::video_stream_profile>();

rs2_intrinsics intrinsics = stream.get_intrinsics();

// Build camera matrix and distortion coefficients for OpenCV
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    intrinsics.fx, 0, intrinsics.ppx,
    0, intrinsics.fy, intrinsics.ppy,
    0, 0, 1);

cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
    intrinsics.coeffs[0], intrinsics.coeffs[1],
    intrinsics.coeffs[2], intrinsics.coeffs[3],
    intrinsics.coeffs[4]);

// Optional: print intrinsics for verification
cout << "Camera intrinsics:" << endl;
cout << " fx = " << intrinsics.fx << ", fy = " << intrinsics.fy << endl;
cout << " cx = " << intrinsics.ppx << ", cy = " << intrinsics.ppy << endl;
cout << " Distortion coeffs = [ ";
for (int i = 0; i < 5; i++) cout << intrinsics.coeffs[i] << " ";
cout << "]" << endl;

    // set up aruco dictionary 
      cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector(dictionary,detectorParams);
bool found = false;

  while(true){
        rs2::frameset frames = pipe.wait_for_frames();        // wait for the frame
        rs2:: frame color_frame = frames.get_color_frame();    // get the color

        //  get frame data and convert it to CV MAT

        cv:: Mat color(cv::Size(640, 480), CV_8UC3, 
                          (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);


        // --detect markers--
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector. detectMarkers(color,corners,ids);

        //-- Draw detected markers--

        if(!ids.empty()){
              
            cv::aruco::drawDetectedMarkers(color,corners,ids);

            cout << "Captured marker data. Total frames: "<< allCorners.size()<< endl;
              for (int id : ids){
                if (id==target.markerId){
              cv::putText(color,
            "Yes, Column found: " + target.label,
            cv::Point(20, 50),
            cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 0), 2);

cv::putText(color,
            "(Marker ID: " + std::to_string(id) + ")",
            cv::Point(20, 90),  // 40 px lower
            cv::FONT_HERSHEY_SIMPLEX, 0.9,
            cv::Scalar(0, 255, 0), 2);


                    std::cout << "Yes, Column found: " << target.label
                              << " (Marker ID: " << id << ")\n";
                    found = true;
                }
            }
                if (!found) {
                       cv::putText(color, "Searching for marker ID: " +
                                       std::to_string(target.markerId),
                            cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                            cv::Scalar(0, 0, 255), 2);  

              }
        }
        else {
            cv::putText(color, "No markers detected", cv::Point(20, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0,
                        cv::Scalar(0, 255, 255), 2);

           
        //     // display number of markers detected
        //     std::string msg = "Markers detected:" +std::to_string(ids.size());
        //     cv::putText(color, msg, cv::Point(20, 40),
        //                     cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        //     cout << msg<< endl;

        //     // save the image as jpeg (essentialy saving the data or results)
        //      std::string filename = "detected_frame_result.jpg";
        //      cv::imwrite(filename,color);
        //      std::cout << "🖼️  Saved annotated image: " << filename << std::endl;
        // } else {
        //     std::string msg = "No markers detected";
        //          cv::putText(color, msg, cv::Point(20, 40),
        //                     cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        // }
          
            
    }
cv::imshow("Real Sense camera live feed", color);     // just to put on screen
          int key =cv::waitKey(1);

    
}
 pipe.stop();
     cv::destroyAllWindows();
     std::cout << "🛑 Stopped.\n";
    }

   

catch(const rs2::error & e) {
      std::cerr << "RealSense error calling " << e.get_failed_function()
                  << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
}
 catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}


