#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
    try {
        // 🧩 Choose your dictionary
        cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // 📏 Marker parameters
        int markerSize = 500;     // marker pixel size
        int borderBits = 1;
        int spacing = 100;        // spacing between markers
        int whiteMargin = 150;    // white border around the whole layout

        // 🧱 Marker IDs you want to include
        //std::vector<int> markerIds = {1,17,31,23,13,19};
        std::vector<int> markerIds = {19};

        // 🖼️ Create each marker
        std::vector<cv::Mat> markers;
        for (int id : markerIds) {
            cv::Mat marker;
            dictionary.generateImageMarker(id, markerSize, marker, borderBits);
            markers.push_back(marker);
        }

        // 🧾 Layout: horizontally concatenate with spacing
        int totalWidth = (int)(markerIds.size() * markerSize + (markerIds.size() - 1) * spacing);
        int totalHeight = markerSize;
        cv::Mat page(totalHeight, totalWidth, CV_8UC1, cv::Scalar(255)); // white background

        for (size_t i = 0; i < markers.size(); ++i) {
            cv::Rect roi(i * (markerSize + spacing), 0, markerSize, markerSize);
            markers[i].copyTo(page(roi));
        }

        // ➕ Add white border around page
        cv::Mat bordered;
        cv::copyMakeBorder(page, bordered, whiteMargin, whiteMargin, whiteMargin, whiteMargin,
                           cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

        // 🧾 Save and show
        std::string filename = "aruco_two_markers_4x4_today.png";
        cv::imwrite(filename, bordered);
        std::cout << "✅ Saved combined markers as: " << filename << std::endl;

        cv::imshow("Two ArUco Markers", bordered);
        cv::waitKey(0);

    } catch (const cv::Exception &e) {
        std::cerr << "❌ OpenCV Error: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "❌ Unknown error occurred!" << std::endl;
        return -1;
    }

    return 0;
}
