#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main()
{
    try
    {
        // ------------------------------------------------------------
        // ⚙️ Board parameters
        // ------------------------------------------------------------
        const int squaresX = 7;             // squares along X
        const int squaresY = 5;             // squares along Y
        const float squareLength = 0.03f;   // 30 mm
        const float markerLength = 0.022f;  // 22 mm
        const int dpi = 300;                // printer DPI (optional)

        // ------------------------------------------------------------
        // 📘 Dictionary (new API)
        // ------------------------------------------------------------
        cv::aruco::Dictionary dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // ------------------------------------------------------------
        // 🧩 Create ChArUco board (new API style)
        // ------------------------------------------------------------
        cv::aruco::CharucoBoard board(
            cv::Size(squaresX, squaresY),
            squareLength,
            markerLength,
            dictionary
        );

        // ------------------------------------------------------------
        // 🖼️ Generate image
        // ------------------------------------------------------------
        // Convert DPI to pixels per meter (1 inch = 0.0254 m)
        int pixelsPerMeter = static_cast<int>(dpi / 0.0254);

        // Calculate board size in pixels
        int width_px  = static_cast<int>(squaresX * squareLength * pixelsPerMeter);
        int height_px = static_cast<int>(squaresY * squareLength * pixelsPerMeter);

        cv::Mat boardImage;
        board.generateImage(cv::Size(width_px, height_px), boardImage, 10, 1);

        // Add white border for printing
        int border = 100;
        cv::copyMakeBorder(boardImage, boardImage,
                           border, border, border, border,
                           cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

        // ------------------------------------------------------------
        // 💾 Save and preview
        // ------------------------------------------------------------
        std::string filename = "charuco_board_4x4_50.png";
        cv::imwrite(filename, boardImage);

        std::cout << "✅ Saved ChArUco board: " << filename << std::endl;
        std::cout << "   Size (pixels): " << width_px << " × " << height_px << std::endl;
        std::cout << "   Each square:  " << squareLength * 1000 << " mm" << std::endl;
        std::cout << "   Each marker:  " << markerLength * 1000 << " mm" << std::endl;
        std::cout << "🖨️  Print at 100% scale (no fit-to-page)\n";

        cv::imshow("ChArUco Board", boardImage);
        cv::waitKey(0);
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "❌ OpenCV Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
