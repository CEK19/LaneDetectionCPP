#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>

cv::Mat abs_sobel_thresh(const cv::Mat &img, char orient, int thresh_min, int thresh_max);
cv::Mat color_thresh_hold(const cv::Mat &rgb_img, const cv::Point2i sthresh, const cv::Point2i vthresh);
cv::Mat region_of_interest(const cv::Mat &preproc_img);
cv::Mat warpPerpesctiveUserDef(const cv::Mat &bitwise_img);

int main( int argc, char** argv )
{
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap("./Image_Video/map_4_lane_ngoai.avi");

    // Check if camera opened successfully
    if (!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    int INDEX = 0;

    while (true){
        // Capture frame-by-frame
        cv::Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()){
            break;
        }
        
        // ----------------------PREPROCESSING IMAGE---------------------- //
        // cv::imshow("RGB Color", frame);
        // STEP 1: GRAD X & GRAD Y (SOBEL) & COLOR_THRESH_HOLD
        cv::Mat gradX_binary        = abs_sobel_thresh(frame, 'x', 40,  255);
        cv::Mat gradY_binary        = abs_sobel_thresh(frame, 'y', 40, 255);
        cv::Mat c_binary            = color_thresh_hold(frame, cv::Point2i(100, 255), cv::Point2i(50, 255));

        // STEP 2: COMBINE GRAD X + GRAD Y + COLOR THRESH_HOLD TOGETHER
        cv::Mat preprocessImage     = ((gradX_binary == 1) & (gradY_binary == 1) | (c_binary == 1));
        // cv::imshow("preprocess Image", preprocessImage);

        // ----------------------CREATE REGION OF INTEREST---------------------- //
        cv::Mat ROI                 = region_of_interest(preprocessImage);
        // cv::imshow("Region Of Interest", ROI);

        // ----------------------WARP PERPESCTIVE---------------------- //
        cv::Mat WARP_PERPESCTIVE    = warpPerpesctiveUserDef(ROI);
        // cv::imshow("Warp Perpesctive", WARP_PERPESCTIVE);
        cv::Point2i TOP_LEFT_COOR   = cv::Point2i(100, 200);
        int WIDTH_CROP              = 450;
        int HEIGHT_CROP             = 120;
        cv::Mat CROP_REGION         = WARP_PERPESCTIVE(cv::Rect(TOP_LEFT_COOR.x, TOP_LEFT_COOR.y, WIDTH_CROP, HEIGHT_CROP));
        // cv::imshow("Crop REGION", CROP_REGION);

        // ----------------------BINARY THRESHHOLDING---------------------- //
        cv::Mat BIN_THRESH;
        cv::threshold(CROP_REGION, BIN_THRESH, 100, 255, cv::THRESH_BINARY);
        cv::imshow("Binary Thresh Hold", BIN_THRESH);

        // ----------------------LANE DETECTION ALGORITHM---------------------- //
        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat contourOutput = BIN_THRESH.clone();
        cv::findContours(contourOutput, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        //Draw the contours
        cv::Mat contourImage(BIN_THRESH.size(), CV_8UC3, cv::Scalar(0,0,0));
        cv::Scalar colors[3];
        colors[0] = cv::Scalar(255, 0, 0);
        colors[1] = cv::Scalar(0, 255, 0);
        colors[2] = cv::Scalar(0, 0, 255);
        for (size_t idx = 0; idx < contours.size(); idx++) {
            cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
        }

        cv::imshow("Input Image", BIN_THRESH);
        cv::imshow("Contours", contourImage);
        cv::imwrite("./IMAGE_RESULT/" + std::to_string(INDEX) + ".jpg", contourImage);
        INDEX = INDEX + 1;
        // Display the resulting frame


        // Press  ESC on keyboard to exit   
        // break;
        char c = (char)cv::waitKey(25);
        if (c == 27){
            break;
        }
    }

    // When everything done, release the video capture object & destroy all windows
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

cv::Mat abs_sobel_thresh(const cv::Mat &rgb_img, char orient = 'x', int thresh_min=25, int thresh_max=255){
    // STEP 1: RGB TO HLS
    const int ROW_IMG = rgb_img.rows;
    const int COL_IMG = rgb_img.cols;

    cv::Mat HLS_IMG;
    cv::cvtColor(rgb_img, HLS_IMG, cv::COLOR_RGB2HLS);

    // STEP 2: SAVE L-CHANNEL TO VARIABLE
    cv::Mat L_CHANNEL = cv::Mat(ROW_IMG, COL_IMG, CV_8U);
    for (int row = 0; row < ROW_IMG;  ++row){
        for (int col  = 0; col < COL_IMG; ++col){
            L_CHANNEL.at<uint8_t>(row, col) = HLS_IMG.at<cv::Vec3b>(row, col)[1];
        }
    }

    // STEP 3: CACULATE ABS SOBEL    
    cv::Mat grad_x_or_y;
    cv::Mat abs_grad_x_or_y;
    cv::Mat scale_sobel;
    double maxValue, minValue;
    
    if (orient == 'x'){
        cv::Sobel(L_CHANNEL, grad_x_or_y, CV_64F, 1, 0);
    }
    if (orient == 'y'){
        cv::Sobel(L_CHANNEL, grad_x_or_y, CV_64F, 0, 1);        
    }
    abs_grad_x_or_y = cv::abs(grad_x_or_y);
    cv::minMaxIdx(abs_grad_x_or_y, &minValue, &maxValue);   
    abs_grad_x_or_y.convertTo(scale_sobel, CV_64F, 255.0/maxValue);
    
    // STEP 4: RESCALE BACK TO 8 BIT INTEGER
    scale_sobel.convertTo(scale_sobel, CV_8U);

    // STEP 5: RETURN BINARY OUTPUT BETWEEN MIN & MAX THRESHHOLD
    cv::Mat BINARY_OUTPUT = ((scale_sobel >= thresh_min) & (scale_sobel <= thresh_max))/255;

    return BINARY_OUTPUT;
}

cv::Mat color_thresh_hold(const cv::Mat &rgb_img, const cv::Point2i sthresh, const cv::Point2i vthresh){

    // STEP 1: CONVERT RGB TO HLS
    const int ROW_IMG = rgb_img.rows;
    const int COL_IMG = rgb_img.cols;

    cv::Mat HLS_IMG;
    cv::cvtColor(rgb_img, HLS_IMG, cv::COLOR_RGB2HLS);

    // STEP 2: SAVE S-CHANNEL TO VARIABLE
    cv::Mat S_CHANNEL = cv::Mat(ROW_IMG, COL_IMG, CV_8U);
    for (int row = 0; row < ROW_IMG;  ++row){
        for (int col  = 0; col < COL_IMG; ++col){
            S_CHANNEL.at<uint8_t>(row, col) = HLS_IMG.at<cv::Vec3b>(row, col)[2];
        }
    }

    cv::Mat S_BINARY = ((S_CHANNEL > sthresh.x) & (S_CHANNEL <= sthresh.y))/255;

    // STEP 3: SAVE V-CHANNEL TO VARIABLE
    cv::Mat HSV_IMG;
    cv::cvtColor(rgb_img, HSV_IMG, cv::COLOR_RGB2HSV);

    cv::Mat V_CHANNEL = cv::Mat(ROW_IMG, COL_IMG, CV_8U);
    for (int row = 0; row < ROW_IMG; ++row){
        for (int col = 0; col < COL_IMG; ++col){
            V_CHANNEL.at<uint8_t>(row, col) = HSV_IMG.at<cv::Vec3b>(row, col)[2];
        }
    }

    cv::Mat V_BINARY = ((V_CHANNEL > vthresh.x) & (V_CHANNEL <= vthresh.y))/255;

    // STEP 4: COMBINE S_CHANNEL AND V_CHANNEL
    cv::Mat COMBINE_S_V_BINARY = ((S_BINARY == 1) & (V_BINARY == 1))/255;
    
    int count = 0; 
    for (int row = 0; row  < COMBINE_S_V_BINARY.rows; ++row){
        for (int col = 0; col < COMBINE_S_V_BINARY.cols; ++col){
            if(static_cast<int>(COMBINE_S_V_BINARY.at<uint8_t>(row, col)) == 1) ++count;
        }
    }
    return COMBINE_S_V_BINARY;
}

cv::Mat region_of_interest(const cv::Mat &preproc_img){       
    const int COL_IMG = preproc_img.cols;
    const int ROW_IMG = preproc_img.rows;    

    cv::Mat zero_mask = cv::Mat(ROW_IMG, COL_IMG, CV_8U, uint8_t(0));
    cv::Mat BITWISE_OUTPUT;

    std::vector<cv::Point> fillContSingle;
    // Add all points of the contour to the vector
    cv::Point left_bot = cv::Point(0, 320);
    cv::Point left_top = cv::Point(244, 255);
    cv::Point right_top = cv::Point(425, 255);
    cv::Point right_bot = cv::Point(COL_IMG, 320);

    fillContSingle.push_back(left_bot);
    fillContSingle.push_back(left_top);
    fillContSingle.push_back(right_top);
    fillContSingle.push_back(right_bot);
    

    std::vector< std::vector<cv::Point> > fillContAll;
    //fill the single contour 
    // (one could add multiple other similar contours to the vector)
    fillContAll.push_back(fillContSingle);
    cv::fillPoly(zero_mask, fillContAll, cv::Scalar(255, 255, 255));
        
    cv::bitwise_and(preproc_img, zero_mask, BITWISE_OUTPUT);
    return BITWISE_OUTPUT;
}

cv::Mat warpPerpesctiveUserDef(const cv::Mat &bitwise_img) {
    cv::Mat RETURN_MAT;
    cv::Point2f SRC[4];
    cv::Point2f DST[4];

    SRC[0] = cv::Point2f(0, 320);
    SRC[1] = cv::Point2f(240, 255);
    SRC[2] = cv::Point2f(425, 255);
    SRC[3] = cv::Point2f(bitwise_img.cols, 320);

    DST[0] = cv::Point2f(SRC[0].x/2 + SRC[1].x/2, SRC[0].y + 50);
    DST[1] = cv::Point2f(SRC[0].x/2 + SRC[1].x/2, SRC[1].y - 50);
    DST[2] = cv::Point2f(SRC[2].x/2 + SRC[3].x/2, SRC[2].y - 50);
    DST[3] = cv::Point2f(SRC[2].x/2 + SRC[3].x/2, SRC[3].y + 50);

    cv::Mat LAMBDA = cv::getPerspectiveTransform(SRC, DST);
    cv::warpPerspective(bitwise_img, RETURN_MAT, LAMBDA, bitwise_img.size());
    return RETURN_MAT;
}