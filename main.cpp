#include <opencv2/opencv.hpp>
#include <time.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <vector>

class DetectComponent {
    private:
        std::vector<cv::Point2i> collectionDataPoint;
        int length; // CACULATE BASED ON LINE INTEGRAL
        cv::Point2i coor_start;
        cv::Point2i coor_end;
    public:
        DetectComponent() {
            this->length        = 0;
            this->coor_start    = cv::Point2i(0, 0);
            this->coor_end      = cv::Point2i(0, 0);
        }

        int getLength() {
            return this->length;
        }

        void setLength(int newLength) {
            this->length = newLength;
        }

        cv::Point2i getCoorStart() {
            return coor_start;
        }

        void setCoorStart(cv::Point2i newPoint) {
            this->coor_start = newPoint;
        }

        cv::Point2i getCoorEnd() {
            return coor_end;
        }

        void setCoorEnd(cv::Point2i newPoint) {
            this->coor_end = newPoint;
        }

        std::vector<cv::Point2i> getCollectionPoint() {
            return this->collectionDataPoint;
        }

        void addPoint(cv::Point2i newPoint) {
            (this->collectionDataPoint).push_back(newPoint);
        }
};

#define BLACK_BINARY_VAL    0
#define WHITE_BINARY_VAL    1
#define BLACK_DEC_VAL       0
#define WHITE_DEC_VAL       255

cv::Mat abs_sobel_thresh(const cv::Mat &img, char orient, int thresh_min, int thresh_max);
cv::Mat color_thresh_hold(const cv::Mat &rgb_img, const cv::Point2i sthresh, const cv::Point2i vthresh);
cv::Mat region_of_interest(const cv::Mat &preproc_img);
cv::Mat warpPerpesctiveUserDef(const cv::Mat &bitwise_img);
cv::Mat line_center_method(const cv::Mat &BIN_THRESH);
cv::Mat lane_detection_algo(const cv::Mat &FILTER_BASE_CENTER);

// ----------------------SET-UP FILE NAME INFORMATION---------------------- //
const std::string DIR_NAME      = "./Image_Video/";
const std::string TAIL_NAME     = ".avi";
const std::string FILE_NAME     = "vatcanvip"; // ABLE TO CHANGE !!!

// ----------------------1. THRESH HOLD VARIABLE---------------------- //
const int GRAD_MIN_THRESH_HOLD  = 40; // ABLE TO CHANGE
const int GRAD_MAX_THRESH_HOLD  = 255; // ABLE TO CHANGE
const cv::Point2i S_THRESH_HOLD = cv::Point2i(100, 255); // ABLE TO CHANGE
const cv::Point2i V_THRESH_HOLD = cv::Point2i(50, 255); // ABLE TO CHANGE
const int MIN_BIN_THRESHOLD     = 100;
const int MAX_BIN_THRESHOLD     = 255;

// ----------------------2. POINT TO WARP PERPESCTIVE--------------------- //
const int COL_ORG_IMG           = 640; // ABLE TO CHANGE
const int WIDTH_ORG_IMG         = 480; // ABLE TO CHANGE
const cv::Point left_bot        = cv::Point(0, 320); // ABLE TO CHANGE
const cv::Point left_top        = cv::Point(244, 255); // ABLE TO CHANGE
const cv::Point right_top       = cv::Point(425, 255); // ABLE TO CHANGE
const cv::Point right_bot       = cv::Point(COL_ORG_IMG, 320); // ABLE TO CHANGE

// ----------------------3. CROPPING PURPOSE (RECTANGLE) ---------------------- //
const cv::Point2i TOP_LEFT_COOR = cv::Point2i(100, 200);
const int WIDTH_CROP            = 450;
const int HEIGHT_CROP           = 120;

int main( int argc, char** argv )
{
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(DIR_NAME + FILE_NAME + TAIL_NAME);
    // Check if camera opened successfully
    if (!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    int INDEX = 0;

    while (true){
        // Capture frame-by-frame
        auto begin_time = std::chrono::high_resolution_clock::now();
        cv::Mat frame;
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()){
            break;
        }
        
        // ----------------------PREPROCESSING IMAGE---------------------- //
        cv::imshow("RGB Color", frame);
        // STEP 1: GRAD X & GRAD Y (SOBEL) & COLOR_THRESH_HOLD
        cv::Mat gradX_binary        = abs_sobel_thresh(frame, 'x', GRAD_MIN_THRESH_HOLD, GRAD_MAX_THRESH_HOLD);
        cv::Mat gradY_binary        = abs_sobel_thresh(frame, 'y', GRAD_MIN_THRESH_HOLD, GRAD_MAX_THRESH_HOLD);
        cv::Mat c_binary            = color_thresh_hold(frame, S_THRESH_HOLD, V_THRESH_HOLD);

        // STEP 2: COMBINE GRAD X + GRAD Y + COLOR THRESH_HOLD TOGETHER
        cv::Mat preprocessImage     = ((gradX_binary == WHITE_BINARY_VAL) & (gradY_binary == WHITE_BINARY_VAL) | (c_binary == WHITE_BINARY_VAL));
        cv::imshow("preprocess Image", preprocessImage);

        // ----------------------CREATE REGION OF INTEREST---------------------- //
        cv::Mat ROI                 = region_of_interest(preprocessImage);

        // ----------------------WARP PERPESCTIVE---------------------- //
        cv::Mat WARP_PERPESCTIVE    = warpPerpesctiveUserDef(ROI);
        // cv::imshow("Warp Perpesctive", WARP_PERPESCTIVE);
        cv::Mat CROP_REGION         = WARP_PERPESCTIVE(cv::Rect(TOP_LEFT_COOR.x, TOP_LEFT_COOR.y, WIDTH_CROP, HEIGHT_CROP));

        // ----------------------BINARY THRESHHOLDING---------------------- //
        cv::Mat BIN_THRESH;
        cv::threshold(CROP_REGION, BIN_THRESH, MIN_BIN_THRESHOLD, MAX_BIN_THRESHOLD, cv::THRESH_BINARY);
        cv::imshow("Binary Thresh Hold", BIN_THRESH);

        // ----------------------LANE DETECTION ALGORITHM---------------------- //
        // PHASE 1: LINE CENTER METHOD
        cv::Mat FILTER_BASE_CENTER  = line_center_method(BIN_THRESH);
        cv::imshow("FILTER", FILTER_BASE_CENTER);

        // PHASE 2: CHOOSE 2 LANE BASE ON GAP, LENGTH AND ANGULAR
        cv::Mat DETECT_LANE         = lane_detection_algo(FILTER_BASE_CENTER);

        // Press  ESC on keyboard to exit   
        // break;
        auto end_time = std::chrono::high_resolution_clock::now();
        float fps = 1/(std::chrono::duration<double, std::milli>(end_time-begin_time).count()/1000);
        std::cout << fps << std::endl; 

        // PHASE 2: DETECT WHICH HAVE HIGHER % PROBALITIES BE LANE ()

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
    cv::Point roi_left_bot = left_bot;
    cv::Point roi_left_top = left_top;
    cv::Point roi_right_top = right_top;
    cv::Point roi_right_bot = right_bot;

    fillContSingle.push_back(roi_left_bot);
    fillContSingle.push_back(roi_left_top);
    fillContSingle.push_back(roi_right_top);
    fillContSingle.push_back(roi_right_bot);
    

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

    SRC[0] = left_bot;
    SRC[1] = left_top;
    SRC[2] = right_top;
    SRC[3] = right_bot;

    DST[0] = cv::Point2f(SRC[0].x/2 + SRC[1].x/2, SRC[0].y + 50);
    DST[1] = cv::Point2f(SRC[0].x/2 + SRC[1].x/2, SRC[1].y - 50);
    DST[2] = cv::Point2f(SRC[2].x/2 + SRC[3].x/2, SRC[2].y - 50);
    DST[3] = cv::Point2f(SRC[2].x/2 + SRC[3].x/2, SRC[3].y + 50);

    cv::Mat LAMBDA = cv::getPerspectiveTransform(SRC, DST);
    cv::warpPerspective(bitwise_img, RETURN_MAT, LAMBDA, bitwise_img.size());
    return RETURN_MAT;
}

cv::Mat line_center_method(const cv::Mat &BIN_THRESH) {
    int ROWS_MAX = BIN_THRESH.rows;
    int COLS_MAX = BIN_THRESH.cols;
    int X_CENTER = COLS_MAX/2;
    cv::Mat FILTER_BASE_CENTER = cv::Mat(ROWS_MAX, COLS_MAX, CV_8U, uint8_t(0));

    for (int y = ROWS_MAX - 1; y >= 0; --y){
        for (int x = X_CENTER - 1; x >= 0; --x){
            if (static_cast<int>(BIN_THRESH.at<uint8_t>(y, x)) != 0){
                FILTER_BASE_CENTER.at<uint8_t>(y, x) = BIN_THRESH.at<uint8_t>(y, x);
                break;
            }
        }
        for (int x = X_CENTER - 1; x < COLS_MAX; ++x){
            if (static_cast<int>(BIN_THRESH.at<uint8_t>(y, x)) != 0){
                FILTER_BASE_CENTER.at<uint8_t>(y, x) = BIN_THRESH.at<uint8_t>(y, x);
                break;
            }
        }
    }
    return FILTER_BASE_CENTER;
}

cv::Mat lane_detection_algo(const cv::Mat &FILTER, const cv::Mat &PRE_FILTER) {
    const int MAX_ROW = FILTER.rows;
    const int MAX_COL = FILTER.cols;

    const int MARKED  = 1;
    const int UNMARKED = 0;
    cv::Mat MARKING_MATRIX = cv::Mat(MAX_ROW, MAX_COL, CV_8U, uint8_t(UNMARKED));

    const int center_point_x = MAX_COL/2;
    const int most_left_pixel_idx = 0;
    const int right_left_pixel_idx = MAX_COL - 1;

    for (int y = MAX_ROW - 1; y >= 0; --y) {
        for (int x = center_point_x; x >=  most_left_pixel_idx; --x) {
            if (static_cast<int>(FILTER.at<uint8_t>(y, x)) == WHITE_BINARY_VAL && 
                static_cast<int>(MARKING_MATRIX.at<uint8_t>(y, x)) == UNMARKED) 
            {
                
            }
        }
    }

    for (int  y = MAX_ROW -  1; y >= 0; --y) {
        for (int x = center_point_x + 1; x <= right_left_pixel_idx; ++x) {

        }
    }
    return ;
}