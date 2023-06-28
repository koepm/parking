#include <parking/parking.hpp>
#include <vector>
#include <sstream>

//--------------------------------------------------------------------------------------------------
/**
 * @brief 스테레오 카메라 만들 때 사용하는 함수, 초기 제작할 때 수평을 맞추기 위해서 사용함
 * @param frame 입력 영상
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 */
void Parking::line_symmetry(const cv::Mat& frame, const int camera)
{
	rectangle(frame, cv::Rect(cv::Point(635,0),cv::Point(645,720)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,680),cv::Point(1280,690)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,640),cv::Point(1280,650)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,600),cv::Point(1280,610)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,560),cv::Point(1280,570)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,520),cv::Point(1280,530)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,480),cv::Point(1280,490)),cv::Scalar(0,255,0),2,4,0);
	rectangle(frame, cv::Rect(cv::Point(0,360),cv::Point(1280,370)),cv::Scalar(0,255,0),2,4,0);

	if(camera == 0)
	{
		imshow("left_line_symmetry", frame);
	}
	else
	{
		imshow("right_line_symmetry", frame);
	}
}

//--------------------------------------------------------------------------------------------------
/**
 * @brief 왜곡 보정 함수
 * @param frame 입력 영상
 */
cv::Mat Parking::undistort_frame(const cv::Mat& frame)
{
	// 카메라 내부 파라미터
	cv::Mat intrinsic_param = cv::Mat::zeros(3,3,CV_64FC1); // zeros로 테스트 중
	//cv::Mat intrinsic_param = Mat::eye(3,3,CV_64FC1); // eye를 쓴 이유가 뭐지?
	intrinsic_param=(cv::Mat1d(3,3) << 509.5140, 0, 321.9972, 0, 510.5093, 258.7457, 0., 0., 1. );

	// 카메라 왜곡 계수
	cv::Mat distortion_coefficient = cv::Mat::zeros(1,5,CV_64FC1);
	distortion_coefficient=(cv::Mat1d(1,5) << 0.0891, -0.1673, 0., 0., 0.);

	// 새로운 카메라 매개변수 생성
	// newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionParameters, { frame.cols, frame.rows }, 1);

	cv::Mat undistorted_frame;
	cv::undistort(frame, undistorted_frame, intrinsic_param, distortion_coefficient);

	return undistorted_frame;
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief HSV값으로 영상을 이진화하는 함수
 * 
 * @param frame 입력하고자 하는 화면
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return Mat 
 */
cv::Mat Parking::add_hsv_filter(const cv::Mat& frame, const int camera) {

	cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
	cv::Mat mask;

	// logitech c930e
	std::vector<int> left_lowerYellow = { 10, 160, 100 };     // Lower limit for yellow
	std::vector<int> left_upperYellow = { 40, 255, 255 };	 // Upper limit for yellow
	std::vector<int> right_lowerYellow = { 10, 160, 100 };     // Lower limit for yellow
	std::vector<int> right_upperYellow = { 40, 255, 255 };	 // Upper limit for yellow

	if(camera == 0)
	{
		inRange(frame, left_lowerYellow, left_upperYellow, mask);
	}
	else
	{
		inRange(frame, right_lowerYellow, right_upperYellow, mask);
	}
	
	return mask;
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 캘리용 함수
 * 
 * @param frame 입력 영상
 * @param mask 이진화된 영상 (HSV로 이진화하든 어쨌든 이진화된 영상)
 * @return Point 
 */
cv::Point Parking::find_ball(const cv::Mat& frame, const cv::Mat& mask)
{

	std::vector<std::vector<cv::Point> > contours;

	/*
	cv::RETR_EXTERNAL: 가장 외곽의 윤곽선만 검색.
	cv::CHAIN_APPROX_SIMPLE: 윤곽선 압축하여 저장. ex)직선 부분은 끝점만 저장, 곡선 부분은 시작점과 끝점 사이의 중간 점을 저장.
	*/
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	

	// Sort the contours to find the biggest one
	sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
		return contourArea(c1, false) < contourArea(c2, false);
	});

	if (contours.size() > 0) {

		std::vector<cv::Point> largestContour = contours[contours.size() - 1];
		cv::Point2f center;
		float radius;
		
		minEnclosingCircle(largestContour, center, radius);
		// 0623 아래 부분 수정함.
		//cv::Moments m = moments(largestContour);
		//cv::Point centerPoint(m.m10 / m.m00, m.m01 / m.m00);
		cv::Point centerPoint(center.x, center.y);

		// Only preceed if the radius is greater than a minimum threshold
		if (radius > 10) {
			// Draw the circle and centroid on the frame
			circle(frame, center, int(radius), (0, 255, 255), 2);
			circle(frame, centerPoint, 5, (0, 0, 255), -1);
		}

		return centerPoint;
	}
	return { 0,0 };
}



//--------------------------------------------------------------------------------------------------
cv::Mat Parking::find_edge(const cv::Mat& frame, const int camera) {


	// y값이 왜 영상의 4분의 1지점이지???
	int center_x = frame.cols /2;
	int center_y = frame.rows /4;

	// int center_x = frame.cols /2 - 150;
	// int center_y = frame.rows /4;

	cv::Mat gray_image;
	cvtColor(frame, gray_image, CV_RGB2GRAY);

	// imshow("gray_image", gray_image); 
	GaussianBlur(gray_image,gray_image, cv::Size(3,3),3);
	// imshow("gau", gray_image);
	
	cv::Mat dx, dy;
	// 소벨 -> 엣지 검출
	Sobel(gray_image, dx, CV_32FC1, 1, 0); // 수평 방향 소벨 필터링
	Sobel(gray_image, dy, CV_32FC1, 0, 1); // 수직 방향 소벨 필터링

	cv::Mat fmag, mag;
	magnitude(dx, dy, fmag);
	fmag.convertTo(mag, CV_8UC1);

	// 특정 강도 이상만 표시?
	cv::Mat edge = mag > 80;

	imshow("edge", edge);


	if(camera != 0)
	{
		center_x = center_x - 30;
	}

	cv::Mat dst = cv::Mat::zeros( frame.size(), CV_8UC1 );

	// for (int r = 0; r < frame.rows; r++)
	// {
    //     for (int c = center_x; c < frame.cols-2; c++)
	// 	{
	// 		if((5*gray_image.at<uchar>(r, c) - gray_image.at<uchar>(r, c+1) - gray_image.at<uchar>(r, c+2) - gray_image.at<uchar>(r, c+3) - gray_image.at<uchar>(r, c+4) - gray_image.at<uchar>(r, c+5)) > 80) // 60
	// 		{
	// 			dst.at<uchar>(r, c-3) = 255;
	// 			dst.at<uchar>(r, c-2) = 255;
	// 			dst.at<uchar>(r, c-1) = 255;
	// 			dst.at<uchar>(r, c) = 255;
	// 			dst.at<uchar>(r, c+1) = 255;
	// 			dst.at<uchar>(r, c+2) = 255;
	// 			dst.at<uchar>(r, c+3) = 255;
	// 		}
    //     }
    // }

	// for (int r = 0; r < frame.rows; r++)
	// {
    //     for (int c = 0; c < center_x; c++)
	// 	{
	// 		if((gray_image.at<uchar>(r, c+1) + gray_image.at<uchar>(r, c+2) + gray_image.at<uchar>(r, c+3) + gray_image.at<uchar>(r, c+4) + gray_image.at<uchar>(r, c+5)) - 5*gray_image.at<uchar>(r, c) > 80) // 60
	// 		{
	// 			dst.at<uchar>(r, c-1) = 255;
	// 			dst.at<uchar>(r, c) = 255;
	// 			dst.at<uchar>(r, c+1) = 255;
	// 		}
    //     }
    // }

	// imshow("dst", dst);

	// subtract(edge, dst, edge);
	// imshow("Minus", edge);

	circle(gray_image, cv::Point(center_x,center_y), 10, cv::Scalar(255, 255, 255), 2);

	if(camera == 0)
	{
		// imshow("left circle mean", gray_image);
	}
	else
	{
		// imshow("right circle mean", gray_image);
	}



	dilate(edge, edge, cv::Mat(), cv::Point(-1, -1), 1);
	// erode(edge, edge, Mat(), Point(-1, -1), 1);

	// imshow("final ", edge);

	return edge;
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 이진화한 영상에서 중심점을 찾고 하단, 중단, 상단의 x,y좌표를 double 형의 자료형으로 저장함
 * 
 * @param img 입력 영상 (이진화된 영상을 넣으면 됨)
 * @param array double자료형의 배열(바닥 x,y, 가운데 x,y, 상단 x,y 를 저장함)
 * @param camera 카메라 번호 0 = 왼쪽, 1 = 오른쪽
 * @return double* 
 */
double* Parking::find_center(const cv::Mat& frame, double array[], const int camera)
{
	int center_x = frame.cols /2;
	int center_y = frame.rows /4;

	cv::Mat mask = frame.clone();
    // contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
    double nowarea = 0;

	std::vector<cv::Point2f> approx;

	int min_distance = 1000;

	for (size_t i = 0; i < contours.size(); i++)
	{
		approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
		int vertex = approx.size();
		
		cv::Moments m = moments(contours[i], true);
		cv::Point p(m.m10/m.m00, m.m01/m.m00);
		
		std::vector<cv::Point> hull;
        cv::convexHull(cv::Mat(contours[i]), hull, false);

		nowarea = cv::contourArea(cv::Mat(hull));

		// cout << nowarea << endl;

		if(vertex > 3)
		{
			if ((nowarea > 35000) && (nowarea < 100000))	//35000	// 일단 25000이었음
			{
				if(sqrt(pow((p.x-center_x),2)+pow((p.y-center_y),2)) < min_distance)
				{
					min_distance = sqrt(pow((p.x-center_x),2)+pow((p.y-center_y),2));
					lrgctridx = i;
					// cout << " 넓이 : " << nowarea << endl;
				}
			}
		}
	}

	// cout << "contour size = " << minarea << endl;

    if(contours.size() > 0)
    {
        cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );

        // 모든 외각선 그리기
        // for(int idx = 0; idx >= 0; idx = hierarchy[idx][0])
        // {
        //     drawContours( drawing, contours, idx, color, 2, LINE_8, hierarchy);
        // }

        // 특정한 외각선만 그리기
        drawContours( drawing, contours, lrgctridx, color, 2, cv::LINE_8, hierarchy);

        std::vector<cv::Point> hull;
        cv::convexHull(cv::Mat(contours[lrgctridx]), hull, false);

        std::vector<std::vector<cv::Point>> fake_hull;
        fake_hull.push_back(hull);
        drawContours(drawing, fake_hull, 0, color, 2, cv::LINE_8);

        int top_x_left = hull[0].x;
	    int top_y_left = hull[0].y;
        int top_num_left = 0;

        int bottom_x_left = hull[0].x;
        int bottom_y_left = hull[0].y;
        int bottom_num_left = 0;

        int top_x_right = hull[0].x;
	    int top_y_right = hull[0].y;
        int top_num_right = 0;

        int bottom_x_right = hull[0].x;
        int bottom_y_right = hull[0].y;
        int bottom_num_right = 0;

        for(int i = 0; i < hull.size(); i++)
        {
            // if(hull[i].y < top_y_left)
            // {
            //     top_x_left = hull[i].x;
            //     top_y_left = hull[i].y;
            //     top_num_left = i;
            // }
            if(sqrt(pow(hull[i].x - frame.cols*7/8, 2) + pow(hull[i].y - 0, 2)) < sqrt(pow(top_x_right - frame.cols*7/8, 2) + pow(top_y_right - 0, 2)))
            {
                top_x_right = hull[i].x;
                top_y_right = hull[i].y;
                top_num_right = i;
            }
            if((1*sqrt(pow(hull[i].x - 0, 2) + pow(hull[i].y - frame.rows, 2)) + 9*hull[i].x) < (1*sqrt(pow(bottom_x_left - 0, 2) + pow(bottom_y_left - frame.rows, 2)) + 9*bottom_x_left))
            {
                bottom_x_left = hull[i].x;
                bottom_y_left = hull[i].y;
                bottom_num_left = i;
            }
            if(sqrt(pow(hull[i].x - frame.cols, 2) + pow(hull[i].y - frame.rows, 2)) < sqrt(pow(bottom_x_right - frame.cols, 2) + pow(bottom_y_right - frame.rows, 2)))
            {
                bottom_x_right = hull[i].x;
                bottom_y_right = hull[i].y;
                bottom_num_right = i;
            }
        }

		double daegaksun = sqrt(pow(bottom_x_left - top_x_right, 2) + pow(bottom_y_left - top_y_right, 2));
		double long_sin = 0;
		double fake_sin;

		for(int j=0; j < hull.size(); j++)
		{
			if((hull[j].y < bottom_y_left -50) && (hull[j].x < top_x_right - 50) && (hull[j].x < bottom_x_right - 50))
			{
				double sasun1 = sqrt(pow(hull[j].x - bottom_x_left, 2) + pow(hull[j].y - bottom_y_left, 2));
				double sasun2 = sqrt(pow(hull[j].x - top_x_right, 2) + pow(hull[j].y - top_y_right, 2));

				double theta = acos((pow(sasun1, 2) - pow(sasun2, 2) + pow(daegaksun, 2)) / (2*sasun1*daegaksun));

				fake_sin = sasun1 * sin(theta);

				if(fake_sin > long_sin)
				{
					long_sin = fake_sin;

					top_x_left = hull[j].x;
					top_y_left = hull[j].y;
				}

			}
		}

        double mean_top_x = (double)((top_x_left + top_x_right) / 2.0);
        double mean_top_y = (double)((top_y_left + top_y_right) / 2.0);
		double mean_mid_x = (double)((top_x_left+top_x_right+bottom_x_left+bottom_x_right)/4.0);
		double mean_mid_y = (double)((top_y_left+top_y_right+bottom_y_left+bottom_y_right)/4.0);
        double mean_bottom_x = (double)((bottom_x_left + bottom_x_right) / 2.0);
        double mean_bottom_y = (double)((bottom_y_left + bottom_y_right) / 2.0);

        circle(drawing, cv::Point((int)mean_top_x, (int)mean_top_y), 10, cv::Scalar(0, 0, 255), -1);
		circle(drawing, cv::Point((int)mean_mid_x, (int)mean_mid_y), 10, cv::Scalar(0, 255, 0), -1);
        circle(drawing, cv::Point((int)mean_bottom_x, (int)mean_bottom_y), 10, cv::Scalar(255, 0, 0), -1);
        
        circle(drawing, cv::Point(top_x_left, top_y_left), 4, cv::Scalar(255, 255, 255), -1);
        circle(drawing, cv::Point(top_x_right, top_y_right), 4, cv::Scalar(255, 255, 255), -1);
        circle(drawing, cv::Point(bottom_x_left, bottom_y_left), 4, cv::Scalar(255, 255, 255), -1);
        circle(drawing, cv::Point(bottom_x_right, bottom_y_right), 4, cv::Scalar(255, 255, 255), -1);

        array[0] = mean_bottom_x;
		array[1] = mean_bottom_y;
		array[2] = mean_mid_x;
		array[3] = mean_mid_y;
		array[4] = mean_top_x;
		array[5] = mean_top_y;

		if(camera == 0)
		{
			imshow("Left", drawing);
		}
		else
		{
			imshow("Right", drawing);
		}
		
		return array;
    }
    else
    {
        return array;
    }
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 실제 erp의 GPS를 기준으로 X, Z값을 계산하는 함수
 * 
 * left_point와 right_point는 서로 대응되는 점을 넣어야 함
 * ex) {left_array[0],left_array[1]}를 넣으면 {right_array[0],right_array[1]}를 넣어야 함
 * 
 * @param left_point find_center 함수로 구한 좌측 카메라에서의 x,y값
 * @param right_point find_center 함수로 구한 우측 카메라에서의 x,y값
 * @param left_frame 왼쪽화면 입력영상
 * @param right_frame 오른쪽화면 입력영상
 * @param alpha 카메라가 고개를 숙인 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @param beta 카메라가 틀어진 각도 (처음 find_ball함수를 이용해 캘리를 하면서 구함)
 * @return Point2d (실제 X거리값, 실제 Z거리값, cm단위임)
 */
cv::Point2d Parking::find_xz(const cv::Point2d circle_left, const cv::Point2d circle_right, \
const cv::Mat& left_frame, const cv::Mat& right_frame, const float alpha, const float beta)
{
	float x_0 = 0;
	float y_0 = 0;
	if ((right_frame.cols == left_frame.cols) && (right_frame.rows == left_frame.rows))
	{	
		x_0 = right_frame.cols/2;
		y_0 = right_frame.rows/2;
	}
	else {
		std::cout << "Left and Right Camera frames do not have the same pixel width" << std::endl;	
	}

	float xLeft = circle_left.x;
	float xRight = circle_right.x;
	float yLeft = circle_left.y;
	float yRight = circle_right.y;

	float realX = 0;
	float realY = 0;
	float realZ = 0;
	float distance = 0;

	if(xLeft != x_0)
	{
		realX = (float)Parking::baseline/(1 - (x_0 - xRight)/(x_0 - xLeft));
		realZ = abs(realX*Parking::focal_pixels/(x_0 - xLeft));
	}
	else if(xRight != x_0)
	{
		realX = -(float)Parking::baseline/(1 - (x_0 - xLeft)/(x_0 - xRight));
		realZ = abs(realX*Parking::focal_pixels/(x_0 - xRight));
		realX = realX + (float)Parking::baseline; //왼쪽 카메라 기준
	}
	else
	{
		realX = 0;
		realY = 0;
	}
	realY = realZ*(2*y_0-yLeft-yRight)/(2*Parking::focal_pixels);

	distance = sqrt(pow(realX,2)+pow(realY,2) + pow(realZ,2));

	std::cout << " realX : " << realX << "   realY : "<< realY << "     realZ : " << realZ << std::endl << std::endl;
	// cout << " distance : " << distance << endl;
	
	// cout << " 영점 조절 : " << realX << endl;
	
	//ERP 기준 좌표로 변환
	Parking::alpha = alpha * CV_PI / 180;
	Parking::beta = beta * CV_PI / 180;

	float fakeZ = realZ;
	float fakeY = realY;

	float theta = atan(fakeY/fakeZ) - alpha;
	realZ = sqrt(pow(fakeZ,2)+pow(fakeY,2))*cos(theta);
	realY = sqrt(pow(fakeZ,2)+pow(fakeY,2))*sin(theta);

	float realZ_copy = realZ;
	float realX_copy = realX;

	float gama = atan(realX/realZ) + beta;
	realZ = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*cos(gama);
	realX = sqrt(pow(realZ_copy,2)+pow(realX_copy,2))*sin(gama);
	
	// float angle = 0;
	// angle = atan(realX/realZ)*180/CV_PI;

	// cout << "realZ : " << realZ << "  realX : " << realX << endl <<endl;
	return {realZ, realX};
}



//--------------------------------------------------------------------------------------------------
/**
 * @brief 그림자 문제를 해결하기 위해 적응형 이진화 함수를 사용해 색공간이 아닌 다른 방법으로 영상을 이진화함
 * 
 * @param src 입력영상 
 * @return Mat 이진화된 영상 
 */
cv::Mat Parking::adapt_th(cv::Mat src)
{
	cv::Mat image;
	cv::Mat binary;

	image = src.clone();

	resize(image, image, cv::Size(640, 480));

	imshow("cap", image);

	cvtColor(image, binary, CV_BGR2GRAY);
	// namedWindow("dst");
	// createTrackbar("Block_Size", "dst", 0, 200, on_trackbar, (void*)&binary);
	// setTrackbarPos("Block_Size", "dst", 11);

	adaptiveThreshold(binary, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 9, 5);

	cv::Mat binary_inv;

	binary_inv = ~binary;

	// morphologyEx(binary, binary, MORPH_OPEN, Mat(), Point(-1, -1), 3);

	morphologyEx(binary_inv, binary_inv, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

	erode(binary_inv, binary_inv, cv::Mat());

	resize(binary_inv, binary_inv, cv::Size(640, 480));

	imshow("dst", binary_inv);

	return binary_inv;
}

//-------------------------  사용 안하는 코드!!!  -------------------------------------
//void line_symmetry(const cv::Mat& frame, const int camera);
/**
 * @brief 평행주차용으로 개발했으나 라바콘이 예상과는 다르게 사용되어 쓸 일이 없음
 * 
 * @param img 
 * @param camera 
 * @return Point
 */
/*
Point StereoVision::back_park(Mat &img, int camera)
{
	Mat mask = img.clone();

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	dilate(mask, mask, Mat::ones(Size(3, 3), CV_8UC1), Point(-1, -1), 2);

	findContours(mask, contours, hierarchy,RETR_LIST, CHAIN_APPROX_SIMPLE);

	Scalar color(rand() & 255, rand() & 255, rand() & 255);

    int lrgctridx = 0;
    int minarea = 1000000;
	int maxarea = 0;
    double nowarea = 0;
	bool find_contour = false;
	Point contour_moment = {0,0};
	int minDistance = 10000;

	for (size_t i = 0; i < contours.size(); i++)
	{
		nowarea = contourArea(contours[i]);
		Moments m = moments(contours[i], true);
		Point p(m.m10/m.m00, m.m01/m.m00);

		float d = sqrt(pow(p.x-img.cols*5/6,2)+pow(p.y-img.rows*5/6,2));

		// if ((nowarea > 4000) && (nowarea < 30000) && d > 300)
		// {
		// 	if(nowarea > maxarea)
		// 	{
		// 		maxarea = nowarea;
		// 		lrgctridx = i;
		// 		contour_moment = p;
		// 		find_contour = true;
		// 		// cout << " size : " << nowarea << endl;
		// 	}
		// }
		// cout << "!!! : " << sqrt(pow(img.cols/6,2)+pow(img.rows/6,2)) << endl;
		if ((nowarea > 2000) && (nowarea < 30000) && (d > 250))
		{
			if(d < minDistance)
			{
				minDistance = d;
				lrgctridx = i;
				contour_moment = p;
				find_contour = true;
				// cout << " size : " << nowarea << endl;
				// cout << " d : " << d << endl;
			}
		}
	}

	int bottom_x = 0;
	int bottom_y = 0;

	if(find_contour == true)
	{
		Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );

		// drawContours( mask, contours, lrgctridx, Scalar(255, 255, 255), 2, LINE_8, hierarchy);

        vector<Point> hull;
        convexHull(Mat(contours[lrgctridx]), hull, false);

        vector<vector<Point>> fake_hull;
        fake_hull.push_back(hull);
        drawContours(drawing, fake_hull, 0, color, 4, LINE_8);

		bottom_x = hull[0].x;
        bottom_y = hull[0].y;

		for(int i = 0; i < hull.size(); i++)
        {
            if(hull[i].y > bottom_y)
            {
                bottom_x = hull[i].x;
                bottom_y = hull[i].y;
            }
        }

		circle(drawing, {bottom_x,bottom_y}, 8, Scalar(0, 255, 0), -1);

        circle(drawing, contour_moment, 8, Scalar(0, 0, 255), -1);

		if(camera == 0)
		{
			imshow("Left", drawing);
		}
		else
		{
			imshow("Right", drawing);
		}
		
		return {bottom_x,bottom_y}; // contour_moment
    }
    else
    {
        return {bottom_x,bottom_y}; //contour_moment
    }
}
*/