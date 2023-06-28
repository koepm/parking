#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <image_transport/image_transport.hpp>


class Parking : public rclcpp::Node
{

	public:
		Parking();

	private:
		void image_processing();
		void lidar_callback(const std_msgs::msg::Bool::SharedPtr msg);
		void mission_callback(const std_msgs::msg::Int16::SharedPtr msg);
		cv::Mat undistort_frame(const cv::Mat& frame); // 이미지 왜곡 보정
		cv::Mat add_hsv_filter(const cv::Mat& frame, const int camera); // HSV 필터 적용
		cv::Point find_ball(const cv::Mat& frame, const cv::Mat& mask);
		void line_symmetry(const cv::Mat& frame, const int camera);
		double* find_center(const cv::Mat& frame, double array[], const int camera);
		cv::Point2d find_xz(const cv::Point2d circle_left, const cv::Point2d circle_right, \
		const cv::Mat& left_frame, const cv::Mat& right_frame, const float alpha, const float beta);
		cv::Mat adapt_th(cv::Mat src);
		cv::Point back_parking(const cv::Mat& frame, const int camera);
		cv::Mat find_edge(const cv::Mat& frame, const int camera);

		//sub
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; // 일단 사용 안함
		rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_flag_; // 현재 진행 미션 플래그
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_flag_; // 라이다 스탑 플래그

		//pub
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_flag_;
		rclcpp::TimerBase::SharedPtr timer_; // 호출 주기

		float baseline = 23;
		float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
		float alpha = 23.9;	//alpha = 카메라 머리 숙인 각도
		float beta = 45.5999; 	//beta = erp 헤딩으로부터 카메라 각도
		float gps_for_camera_x = 30; //cm
		float gps_for_camera_z = -50; //cm
		int target_x = 135;
		int target_z = 135;
		
};