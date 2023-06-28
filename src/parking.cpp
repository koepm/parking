#include "parking/parking_func.cpp"
#include <chrono>
#include <thread>
#define LEFT_CAM 0
#define RIGHT_CAM 4

// 아직 변수 정리 중
int array_count = 0;
int boom_count = 0;
int mission_flag = 0;
bool Lidar_Stop = false;

cv::Mat img_color;
cv::Mat img_color_2;
int H, S, V;

bool finish_park = false;

bool impulse = false;

float sum_array[6] ={0,0,0,0,0,0};
cv::Point ptOld1;

Parking::Parking()
: Node("parking_node")
{

    lidar_flag_= this->create_subscription<std_msgs::msg::Bool>(
        "lidar_flag", 10, std::bind(&Parking::lidar_callback, this, std::placeholders::_1));
	
	mission_flag_= this->create_subscription<std_msgs::msg::Int16>(
        "mission_flag", 10, std::bind(&Parking::mission_callback, this, std::placeholders::_1));

	//센터 포인트 pub해야됨!!
	//center_XZ_pub = nh.advertise<std_msgs::Float64MultiArray>("Vision/diag_points", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Parking::image_processing, this));
}

/**
 * @brief 라이더로부터 플래그를 받음 Lidar_Stop
 * Lidar_Stop = true 인 경우 level2로 넘어감
 * @param msg
 */
void Parking::lidar_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
	Lidar_Stop = msg->data;
}

/**
 * @brief 플래닝으로부터 미션 번호를 받음
 * 10 = 뭐야
 * 6 = 또 뭐야
 * @param msg
 */
void Parking::mission_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
	mission_flag = msg->data;
}


void Parking::image_processing()
{
    cv::VideoCapture capLeft;
	cv::VideoCapture capRight;

	// 카메라 열기
	if (!capLeft.open(LEFT_CAM)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open left cam");
		return;
	}
	else if (!capRight.open(RIGHT_CAM)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open right cam");
		return;
	}

	capLeft.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capLeft.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capRight.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capRight.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	cv::Mat leftFrame, rightFrame;
    cv::Mat leftMask, rightMask;

	capLeft.read(leftFrame);
	capRight.read(rightFrame);
	
    cv::Point leftCircle, rightCircle; // using yellow ball
    cv::Point2d top_XZ, mid_XZ, bottom_XZ; // using parking

	Parking::line_symmetry(leftFrame, LEFT_CAM);
	Parking::line_symmetry(rightFrame, RIGHT_CAM);

	cv::rectangle(leftFrame, cv::Rect(0, 0, 1280, 100), cv::Scalar(0, 0, 0), -1); //상단 for koreatech
	cv::rectangle(rightFrame, cv::Rect(0, 0, 1280, 100), cv::Scalar(0, 0, 0), -1); //상단 for koreatech
	// rectangle(leftFrame, Rect(0, 400, 1280, 720), Scalar(0, 0, 0), -1); //for k-citys
	// rectangle(rightFrame, Rect(0, 0, 1280, 200), Scalar(0, 0, 0), -1); //for k-citys

	//imshow("rightFrame", rightFrame);
	//imshow("leftFrame", leftFrame);

	if((mission_flag == 10)||(mission_flag == 6))
	{
		// =================[ using parking ]=========================
		// system("clear");
		// cout << "---------------------------------------------" << endl;
		// cout << "Lidar_Stop : " << Lidar_Stop << "  finish_park : " << finish_park << endl;	

		// imshow("rightFrame", rightFrame);
		// imshow("leftFrame", leftFrame);

		// leftMask = stereovision.find_edge(leftFrame, 0);
		// rightMask = stereovision.find_edge(rightFrame, 1);

		// =================[ using yellow ball ]=======================
		leftMask = Parking::add_hsv_filter(leftFrame, LEFT_CAM);
		rightMask = Parking::add_hsv_filter(rightFrame, RIGHT_CAM);
		cv::Point2f ball_XZ;
		leftCircle = Parking::find_ball(leftMask, leftMask);
		rightCircle = Parking::find_ball(rightMask, rightMask);
		ball_XZ = Parking::find_xz(leftCircle, rightCircle, leftFrame, rightFrame, Parking::alpha, Parking::beta);
		
		// ==================[ using mouse_callback ]===================
		// img_color = leftFrame.clone();
		// img_color_2 = rightFrame.clone();
		// setMouseCallback("Left Frame", mouse_callback);
		// setMouseCallback("Right Frame", mouse_callback_2);

		// setMouseCallback("Left Frame",on_mouse);
		// setMouseCallback("Right Frame",on_mouse);

		imshow("rightMask", rightMask);
		imshow("leftMask", leftMask);

		if((Lidar_Stop == false) && (finish_park == false))
		{
			// cout << " Wating Lidar Stop" << endl;
			// imshow("Left Frame", leftFrame);
			// imshow("Right Frame", rightFrame);
		}
		else if((Lidar_Stop == true) && (finish_park == false))
		{	
			if(impulse == false)
			{
				// erp정지시 흔들림에 의해 생기는 오차 방지
				std::cout << "impulse !!" << std::endl;
				//ros::Duration(1.5).sleep(); // (구)1.5초 정지 코드
				std::chrono::milliseconds duration(1500);
    			std::this_thread::sleep_for(duration);
				impulse = true;
			}
		
			imshow("Left_mask", leftMask);
			imshow("Right_mask", rightMask);

			double left_array[6] = {0,0,0,0,0,0};
			double right_array[6] = {0,0,0,0,0,0};
			double array[6] = {0,0,0,0,0,0};
			double pub_array[6] = {0,0,0,0,0,0};
			double default_array[6] = {2.4, -2.2, 3.65, -4.2, 4.9, -6.2}; // k-city에 맞춘 이상적인 주차구역 좌표


			double *ptr_left = Parking::find_center(leftMask, left_array, LEFT_CAM);
			double *ptr_right = Parking::find_center(rightMask, right_array, RIGHT_CAM);

			if (left_array[0] && right_array[0])
			{
				bottom_XZ = Parking::find_xz({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame, Parking::alpha, Parking::beta);
				// mid_XZ = stereovision.`({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame, Parking::alpha, Parking::beta);
				top_XZ = Parking::find_xz({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame, Parking::alpha, Parking::beta);
				mid_XZ.x = (bottom_XZ.x + top_XZ.x)/2.0;
				mid_XZ.y = (bottom_XZ.y + top_XZ.y)/2.0;

				std::cout << "bottom_XZ : " << bottom_XZ << std::endl;
				std::cout << "mid_XZ : " << mid_XZ << std::endl;
				std::cout << "top_XZ : " << top_XZ << std::endl;

				array[0]= (bottom_XZ.x + Parking::gps_for_camera_z)/100.00;
				array[1]= -(bottom_XZ.y + Parking::gps_for_camera_x)/100.00;
				array[2]= (mid_XZ.x + Parking::gps_for_camera_z)/100.00;
				array[3]= -(mid_XZ.y + Parking::gps_for_camera_x)/100.00;
				array[4]= (top_XZ.x + Parking::gps_for_camera_z)/100.00;
				array[5]= -(top_XZ.y + Parking::gps_for_camera_x)/100.00;

				//############################################ 보험용 #################################################################################################
				if((abs(array[0]-default_array[0]) > 1.3) || (abs(array[1]-default_array[1]) > 1.3) || \
				(abs(array[2]-default_array[2]) > 1.3) || (abs(array[3]-default_array[3]) > 1.3) || \
				(abs(array[4]-default_array[4]) > 1.3) || (abs(array[5]-default_array[5]) > 1.3))
				{
					std::cout << "고정점 ++" << std::endl;
					boom_count++;

					for(int i=0; i < 6; i++)
					{
						array[i] = default_array[i];
					}
				}
				
				//find_xz 이거 왜 있는거지?????????????

				for(int i=0; i<6; i++)
				{
					sum_array[i] = sum_array[i] + array[i];
				}				

				array_count++;
				// cout << "array_count : " << array_count << endl;

				// if(array_count == 20)
				// {
				// 	if(boom_count > 15)
				// 	{
				// 		cout << "주차실패 ㅠㅠㅠㅠ" << endl;
				// 	}
				// 	else
				// 	{
				// 		cout << " 봄 ? \n 이게 바로 비전 클라스 우리 잘못 아니니 뭐라 하려면 제어탓. \n ^~^" << endl;
				// 	}
					
				// 	cout << "!!!!!!!!!!!!!!!!!!" << endl;
				// 	std_msgs::Float64MultiArray center_XZ_msg; 
				// 	center_XZ_msg.data.clear();

				// 	for(int i=0; i<6; i++)
				// 	{
				// 		pub_array[i] = sum_array[i]/(double)array_count;
				// 		center_XZ_msg.data.push_back(pub_array[i]);
				// 		printf("pub_array[%d] : %f\n", i, pub_array[i]);
				// 	}
					
				// 	center_XZ_pub.publish(center_XZ_msg);
				// 	finish_park = true;
				// 	cout << " Finish !!!! " << endl;
				// }
			}
		}
		else if((Lidar_Stop == true) && (finish_park == true))
		{
			std::cout << " Finish !!!! " << std::endl;
		}
	}

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Parking>());
    rclcpp::shutdown();
    return 0;
}































/**
 * @brief 마우스 왼쪽 클릭을 하면 마우스가 있는 지점의 HSV 색상을 알려줌
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 * @param param 
 */
void mouse_callback(int event, int x, int y, int flags, void *param)
{
	if (event == cv::EVENT_LBUTTONDBLCLK)
	{
		cv::Vec3b color_pixel = img_color.at<cv::Vec3b>(y, x);

		cv::Mat hsv_color = cv::Mat(1, 1, CV_8UC3, color_pixel);


		H = hsv_color.at<cv::Vec3b>(0, 0)[0];
		S = hsv_color.at<cv::Vec3b>(0, 0)[1];
		V = hsv_color.at<cv::Vec3b>(0, 0)[2];

		std::cout << "left H= " << H << std::endl;
		std::cout << "left S= " << S << std::endl;
		std::cout << "left V = " << V << "\n" << std::endl;

	}
}

// float camera_values()
// {
//     cv::Mat K(3,3,CV_64F);

//     K = (Mat_<_Float64>(3, 3) << 538.39128993937,   0.,   308.049009633327, 0.,  539.777910345317,  248.065244763797, 0., 0., 1.);

//     cout << "K : " << K << endl;


//     cv::Size imageSize(640,480);
//     double apertureWidth = 0;
//     double apertureHeight = 0;
//     double fieldOfViewX;
//     double fieldOfViewY;
//     double focalLength2;
//     cv::Point2d principalPoint;
//     double aspectRatio;
//     cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength2, principalPoint, aspectRatio);

    
//     cout << fieldOfViewX << endl;

//     return (float)focalLength2;
// }

/**
 * @brief 마우스 왼쪽 버튼을 누르고 땔 때마다 그 위치의 좌표를 알려줌
 * 
 * @param event 
 * @param x 
 * @param y 
 * @param flags 
 */

void on_mouse(int event, int x, int y, int flags, void *)
{
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		ptOld1 = cv::Point(x, y);
		std::cout << "EVENT_LBUTTONDOWN: " << x << ", " << y << std::endl;
		break;
	case cv::EVENT_LBUTTONUP:
		std::cout << "EVENT_LBUTTONUP: " << x << ", " << y << std::endl;
		break;
	}
}

