#include "parking/parking_func.cpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <thread>
#define LEFT_CAM 0
#define RIGHT_CAM 4

// 아직 변수 정리 중
int array_count = 0;
int boom_count = 0;
int mission_flag = 0;
bool lidar_stop = false;
bool finish_park = false;
bool impulse = false;
float sum_array[6] ={0,0,0,0,0,0};

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("parking_node")
  {

	lidar_flag_ = this->create_subscription<std_msgs::msg::Bool>(
    "lidar_flag", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {lidar_callback(msg);});

	mission_flag_ = this->create_subscription<std_msgs::msg::Int16>(
    "mission_flag", 10, [this](const std_msgs::msg::Int16::SharedPtr msg) {mission_callback(msg);});

	center_XZ_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("where_center_point_needs",10);

    cap_left_.open(LEFT_CAM);
    cap_left_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap_left_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

	cap_right_.open(RIGHT_CAM);
    cap_right_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap_right_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    if (!cap_left_.isOpened())
    {
    	RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    	return;
    }
	else if (!cap_right_.isOpened()){
		RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    	return;
	}
	else return;

	// 이미지 처리
    image_processing();
  }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_XZ_pub_;
    cv::VideoCapture cap_left_;
	cv::VideoCapture cap_right_;

	//sub
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; // 일단 사용 안함
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_flag_; // 현재 진행 미션 플래그
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_flag_; // 라이다 스탑 플래그

	float baseline = 23;
	float focal_pixels = 800; // size(1280, 720) 일때 focal_pixels
	int target_x = 135;
	int target_z = 135;
	float alpha = 23.9;	//alpha = 카메라 머리 숙인 각도
	float beta = 45.5999; 	//beta = erp 헤딩으로부터 카메라 각도
	float gps_for_camera_x = 30; //cm
	float gps_for_camera_z = -50; //cm


    void image_processing()
    {
		while (rclcpp::ok())
		{
			cv::Mat leftFrame, rightFrame;
			cv::Mat leftMask, rightMask;

			cap_left_ >> leftFrame;
			cap_right_ >> rightFrame;
			
			cv::Point leftCircle, rightCircle; // using yellow ball
			cv::Point2d top_XZ, mid_XZ, bottom_XZ; // using parking

			line_symmetry(leftFrame, LEFT_CAM);
			line_symmetry(rightFrame, RIGHT_CAM);

			cv::rectangle(leftFrame, cv::Rect(0, 0, 1280, 100), cv::Scalar(0, 0, 0), -1); //상단 for koreatech
			cv::rectangle(rightFrame, cv::Rect(0, 0, 1280, 100), cv::Scalar(0, 0, 0), -1); //상단 for koreatech
			// rectangle(leftFrame, Rect(0, 400, 1280, 720), Scalar(0, 0, 0), -1); //for k-citys
			// rectangle(rightFrame, Rect(0, 0, 1280, 200), Scalar(0, 0, 0), -1); //for k-citys

			//imshow("rightFrame", rightFrame);
			//imshow("leftFrame", leftFrame);

			if((mission_flag == 10)||(mission_flag == 6))
			{
				// =================[ sub하는 변수값 확인 ]=========================
				// system("clear");
				// std::cout << "---------------------------------------------" << std::endl;
				// std::cout << "lidar_stop : " << lidar_stop << "  finish_park : " << finish_park << std::endl;	

				// imshow("rightFrame", rightFrame);
				// imshow("leftFrame", leftFrame);

				// leftMask = find_edge(leftFrame, 0);
				// rightMask = find_edge(rightFrame, 1);

				// =================[ 노란공을 이용한 초기 대칭 확인 ]=======================
				// leftMask = add_hsv_filter(leftFrame, LEFT_CAM);
				// rightMask = add_hsv_filter(rightFrame, RIGHT_CAM);
				// cv::Point2f ball_XZ;
				// leftCircle = find_ball(leftMask, leftMask);
				// rightCircle = find_ball(rightMask, rightMask);
				// ball_XZ = find_xz(leftCircle, rightCircle, leftFrame, rightFrame, alpha, beta);
				
				// ==================[ using mouse_callback ]===================
				// img_color = leftFrame.clone();
				// img_color_2 = rightFrame.clone();
				// cv::setMouseCallback("Left Frame", mouse_callback);
				// cv::setMouseCallback("Right Frame", mouse_callback_2);

				// setMouseCallback("Left Frame",on_mouse);
				// setMouseCallback("Right Frame",on_mouse);

				// imshow("rightMask", rightMask);
				// imshow("leftMask", leftMask);

				if((lidar_stop == false) && (finish_park == false))
				{
					// cout << " Wating Lidar Stop" << endl;
					// imshow("Left Frame", leftFrame);
					// imshow("Right Frame", rightFrame);
				}
				else if((lidar_stop == true) && (finish_park == false))
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


					double *ptr_left = find_center(leftMask, left_array, LEFT_CAM);
					double *ptr_right = find_center(rightMask, right_array, RIGHT_CAM);

					if (left_array[0] && right_array[0])
					{
						bottom_XZ = find_xz({left_array[0],left_array[1]}, {right_array[0],right_array[1]}, leftFrame, rightFrame, alpha, beta);
						// mid_XZ = stereovision.`({left_array[2],left_array[3]}, {right_array[2],right_array[3]}, leftFrame, rightFrame, alpha, beta);
						top_XZ = find_xz({left_array[4],left_array[5]}, {right_array[4],right_array[5]}, leftFrame, rightFrame, alpha, beta);
						mid_XZ.x = (bottom_XZ.x + top_XZ.x)/2.0;
						mid_XZ.y = (bottom_XZ.y + top_XZ.y)/2.0;

						std::cout << "bottom_XZ : " << bottom_XZ << std::endl;
						std::cout << "mid_XZ : " << mid_XZ << std::endl;
						std::cout << "top_XZ : " << top_XZ << std::endl;

						array[0]= (bottom_XZ.x + gps_for_camera_z)/100.00;
						array[1]= -(bottom_XZ.y + gps_for_camera_x)/100.00;
						array[2]= (mid_XZ.x + gps_for_camera_z)/100.00;
						array[3]= -(mid_XZ.y + gps_for_camera_x)/100.00;
						array[4]= (top_XZ.x + gps_for_camera_z)/100.00;
						array[5]= -(top_XZ.y + gps_for_camera_x)/100.00;

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
						std::cout << "array_count : " << array_count << std::endl;

						if(array_count == 20)
						{
							if(boom_count > 15)
							{
								std::cout << "주차실패 ㅠㅠㅠㅠ" << std::endl;
							}
							else
							{
								std::cout << " 봄 ? \n 이게 바로 비전 클라스 우리 잘못 아니니 뭐라 하려면 제어탓. \n ^~^" << std::endl;
							}
							
							std::cout << "!!!!!!!!!!!!!!!!!!" << std::endl;
							std_msgs::msg::Float64MultiArray center_XZ_msg; 
							center_XZ_msg.data.clear();

							for(int i=0; i<6; i++)
							{
								pub_array[i] = sum_array[i]/(double)array_count;
								center_XZ_msg.data.push_back(pub_array[i]);
								printf("pub_array[%d] : %f\n", i, pub_array[i]);
							}
							
							center_XZ_pub_->publish(center_XZ_msg);
							finish_park = true;
							std::cout << " Finish !!!! " << std::endl;
						}
					}
				}
				else if((lidar_stop == true) && (finish_park == true))
				{
					std::cout << " Finish !!!! " << std::endl;
				}
			}
			rclcpp::spin_some(this->get_node_base_interface());
		}
    }

	/**
 * @brief 라이더로부터 플래그를 받음 lidar_stop
 * lidar_stop = true 인 경우 level2로 넘어감
 * @param msg
 */
	void lidar_callback(const std_msgs::msg::Bool::SharedPtr msg)
	{
		lidar_stop = msg->data;
	}

	/**
	 * @brief 플래닝으로부터 미션 번호를 받음
	 * 10 = 뭐야
	 * 6 = 또 뭐야
	 * @param msg
	 */
	void mission_callback(const std_msgs::msg::Int16::SharedPtr msg)
	{
		mission_flag = msg->data;
	}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}

