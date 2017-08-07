#include "ros/ros.h"
#include "/home/ubuntu/auto_car/devel/include/armor_find/armor_msg.h"
#include "std_msgs/String.h"
#include <sstream>

#include <memory>
#include <math.h>
#include <iostream>

#include "carfinder.h"

using namespace std;
using namespace cv;

const bool findRed = true;
std::vector<cv::Point2f> armorRec;
bool armorD = false;
int armorX = 0;
int armorY = 0;
int armorS = 0;

const int imgWidth = 1280;
const int imgHeight = 720;

cv::Mat imgSource;
cv::Mat grayImg;
cv::Mat bgrSplit[3];
cv::Mat splitDiff;

double camPara[9] = {932.162968, 0.000000,   658.663550,
		     0.00000,    933.417581, 343.299587,
		     0.00000,    0.00000,    1.0000};

double distor[5] = {0.143583, -0.222515, -0.003117,0.001501,0.0000};

int main(int argc, char **argv){ 

vector<Point3f> obj_p;
obj_p.push_back(cv::Point3f(0.0,   0.0,  0.0));
obj_p.push_back(cv::Point3f(125,   0.0,  0.0));
obj_p.push_back(cv::Point3f(125,   60,   0.0));
obj_p.push_back(cv::Point3f(0.0,   60,   0.0));

Mat camera_matrix = cv::Mat(3, 3, CV_64F, camPara);
Mat dist_coeffs   = cv::Mat(5, 1, CV_64F, distor);

Mat rvec = cv::Mat::ones(3,1,CV_64F);
Mat tvec = cv::Mat::ones(3,1,CV_64F);


 ros::init(argc,argv,"armorfind_node");
 ros::NodeHandle nh;
 
 ros::Publisher armor_pub = nh.advertise<armor_find::armor_msg>("armor_info", 100);
 ros::Rate loop_rate(10);
 armor_find::armor_msg armorMsg;

string CamWindow = "CamWindow";
namedWindow(CamWindow,WINDOW_AUTOSIZE);
VideoCapture cap(0);
cap.open(0);

//cap>>imgSource;
//cap.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
cap.set(CV_CAP_PROP_FRAME_WIDTH,imgWidth);
cap.set(CV_CAP_PROP_FRAME_HEIGHT,imgHeight);
//cap.set(CV_CAP_PROP_BRIGHTNESS,0.45);


while(ros::ok()){
		
            cap>>imgSource;
            if(!imgSource.empty()){             
                vector<KeyPoint> keyPoints;
                split(imgSource,bgrSplit);
		if(findRed){
                splitDiff = bgrSplit[2] - bgrSplit[0];
		}
		else{
		splitDiff = bgrSplit[0] - bgrSplit[2];
		}

                //cvtColor(imgSource,grayImg,CV_BGR2GRAY);
               

                //struct timeval start, end;
                //long mtime, seconds, useconds;

                //auto start = std::chrono::high_resolution_clock::now();
                //gettimeofday(&start, NULL);
                findLights(imgSource);
                //gettimeofday(&end, NULL);
                //seconds  = end.tv_sec  - start.tv_sec;
                //useconds = end.tv_usec - start.tv_usec;
                //mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
                //printf("Elapsed time: %ld milliseconds\n", mtime);
                //auto end   = std::chrono::high_resolution_clock::now();
                //auto duration = duration_cast<microseconds>(end - start);
                //std::chrono::duration<double> diff = end-start;
                //std::cout << " ints : " << diff.count() * 1000 << " ms\n";
                
                imshow(CamWindow,imgSource);
            }

            if(27==waitKey(30))
            {   cout<<"User EXIT"<<endl;
                break;
            }

	if(armorD){
		cv::solvePnP(obj_p,armorRec, camera_matrix, dist_coeffs, rvec, tvec);
		double enemy_dis = tvec.at<double>(2);//Z distance
                double enemy_ang = rvec.at<double>(1);//Rotate around y axis
		armorMsg.detected = true;
		armorMsg.d = enemy_dis;
		armorMsg.x = (armorX - 658.663550) /932.162968 * armorMsg.d ;//armor_->armor.center.x - 320;
		armorMsg.y = (armorX - 343.299587)/933.417581 * armorMsg.d ;//armor_->armor.center.y - 240;
		
	}
	else{
	    armorMsg.detected = false;
	    armorMsg.d = 0;
            armorMsg.x = 0;
            armorMsg.y = 0;

	}
	armorD = false;
	armor_pub.publish(armorMsg);
	//ROS_INFO("%s","send armor info");
	
	
	
}

//ros::spinOnce();
//loop_rate.sleep();


  return 0;
}
