#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

extern const bool findRed;

extern cv::Mat imgSource;
extern cv::Mat grayImg;
extern cv::Mat bgrSplit[3];
extern cv::Mat splitDiff;

extern bool armorD;
extern int armorX;
extern int armorY; 
extern int armorS;

extern const int imgWidth;
extern const int imgHeight;



class lightInfo;

void bfsMat(cv::Mat& inputMat,cv::Mat& labelMat,const cv::Point2i& startP,int threshold,cv::Point2i& cCenter);
void bfsMat(cv::Mat& inputMat,cv::Mat& labelMat,const cv::Point2i& startP,int threshold,lightInfo& clight);
void findLights(cv::Mat& inputMat);
void findArmors(cv::Mat& inputMat,std::vector<cv::Rect> inRectList);
void drawRoRect(cv::Mat& inputMat,const cv::RotatedRect& inRect);

class lightInfo{
public:
	cv::Point2i lCenter;
	int lWidth;
	int lHeight;
	int pixelCount;
	bool pairFlag = false;
	bool validFlag = false;
	cv::Rect lRect;
	cv::Rect getRect();
};

class highRegion{
public:
	highRegion(cv::Rect& inRect);
	bool activeFlag = true;
	int actCount = 0;
	int allCount = 0;
	cv::Rect rRect;
	float actRatio();
	bool hitCheck(cv::Rect& inRect);
	void preCheck(cv::Mat& inputMat);
};

