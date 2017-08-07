#include "carfinder.h"
using namespace cv;
using namespace std;

const int rLThresh = 100;
const int bLThresh = 100;
const int grayThresh = 240;


vector<highRegion> hRegiList;

// lightInfo::lightInfo(cv::Point2i cCenter,int idth,int height,int pNum):\
// lCeneter(cCenter),lWidth(width),lHeight(height),pixelCount(pNum)
// {

// }
Rect lightInfo::getRect()
{
    return Rect(lCenter.x - lWidth / 2, lCenter.y - lHeight / 2, \
                lWidth, lHeight);
}

highRegion::highRegion(Rect &inRect): rRect(inRect)
{

}

float highRegion::actRatio()
{
    float ratio = 1.0 * actCount / allCount;
    if (ratio > 0.5)
    {
        activeFlag = true;
        return ratio;
    }
    else
    {
        activeFlag = false;
        return 0.0;
    }

}

bool highRegion::hitCheck(Rect &inRect)
{
    ++allCount;
        if (  (inRect & rRect).area() > 0)
        {
            ++actCount;
            rRect.x += (int) 1.2 * (inRect.x - rRect.x);
            rRect.y += (int) 1.2 * (inRect.y - rRect.y);
            rRect.width -= rRect.width/4;
            rRect.height -= rRect.height/4;
        }
        else
        {
            if (rRect.x > (rRect.width/4 + 5))
            {
                rRect.x -= rRect.width/4;
            }
            if (rRect.y > (rRect.height/4 + 5))
            {
                rRect.y -= rRect.height/4;
            }
            rRect.width += rRect.width / 2;
            if(rRect.width > imgWidth/2){rRect.width = imgWidth/2;}
            rRect.height += rRect.height / 2;
            if(rRect.height > imgHeight/2){rRect.height = imgHeight/2;}
        }
}

void highRegion:: preCheck(Mat& inputMat){
    if (inputMat.channels() == 3)
    {
        Mat preMat = inputMat(rRect);
        Mat preSplit[3];
        split(preMat,preSplit);
        Mat colorBinMat(preMat.size(),CV_8UC1);
        colorBinMat = (preMat > rLThresh);
        //findArmors()
    }
     

}
void bfsMat(Mat &inputMat, Mat &labelMat, const Point2i &startP, int threshold, Point2i &cCenter)
{
    int maxWidth = 50;
    int maxHeight = 50;
    int leftMost = 9999;
    int rightMost = 0;
    int upMost = 9999;
    int downMost = 0;
    uchar *inputPtr = (uchar *) inputMat.data;
    uchar *labelPtr = (uchar *) labelMat.data;
    if (labelPtr[startP.y * imgWidth + startP.x])
    {
        return;
    }
    int bfsQueue[30000][2];
    long lstPtr = 0;
    long curPtr = 0;
    int curVal;
    int left, right, up, down;
    double sumX(0) , sumY(0) , sumVal(0);
    int aveX, aveY, aveVal;
    int tempVal;
    curPtr++;
    bfsQueue[lstPtr][0] = startP.x;
    bfsQueue[lstPtr][1] = startP.y;
    bfsQueue[curPtr][0] = startP.x;
    bfsQueue[curPtr][1] = startP.y;
    Point2i cenPt;

    while(lstPtr != curPtr && curPtr < 28000)
    {
        cenPt.x = bfsQueue[lstPtr][0];
        cenPt.y = bfsQueue[lstPtr][1];
        if (cenPt.x < 2 || cenPt.x > (imgWidth - 2) || cenPt.y < 2 || cenPt.y > (imgHeight - 2))
        {
            lstPtr++;
            continue;
        }

        curVal = inputPtr[cenPt.y * imgWidth + cenPt.x];

        left = cenPt.x - 1;
        right = cenPt.x + 1;
        up = cenPt.y - 1;
        down = cenPt.y + 1;

        if (!labelPtr[cenPt.y * imgWidth + right])
        {
            tempVal = inputPtr[cenPt.y * imgWidth + right];
            if(abs(tempVal - curVal) < threshold)
            {
                if (rightMost < right )
                {
                    rightMost = right;
                }
                curPtr++;
                bfsQueue[curPtr][0] = right;
                bfsQueue[curPtr][1] = cenPt.y;
                labelPtr[cenPt.y * imgWidth + right] = 255;
                sumX += right;
                sumY += cenPt.y;
                sumVal += tempVal;
            }
        }
        if (!labelPtr[cenPt.y * imgWidth + left])
        {
            tempVal = inputPtr[cenPt.y * imgWidth + left];
            if(abs(tempVal - curVal) < threshold)
            {
                if (leftMost > left )
                {
                    leftMost = left;
                }
                curPtr++;
                bfsQueue[curPtr][0] = left;
                bfsQueue[curPtr][1] = cenPt.y;
                labelPtr[cenPt.y * imgWidth + left] = 255;
                sumX += left;
                sumY += cenPt.y;
                sumVal += tempVal;
            }
        }
        if (!labelPtr[up * imgWidth + cenPt.x])
        {
            tempVal = inputPtr[up * imgWidth + cenPt.x];
            if(abs(tempVal - curVal) < threshold)
            {
                if(upMost > up)
                {
                    upMost = up;
                }
                curPtr++;
                bfsQueue[curPtr][0] = cenPt.x;
                bfsQueue[curPtr][1] = up;
                labelPtr[up * imgWidth + cenPt.x] = 255;
                sumX += cenPt.x;
                sumY += up;
                sumVal += tempVal;
            }
        }
        if (!labelPtr[down * imgWidth + cenPt.x])
        {
            tempVal = inputPtr[down * imgWidth + cenPt.x];
            if(abs(tempVal - curVal) < threshold)
            {
                if(downMost < down)
                {
                    downMost = down;
                }
                curPtr++;
                bfsQueue[curPtr][0] = cenPt.x;
                bfsQueue[curPtr][1] = down;
                labelPtr[down * imgWidth + cenPt.x] = 255;
                sumX += cenPt.x;
                sumY += down;
                sumVal += tempVal;
            }
        }
         lstPtr++;
        if((rightMost - leftMost) > maxWidth ||
                (downMost - upMost) > maxHeight)
        {
            return;
        }


    }
    if (curPtr < 20 || curPtr > 3000)
    {
        return;
    }
    if (  (float(rightMost - leftMost) / (downMost - upMost)) > 0.5 && \
            (float(rightMost - leftMost) / (downMost - upMost)) < 2.0 )
    {
        return ;
    }

    cCenter.x = (int)(sumX * 1.0 / curPtr);
    cCenter.y = (int)(sumY * 1.0 / curPtr);

    //aveVal = (int)sumVal * 1.0 / curPtr;
    return ;
}

void bfsMat(Mat &inputMat, Mat &labelMat, const Point2i &startP, int threshold, lightInfo &plight)
{
    int maxWidth = 150;
    int maxHeight = 150;
    int leftMost = 9999;
    int rightMost = 0;
    int upMost = 9999;
    int downMost = 0;
    uchar *inputPtr = (uchar *) inputMat.data;
    uchar *labelPtr = (uchar *) labelMat.data;
    if (labelPtr[startP.y * imgWidth + startP.x])
    {
        return;
    }
    int bfsQueue[30000][2];
    long lstPtr = 0;
    long curPtr = 0;
    int curVal;
    int left, right, up, down;
    long sumX(0) , sumY(0) , sumVal(0);
    int aveX, aveY, aveVal;
    int tempVal;
    curPtr++;
    bfsQueue[lstPtr][0] = startP.x;
    bfsQueue[lstPtr][1] = startP.y;
    bfsQueue[curPtr][0] = startP.x;
    bfsQueue[curPtr][1] = startP.y;
    Point2i cenPt;

    while(lstPtr != curPtr && curPtr < 29000)
    {
        cenPt.x = bfsQueue[lstPtr][0];
        cenPt.y = bfsQueue[lstPtr][1];
        if (cenPt.x < 2 || cenPt.x > (imgWidth - 2) || \
                cenPt.y < 2 || cenPt.y > (imgHeight - 2))
        {
            lstPtr++;
            continue;
        }

        if (leftMost > cenPt.x ){leftMost = cenPt.x;}
        if (rightMost < cenPt.x){rightMost = cenPt.x;}
        if (upMost > cenPt.y){upMost = cenPt.y;}
        if (downMost < cenPt.y){downMost = cenPt.y;}

        if((rightMost - leftMost) > maxWidth ||
            (downMost - upMost) > maxHeight){
            return;
        }

        curVal = inputPtr[cenPt.y * imgWidth + cenPt.x];

        left = cenPt.x - 1;
        right = cenPt.x + 1;
        up = cenPt.y - 1;
        down = cenPt.y + 1;

        if (!labelPtr[up * imgWidth + cenPt.x])
        {
            tempVal = inputPtr[up * imgWidth + cenPt.x];
            if(abs(tempVal - curVal) < threshold)
            {
                curPtr++;
                bfsQueue[curPtr][0] = cenPt.x;
                bfsQueue[curPtr][1] = up;
                labelPtr[up * imgWidth + cenPt.x] = 255;
                sumX += cenPt.x;
                sumY += up;
                sumVal += tempVal;
            }
        }
        if (!labelPtr[down * imgWidth + cenPt.x])
        {
            tempVal = inputPtr[down * imgWidth + cenPt.x];
            if(abs(tempVal - curVal) < threshold)
            {
                curPtr++;
                bfsQueue[curPtr][0] = cenPt.x;
                bfsQueue[curPtr][1] = down;
                labelPtr[down * imgWidth + cenPt.x] = 255;
                sumX += cenPt.x;
                sumY += down;
                sumVal += tempVal;
            }
        }

        if (!labelPtr[cenPt.y * imgWidth + right])
        {
            tempVal = inputPtr[cenPt.y * imgWidth + right];
            if(abs(tempVal - curVal) < threshold)
            {
                curPtr++;
                bfsQueue[curPtr][0] = right;
                bfsQueue[curPtr][1] = cenPt.y;
                labelPtr[cenPt.y * imgWidth + right] = 255;
                sumX += right;
                sumY += cenPt.y;
                sumVal += tempVal;
            }
        }
        if (!labelPtr[cenPt.y * imgWidth + left])
        {
            tempVal = inputPtr[cenPt.y * imgWidth + left];
            if(abs(tempVal - curVal) < threshold)
            {
                curPtr++;
                bfsQueue[curPtr][0] = left;
                bfsQueue[curPtr][1] = cenPt.y;
                labelPtr[cenPt.y * imgWidth + left] = 255;
                sumX += left;
                sumY += cenPt.y;
                sumVal += tempVal;
            }
        }

        

        lstPtr++;
    }
    if (curPtr < 15 || curPtr > 3000)
    {
        return;
    }
    // if (  (float(rightMost - leftMost)/(downMost - upMost)) > 0.5 && \
    //       (float(rightMost - leftMost)/(downMost - upMost)) < 2.0 ){
    //         return ;
    //     }
    if(leftMost == 9999)
    {
        return;
    }
    plight.lCenter.x = (int)(sumX / curPtr);
    plight.lCenter.y = (int)(sumY / curPtr);
    plight.lWidth = rightMost - leftMost;
    plight.lHeight = downMost - upMost;
    plight.pixelCount = curPtr;
    plight.validFlag = true;
    //circle(imgSource,cv::Point2i(plight.lCenter.x,upMost),5,Scalar(255,255,255),2);
    //circle(imgSource,cv::Point2i(plight.lCenter.x,downMost),5,Scalar(255,0,0),2);
    //aveVal = (int)sumVal/ curPtr;
    return ;
}


void findArmors(Mat& inputMat,vector<Rect> inRectList){
    Mat tempMat(inputMat.size(),CV_8UC1);
    for(int i = 0; i < inRectList.size();++i){
      Rect bigRect(inRectList[i]);
      if(bigRect.y > (inRectList[i].height/3 + 10) && bigRect.y < (imgHeight-10-1.7*inRectList[i].height) ){
        bigRect.y -= inRectList[i].height/3;
        bigRect.height *= 1.7;
        bigRect.x += 0.15 * bigRect.width;
        bigRect.width *= 0.7;
        //equalizeHist(inputMat(bigRect),inputMat(bigRect));
        vector<vector<Point2i>> pCircles;
        inRange(inputMat(bigRect),Scalar(100),Scalar(245),tempMat(bigRect));
        dilate(tempMat(bigRect),tempMat(bigRect),Mat::ones(5,5,CV_8UC1));
        erode(tempMat(bigRect),tempMat(bigRect),Mat::ones(5,5,CV_8UC1));

        //morphologyEx(tempMat(inRectList[i]),tempMat(inRectList[i]),MORPH_CLOSE,Mat(5,5,CV_8UC1));
        findContours(tempMat(bigRect),pCircles,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,bigRect.tl());
        if(pCircles.size() > 0){
            if(pCircles.size() > 1){
                std::sort(pCircles.begin(),pCircles.end(),[](vector<Point2i> a, vector<Point2i> b)\
                { return a.size() > b.size(); });
                for(int i = 0; i < pCircles.size(); ++i){
                    double cLen = arcLength(pCircles[i],true);
                    double cArea = contourArea(pCircles[i],false);
                    Rect cRect = boundingRect(pCircles[i]);
                    //RotatedRect = minEllipse(pCircles[i]);
                    //drawContours(imgSource,pCircles,i,Scalar(255,255,255),4);
                    if (cLen > (cRect.width + cRect.height)  && cArea > (cRect.area()*9/16) &&
                        cArea > (bigRect.area()/8) && cLen < 2*(cRect.width + cRect.height) )
                    {
                        Point2f cCenter;
                        float cRadius;
                        minEnclosingCircle(pCircles[i],cCenter,cRadius);

			armorD = true;
		        armorX =(int)cCenter.x;
			armorY =(int)cCenter.y; 
			armorS = (int)cRect.height;
			armorRec.clear();
			armorRec.push_back(Point2f(inRectList[i].x,inRectList[i].y));
			armorRec.push_back(Point2f(inRectList[i].x + inRectList[i].width,inRectList[i].y));
			armorRec.push_back(Point2f(inRectList[i].x + inRectList[i].width,inRectList[i].y + inRectList[i].height));
			armorRec.push_back(Point2f(inRectList[i].x,inRectList[i].y + inRectList[i].height));
                        //RotatedRect numRect = minAreaRect(pCircles[i]);
                        circle(imgSource,cCenter,cRadius,Scalar(0,255,0),2);
                        //drawRoRect(imgSource,numRect);
                    }
                }
            }
            else{
                double cLen = arcLength(pCircles[0],true);
                double cArea = contourArea(pCircles[0],false);
                Rect cRect = boundingRect(pCircles[0]);
                //drawContours(imgSource,pCircles,i,Scalar(255,255,255),4);
                if (cLen > (cRect.width + cRect.height)  && cArea > (cRect.area()*9/16) &&
                    cArea > (bigRect.area()/8)  && cLen < 2*(cRect.width + cRect.height)    )
                {
                    Point2f cCenter;
                    float cRadius;
                    minEnclosingCircle(pCircles[0],cCenter,cRadius);

		    armorD = true;
		    armorX =(int)cCenter.x;
	            armorY =(int)cCenter.y; 				

                    //RotatedRect numRect = minAreaRect(pCircles[i]);
                    circle(imgSource,cCenter,cRadius,Scalar(0,255,0),2);
                    //drawRoRect(imgSource,numRect);
                }
            }
        }
    }
  }
  //imshow("temp", tempMat);
  tempMat.setTo(0);
}

void findArmors(Mat& inputMat,vector<highRegion> hRegions){
    Mat tempMat(inputMat.size(),CV_8UC1);
    for(int i = 0; i < hRegions.size();++i){
        Rect bigRect(hRegions[i].rRect);
        if(hRegions[i].rRect.y > 4 * hRegions[i].rRect.height ){
            bigRect.y -= 4 *hRegions[i].rRect.height;
            bigRect.height *=8 ;
            vector<vector<Point2i>> pCircles;
            inRange(inputMat(bigRect),Scalar(100),Scalar(245),tempMat(bigRect));
            dilate(tempMat(bigRect),tempMat(bigRect),Mat::ones(5,5,CV_8UC1));
            erode(tempMat(bigRect),tempMat(bigRect),Mat::ones(5,5,CV_8UC1));
            //morphologyEx(tempMat(inRectList[i]),tempMat(inRectList[i]),MORPH_CLOSE,Mat(5,5,CV_8UC1));
            findContours(tempMat(bigRect),pCircles,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,bigRect.tl());
            if(pCircles.size() > 0){
                for (int i = 0; i < pCircles.size(); ++i){
                    double cLen = arcLength(pCircles[i],true);
                    double cArea = contourArea(pCircles[i],false);
                    if (cLen > (bigRect.width + bigRect.height)/2  && cArea > (bigRect.area()/6) &&
                        cLen*cLen/cArea < 4.5 * CV_PI && cLen*cLen/cArea > 3.5 * CV_PI)
                    {
                        Point2f cCenter;
                        float cRadius;
                        //minEnclosingCircle(pCircles[i],cCenter,cRadius);
                        RotatedRect numRect = minAreaRect(pCircles[i]);
                        //circle(imgSource,cCenter,cRadius,Scalar(0,255,0),2);
                        drawRoRect(imgSource,numRect);

                    }
                }
            }

        }

    }
}

void getWorldInfo(){
    const int aTWidth = 135;
    const int aTHeight = 125;
    const int lTHeight = 60;
    const int lTWidth = 10;
    // int pinWidth = 94;
    // int pinHeight = 95;
    // int numTRadius = 100;
    const int numTRadius = 50;
    const int aITWidth = 83;
    const int aITHeight = 73;
    double armPitch = 3/4 * CV_PI;

    int widIdx0,widIdx1;
    int heiIdx0,heiIdx1;
    int focalLen;

    float camYaw = atan2(0.5*(widIdx0 + widIdx1) - 0.5*imgWidth,1.0*focalLen);
    float camPitch = atan2(0.5*(heiIdx0 + heiIdx1) - 0.5*imgHeight,1.0*focalLen);
    float distance =(int) 1.0 * focalLen * aITWidth / abs(widIdx0 - widIdx1);
    double pRatio = (double) 1.0*((widIdx0 + widIdx1) + abs(widIdx0 - widIdx1))/ \
                     ((widIdx0 + widIdx1) - abs(widIdx0 - widIdx1));
    //double objYaw = acos(), Todo

}

void findLights(Mat& inputMat)
{
    //Mat grayBinMat = (grayImg > grayThresh);

    //equalizeHist(BGRSplit[0],BGRSplit[0]);
    Mat colorBinMat =  (splitDiff > (findRed ? 80 : 100 ) ); // Blue 100
    //colorBinMat = colorBinMat & grayBinMat;
    dilate(colorBinMat, colorBinMat, Mat::ones(11, 11, CV_8UC1));
    colorBinMat = (colorBinMat & (bgrSplit[(findRed ? 2 : 0 )] > 250));//blue [0]
    //dilate(colorBinMat, colorBinMat, Mat::ones(5, 5, CV_8UC1));
    //erode(colorBinMat, colorBinMat, Mat::ones(5, 5, CV_8UC1));

    vector<Rect> lRectList;
    vector<Rect> armRectList;

//    int nzCount = countNonZero(colorBinMat);
//    if(nzCount > 0)
//    {
//        Mat labelMat = Mat::zeros(imgWidth, imgHeight, CV_8UC1);
//        //TODO sort

//        //for(int i = 0; i < hRegiList.size();++i){

//        //  }
//        vector<Point2i> nzPoints(nzCount);
//        findNonZero(colorBinMat, nzPoints);
//        for (int i = 0; i < nzCount; ++i)
//        {
//            lightInfo poLight;
//            bfsMat(colorBinMat, labelMat, nzPoints[i], 5, poLight);
//            if (poLight.validFlag)
//            {
//                lRectList.push_back(poLight.getRect());
//                //circle(imgSource,Point2i(poLight.lCenter.x,poLight.lCenter.y),5,Scalar(0,255,255),2);
//                rectangle(imgSource, lRectList.back(), Scalar(0, 255, 255), 2);
//            }
//        }
//     }

     vector<vector<Point2i> > lContours;
     findContours(colorBinMat,lContours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
     int cSizeSum = 0;
     for(int i = 0; i < lContours.size();++i){
        lRectList.push_back(boundingRect(lContours[i]));
        rectangle(imgSource, lRectList.back(), Scalar(0, 255, 255), 2);
       // cSizeSum += lContours[i].size();
     }

//To DO USE SORT 
//     if(lContours.size() && cSizeSum/lContours.size());

//     sort(lContours.begin(),lContours.end(),[](vector<Point2i>& a, vector<Point2i>& b)\
//     {return a.size() > b.size();});
//***************//

     if(lRectList.size()){
        for(int i = 0; i < lRectList.size(); ++i)
        {
            float wdRatio = lRectList[i].width * 1.0 / lRectList[i].height;
            if (wdRatio < 0.1|| wdRatio > 2)
            {
                if(wdRatio > 1){
                //TODO
                }
                continue;
            }
            for (int j = i + 1; j < lRectList.size(); ++j)
            {
                float wdRatio = lRectList[j].width * 1.0 / lRectList[j].height;
                if (wdRatio < 0.1 || wdRatio > 2)
                {
                    if(wdRatio > 1){
                    //TODO
                    }
                    continue;
                }
                Rect armRect = lRectList[i] | lRectList[j];
                float aRatio = armRect.width * 1.0 / armRect.height;
                if( aRatio > 3 || aRatio < 1){
                    continue;
                }
                float xDis = abs(lRectList[i].x - lRectList[j].x) * 1.0;
                float yDis = abs(lRectList[i].y - lRectList[j].y) * 1.0;
                float widRatio = lRectList[i].width * 1.0 / lRectList[j].width;
                float heiRatio = lRectList[i].height * 1.0 / lRectList[j].height;
                float arRatio = lRectList[i].area() * 1.0 / lRectList[j].area();
                float siRatio = abs(lRectList[i].width * 1.0 / lRectList[i].height - \
                                    lRectList[j].width * 1.0 / lRectList[j].height);
                if ((arRatio < 1.5 || arRatio > 0.65)  &&
                    (widRatio < 1.3 || widRatio > 0.75) &&
                    (heiRatio < 1.3 || heiRatio > 0.75) &&
                    (siRatio < 0.2) &&
                    xDis < 6.7 * (lRectList[i].width + lRectList[j].width) &&
                    xDis < 2.6 * (lRectList[i].height + lRectList[j].height) &&
                    xDis > 2 * (lRectList[i].width + lRectList[j].width) &&
                    yDis < 0.2 * (lRectList[i].height + lRectList[j].height))
                {       
                    armRectList.push_back(armRect);
                }
            }
        }

        findArmors(bgrSplit[(findRed ? 0:2)],armRectList);

        int curRegiNum = hRegiList.size();
        vector<Rect> actRects;
        for(int i = 0; i < armRectList.size(); ++i)
        {
            int curHitNum = 0;
            for(int j = 0; j < curRegiNum; ++j)
            {
                if(hRegiList[j].hitCheck(armRectList[i]))
                {
                    curHitNum++;
                }
            }
            if (!curHitNum)
            {
                hRegiList.push_back(highRegion(armRectList[i]));
            }
            else{
                actRects.push_back(armRectList[i]);
                rectangle(imgSource, armRectList[i], Scalar(255, 255, 0), 2);
            }
        }
        int j = 0;
        for(int i = 0; j < curRegiNum; ++j)
        {
            hRegiList[i].actRatio();
            if(!hRegiList[i].activeFlag)
            {
                hRegiList.erase(hRegiList.begin() + i);
            }
            else
            {
                ++i;
            }
        }

        //imshow("colorBin", colorBinMat);
	return ;
    }
}

void drawRoRect(cv::Mat& inputMat,const cv::RotatedRect& inRect){
    Point2f rect_points[4]; inRect.points(rect_points);
    for(int i = 0; i <4; ++i){
        line(inputMat, rect_points[i], rect_points[(i+1)%4],Scalar(255),4);
    }
}
