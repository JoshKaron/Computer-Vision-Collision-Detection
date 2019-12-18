
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

double distance(cv::Point2f& p1, cv::Point2f& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0));
}

// Task 3
// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // calculate keypoint distance mean
    double disMean = 0;
    for(cv::DMatch dmatch : kptMatches)
    {
        cv::KeyPoint kpPrev = kptsPrev[dmatch.queryIdx];
        cv::KeyPoint kpCurr = kptsCurr[dmatch.trainIdx];

        disMean += distance(kpPrev.pt, kpCurr.pt);
    }
    disMean /= kptMatches.size();

    // calculate keypoint distance standard deviation
    double disStd = 0;
    for(cv::DMatch dmatch : kptMatches)
    {
        cv::KeyPoint kpPrev = kptsPrev[dmatch.queryIdx];
        cv::KeyPoint kpCurr = kptsCurr[dmatch.trainIdx];

        disStd += std::pow(distance(kpPrev.pt, kpCurr.pt) - disMean, 2.0);
    }
    disStd /= kptMatches.size();

    for(cv::DMatch dmatch : kptMatches)
    {
        cv::KeyPoint kpPrev = kptsPrev[dmatch.queryIdx];
        cv::KeyPoint kpCurr = kptsCurr[dmatch.trainIdx];

        // elimiate outliers
        double dis = distance(kpPrev.pt, kpCurr.pt);
        if(dis > disMean + 2.0 * disStd ||
           dis < disMean - 2.0 * disStd)
        {
            continue;
        }

        if(boundingBox.roi.contains(kpPrev.pt) &&
           boundingBox.roi.contains(kpCurr.pt))
        {
            //boundingBox.keypoints.push_back(kpCurr);
            boundingBox.kptMatches.push_back(dmatch);
        }
    }

    if(false)
    {
        cout << "BoundingBoxId: " << boundingBox.boxID << " has " << boundingBox.kptMatches.size() << " matches" << endl;
    }
}

double mean(std::vector<double>& vec)
{
    double mean = 0;
    for(double x : vec)
    {
        mean += x;
    }
    return mean / vec.size();
}

double stDev(std::vector<double>& vec, double& mean)
{
    double stdev = 0;
    for(double x : vec)
    {
        stdev += std::pow(x - mean, 2.0);
    }
    return stdev / vec.size();
}

// Task 4
// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> disRatios;
    double minDis = 100;

    for(auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; it1++)
    {
        cv::KeyPoint kptOuterPrev = kptsPrev.at(it1->queryIdx);
        cv::KeyPoint kptOuterCurr = kptsCurr.at(it1->trainIdx);

        for(auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); it2++)
        {
            cv::KeyPoint kptInnerPrev = kptsPrev.at(it2->queryIdx);
            cv::KeyPoint kptInnerCurr = kptsCurr.at(it2->trainIdx);

            double disCurr = cv::norm(kptOuterCurr.pt - kptInnerCurr.pt);
            double disPrev = cv::norm(kptOuterPrev.pt - kptInnerPrev.pt);

            if(disPrev > std::numeric_limits<double>::epsilon() && disCurr >= minDis)
            {
                double disRatio = disCurr / disPrev;

                disRatios.push_back(disRatio);
            }
        }
    }

    if(disRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    
    std::sort(disRatios.begin(), disRatios.end());
    int medianIdx = std::floor(disRatios.size() / 2);
    double medianDisRatio = disRatios.size() % 2 == 0 ? (disRatios[medianIdx - 1] + disRatios[medianIdx]) / 2.0 : disRatios[medianIdx];

    double dt = 1.0 / frameRate;
    TTC = -dt / (1.0 - medianDisRatio);

    if(false)
    {
        cout << "distance ratios: " << endl;
        for(double dis : disRatios)
        {
            cout << dis << endl;
        }
        cout << "median: " << medianDisRatio << endl;
        

    }

    if(false)
    {
        cout << "TTC Camera: " << TTC << endl;
    }
}

// Task 2
bool sortCondition(LidarPoint lp1, LidarPoint lp2)
{
    return lp1.x < lp2.x;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //double laneWidth = 4.0;

    // sort vectors to get the median in the x direction
    std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), sortCondition);
    std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), sortCondition);

    int IdxPrev = lidarPointsPrev.size() / 2;
    int IdxCurr = lidarPointsCurr.size() / 2;

    double medianXPrev = lidarPointsPrev[IdxPrev].x;
    double medianXCurr = lidarPointsCurr[IdxCurr].x;

    double dt = 1.0 / frameRate;
    TTC = medianXCurr * dt / (medianXPrev - medianXCurr);
    
    if(false)
    {
        cout << "TTC Lidar: " << TTC << endl;
    }
}



// Task 1
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // initalize accumulator to store counts of object correspondences
    int nPrev = prevFrame.boundingBoxes.size() + 1;
    int nCurr = currFrame.boundingBoxes.size() + 1;
    int accum[nPrev][nCurr] = {};

    // loop through matches
    for(auto match : matches)
    {
        cv::KeyPoint kpPrev = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint kpCurr = currFrame.keypoints[match.trainIdx];

        // find bounding box for kpPrev
        for(auto bBoxPrev : prevFrame.boundingBoxes)
        {
            if(bBoxPrev.roi.contains(kpPrev.pt))
            {
                // find bouding box for kpCurr
                for(auto bBoxCurr : currFrame.boundingBoxes)
                {
                    if(bBoxCurr.roi.contains(kpCurr.pt))
                    {
                        // add to accumulator
                        accum[bBoxPrev.boxID][bBoxCurr.boxID]++; 
                    }
                }
            }
        }
    }

    // go thorugh the accumulator and make the object corrispondences with most matches
    // this code matches each box in the prev image with a box in the current img
    /*
    for(int prevIdx = 0; prevIdx < nPrev; prevIdx++)
    {
        int maxCurrIdx = 0;
        int maxVal = 0;
        for(int currIdx = 0; currIdx < nCurr; currIdx++)
        {
            if(accum[prevIdx][currIdx] > maxVal)
            {
                maxVal = accum[prevIdx][currIdx];
                maxCurrIdx = currIdx;
            }
        }

        if(maxVal < 1) { continue; }

        // check if match is revesable
        //int maxReverseIdx = 0;
        //int maxReverseVal = 0;
        //for(int i = 0; i < nPrev; i++)
        //{
        //    if(accum[i][maxCurrIdx] > maxReverseVal)
        //    {
        //        maxReverseVal = accum[i][maxCurrIdx];
        //        maxReverseIdx = i;
        //    }
        //}

        //if(maxReverseIdx != prevIdx) { continue; }
        // end reversable check

        bbBestMatches.insert({prevIdx, maxCurrIdx});

        if(true)
        {
            cout << prevIdx << " -> " << maxCurrIdx << " with val of: "<< maxVal << endl;
        }
    }*/

    for(int currIdx = 0; currIdx < nCurr; currIdx++)
    {
        int maxPrevIdx = 0;
        int maxVal = 0;
        for(int prevIdx = 0; prevIdx < nPrev; prevIdx++)
        {
            if(accum[prevIdx][currIdx] > maxVal)
            {
                maxVal = accum[prevIdx][currIdx];
                maxPrevIdx = prevIdx;
            }
        }

        if(maxVal < 1) { continue; }

        bbBestMatches.insert({maxPrevIdx, currIdx});

        if(false)
        {
            cout << maxPrevIdx << " -> " << currIdx << " with val of: "<< maxVal << endl;
        }
    }

    
}
