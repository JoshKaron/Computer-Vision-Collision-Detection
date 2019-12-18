#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    
    if (matcherType == "MAT_BF")
    {
        int normType = descriptorType == "DES_BINARY" ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType == "MAT_FLANN")
    {
        matcher = cv::FlannBasedMatcher::create();
        descSource.convertTo(descSource, CV_32F);
        descRef.convertTo(descRef, CV_32F);
    }

    double t = (double)cv::getTickCount();

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1

        std::vector<cv::DMatch> goodMatches;
        for(int i = 0; i < matches.size(); i++)
        {
            if(matches[i].distance >= 0.8)
            {
                goodMatches.push_back(matches[i]);
            }
        }
        matches = goodMatches;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2);

        std::vector<cv::DMatch> goodMatches;

        for(int i = 0; i < knnMatches.size(); i++)
        {
            std::vector<cv::DMatch> pair = knnMatches[i];
            if(pair[0].distance < 0.8 * pair[1].distance)
            {
                goodMatches.push_back(pair[0]);
            }

            matches = goodMatches;
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    return t;

    
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
// BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == "BRISK")
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType == "BRIEF")
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType == "ORB")
    {
        //cout << "in ORB" << endl;
        extractor = cv::ORB::create();
    }
    else if (descriptorType == "FREAK")
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType =="AKAZE")
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType == "SIFT")
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();

    extractor->compute(img, keypoints, descriptors);
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    return t;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsHarris(vector<cv::KeyPoint>& keypoints, cv::Mat& img, bool bVis)
{
    int blockSize = 2;
    int apatureSize = 3;
    double k = 0.04;
    int thresh = 125;

    double t = (double)cv::getTickCount();

    cv::Mat output = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, output, blockSize, apatureSize, k);

    cv::Mat norm, norm_scaled;
    cv::normalize( output, norm, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs( norm, norm_scaled );
    for( int i = 0; i < norm.rows ; i++ )
    {
        for( int j = 0; j < norm.cols; j++ )
        {
            if( (int) norm.at<float>(i,j) > thresh )
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(j, i);
                newKeyPoint.size = blockSize;
                keypoints.push_back(newKeyPoint);
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Harris Corner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsModern(vector<cv::KeyPoint>& keypoints, cv::Mat& img, std::string detectorType, bool bVis)
{
    // initialize detector
    cv::Ptr<cv::FeatureDetector> detector;
    
    if (detectorType == "FAST")
    {
        int threshold = 100;
        detector = cv::FastFeatureDetector::create(threshold);
    }
    else if (detectorType == "BRISK")
    {
        int threshold = 100;
        detector = cv::BRISK::create(threshold);
    }
    else if (detectorType == "ORB")
    {
        int nFeatures = 200;
        detector = cv::ORB::create(nFeatures);
    }
    else if (detectorType == "AKAZE")
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType == "SIFT")
    {
        detector = cv::xfeatures2d::SIFT::create();
    }

    double t = (double)cv::getTickCount();

    detector->detect(img, keypoints, cv::Mat());

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + "Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}