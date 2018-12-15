/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <opencv2/highgui/highgui.hpp>
#include "gms_matcher.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    //cv::Mat depth = *((cv::Mat*)userdata);
    int* pos = (int*)userdata;
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        //cout << "(" << x << ", " << y << "):    "<<depth.at<unsigned short>(y,x)<< endl;
        cout << "(" << x << ", " << y << ")"<< endl;
        pos[0] = x;
        pos[1] = y;
    }
}


void normalizeDepthImage(cv::Mat& depth, cv::Mat& disp)
{

    ushort* depthData = (ushort*)depth.data;
    int width = depth.cols;
    int height = depth.rows;

    static ushort max = *std::max_element(depthData, depthData + width*height);
    static ushort min = *std::min_element(depthData, depthData + width*height);

    disp = cv::Mat(depth.size(), CV_8U);
    uchar* dispData = disp.data;


    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            //if(depthData[i*width + j]<)
            dispData[i*width + j] = (((double)(depthData[i*width + j] - min)) / ((double)(max - min))) * 255;
        }

}




int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    bool show_matching = true;
    // Retrieve paths to images
    vector<string> vstrImageFilenamesLeft;
    vector<string> vstrImageFilenamesRight;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesLeft, vstrImageFilenamesRight, vTimestamps);
    cv::Mat im0 = cv::imread(string(argv[3])+"/"+vstrImageFilenamesLeft[0],cv::IMREAD_UNCHANGED);
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesLeft.size();
    if(vstrImageFilenamesLeft.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesRight.size()!=vstrImageFilenamesLeft.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
            3.3561143191038963e+02, 0, 3.6495411045338960e+02,
            0,3.3561143191038963e+02, 2.4697793407278658e+02,
            0,0,1);

    cv::Mat K2 = (cv::Mat_<double>(3,3) <<
            3.3348017604083498e+02, 0, 3.7658453490326741e+02,
            0, 3.3348017604083498e+02,  2.4108772323997113e+02,
            0,0,1);

    cv::Mat dist1 = (
            cv::Mat_<double>(5,1)<<-7.5258164091129703e-02, 4.1505130271465580e-02, -3.1686006775296914e-03, -1.6823993785786953e-03, 5.5918352998534085e-03
    );

    cv::Mat dist2 = (cv::Mat_<double>(5,1)<<-9.0381074354388399e-02, 9.3731289312594043e-02, -3.5218662942246307e-02, 1.1614056654231642e-03, -5.7202096279267596e-03);


    cv::Mat R = (cv::Mat_<double>(3,3)<<
            0.9840, 0.0128, 0.1777,
            -0.0109, 0.9999, -0.0112,
            -0.1779, 0.0091, 0.9840);

    cv::Mat t = (cv::Mat_<double>(3,1)<<
                                      -515.9966, -31.0014,  -41.7613);

    //R = R.t();
    //t = -R.t()*t;
    cv::Mat R1,R2,P1,P2,Q;

    cv::Mat remap_left1,remap_left2,remap_right1,remap_right2;
    cv::Rect validROI_left,validROI_right;
    cv::Size image_size = im0.size();


    cv::stereoRectify(K1,dist1,K2,dist2,image_size,R,t,R1,R2,P1,P2,Q,
                      cv::CALIB_ZERO_DISPARITY, 0, image_size, &validROI_left, &validROI_right);
    cv::initUndistortRectifyMap(K1, dist1, R1, P1, image_size, CV_16SC2, remap_left1, remap_left2);
    cv::initUndistortRectifyMap(K2, dist2, R2, P2, image_size, CV_16SC2, remap_right1, remap_right2);


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    // Main loop
    cv::Mat imLeft, imRight;

    std::string command = "mkdir -p "+string(argv[3])+"rectify/";

    system((command+"left").c_str());
    system((command+"right").c_str());

    std::cout<<"P1"<<std::endl<<P1<<std::endl;
    std::cout<<"P2"<<std::endl<<P2<<std::endl;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imLeft = cv::imread(string(argv[3])+"/"+vstrImageFilenamesLeft[ni],cv::IMREAD_UNCHANGED);
        imRight = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRight[ni],cv::IMREAD_UNCHANGED);
        cv::Mat rect_left,rect_right;
        cv::remap(imLeft, rect_left, remap_left1, remap_left2, cv::INTER_CUBIC);
        cv::remap(imRight, rect_right, remap_right1, remap_right2, cv::INTER_CUBIC);

        int leftX = std::max(validROI_left.x, validROI_right.x);
        int leftY = std::max(validROI_left.y, validROI_right.y);
        int width = std::min(validROI_left.width+validROI_left.x, validROI_right.width+validROI_right.x)-leftX;
        int height = std::min(validROI_left.height+validROI_left.y, validROI_right.height+validROI_right.y)-leftY;
        cv::Rect validRegion(leftX, leftY, width, height);
        //std::cout<<"width: "<<width<<" height: "<<height<<std::endl;

        rect_left = imLeft;
        rect_right = imRight;
        cv::imwrite(string(argv[3])+"/"+"rectify/"+vstrImageFilenamesLeft[ni], rect_left(validRegion));
        cv::imwrite(string(argv[3])+"/"+"rectify/"+vstrImageFilenamesRight[ni], rect_right(validRegion));

        cv::imshow("left",rect_left(validRegion));
        cv::imshow("right",rect_right(validRegion));



        if(show_matching)
        {
            std::vector<cv::KeyPoint> kp1, kp2;
            std::vector<cv::Point2f> kp1_align, kp2_align;

            cv::Mat d1, d2;
            vector<cv::DMatch> matches_all, matches_gms;
            cv::Ptr<cv::ORB> orb = cv::ORB::create(50000,1.2,8,20,0,2,cv::ORB::FAST_SCORE);


            //Extract feature points
            orb->detect(rect_left,kp1);
            orb->compute(rect_left,kp1,d1);
            orb->detect(rect_right,kp2);
            orb->compute(rect_right,kp2,d2);

            cv::BFMatcher matcher(cv::NORM_HAMMING);
            matcher.match(d1, d2, matches_all);



            int num_inliers = 0;
            std::vector<bool> vbInliers;
            gms_matcher gms(kp1,rect_left.size(), kp2,rect_right.size(), matches_all);
            num_inliers = gms.GetInlierMask(vbInliers, false, false);



            // draw matches
            for (size_t i = 0; i < vbInliers.size(); ++i)
            {
                if (vbInliers[i] == true)
                {
                    matches_gms.push_back(matches_all[i]);
                }
            }

                    //***********************************
            for(int i = 0;i<matches_gms.size();i++)
            {
                kp1_align.push_back(kp1[matches_gms[i].queryIdx].pt);
                kp2_align.push_back(kp2[matches_gms[i].trainIdx].pt);

            }
            cv::Mat F =  cv::findFundamentalMat(kp1_align,kp2_align);
            //std::cout<<F<<std::endl;
            cv::Mat H1,H2;
            cv::stereoRectifyUncalibrated(kp1_align,kp2_align,F,image_size,H1,H2);
            std::cout<<H2.t().inv()*F*H1.inv()<<std::endl;
            //************************************



            cv::Mat show = DrawInlier(rect_left, rect_right, kp1, kp2, matches_gms, 1);
            cv::namedWindow("show",0);
            cv::imshow("show", show);

        }


        cv::waitKey(1);
    }




    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesLeft,
                vector<string> &vstrImageFilenamesRight, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesLeft.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesRight.push_back(sD);

        }
    }
}
