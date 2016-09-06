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
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


void DrawMapPoints(cv::Mat& im, cv::Mat pose, ORB_SLAM2::Tracking* mpTracker);
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // argv[1] == Vocabulary/ORBvoc.txt: trained out from a huge database, which is more or less a kind of generic
    // argv[2] == Examples/Monocular/TUMX.yaml: parameters and camera parameter
    //ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Image showing window
    cv::namedWindow("Slam_Show", CV_WINDOW_AUTOSIZE);
    cv::Mat pose_;
    int lost_counter = 0;

    // Main loop
    cv::Mat im;
    //cv::Mat out_im;
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat pose = SLAM.TrackMonocular(im, tframe);
        if (!pose.empty()) {
            pose_ = pose;
            lost_counter = 0;
        } else {
            lost_counter++;
            cout << "lost_counter: " << lost_counter << endl;
        } 

        if (!pose_.empty() && lost_counter < 60) {
            DrawMapPoints(im, pose, SLAM.mpTracker);
        }
        cv::imshow("Slam_Show", im);
        cv::waitKey(150);
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast < std::chrono::duration<double> >(t2 - t1).count();
        cout << ni << ":" << ttrack << endl;
        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni > 0)
            T = tframe-vTimestamps[ni-1];

        if (ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();
    cv::destroyWindow("Slam_Show");

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
            string sRGB;
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}


void DrawMapPoints(cv::Mat& im, cv::Mat pose, ORB_SLAM2::Tracking* mpTracker)
{
    cv::Mat rVec;
    cv::Rodrigues(pose.colRange(0, 3).rowRange(0, 3), rVec);
    cv::Mat tVec = pose.col(3).rowRange(0, 3);

    const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpTracker->mpMap->GetAllMapPoints();
    const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpTracker->mpMap->GetReferenceMapPoints();
    set<ORB_SLAM2::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if (vpMPs.size() > 0) {
        std::vector<cv::Point3f> allmappoints;
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])) continue;
            cv::Point3f pos = cv::Point3f(vpMPs[i]->GetWorldPos());
            allmappoints.push_back(pos);
        }
        if (allmappoints.size() > 0) {
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(allmappoints, rVec, tVec, mpTracker->mK, mpTracker->mDistCoef, projectedPoints);
            for (size_t j = 0; j < projectedPoints.size(); ++j) {
                cv::Point2f r1 = projectedPoints[j];
                cv::circle(im, cv::Point(r1.x, r1.y), 2, cv::Scalar(255, 0, 0), 1, 8);
            }
        }

        std::vector<cv::Point3f> refmappoints;
        for (set<ORB_SLAM2::MapPoint*>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad()) continue;
            cv::Point3f pos = cv::Point3f((*sit)->GetWorldPos());
            refmappoints.push_back(pos);
        }
        if (refmappoints.size() > 0) {
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(refmappoints, rVec, tVec, mpTracker->mK, mpTracker->mDistCoef, projectedPoints);
            for (size_t j = 0; j < projectedPoints.size(); ++j) {
                cv::Point2f r1 = projectedPoints[j];
                cv::circle(im, cv::Point(r1.x, r1.y), 2, cv::Scalar(0, 255, 0), 1, 8);
            }
        }
    }
}
