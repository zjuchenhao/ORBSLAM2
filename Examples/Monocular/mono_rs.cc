#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#include<time.h>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Eigen>

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps, int numberOfImage);

int main(int argc, char **argv)
{
    if (argc != 7)
    {
        cerr << endl
             << "Usage: ./mono_rs [1]path_to_vocabulary [2]path_to_settings [3]path_to_sequence [4]start_number_of_image [5]end_number_of_image [6]skip_number_of_image." << endl;
        return 1;
    }

    string path = string(argv[3]);
    int nStart = atoi(argv[4]);
    int nEnd = atoi(argv[5]);
    int nSkip = atoi(argv[6]);
    int nImages = nEnd - nStart;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nEnd - nStart << endl
         << endl;

    time_t t = time(NULL);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S.txt", localtime(&t));
    puts(tmp);
    ofstream traj_file(tmp);

    // Main loop
    cv::Mat im;
    double sum_t = 0;
    double count = 0;
    for (int ni = nStart; ni < nEnd; ni+=nSkip)
    {
        char tmp[10];
        sprintf(tmp, "%05d.png", ni);
        // sprintf(tmp, "%d", ni);

        // Read image from file
        std::string fileName = path + "/" + string(tmp);
        im = cv::imread(fileName, CV_LOAD_IMAGE_UNCHANGED);

        if (im.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << fileName << endl;
            return 1;
        }

        // cv::resize(im, im, cv::Size(), 0.5, 0.5);

#ifdef COMPILEDWITHC11
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        auto start = std::chrono::system_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocular(im, ni);

#ifdef COMPILEDWITHC11
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        auto end = std::chrono::system_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        double ttrack = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();

        sum_t += ttrack;
        count++;
        vTimesTrack.push_back(ttrack);

        if(!Tcw.empty()){
                Eigen::Matrix4d T_c_w_;
                T_c_w_ << Tcw.at<float>(0,0),
                        Tcw.at<float>(0,1),
                        Tcw.at<float>(0,2),
                        Tcw.at<float>(0,3),
                        Tcw.at<float>(1,0),
                        Tcw.at<float>(1,1),
                        Tcw.at<float>(1,2),
                        Tcw.at<float>(1,3),
                        Tcw.at<float>(2,0),
                        Tcw.at<float>(2,1),
                        Tcw.at<float>(2,2),
                        Tcw.at<float>(2,3),
                        Tcw.at<float>(3,0),
                        Tcw.at<float>(3,1),
                        Tcw.at<float>(3,2),
                        Tcw.at<float>(3,3);
                Eigen::Matrix4d pose_m = T_c_w_.inverse();
                traj_file << pose_m(0, 0) << ' '
                        << pose_m(0, 1) << ' '
                        << pose_m(0, 2) << ' '
                        << pose_m(0, 3) << ' '
                        << pose_m(1, 0) << ' '
                        << pose_m(1, 1) << ' '
                        << pose_m(1, 2) << ' '
                        << pose_m(1, 3) << ' '
                        << pose_m(2, 0) << ' '
                        << pose_m(2, 1) << ' '
                        << pose_m(2, 2) << ' '
                        << pose_m(2, 3) << '\n';
        }else
        {
                traj_file << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << ' '
                          << 0.0 << '\n';
        }
        



        usleep(2e5);
    }

    traj_file.close();

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    cout << "mean FPS: " << 1e6/(sum_t/count) << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
