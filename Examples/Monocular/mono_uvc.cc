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
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include <System.h>

#include <OpenNI.h>

using namespace std;
using namespace openni;

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 400
#define DEPTH_FPS 30
#define DEPTH_UNIT openni::PIXEL_FORMAT_DEPTH_100_UM
#define RGB_WIDTH 640
#define RGB_HEIGHT 480
#define RGB_FPS 30

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Initialize uvc camera
    cv::VideoCapture uvcCap;
    uvcCap.open(-1);
    if (uvcCap.isOpened())
    {
        cout << endl
             << "UVC camera started" << endl;
    }
    uvcCap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));      // set to MJPG mode first, otherwise it's RAW by default which doesnt support high fps
    uvcCap.set(cv::CAP_PROP_FRAME_WIDTH, RGB_WIDTH);
    uvcCap.set(cv::CAP_PROP_FRAME_HEIGHT, RGB_HEIGHT);
    uvcCap.set(cv::CAP_PROP_FPS, 30);

    // Get intrinsics
    openni::Status rc = openni::STATUS_OK;
    openni::Device device;
    openni::VideoStream depth, ir, color;
    // openni::VideoStream *streams[] = {&depth};
    openni::VideoFrameRef depthFrame;
    const char *deviceURI = openni::ANY_DEVICE;
    rc = openni::OpenNI::initialize();
    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }
    else
    {
        printf("Device uri: %s \n", device.getDeviceInfo().getUri());
    }
    OBCameraParams m_CamParams;
    int dataSize = sizeof(OBCameraParams);
    device.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&m_CamParams, &dataSize);
    printf("RGB camera intrinsics [fx,fy,cx,cy]: %f %f %f %f\n", m_CamParams.r_intr_p[0], m_CamParams.r_intr_p[1], m_CamParams.r_intr_p[2], m_CamParams.r_intr_p[3]);
    printf("RGB camera distortions [k1,k2,p1,p2,k3]: %f %f %f %f %f\n", m_CamParams.r_k[0], m_CamParams.r_k[1], m_CamParams.r_k[2], m_CamParams.r_k[3], m_CamParams.r_k[4]);
    printf("RGB resolution = %f x %f;    fps = %f\n", uvcCap.get(cv::CAP_PROP_FRAME_WIDTH), uvcCap.get(cv::CAP_PROP_FRAME_HEIGHT), uvcCap.get(cv::CAP_PROP_FPS));
    printf("Depth camera intrinsics [fx,fy,cx,cy]: %f %f %f %f\n", m_CamParams.l_intr_p[0], m_CamParams.l_intr_p[1], m_CamParams.l_intr_p[2], m_CamParams.l_intr_p[3]);
    printf("Depth camera distortions [k1,k2,p1,p2,k3]: %f %f %f %f %f\n", m_CamParams.l_k[0], m_CamParams.l_k[1], m_CamParams.l_k[2], m_CamParams.l_k[3], m_CamParams.l_k[4]);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    // vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;
    int nImages = 0;
    // press ESC key to exit
    while (cv::waitKey(10)!=27)
    {
        // Read image from file
        uvcCap >> im;
        // cout << std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() << endl;
        cout.precision(std::numeric_limits<double>::max_digits10);
        cout << uvcCap.get(cv::CAP_PROP_POS_MSEC) << endl << endl;

        if(im.empty())
        {
            cerr << endl << "Failed to capture uvc image" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // cout<<std::chrono::duration<double>(t1.time_since_epoch()).count()<<endl;
        // cout<<(t1.time_since_epoch()).count()/1000000000.0<<endl;
        SLAM.TrackMonocular(im, (t1.time_since_epoch()).count()/1000000000.0);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // vTimesTrack[nImages] = ttrack;
        vTimesTrack.push_back(ttrack);

        nImages++;
    }

    // Stop all threads
    SLAM.Shutdown();

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

