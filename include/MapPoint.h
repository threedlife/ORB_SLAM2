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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

// MapPoint is the bridge between keyframes and observations(keypoints)
//
//         KeyFrame  <--- N:1 ---  MapPoint  --- 1:N --->  Observations (Keypoints)
//                   ---- 1:N -->            <-- 1:1 ----
class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    // Param idxF: index of the keypoint inside the Frame
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    // add index of MapPoint observable by the KF into the KF-index map mObservations
    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();
    
    // Replace MP after loop closure
    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    // Compute the best (min mean distance to all observations) descriptor for the MapPoint, since one MapPoint can be observed by many keyframes. 
    // It shall be called after keyframe is inserted 
    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    // Update normal and depth (max/min distance) by averaging all observations of the MapPoint
    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    
    // Returns a predicted pyramid scale in which a MapPoint at currentDist will likely appear in the (Key)Frame
    int PredictScale(const float &currentDist, KeyFrame *pKF);
    // Returns a predicted pyramid scale in which a MapPoint at currentDist will likely appear in the Frame
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    // u coordinate (in pixel) of the MapPoint
    float mTrackProjX;
    // v coordinate (in pixel) of the MapPoint
    float mTrackProjY;
    // u coordinate (in pixel) of the MapPoint in the right image of Stereo mode
    float mTrackProjXR;
    // Set by Frame::isInFrustum(), true when the MapPoint inside the FOV of the Frame
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     /* near features can be observed at higher level; far features can be observed at higher resolution
                      __
        Nearer      /____\      level: n-1 ----> dmin,   d/dmin = 1.2^(n-1 - m)
                   /______\
                  /________\    level: m   ----> d,      m = ceil(log(dmax/d) / log(1.2))
        Farther  /__________\   
                /____________\  level: 0   ----> dmax,   dmax/d = 1.2^m
     */
     // Scale invariance distances, i.e. from what range the feature can be observed across all scale levels
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
