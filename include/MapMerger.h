
#ifndef CSLAM_MAPMERGER_H_
#define CSLAM_MAPMERGER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

// //ROS,离线测试应该不需要ros
// #include <ros/ros.h>
// #include <ros/publisher.h>

//CSLAM
// #include <config.h>
#include <estd.h>
#include <Converter.h>
// #include <Datatypes.h>
// #include <CentralControl.h>
#include <Map.h>
#include <KeyFrame.h>
#include <MapMatcher.h>
// #include <ClientHandler.h>
#include <MapPoint.h>

// #include <LoopClosing.h>
#include <Optimizer.h>

//THIRDPARTY
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
// using namespace estd;

namespace ORB_SLAM2{

//forward decs
class MapMatcher;
// class ClientHandler;
class Map;
// class CentralControl;
class KeyFrame;
class MapPoint;
struct MapMatchHit;
//-----------------

class MapMerger
{
public:
    class Map;
    typedef boost::shared_ptr<MapMerger> mergeptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    // typedef boost::shared_ptr<ClientHandler> chptr;
    // typedef boost::shared_ptr<CentralControl> ccptr;
    typedef pair<size_t,size_t> idpair;

public:
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
        

public:
    MapMerger(matchptr pMatcher);
    Map* MergeMaps(Map* pMapCurr, Map* pMapMatch, vector<MapMatchHit> vMatchHits);

    bool isBusy();
private:
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<MapPoint*> vpLoopMapPoints);
    void SetBusy();
    void SetIdle();

    

    bool bIsBusy;
    matchptr mpMatcher;
    mutex mMutexBusy;

    std::vector<KeyFrame*> mvpCurrentConnectedKFs;

    void RunGBA(idpair nLoopKf, Map* pFusedMap);
};

} //end namespace

#endif
