
#ifndef CSLAM_MAPMATCHER_H_
#define CSLAM_MAPMATCHER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>

//ROS
// #include <visualization_msgs/Marker.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>

//CSLAM
// #include <config.h>
#include <estd.h>
// #include <Datatypes.h>
#include <KeyFrameDatabase.h>
#include <Map.h>
#include <KeyFrame.h>
#include <ORBVocabulary.h>
#include <ORBmatcher.h>
#include <Sim3Solver.h>
#include <Converter.h>
#include <Optimizer.h>
#include <MapMerger.h>

using namespace std;
// using namespace estd;

namespace ORB_SLAM2{

//forward decs
class KeyFrameDatabase;
class MapMerger;
class Map;
//-------------

struct MapMatchHit
{
public:
    MapMatchHit(KeyFrame* pKFCurr = nullptr, KeyFrame* pKFMatch = nullptr, g2o::Sim3 g2oScw = g2o::Sim3(), std::vector<MapPoint*> vpLoopMapPoints = /*todo*/std::vector<MapPoint*>(), std::vector<MapPoint*> vpCurrentMatchedPoints = std::vector<MapPoint*>())
        : mpKFCurr(pKFCurr), mpKFMatch(pKFMatch), mg2oScw(g2oScw),
          mvpLoopMapPoints(vpLoopMapPoints), mvpCurrentMatchedPoints(vpCurrentMatchedPoints) {}//构造函数
    KeyFrame* mpKFCurr;
    KeyFrame* mpKFMatch;
    g2o::Sim3 mg2oScw;
    std::vector<MapPoint*> mvpLoopMapPoints;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
};

class MapMatcher : public boost::enable_shared_from_this<MapMatcher>
{
public:
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<MapMerger> mergeptr;

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;

    typedef pair<size_t,size_t> idpair;

public:
    MapMatcher(KeyFrameDatabase* pDB, ORBVocabulary* pVoc, Map* pMap, Map* pMap0, Map* pMap1, Map* pMap2, Map* pMap3);
    MapMatcher();//自己写的一个构造函数

    void Run();
    // void InsertKF(KeyFrame* pKF);
    void InsertKF(const std::map<idpair, KeyFrame*>& mmpkf);
    void EraseKFs(vector<KeyFrame*> vpKFs);
    int GetNumKFsinQueue();

private:
    bool CheckKfQueue();
    bool DetectLoop();
    bool ComputeSim3();
    void CorrectLoop();
    void PublishLoopEdges();
    void ClearLoopEdges();

    //Note: No need for a reset function for the MapMatcher

    //comment by chj
    // ros::NodeHandle mNh;
    // ros::NodeHandle mNhPrivate;

    // ros::Publisher mPubMarker;

    // visualization_msgs::Marker mMapMatchEdgeMsg;
    // tf::TransformListener mTfListen;

    KeyFrameDatabase* mpKFDB;
    ORBVocabulary* mpVoc;
    map<size_t,Map*> mmpMaps;
    set<Map*> mspMaps;

    Map* mpMap0;
    Map* mpMap1;
    Map* mpMap2;
    Map* mpMap3;

    mergeptr mpMapMerger;
//ccm中闭环检测和地图合并要处理的帧存放在同一个变量中，能否各自存一个变量？暂时认为可以，所以以下变量就是orb2中的地图合并用的,不过加载的时候已经存放在 mlploopkeyframe ，改成mlKfInQueue？这样的话就是单独调试地图合并，不管闭环检测；除非把其他地方的mlploopkeyframe也都改了
    std::list<KeyFrame*> mlKfInQueue;
    std::mutex mMutexKfInQueue;

    // Loop detector parameters
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    Map* mpCurrMap;

    float mnCovisibilityConsistencyTh;
    std::map<Map*,std::vector<ConsistentGroup>> mmvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    cv::Mat mMatchMatrix;
    std::map<Map*,std::map<Map*,vector<MapMatchHit>>> mFoundMatches;

    // Fix scale in the stereo/RGB-D case - reamnant from ORB-SLAM, always set to false
    bool mbFixScale;
};

} //end namespace

#endif
