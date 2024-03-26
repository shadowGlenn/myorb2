
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<unistd.h>
#include<MapMatcher.h>
#include<MapMerger.h>



using namespace std;

class MapMatcher;
class Map;
class KeyFrame;

typedef pair<size_t,size_t> idpair;



int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.在里面增加了mapmatcher初始化，只不过构造函数是空的；此外把mapmatcher线程移动到insertkf之后，因为现在是离线模式
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // ORB_SLAM2::Map mymap;
    //把kf添加到了 mlKfInQueue 中,注意是Map::mlKfInQueue;此外，加载的两个地图都存放在同一个mpMap中
    SLAM.LoadMap("/home/chj/下载/ORB_SLAM2_detailed_comments-master-master/Examples/Monocular/map0_test.bin");
    SLAM.LoadMap("/home/chj/下载/ORB_SLAM2_detailed_comments-master-master/Examples/Monocular/map1.bin");
    
    // mpMapMatcher->InsertKF();//
    // SLAM.mmpKeyFrames = mpMap->mmpKeyFrames;

    // std::map<idpair,KeyFrame*> mmpKeyFrames_temp = SLAM.mpMap->GetmmpKeyFrames();
    SLAM.mpMapMatcher->InsertKF(SLAM.mpMap->GetmmpKeyFrames());

    SLAM.mptMapMatching = new thread(&ORB_SLAM2::MapMatcher::Run,SLAM.mpMapMatcher);
    SLAM.mptMapMatching->join();
    

    // SLAM.Shutdown();

    return 0;
}


