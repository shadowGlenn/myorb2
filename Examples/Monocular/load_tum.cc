
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<unistd.h>
using namespace std;


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    // vector<string> vstrImageFilenames;
    // vector<double> vTimestamps;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    SLAM.LoadMap("/home/chj/下载/ORB_SLAM2_detailed_comments-master-master/Examples/Monocular/map0.bin");
    SLAM.LoadMap("/home/chj/下载/ORB_SLAM2_detailed_comments-master-master/Examples/Monocular/map1.bin");


    SLAM.Shutdown();

    return 0;
}


