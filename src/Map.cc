/**
 * @file Map.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 地图的实现
 * @version 0.1
 * @date 2019-02-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

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


#include "Map.h"
#include "Converter.h"


#include<mutex>

namespace ORB_SLAM2
{

//构造函数,地图点中最大关键帧id归0
Map::Map():mnMaxKFid(0)
{
}
// Map::Map(size_t MapId):mnMaxKFid(0),mMapId(MapId)
// {
// }

/*
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
//在地图中插入关键帧,同时更新关键帧的最大id
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

/*
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
//向地图中插入地图点
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

/**
 * @brief 从地图中删除地图点,但是其实这个地图点所占用的内存空间并没有被释放
 * 
 * @param[in] pMP 
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    //下面是作者加入的注释. 实际上只是从std::set中删除了地图点的指针, 原先地图点
    //占用的内存区域并没有得到释放
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    //是的,根据值来删除地图点
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * @param vpMPs Local MapPoints
 */
// 设置参考地图点用于绘图显示局部地图点（红色）
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

//REVIEW 这个好像没有用到
void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

//这个在原版的泡泡机器人注释的版本中是没有这个函数和上面的函数的
//REVIEW 目测也是当前在程序中没有被被用到过
int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

//获取地图中的所有关键帧
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

//获取地图中的所有地图点
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

//获取地图点数目
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

//获取地图中的关键帧数目
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

//获取参考地图点
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

//获取地图中最大的关键帧id
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

//清空地图中的数据
void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

//保存地图信息
void Map::Save ( const string& filename )
{
    //Print the information of the saving map
    cerr<<"Map.cc :: Map Saving to "<<filename <<endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);
 
    //Number of MapPoints
    unsigned long int nMapPoints = mspMapPoints.size();
    f.write((char*)&nMapPoints, sizeof(nMapPoints) );
    //Save MapPoint sequentially
    for ( auto mp: mspMapPoints ){
        //Save MapPoint
        SaveMapPoint( f, mp );
        // cerr << "Map.cc :: Saving map point number: " << mp->mnId << endl;
    }
 
    //Print The number of MapPoints
    cerr << "Map.cc :: The number of MapPoints is :"<<mspMapPoints.size()<<endl;
        
 
    //Grab the index of each MapPoint, count from 0, in which we initialized mmpnMapPointsIdx  
    GetMapPointsIdx(); 
 
    //Print the number of KeyFrames
    cerr <<"Map.cc :: The number of KeyFrames:"<<mspKeyFrames.size()<<endl;
    //Number of KeyFrames
    unsigned long int nKeyFrames = mspKeyFrames.size();
    f.write((char*)&nKeyFrames, sizeof(nKeyFrames));
 
    //Save KeyFrames sequentially
    for ( auto kf: mspKeyFrames )
        SaveKeyFrame( f, kf );
 
    for (auto kf:mspKeyFrames )
    {
        //Get parent of current KeyFrame and save the ID of this parent
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if ( parent )
            parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));
 
        //Get the size of the Connected KeyFrames of the current KeyFrames
        //and then save the ID and weight of the Connected KeyFrames
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con, sizeof(nb_con));
        for ( auto ckf: kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }
 
    // Save last Frame ID
    // SaveFrameID(f);
 
    f.close();
    cerr<<"Map.cc :: Map Saving Finished!"<<endl;
}
 
//保存地图点
void Map::SaveMapPoint( ofstream& f, MapPoint* mp)
{   
    //Save ID and the x,y,z coordinates of the current MapPoint
    f.write((char*)&mp->mnId, sizeof(mp->mnId));
    cv::Mat mpWorldPos = mp->GetWorldPos();
    f.write((char*)& mpWorldPos.at<float>(0),sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(1),sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(2),sizeof(float));
}
 
//存储关键帧函数
//保存关键帧的函数稍微复杂一点，
//首先需要明白一幅关键帧包含特征点，描述符，以及哪些特征点通过三角化成为了地图点。
void Map::SaveKeyFrame( ofstream &f, KeyFrame* kf )//wuhoup20190923
{
    //保存当前关键帧的ID和时间戳
    //Save the ID and timesteps of current KeyFrame
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    //cout << "saving kf->mnId = " << kf->mnId <<endl;
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
    //保存当前关键帧的位姿矩阵
    //Save the Pose Matrix of current KeyFrame
    cv::Mat Tcw = kf->GetPose();

    //直接保存旋转矩阵
    //Save the rotation matrix
    // for ( int i = 0; i < Tcw.rows; i ++ )
    // {
    //     for ( int j = 0; j < Tcw.cols; j ++ )
    //     {
    //         f.write((char*)&Tcw.at<float>(i,j), sizeof(float));
    //         //cerr<<"Tcw.at<float>("<<i<<","<<j<<"):"<<Tcw.at<float>(i,j)<<endl;
    //     }
    // }

    //通过四元数保存旋转矩阵
    //Save the rotation matrix in Quaternion
    std::vector<float> Quat = Converter::toQuaternion(Tcw);
    for ( int i = 0; i < 4; i ++ )
        f.write((char*)&Quat[i],sizeof(float));
    //保存平移矩阵
    //Save the translation matrix
    for ( int i = 0; i < 3; i ++ )
        f.write((char*)&Tcw.at<float>(i,3),sizeof(float));



    //保存当前关键帧包含的ORB特征数目
    //Save the size of the ORB features current KeyFrame
    //cerr<<"kf->N:"<<kf->N<<endl;
    f.write((char*)&kf->N, sizeof(kf->N));
    //保存每一个ORB特征点
    //Save each ORB features
    for( int i = 0; i < kf->N; i ++ )
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle,sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        //保存当前特征点的描述符
	//Save the Descriptors of current ORB features
        f.write((char*)&kf->mDescriptors.cols, sizeof(kf->mDescriptors.cols)); //kf->mDescriptors.cols is always 32 here.
        for (int j = 0; j < kf->mDescriptors.cols; j ++ )
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        //保存当前ORB特征对应的MapPoints的索引值
	//Save the index of MapPoints that corresponds to current ORB features
        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL  )
            mnIdx = ULONG_MAX;
        else
            mnIdx = mmpnMapPointsIdx[mp];

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }
}


//获取地图点ID
void Map::GetMapPointsIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    unsigned long int i = 0;
    for ( auto mp: mspMapPoints )
    {
        mmpnMapPointsIdx[mp] = i;
        i += 1;
    }
}

//地图加载函数
// void Map::Load ( const string &filename, SystemSetting* mySystemSetting)
void Map::Load ( const string &filename, SystemSetting* mySystemSetting,KeyFrameDatabase* mpKeyFrameDatabase)
{
    cerr << "Map.cc :: Map reading from:"<<filename<<endl;
    ifstream f;
    f.open( filename.c_str() );
 
    // Same as the sequence that we save the file, we first read the number of MapPoints.
    unsigned long int nMapPoints;
    f.read((char*)&nMapPoints, sizeof(nMapPoints));
 
    // Then read MapPoints one after another, and add them into the map
    cerr<<"Map.cc :: The number of MapPoints:"<<nMapPoints<<endl;
    for ( unsigned int i = 0; i < nMapPoints; i ++ )
    {
        MapPoint* mp = LoadMapPoint(f);
        AddMapPoint(mp);//地图加载时，添加的点默认是存放在当前系统中唯一的一个地图，如何存放到不同的地图？
        //1.map<size_t,map*> mspMaps;
        //2.把pMap加入到mspMaps中
    }
 
    // Get all MapPoints
    std::vector<MapPoint*> vmp = GetAllMapPoints();
 
    // Read the number of KeyFrames
    unsigned long int nKeyFrames;
    f.read((char*)&nKeyFrames, sizeof(nKeyFrames));
    cerr<<"Map.cc :: The number of KeyFrames:"<<nKeyFrames<<endl;
 
    // Then read KeyFrames one after another, and add them into the map
    vector<KeyFrame*>kf_by_order;
    for( unsigned int i = 0; i < nKeyFrames; i ++ )
    {
        KeyFrame* kf = LoadKeyFrame(f, mySystemSetting);
        AddKeyFrame(kf);//加载第二个地图时，调试到这里就报异常，应该是上面的函数就异常了
        kf_by_order.push_back(kf);
                //将关键帧添加到关键帧数据库中
        mpKeyFrameDatabase->add(kf);
        
    }
 
   
 
    cerr<<"Map.cc :: Max KeyFrame ID is: " << mnMaxKFid << ", and I set mnId to this number" <<endl;
    
 
    cerr<<"Map.cc :: KeyFrame Load OVER!"<<endl;
 
    // Read Spanning Tree(open loop trajectory)
    map<unsigned long int, KeyFrame*> kf_by_id;
    for ( auto kf: mspKeyFrames )
        kf_by_id[kf->mnId] = kf;
    cerr<<"Map.cc :: Start Load The Parent!"<<endl;
    for( auto kf: kf_by_order )
    {
        // Read parent_id of current KeyFrame.
        unsigned long int parent_id;
        f.read((char*)&parent_id, sizeof(parent_id));
 
        // Add parent KeyFrame to current KeyFrame.
        // cout<<"Map::Load : Add parent KeyFrame to current KeyFrame"<<endl;
        if ( parent_id != ULONG_MAX )
            kf->ChangeParent(kf_by_id[parent_id]);
 
        // Read covisibility graphs.
        // Read the number of Connected KeyFrames of current KeyFrame.
        unsigned long int nb_con;
        f.read((char*)&nb_con, sizeof(nb_con));
        // Read id and weight of Connected KeyFrames of current KeyFrame, 
        // and add Connected KeyFrames into covisibility graph.
        // cout<<"Map::Load : Read id and weight of Connected KeyFrames"<<endl;
        for ( unsigned long int i = 0; i < nb_con; i ++ )
        {
            unsigned long int id;
            int weight;
            f.read((char*)&id, sizeof(id));
            f.read((char*)&weight, sizeof(weight));
            kf->AddConnection(kf_by_id[id],weight);
        }
   }
   cerr<<"Map.cc :: Parent Load OVER!"<<endl;
   for ( auto mp: vmp )
   {
       // cout << "Now mp = "<< mp << endl;
       if(mp)
       {
            // cout << "compute for mp = "<< mp << endl;
            mp->ComputeDistinctiveDescriptors();
            // cout << "Computed Distinctive Descriptors." << endl;
            mp->UpdateNormalAndDepth();
            // cout << "Updated Normal And Depth." << endl;
        }
   }
    f.close();
    cerr<<"Map.cc :: Load IS OVER!"<<endl;
    return;
}
 
//地图点加载函数
MapPoint* Map::LoadMapPoint( ifstream &f )
{
        // Position and Orientation of the MapPoints.
        cv::Mat Position(3,1,CV_32F);
        long unsigned int id;
        f.read((char*)&id, sizeof(id));
 //从文件流中依次读取三个浮点数，分别表示地图点在x、y、z轴上的位置，然后将这些值分别存储到Position的对应位置。
        f.read((char*)&Position.at<float>(0), sizeof(float));
        f.read((char*)&Position.at<float>(1), sizeof(float));
        f.read((char*)&Position.at<float>(2), sizeof(float));
 
        // Initialize a MapPoint, and set its id and Position.
        MapPoint* mp = new MapPoint(Position, this );
        mp->mnId = id;
        mp->SetWorldPos( Position );
 
        return mp;
}
 
//关键帧加载函数
KeyFrame* Map::LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting )
{
    InitKeyFrame initkf(*mySystemSetting);
 
    // Read ID and TimeStamp of each KeyFrame.
    f.read((char*)&initkf.nId, sizeof(initkf.nId));
    f.read((char*)&initkf.TimeStamp, sizeof(double));
 
    // Read position and quaternion
    cv::Mat T = cv::Mat::zeros(4,4,CV_32F);
    std::vector<float> Quat(4);
    //Quat.reserve(4);
    for ( int i = 0; i < 4; i ++ )
        f.read((char*)&Quat[i],sizeof(float));
    cv::Mat R = Converter::toCvMat(Quat);
    for ( int i = 0; i < 3; i ++ )
        f.read((char*)&T.at<float>(i,3),sizeof(float));
    for ( int i = 0; i < 3; i ++ )
        for ( int j = 0; j < 3; j ++ )
            T.at<float>(i,j) = R.at<float>(i,j);
    T.at<float>(3,3) = 1;
    
    // Read feature point number of current Key Frame
    f.read((char*)&initkf.N, sizeof(initkf.N));
    initkf.vKps.reserve(initkf.N);//异常的起始点！！！
    initkf.Descriptors.create(initkf.N, 32, CV_8UC1);
    vector<float>KeypointDepth;
 
    std::vector<MapPoint*> vpMapPoints;
    vpMapPoints = vector<MapPoint*>(initkf.N,static_cast<MapPoint*>(NULL));
    // Read Keypoints and descriptors of current KeyFrame
    std::vector<MapPoint*> vmp = GetAllMapPoints();
    for(int i = 0; i < initkf.N; i ++ )
    {
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.read((char*)&kp.size, sizeof(kp.size));
        f.read((char*)&kp.angle,sizeof(kp.angle));
        f.read((char*)&kp.response, sizeof(kp.response));
        f.read((char*)&kp.octave, sizeof(kp.octave));
 
        initkf.vKps.push_back(kp);
        
        // Read descriptors of keypoints
        f.read((char*)&initkf.Descriptors.cols, sizeof(initkf.Descriptors.cols));
        // for ( int j = 0; j < 32; j ++ ) // Since initkf.Descriptors.cols is always 32, for loop may also write like this.
        for ( int j = 0; j < initkf.Descriptors.cols; j ++ )
            f.read((char*)&initkf.Descriptors.at<unsigned char>(i,j),sizeof(char));
 
        // Read the mapping from keypoints to MapPoints.
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));
 
        // Look up from vmp, which contains all MapPoints, MapPoint of current KeyFrame, and then insert in vpMapPoints.
        if( mpidx == ULONG_MAX )
                vpMapPoints[i] = NULL;
        else
                vpMapPoints[i] = vmp[mpidx];
    }
 
  
    initkf.vRight = vector<float>(initkf.N,-1);
    initkf.vDepth = vector<float>(initkf.N,-1);
    //initkf.vDepth = KeypointDepth;
    initkf.UndistortKeyPoints();
    initkf.AssignFeaturesToGrid();
 
    // Use initkf to initialize a KeyFrame and set parameters
    KeyFrame* kf = new KeyFrame( initkf, this, NULL, vpMapPoints );
    kf->mnId = initkf.nId;
    kf->SetPose(T);
    kf->ComputeBoW();
 
    for ( int i = 0; i < initkf.N; i ++ )
    {
        if ( vpMapPoints[i] )
        {
            vpMapPoints[i]->AddObservation(kf,i);
            if( !vpMapPoints[i]->GetReferenceKeyFrame())
                vpMapPoints[i]->SetReferenceKeyFrame(kf);
        }
    }
    return kf;
}
 
 
} //namespace ORB_SLAM
