
#include <MapMatcher.h>

namespace ORB_SLAM2
{
//自己写的构造函数
MapMatcher::MapMatcher()
{

}
MapMatcher::MapMatcher(KeyFrameDatabase* pDB, ORBVocabulary* pVoc, Map* pMap, Map* pMap0, Map* pMap1, Map* pMap2, Map* pMap3)
    :
        mpKFDB(pDB), mpVoc(pVoc), mpMap0(pMap0), mpMap1(pMap1), mpMap2(pMap2), mpMap3(pMap3),
        mLastLoopKFid(0),
        mbFixScale(false),
        mnCovisibilityConsistencyTh(3)
        // mnCovisibilityConsistencyTh(params::placerec::miCovisibilityConsistencyTh)//3
{

    if (pMap0)
        mmpMaps[*(pMap0->msuAssClients.begin())] = pMap0;
    if (pMap1)
        mmpMaps[*(pMap1->msuAssClients.begin())] = pMap1;
    if (pMap2)
        mmpMaps[*(pMap2->msuAssClients.begin())] = pMap2;
    if (pMap3)
        mmpMaps[*(pMap3->msuAssClients.begin())] = pMap3;

    if (pMap0)
        mspMaps.insert(pMap0);
    if (pMap1)
        mspMaps.insert(pMap1);
    if (pMap2)
        mspMaps.insert(pMap2);
    if (pMap3)
        mspMaps.insert(pMap3);

    // mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMatcherMarkers", 10);

    // mMatchMatrix = cv::Mat::zeros(4, 4, 2);

    // mMapMatchEdgeMsg.header.frame_id = "world";
    // mMapMatchEdgeMsg.header.stamp = ros::Time::now();
    // mMapMatchEdgeMsg.ns = "MapMatchEdges_red";
    // mMapMatchEdgeMsg.type = visualization_msgs::Marker::LINE_LIST;
    // mMapMatchEdgeMsg.color = Colors::msgRed();
    // mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    // mMapMatchEdgeMsg.scale.x = params::vis::mfLoopMarkerSize;
    // mMapMatchEdgeMsg.id = 1;
}

void MapMatcher::Run()
    {
        // double CovGraphMarkerSize;
        // mNhPrivate.param("MarkerSizeServer", CovGraphMarkerSize, 0.001);
        // mpMapMerger.reset(new MapMerger(shared_from_this()));

#ifdef LOGGING
        KeyFrame::ccptr pCC = mpMap0->GetCCPtr(0);
        if (!pCC)
        {
            std::cout << COUTERROR << "pCC not valid" << std::endl;
            KILLSYS
        }
#endif

        while (1)
        {
#ifdef LOGGING
            pCC->mpLogger->SetMatch(__LINE__, 0);
#endif

            if (CheckKfQueue())//查询传送到server的所有帧，存放在mlKfInQueue
            {
                bool bDetect = DetectLoop();
                if (bDetect)
                {
                    bool bSim3 = ComputeSim3();
                    if (bSim3)
                    {
                        // Perform loop fusion and pose graph optimization
                        CorrectLoop();
                    }
                }
            }

#ifdef LOGGING
            pCC->mpLogger->SetMatch(__LINE__, 0);
#endif

            usleep(5000);
        }
    }
    
    bool MapMatcher::DetectLoop()
    {
        {
            unique_lock<mutex> lock(mMutexKfInQueue);
            mpCurrentKF = mlKfInQueue.front();

            mlKfInQueue.pop_front();
            // Avoid that a keyframe can be erased while it is being process by this thread
            mpCurrentKF->SetNotErase();
        }

        // If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
        if (mpCurrentKF->mId.first < 1) //<30,也就是说前30帧不进行回环检测（可能还在初始化？）;这里我改成1了，因为我的mId.first超过30的好像比较少
        // if (mpCurrentKF->mId.first < params::placerec::miStartMapMatchingAfterKf) //<30,也就是说前30帧不进行回环检测
        {
            mpCurrentKF->SetErase();
            return false;
        }

        mpCurrMap = mpCurrentKF->GetMapptr(); // get map of KF，没错就是这里，首先判断这个帧是来自哪个地图的，如果来自不同地图才进行的是地图匹配和融合，否则就是回环检测，假设当前帧所属的地图是Map1。其实目前两个地图的帧都存放在一个mpMap中
        
        if (!mpCurrMap)
        {
            cout << ": In \"MapMatcher::DetectLoop()\": mpCurrMap is nullptr -> KF not contained in any map" << endl;
            // throw estd::infrastructure_ex();
        }

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
        const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
        float minScore = 1;
        for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++)
        {
            KeyFrame* pKF = vpConnectedKeyFrames[i];
            // add by chj
            if(!pKF)
                continue;
            if (pKF->isBad())
                continue;
            const DBoW2::BowVector &BowVec = pKF->mBowVec;

            float score = mpVoc->score(CurrentBowVec, BowVec);//调试报段错误，发现是mpVoc在构造函数中没定义导致的

            if (score < minScore)
                minScore = score;
        }

        // Query the database imposing the minimum score
        vector<KeyFrame*> vpCandidateKFs = mpKFDB->DetectMapMatchCandidates(mpCurrentKF, minScore, mpCurrMap);

        // If there are no loop candidates, just add new keyframe and return false
        if (vpCandidateKFs.empty())
        {
            mmvConsistentGroups[mpCurrMap].clear();
            mpCurrentKF->SetErase();
            return false;
        }

        // For each loop candidate check consistency with previous loop candidates
        // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
        // A group is consistent with a previous group if they share at least a keyframe
        // We must detect a consistent loop in several consecutive keyframes to accept it
        mvpEnoughConsistentCandidates.clear();

        vector<ConsistentGroup> vCurrentConsistentGroups; // pair <set<KF*>,int> --> int counts consistent groups found for this group
        vector<bool> vbConsistentGroup(mmvConsistentGroups[mpCurrMap].size(), false);
        // mvConsistentGroups stores the last found consistent groups.

        for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++)
        {
            KeyFrame* pCandidateKF = vpCandidateKFs[i];

            set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
            spCandidateGroup.insert(pCandidateKF);
            // group with candidate and connected KFs

            bool bEnoughConsistent = false;
            bool bConsistentForSomeGroup = false;
            for (size_t iG = 0, iendG = mmvConsistentGroups[mpCurrMap].size(); iG < iendG; iG++)
            {
                set<KeyFrame*> sPreviousGroup = mmvConsistentGroups[mpCurrMap][iG].first;

                bool bConsistent = false;
                for (set<KeyFrame*>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++)
                {
                    if (sPreviousGroup.count(*sit))
                    {
                        // KF found that is contained in candidate's group and comparison group
                        bConsistent = true;
                        bConsistentForSomeGroup = true;
                        break;
                    }
                }

                if (bConsistent)
                {
                    int nPreviousConsistency = mmvConsistentGroups[mpCurrMap][iG].second;
                    int nCurrentConsistency = nPreviousConsistency + 1;
                    if (!vbConsistentGroup[iG])
                    {
                        ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
                        vCurrentConsistentGroups.push_back(cg);
                        vbConsistentGroup[iG] = true; // this avoid to include the same group more than once
                    }
                    if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent)
                    {
                        mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                        bEnoughConsistent = true; // this avoid to insert the same candidate more than once
                    }
                }
            }

            // If the group is not consistent with any previous group insert with consistency counter set to zero
            if (!bConsistentForSomeGroup)
            {
                ConsistentGroup cg = make_pair(spCandidateGroup, 0); // For "ConsistentGroup" the "int" is initialized with 0
                vCurrentConsistentGroups.push_back(cg);
            }
        }

        // Update Covisibility Consistent Groups
        mmvConsistentGroups[mpCurrMap] = vCurrentConsistentGroups;

        if (mvpEnoughConsistentCandidates.empty())
        {
            mpCurrentKF->SetErase();
            return false;
        }
        else
        {
            return true;
        }

        mpCurrentKF->SetErase();
        return false;
    }

    bool MapMatcher::ComputeSim3()
    {
        const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

        // We compute first ORB matches for each candidate
        // If enough matches are found, we setup a Sim3Solver
        ORBmatcher matcher(0.75, true);

        vector<Sim3Solver *> vpSim3Solvers;
        vpSim3Solvers.resize(nInitialCandidates);

        vector<vector<MapPoint*>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);

        int nCandidates = 0; // candidates with enough matches

        for (int i = 0; i < nInitialCandidates; i++)
        {
            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // avoid that local mapping erase it while it is being processed in this thread
            pKF->SetNotErase();

            if (pKF->isBad())
            {
                vbDiscarded[i] = true;
                continue;
            }

            int nmatches = matcher.SearchByBoW(mpCurrentKF, pKF, vvpMapPointMatches[i]);

            // if (nmatches < params::opt::mMatchesThres)
            if (nmatches < 20)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                Sim3Solver *pSolver = new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);

                pSolver->SetRansacParameters(0.99,6,300);
                // pSolver->SetRansacParameters(params::opt::mProbability, params::opt::mMinInliers, params::opt::mMaxIterations);
                vpSim3Solvers[i] = pSolver;
            }

            nCandidates++;
        }

        bool bMatch = false;

        // Perform alternatively RANSAC iterations for each candidate
        // until one is succesful or all fail
        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nInitialCandidates; i++)
            {
                if (vbDiscarded[i])
                    continue;

                KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                Sim3Solver *pSolver = vpSim3Solvers[i];
                cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
                // cv::Mat Scm = pSolver->iterate(params::opt::mSolverIterations, bNoMore, vbInliers, nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                if (!Scm.empty())
                {
                    vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++)
                    {
                        if (vbInliers[j])
                            vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                    }

                    cv::Mat R = pSolver->GetEstimatedRotation();
                    cv::Mat t = pSolver->GetEstimatedTranslation();
                    const float s = pSolver->GetEstimatedScale();
                    matcher.SearchBySim3(mpCurrentKF, pKF, vpMapPointMatches, s, R, t, 7.5);

                    g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
                    const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                    // If optimization is succesful stop ransacs and continue
                    if (nInliers >= 20)
                    // if (nInliers >= params::opt::mInliersThres)
                    {
                        bMatch = true;
                        mpMatchedKF = pKF;
                        g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()), Converter::toVector3d(pKF->GetTranslation()), 1.0);
                        mg2oScw = gScm * gSmw;
                        mScw = Converter::toCvMat(mg2oScw);

                        mvpCurrentMatchedPoints = vpMapPointMatches;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }

        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
        vpLoopConnectedKFs.push_back(mpMatchedKF);
        mvpLoopMapPoints.clear();
        for (vector<KeyFrame*>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++)
        {
            KeyFrame* pKF = *vit;
            vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
            {
                MapPoint* pMP = vpMapPoints[i];
                if (pMP)
                {
                    if (!pMP->isBad() && pMP->mLoopPointForKF_MM != mpCurrentKF->mId) // ID Tag
                    {
                        mvpLoopMapPoints.push_back(pMP);
                        pMP->mLoopPointForKF_MM = mpCurrentKF->mId;
                    }
                }
            }
        }

        // Find more matches projecting with the computed Sim3
        matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10);

        // If enough matches accept Loop
        int nTotalMatches = 0;
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
        {
            if (mvpCurrentMatchedPoints[i])
                nTotalMatches++;
        }

        if (nTotalMatches >= 40)
        // if (nTotalMatches >= params::opt::mTotalMatchesThres)
        {
            for (int i = 0; i < nInitialCandidates; i++)
                if (mvpEnoughConsistentCandidates[i] != mpMatchedKF)
                    mvpEnoughConsistentCandidates[i]->SetErase();
            return true;
        }
        else
        {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            mpCurrentKF->SetErase();
            return false;
        }
    }

    void MapMatcher::CorrectLoop()
    {
        cout << "\033[1;32m!!! MAP MATCH FOUND !!!\033[0m" << endl;

        set<size_t> suAssCLientsCurr = mpCurrentKF->GetMapptr()->msuAssClients;
        set<size_t> suAssCLientsMatch = mpMatchedKF->GetMapptr()->msuAssClients;

        for (set<size_t>::iterator sit = suAssCLientsCurr.begin(); sit != suAssCLientsCurr.end(); ++sit)
        {
            size_t idc = *sit;
            for (set<size_t>::iterator sit2 = suAssCLientsMatch.begin(); sit2 != suAssCLientsMatch.end(); ++sit2)
            {
                size_t idm = *sit2;

                if (idc == idm)
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Associated Clients of matched and current map intersect" << endl; // 这句话的意思是匹配的地图和当前地图的关联客户端有交集。在代码中，这句话出现在检查两个地图关联客户端集合中是否有相同客户端的部分。

                mMatchMatrix.at<uint16_t>(idc, idm) = mMatchMatrix.at<uint16_t>(idc, idm) + 1;
                mMatchMatrix.at<uint16_t>(idm, idc) = mMatchMatrix.at<uint16_t>(idm, idc) + 1;
            }
        }

        if (mpCurrentKF->mId.second == mpMatchedKF->mId.second)
            cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belong to same client" << endl;
        if (!mpCurrMap->msuAssClients.count(mpCurrentKF->mId.second))
            cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Current KFs does not belong to current map" << endl;
        if (mpCurrMap->msuAssClients.count(mpMatchedKF->mId.second))
            cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belongs to current map" << endl;

        // if (1)
        // if (params::vis::mbActive)
            //PublishLoopEdges();//todo

        Map* pMatchedMap = mpMatchedKF->GetMapptr();

        MapMatchHit MMH(mpCurrentKF, mpMatchedKF, mg2oScw, mvpLoopMapPoints, mvpCurrentMatchedPoints);
        mFoundMatches[mpCurrMap][pMatchedMap].push_back(MMH);
        mFoundMatches[pMatchedMap][mpCurrMap].push_back(MMH);

        if (mFoundMatches[mpCurrMap][pMatchedMap].size() >= 1)
        {
            vector<MapMatchHit> vMatches = mFoundMatches[mpCurrMap][pMatchedMap];

            // Map* pMergedMap = mpMapMerger->MergeMaps(mpCurrMap, pMatchedMap, vMatches);
            // Map* pMergedMap = mpMapMerger->MergeMaps(static_cast<MapMerger::Map*>(mpCurrMap), pMatchedMap, vMatches);
        }//报错提示第一个参数的类型要转换

        // this->ClearLoopEdges();
    }

    // void MapMatcher::PublishLoopEdges()
    // {
    //     mMapMatchEdgeMsg.points.clear();

    //     tf::StampedTransform Tf_W_Curr, Tf_W_Matched;
    //     string FrameIdCurr, FrameIdMatched;

    //     FrameIdCurr = mpCurrentKF->GetMapptr()->mOdomFrame;
    //     FrameIdMatched = mpMatchedKF->GetMapptr()->mOdomFrame;

    //     try
    //     {
    //         mTfListen.lookupTransform("world", FrameIdCurr, ros::Time(0), Tf_W_Curr);
    //         mTfListen.lookupTransform("world", FrameIdMatched, ros::Time(0), Tf_W_Matched);
    //     }
    //     catch (tf::TransformException ex)
    //     {
    //         ROS_ERROR("%s", ex.what());
    //         return;
    //     }

    //     cv::Mat TCurr = mpCurrentKF->GetPoseInverse();
    //     cv::Mat TMatch = mpMatchedKF->GetPoseInverse();

    //     tf::Point PTfCurr{params::vis::mfScaleFactor * ((double)(TCurr.at<float>(0, 3))), params::vis::mfScaleFactor * ((double)(TCurr.at<float>(1, 3))), params::vis::mfScaleFactor * ((double)(TCurr.at<float>(2, 3)))};
    //     tf::Point PTfMatch{params::vis::mfScaleFactor * ((double)(TMatch.at<float>(0, 3))), params::vis::mfScaleFactor * ((double)(TMatch.at<float>(1, 3))), params::vis::mfScaleFactor * ((double)(TMatch.at<float>(2, 3)))};

    //     PTfCurr = Tf_W_Curr * PTfCurr;
    //     PTfMatch = Tf_W_Matched * PTfMatch;

    //     geometry_msgs::Point PCurr;
    //     geometry_msgs::Point PMatch;

    //     tf::pointTFToMsg(PTfCurr, PCurr);
    //     tf::pointTFToMsg(PTfMatch, PMatch);

    //     mMapMatchEdgeMsg.points.push_back(PCurr);
    //     mMapMatchEdgeMsg.points.push_back(PMatch);

    //     mPubMarker.publish(mMapMatchEdgeMsg);
    // }

    // void MapMatcher::ClearLoopEdges()
    // {
    //     mMapMatchEdgeMsg.action = 3;
    //     mPubMarker.publish(mMapMatchEdgeMsg);
    //     mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    // }

    // void MapMatcher::InsertKF(KeyFrame* pKF)
    // {
    //     unique_lock<mutex> lock(mMutexKfInQueue);

    //     mlKfInQueue.push_back(pKF);
    // }

    void MapMatcher::InsertKF(const std::map<idpair, KeyFrame*>& mmpkf) 
    {
        unique_lock<mutex> lock(mMutexKfInQueue);
        for (const auto& pair : mmpkf) 
            mlKfInQueue.push_back(pair.second);
        cerr << "kfs have been inserted!" << endl;
        int kf_num = mlKfInQueue.size();
        cerr << "mlKfInQueue.size is:" << kf_num << endl;
    }

    void MapMatcher::EraseKFs(vector<KeyFrame*> vpKFs)
    {
        unique_lock<mutex> lock(mMutexKfInQueue);

        for (vector<KeyFrame*>::iterator vit = vpKFs.begin(); vit != vpKFs.end(); ++vit)
        {
            KeyFrame* pKF = *vit;
            std::list<KeyFrame*>::iterator lit = find(mlKfInQueue.begin(), mlKfInQueue.end(), pKF);
            if (lit != mlKfInQueue.end())
                mlKfInQueue.erase(lit);
        }
    }

    int MapMatcher::GetNumKFsinQueue()
    {
        unique_lock<mutex> lock(mMutexKfInQueue, defer_lock);
        if (lock.try_lock())
        {
            return mlKfInQueue.size();
        }
        else
            return -1;
    }

    bool MapMatcher::CheckKfQueue()
    {
        unique_lock<mutex> lock(mMutexKfInQueue, defer_lock);
        if (lock.try_lock()) {
            // 互斥锁成功加锁，可以继续执行需要<保护的代码
            return (!mlKfInQueue.empty());
        }
        else
            return false;
    }

}
