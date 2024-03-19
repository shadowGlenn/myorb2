
#ifndef ESTD_H_
#define ESTD_H_

#include <vector>
#include <thread>
#include <mutex>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <numeric>


using namespace std;
namespace estd{

typedef pair<size_t,size_t> idpair;
typedef boost::shared_ptr<std::thread> threadptr;
// class UniqueIdDispenser;
// typedef boost::shared_ptr<UniqueIdDispenser> uidptr;

} //end ns

#endif
