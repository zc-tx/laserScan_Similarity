#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <dirent.h>
#include <fstream>
#include <vector>

using namespace std;
using namespace PointMatcherSupport;

class scan
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
public:
    PM::DataPoints pointCloud;
    int pointCount;
    float minRange;
    float maxRange;
    float sectionLength;
    vector<int> sectionCountVector;
    vector<float> sectionRatioVector;
    vector<float> rangeOfPointVector;
    void clear();
private:

protected:

};

void scan::clear()
{
    pointCount = 0;
    minRange = 9999;
    maxRange = 0;
    sectionCountVector.clear();
    sectionRatioVector.clear();
    rangeOfPointVector.clear();
}

class Similarity
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
public:
    Similarity(ros::NodeHandle &n);
    ~Similarity();

    void getEMD1D();
    void forScan0();
    void forScan1();
    void scan0Process();
    void scan1Process();
    float calculateRange(Eigen::Vector3f inputOrdinate);
    int judgeBucket(float range, float minRange, float sectionLength);

    void getRawScan(const sensor_msgs::PointCloud2& cloudMsgIn);

private:
    ros::Subscriber scanPointCloudSub;

    scan scan0;
    scan scan1;

    ros::NodeHandle& n;
    const int sectionNum;
    const bool isLog;
    const int truncatedStart;
    const int inputQueueSize;
    bool first = true;
    const int ringStart;
    const int ringEnd;
    int ringRow;

    float EMD;
    vector<vector<float>> scan0RingRatioVector;
    vector<vector<float>> scan1RingRatioVector;
    PM::DataPoints *scan0All;
    PM::DataPoints *scan1All;
protected:

};

Similarity::Similarity(ros::NodeHandle& n):
    n(n),
    sectionNum(getParam<int>("sectionNum", 100)),
    isLog(getParam<bool>("isLog", false)),
    truncatedStart(getParam<int>("truncatedStart", 0)),
    inputQueueSize(getParam<int>("inputQueueSize", 1)),
    ringStart(getParam<int>("ringStart", 0)),
    ringEnd(getParam<int>("ringEnd", 0))
{
    scanPointCloudSub = n.subscribe("/velodyne_points", inputQueueSize, &Similarity::getRawScan, this);
}

void Similarity::getRawScan(const sensor_msgs::PointCloud2 &cloudMsgIn)
{
    cout<<"---------------------------------"<<endl;
    if(first)
    {
        scan0All = new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
        this->forScan0();
        scan1All = new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
        this->forScan1();

        first = false;
    }
    else
    {
        scan1All = new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));
        this->forScan1();
    }

    this->getEMD1D();
    scan1.clear();
}

Similarity::~Similarity()
{

}

void Similarity::forScan0()
{
    //Process by Ring
    int ringRow = scan0All->getDescriptorStartingRow("ring");

    for(int i = ringStart; i <= ringEnd; i++)
    {
        scan0.pointCloud = scan0All->createSimilarEmpty();
        int count = 0;
        for(int j = 0; j < scan0All->features.cols(); j++)
        {
            if(scan0All->descriptors(ringRow, j) == i)
            {
                scan0.pointCloud.setColFrom(count, *scan0All, j);
                count++;
            }
        }
        scan0.pointCloud.conservativeResize(count);

        this->scan0Process();

    }
}

void Similarity::scan0Process()
{

    //Scan0 Pretreated
    scan0.pointCount = scan0.pointCloud.features.cols();
    scan0.maxRange = 0;
    scan0.minRange = 9999;
    for(int i = 0; i < scan0.pointCount; i++)
    {
        float rangeOfPoint = calculateRange(scan0.pointCloud.features.col(i).head(3));
        scan0.rangeOfPointVector.push_back(rangeOfPoint);
        if(rangeOfPoint > scan0.maxRange)
            scan0.maxRange = rangeOfPoint;
        if(rangeOfPoint < scan0.minRange)
            scan0.minRange = rangeOfPoint;
    }
    scan0.sectionLength = (scan0.maxRange - scan0.minRange) / sectionNum;
    for(int i = 0; i < sectionNum; i++)
    {
        scan0.sectionCountVector.push_back(0);
    }
//    cout<<"in Scan0 maxRange:  "<<scan0.maxRange
//       <<"  minRange:  "<<scan0.minRange
//       <<"  count:  "<<scan0.pointCount
//      <<"  sectionLength:  "<<scan0.sectionLength<<endl;

    ///Scan0
    //Count
    for(int i = 0; i < scan0.pointCount; i++)
    {
        int index = this->judgeBucket(scan0.rangeOfPointVector.at(i), scan0.minRange, scan0.sectionLength);
        scan0.sectionCountVector.at(index)++;
    }
    //Ratio
    for(int i = 0; i < sectionNum; i++)
    {
        scan0.sectionRatioVector.push_back((float)scan0.sectionCountVector.at(i) / (float)scan0.pointCount);
    }
//    //Check
//    float sumRatio = 0;
//    int sumCount = 0;
//    for(int i = 0; i < sectionNum; i++)
//    {
//        sumRatio += scan0.sectionRatioVector.at(i);
//        sumCount += scan0.sectionCountVector.at(i);
//    }
//    cout<<"sum0: "<<sumRatio<<"  "<<sumCount<<endl;

    scan0RingRatioVector.push_back(scan0.sectionRatioVector);
}

void Similarity::forScan1()
{
    scan1RingRatioVector.clear();

    //Process by Ring
    int ringRow = scan1All->getDescriptorStartingRow("ring");

    for(int i = ringStart; i <= ringEnd; i++)
    {
        scan1.pointCloud = scan1All->createSimilarEmpty();
        int count = 0;
        for(int j = 0; j < scan1All->features.cols(); j++)
        {
            if(scan1All->descriptors(ringRow, j) == i)
            {
                scan1.pointCloud.setColFrom(count, *scan1All, j);
                count++;
            }
        }
        scan1.pointCloud.conservativeResize(count);

        this->scan1Process();

    }

}

void Similarity::scan1Process()
{

    //Scan1 PreTreated
    scan1.pointCount = scan1.pointCloud.features.cols();
    scan1.maxRange = 0;
    scan1.minRange = 9999;
    for(int i = 0; i < scan1.pointCount; i++)
    {
        float rangeOfPoint = calculateRange(scan1.pointCloud.features.col(i).head(3));
        scan1.rangeOfPointVector.push_back(rangeOfPoint);
        if(rangeOfPoint > scan1.maxRange)
            scan1.maxRange = rangeOfPoint;
        if(rangeOfPoint < scan1.minRange)
            scan1.minRange = rangeOfPoint;
    }
    scan1.sectionLength = (scan1.maxRange - scan1.minRange) / sectionNum;
    for(int i = 0; i < sectionNum; i++)
    {
        scan1.sectionCountVector.push_back(0);
    }
//    cout<<"in Scan1 maxRange:  "<<scan1.maxRange
//       <<"  minRange:  "<<scan1.minRange
//        <<"  count:  "<<scan1.pointCount
//       <<"  sectionLength:  "<<scan1.sectionLength<<endl;

    ///Scan1
    //Count
    for(int i = 0; i < scan1.pointCount; i++)
    {
        int index = this->judgeBucket(scan1.rangeOfPointVector.at(i), scan1.minRange, scan1.sectionLength);
        scan1.sectionCountVector.at(index)++;
    }
    //Ratio
    for(int i = 0; i < sectionNum; i++)
    {
        scan1.sectionRatioVector.push_back((float)scan1.sectionCountVector.at(i) / (float)scan1.pointCount);
    }
//    //Check
//    sumRatio = 0;
//    sumCount = 0;
//    for(int i = 0; i < sectionNum; i++)
//    {
//        sumRatio += scan1.sectionRatioVector.at(i);
//        sumCount += scan1.sectionCountVector.at(i);
//    }
//    cout<<"sum1: "<<sumRatio<<"  "<<sumCount<<endl;

    scan1RingRatioVector.push_back(scan1.sectionRatioVector);

    if(isLog)
    {
        cout<<"Log two vectors"<<endl;
        ofstream recordVector0, recordVector1;
        stringstream recordName0, recordName1;

        //ugly code for test
        recordName0 << "/home/yh/0.txt";
        recordName1 << "/home/yh/1.txt";

        recordVector0.open(recordName0.str());
        recordVector1.open(recordName1.str());

        for(int i = 0; i < sectionNum; i++)
        {
            recordVector0 <<std::fixed<< scan0.sectionRatioVector.at(i) <<endl;
            recordVector1 <<std::fixed<< scan1.sectionRatioVector.at(i) <<endl;
        }

        recordVector0.close();
        recordVector1.close();
    }

}

void Similarity::getEMD1D()
{
    EMD = 0;
    for(int m = 0; m <= ringEnd - ringStart; m++)
    {
        for(int i = this->truncatedStart; i < sectionNum; i++)
        {
            for(int j = 0; j <= i; j++)
            {
                EMD += abs(scan0RingRatioVector[m][j] - scan1RingRatioVector[m][j]);
            }
        }
    }
    ///?
    EMD /= (sectionNum * (ringEnd - ringStart + 1));
    ///?
    cout<<"EMD Distance:  "<<EMD<<endl;

    //save in the EMD.txt
    if(1)
    {
        ofstream recordEMD;
        stringstream ssEMD;

        ssEMD <<"/home/yh/EMD1D.txt";
        recordEMD.open(ssEMD.str(), ios::app);
        recordEMD << std::fixed << EMD << endl;
    }
}

float Similarity::calculateRange(Eigen::Vector3f inputOrdinate)
{
    return pow(pow(inputOrdinate(0), 2) + pow(inputOrdinate(1), 2) + pow(inputOrdinate(2), 2), 0.5);
}

int Similarity::judgeBucket(float range, float minRange, float sectionLength)
{
    for(int i = 0; i < sectionNum; i++)
    {
        if(range >= minRange + i*sectionLength && range <= minRange + (i+1)*sectionLength)
            return i;
    }
    ///Problem exists
//    cout<<"Not in the Bucket!  Return the last Setcion!  Length:  "<<range<<endl;
    return sectionNum-1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserScan_Similarity_Rings_Continuous");
    ros::NodeHandle n;

    Similarity similarity(n);


    ros::spin();
    return 0;
}
