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

    //Range
    float minRange;
    float maxRange;
    float sectionLength;
    vector<int> sectionCountVector;
    vector<float> sectionRatioVector;
    vector<float> rangeOfPointVector;

    //Intensity Added
    float minIntensity;
    float maxIntensity;
    float sectionLengthOfI;
    vector<int> sectionCountVectorOfI;
    vector<float> sectionRatioVectorOfI;
    vector<float> intensityOfPointVector;

    void clear();
private:

protected:

};

void scan::clear()
{
    pointCount = 0;

    //Range
    minRange = 9999;
    maxRange = 0;
    sectionCountVector.clear();
    sectionRatioVector.clear();
    rangeOfPointVector.clear();

    //Intensity
    minIntensity = 9999;
    maxIntensity = 0;
    sectionCountVectorOfI.clear();
    sectionRatioVectorOfI.clear();
    intensityOfPointVector.clear();
}

class Similarity
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
public:
    Similarity(ros::NodeHandle &n);
    ~Similarity();

    void getEMD1D();
    void getEMD1DRings();
    scan processScan(scan scanInput);
    float calculateRange(float inputOrdinate);
    float calculateRangeIn2DPlane(Eigen::Vector2f inputOrdinate);
    float calculateRange(Eigen::Vector3f inputOrdinate);
    int judgeBucket(float range, float minRange, float sectionLength, int sectionNum);

    void getRawScan(const sensor_msgs::PointCloud2& cloudMsgIn);
    DP filterByRange(DP* rawScan);

private:
    ros::Subscriber scanPointCloudSub;
    ros::Publisher scanPointCloudPub;
    PM::DataPointsFilters voxelFilters;

    scan scan0;
    scan scan1;

    vector<scan> rings0Vector;    //scan is ring, ring is scan :)
    vector<scan> rings1Vector;    //complex   :(

    ros::NodeHandle& n;
    const int sectionNumOfR;
    const int sectionNumOfI;
    const int inputQueueSize;
    const float rangeThreshold;
    const bool isVoxelFilter;
    const float minRange;
    const float maxRange;
    const float minIntensity;
    const float maxIntensity;
    const float durationSecond;

    bool first = true;
protected:

};

Similarity::Similarity(ros::NodeHandle& n):
    n(n),
    sectionNumOfR(getParam<int>("sectionNumOfR", 10)),
    inputQueueSize(getParam<int>("inputQueueSize", 1)),
    rangeThreshold(getParam<float>("rangeThreshold", 0)),
    isVoxelFilter(getParam<bool>("isVoxelFilter", false)),
    sectionNumOfI(getParam<int>("sectionNumOfI", 10)),
    minRange(getParam<float>("minRange", 0.0)),
    maxRange(getParam<float>("maxRange", 100.0)),
    minIntensity(getParam<float>("minIntensity", 0.0)),
    maxIntensity(getParam<float>("maxIntensity", 200.0)),
    durationSecond(getParam<float>("durationSecond", 1.0))
{
    //load filterConfig  --->  Voxel Grid Filter, thx to Lv
    string filterConfigName;

    if (ros::param::get("~voxelFilterConfig", filterConfigName))
    {
        ifstream ifs(filterConfigName.c_str());
        if (ifs.good())
        {
            voxelFilters = PM::DataPointsFilters(ifs);
        }
        else
        {
            ROS_ERROR_STREAM("Cannot load voxelFilters config from YAML file " << filterConfigName);
        }
    }
    else
    {
        ROS_INFO_STREAM("No voxelFilters config file given, not using these filters");
    }

    //recieve -> loop
    scanPointCloudSub = n.subscribe("/velodyne_points", inputQueueSize, &Similarity::getRawScan, this);
    scanPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("after_filtered_points", 2, true);
}

void Similarity::getRawScan(const sensor_msgs::PointCloud2 &cloudMsgIn)
{
    cout<<"---------------------------------------"<<endl;

    DP* rawScan = new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    //Apply Voxel Filter
    if(this->isVoxelFilter)
    {
        voxelFilters.apply(*rawScan);
    }

    //    if(first)
    //    {
    //        scan0.pointCloud = this->filterByRange(rawScan);

    //        scan0 = this->processScan(scan0);

    //        scan1.pointCloud = this->filterByRange(rawScan);

    //        scan1 = this->processScan(scan1);

    //        first = false;
    //    }
    //    else
    //    {
    //        scan1.pointCloud = this->filterByRange(rawScan);

    //        scan1 = this->processScan(scan1);
    //    }

    //    this->getEMD1D();

    //    //Pub
    //    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scan1.pointCloud, "velodyne", ros::Time::now()));

    //    //Clear Scan1
    //    scan1.clear();

    int ringRow = rawScan->getDescriptorStartingRow("ring");

    for(int i = 0; i < 16; i++)
    {
        //get the ring of rawScans
        DP tempScan = rawScan->createSimilarEmpty();
        int count = 0;
        for(int j = 0; j < rawScan->features.cols(); j++)
        {
            if(rawScan->descriptors(ringRow, j) == i)
            {
                tempScan.setColFrom(count, *rawScan, j);
                count++;
            }
            else
            {
                continue;
            }
        }
        tempScan.conservativeResize(count);

        //Regular processing
        if(first)
        {
            scan0.pointCloud = tempScan;
            scan0 = this->processScan(scan0);
            rings0Vector.push_back(scan0);

            scan1.pointCloud = tempScan;
            scan1 = this->processScan(scan1);
            rings1Vector.push_back(scan1);

            first = false;
        }
        else
        {
            scan1.pointCloud = tempScan;
            scan1 = this->processScan(scan1);
            rings1Vector.push_back(scan1);
        }

    }

    this->getEMD1DRings();

    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(rings1Vector.at(0).pointCloud, "velodyne", ros::Time::now()));

    scan1.clear();
    rings1Vector.clear();

    //Duartion
    ros::Duration(this->durationSecond).sleep();

}

Similarity::DP Similarity::filterByRange(DP *rawScan)
{
    DP tempScan;
    int count = 0;
    tempScan = rawScan->createSimilarEmpty();
    for(int i = 0; i < rawScan->features.cols(); i++)
    {
        float rangeOfPoint = calculateRangeIn2DPlane(rawScan->features.col(i).head(2));
        if(rangeOfPoint > this->rangeThreshold)
        {
            tempScan.setColFrom(count, *rawScan, i);
            count++;
        }
    }
    tempScan.conservativeResize(count);

    cout<<"Range filtered:  "<<rawScan->features.cols()-tempScan.features.cols()<<endl;

    return tempScan;
}

Similarity::~Similarity()
{

}

scan Similarity::processScan(scan scanInput)
{
    scanInput.pointCount = scanInput.pointCloud.features.cols();

    ///scanInput Processing by Range
    {
        scanInput.maxRange = -1;
        scanInput.minRange = 9999;
        for(int i = 0; i < scanInput.pointCount; i++)
        {
            float rangeOfPoint = calculateRange(scanInput.pointCloud.features.col(i).head(3));   //the range
//            float rangeOfPoint = calculateRange(scanInput.pointCloud.features(2, i)); //Z-axis range only
            scanInput.rangeOfPointVector.push_back(rangeOfPoint);

            ///Thanks to Tang Li's advice, ring?? #TODO
            if(rangeOfPoint > scanInput.maxRange)
                scanInput.maxRange = rangeOfPoint;
            if(rangeOfPoint < scanInput.minRange)
                scanInput.minRange = rangeOfPoint;

        }

//        scanInput.minRange = this->minRange;
//        scanInput.maxRange = this->maxRange;
        scanInput.sectionLength = (scanInput.maxRange - scanInput.minRange) / sectionNumOfR;

        for(int i = 0; i < sectionNumOfR; i++)
        {
            scanInput.sectionCountVector.push_back(0);
        }


        ///scanInput statistics
        //Count
        for(int i = 0; i < scanInput.pointCount; i++)
        {
            int index = this->judgeBucket(scanInput.rangeOfPointVector.at(i),
                                          scanInput.minRange,
                                          scanInput.sectionLength,
                                          this->sectionNumOfR);
            scanInput.sectionCountVector.at(index)++;
        }
        //Ratio
        for(int i = 0; i < sectionNumOfR; i++)
        {
            scanInput.sectionRatioVector.push_back((float)scanInput.sectionCountVector.at(i) / (float)scanInput.pointCount);
        }
    }

    ///scanInput Processing by Intensity
    {
        int intensityRow = scanInput.pointCloud.getDescriptorStartingRow("intensity");
        scanInput.maxIntensity = -1;
        scanInput.minIntensity = 9999;

        for(int i = 0; i < scanInput.pointCount; i++)
        {
            float intensityOfPoint = scanInput.pointCloud.descriptors(intensityRow, i);
            scanInput.intensityOfPointVector.push_back(intensityOfPoint);

            ///Thanks to Tang Li's advice, ring?? #TODO
            if(intensityOfPoint > scanInput.maxIntensity)
                scanInput.maxIntensity = intensityOfPoint;
            if(intensityOfPoint < scanInput.minIntensity)
                scanInput.minIntensity = intensityOfPoint;

        }

//        scanInput.minIntensity = this->minIntensity;
//        scanInput.maxIntensity = this->maxIntensity;
        scanInput.sectionLengthOfI = (scanInput.maxIntensity - scanInput.minIntensity) / sectionNumOfI;

        for(int i = 0; i < sectionNumOfI; i++)
        {
            scanInput.sectionCountVectorOfI.push_back(0);
        }

        ///scanInput statistics
        //Count
        for(int i = 0; i < scanInput.pointCount; i++)
        {
            int index = this->judgeBucket(scanInput.intensityOfPointVector.at(i),
                                          scanInput.minIntensity,
                                          scanInput.sectionLengthOfI,
                                          this->sectionNumOfI);
            scanInput.sectionCountVectorOfI.at(index)++;
        }
        //Ratio
        for(int i = 0; i < sectionNumOfI; i++)
        {
            scanInput.sectionRatioVectorOfI.push_back((float)scanInput.sectionCountVectorOfI.at(i) / (float)scanInput.pointCount);
        }
    }

    return scanInput;
}

void Similarity::getEMD1D()
{
    float EMDRange = 0;
    for(int i = 0; i < sectionNumOfR; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            EMDRange += abs(scan0.sectionRatioVector.at(j) - scan1.sectionRatioVector.at(j));
        }
    }
    EMDRange /= sectionNumOfR;
    cout<<"EMD Range Distance:  "<<EMDRange<<endl;

    float EMDIntensity = 0;
    for(int i = 0; i < sectionNumOfI; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            EMDIntensity += abs(scan0.sectionRatioVectorOfI.at(j) - scan1.sectionRatioVectorOfI.at(j));
        }
    }
    EMDIntensity /= sectionNumOfI;
    cout<<"EMD Intensity Distance:  "<<EMDIntensity<<endl;

    //save the 1D EMD Distance in the EMD.txt
    if(1)
    {
        ofstream recordEMDRange;
        stringstream ssEMDRange;

        ssEMDRange <<"/home/yh/EMDRange.txt";
        recordEMDRange.open(ssEMDRange.str(), ios::app);
        recordEMDRange << std::fixed << EMDRange << endl;

        ofstream recordEMDIntensity;
        stringstream ssEMDIntensity;

        ssEMDIntensity <<"/home/yh/EMDIntensity.txt";
        recordEMDIntensity.open(ssEMDIntensity.str(), ios::app);
        recordEMDIntensity << std::fixed << EMDIntensity << endl;
    }

    //save the origin data to draw the Similarity Matrix
    if(1)
    {
        ofstream recordBinRange;
        stringstream ssBinRange;

        ssBinRange << "/home/yh/BinRange.txt";
        recordBinRange.open(ssBinRange.str(), ios::app);

        for(int i = 0; i < sectionNumOfR; i++)
        {
            recordBinRange << std::fixed << scan1.sectionRatioVector.at(i)<<" ";
        }

        recordBinRange <<endl;

        ofstream recordBinIntensity;
        stringstream ssBinIntensity;

        ssBinIntensity << "/home/yh/BinIntensity.txt";
        recordBinIntensity.open(ssBinIntensity.str(), ios::app);

        for(int i = 0; i < sectionNumOfI; i++)
        {
            recordBinIntensity << std::fixed << scan1.sectionRatioVectorOfI.at(i)<<" ";
        }

        recordBinIntensity <<endl;

    }

}

void Similarity::getEMD1DRings()
{
    for(int i = 0; i < 16; i++)
    {
        //for cout the information of each ring
        cout<<"Ring:  "<<i<<"  Count:  "<<rings1Vector.at(i).pointCount
            <<"  minRange:  "<<rings1Vector.at(i).minRange
            <<"  maxRange:  "<<rings1Vector.at(i).maxRange
            <<"  minIntensity:  "<<rings1Vector.at(i).minIntensity
            <<"  maxIntensity:  "<<rings1Vector.at(i).maxIntensity<<endl;

    }
}

float Similarity::calculateRange(Eigen::Vector3f inputOrdinate)
{
    return pow(pow(inputOrdinate(0), 2) + pow(inputOrdinate(1), 2) + pow(inputOrdinate(2), 2), 0.5);
}

float Similarity::calculateRangeIn2DPlane(Eigen::Vector2f inputOrdinate)
{
    return pow(pow(inputOrdinate(0), 2) + pow(inputOrdinate(1), 2), 0.5);
}

float Similarity::calculateRange(float inputOrdinate)
{
    return inputOrdinate;
}

int Similarity::judgeBucket(float range, float minRange, float sectionLength, int sectionNum)
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
