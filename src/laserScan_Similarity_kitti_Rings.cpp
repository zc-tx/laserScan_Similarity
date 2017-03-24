#include "ros/ros.h"
#include "ros/console.h"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include <dirent.h>
#include <fstream>
#include <vector>
#include <math.h>

#include <stdlib.h>

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

    void getEMDRings();
    scan processScan(scan scanInput);
    float calculateRange(float inputOrdinate);
    float calculateRangeIn2DPlane(Eigen::Vector2f inputOrdinate);
    float calculateRange(Eigen::Vector3f inputOrdinate);
    int judgeBucket(float range, float minRange, float sectionLength, int sectionNum);

//    void getRawScan(const sensor_msgs::PointCloud2& cloudMsgIn);
    DP filterByRange(DP* rawScan);

    DP readFromDir(string fileName);

    int getRingOfPoint(Eigen::Vector3f inputXYZ);

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

    bool first = true;

    float averageRange;
    const string baseDirKitti;
    const int ringsCount;
    const int upStart;
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
    baseDirKitti(getParam<string>("baseDirKitti", ".")),
    ringsCount(getParam<int>("ringsCount", 16)),
    upStart(getParam<int>("upStart", 0))
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

    scanPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("after_filtered_points", 2, true);

    //read from dir in kittti dataset, ONE BY ONE
    //rings added
    {
        //Holy Shit!
        ///FUCK READING FILE
        DIR *dir;
        struct dirent *ptr;
        vector<string> fileNameVector;

        if ((dir = opendir(baseDirKitti.c_str())) == NULL)
        {
            cout<<"OPEN DIR ERROR"<<endl;
            return;
        }

        while((ptr=readdir(dir)) != NULL)
        {
            if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
                continue;
            string temp = ptr->d_name;
            fileNameVector.push_back(temp.substr(0, temp.length()-4).c_str());
        }

        //Sort the fileName!
        sort(fileNameVector.begin(), fileNameVector.end());

        ///i, j, k
        for(int i = 0; i < fileNameVector.size(); i++)
        {
            cout<<"--------------------------------------"<<endl;

            string fileName = baseDirKitti +
                    fileNameVector.at(i) +
                    ".bin";

            cout<<"fileName:  "<<fileName<<endl;

            //process ring by ring
            DP rawScan = this->readFromDir(fileName);
            int ringRow = rawScan.getDescriptorStartingRow("ring");

            for(int j = this->upStart; j < this->ringsCount; j++)
            {
                DP tempScan = rawScan.createSimilarEmpty();
                int count = 0;
                for(int k = 0; k < rawScan.features.cols(); k++)
                {
                    if(rawScan.descriptors(ringRow, k) == j)
                    {
                        tempScan.setColFrom(count, rawScan, k);
                        count++;
                    }
                    else
                    {
                        continue;
                    }
                }
                tempScan.conservativeResize(count);

                //Regular processing
                /// first scan is not useful anymore
                if(i==0)
                {
                    scan0.pointCloud = tempScan;
                    scan0 = this->processScan(scan0);
                    rings0Vector.push_back(scan0);

                    scan1.pointCloud = tempScan;
                    scan1 = this->processScan(scan1);
                    rings1Vector.push_back(scan1);

                }
                else
                {
                    scan1.pointCloud = tempScan;
                    scan1 = this->processScan(scan1);
                    rings1Vector.push_back(scan1);
                }

                ///reset ???
                scan1.clear();
            }
            this->getEMDRings();

            //Pub
            scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(rawScan, "velodyne", ros::Time::now()));

            //Clear Scan1
            rings1Vector.clear();

        }


    }

    //recieve -> loop
    ///NO ROS BAG
//    scanPointCloudSub = n.subscribe("/velodyne_points", inputQueueSize, &Similarity::getRawScan, this);
//    scanPointCloudPub = n.advertise<sensor_msgs::PointCloud2>("after_filtered_points", 2, true);
}

/*
void Similarity::getRawScan(const sensor_msgs::PointCloud2 &cloudMsgIn)
{
    cout<<"---------------------------------------"<<endl;

    averageRange = 0;

    DP* rawScan = new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn));

    //Apply Voxel Filter
    if(this->isVoxelFilter)
    {
        voxelFilters.apply(*rawScan);
    }

    if(first)
    {
        scan0.pointCloud = this->filterByRange(rawScan);

        scan0 = this->processScan(scan0);

        scan1.pointCloud = this->filterByRange(rawScan);

        scan1 = this->processScan(scan1);

        first = false;
    }
    else
    {
        scan1.pointCloud = this->filterByRange(rawScan);

        scan1 = this->processScan(scan1);
    }

    this->getEMD1D();

    //Pub
    scanPointCloudPub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scan1.pointCloud, "velodyne", ros::Time::now()));

    //Clear Scan1
    scan1.clear();
}
*/

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

//For kitti dataset
Similarity::DP Similarity::readFromDir(string fileName)
{
    DP tempScan;

    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (fileName.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;

    //ethz data structure
    tempScan.addFeature("x", PM::Matrix::Zero(1, num));
    tempScan.addFeature("y", PM::Matrix::Zero(1, num));
    tempScan.addFeature("z", PM::Matrix::Zero(1, num));
    tempScan.addDescriptor("intensity", PM::Matrix::Zero(1, num));
    tempScan.addDescriptor("ring", PM::Matrix::Zero(1, num));

    int x = tempScan.getFeatureStartingRow("x");
    int y = tempScan.getFeatureStartingRow("y");
    int z = tempScan.getFeatureStartingRow("z");
    int intensity = tempScan.getDescriptorStartingRow("intensity");
    int ring = tempScan.getDescriptorStartingRow("ring");

    int ringOfPt = 0;
    int ring_size = num / this->ringsCount;

    for (int32_t i = 0; i < num; i++)
    {
        if (i != 0 && (i % ring_size) == 0)
        {
            ringOfPt++;
        }

        Eigen::Vector3f range;
        range(0) = *px;
        range(1) = *py;
        range(2) = *pz;

        //thx to https://github.com/robofit/but_velodyne_lib
        //filer by maxRange
        if (ringOfPt < this->ringsCount && range.norm() < this->maxRange)
        {
           tempScan.descriptors(ring, i) = ringOfPt;
           tempScan.descriptors(intensity,i) = *pr;
           tempScan.features(x,i) = *px;
           tempScan.features(y,i) = *py;
           tempScan.features(z,i) = *pz;
        }
        else
        {
            //nothing
        }

        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);

    ///free the ptr, crash or not
    {
        free(data);
    }

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

//            ///Thanks to Tang Li's advice
//            if(rangeOfPoint > scanInput.maxRange)
//                scanInput.maxRange = rangeOfPoint;
//            if(rangeOfPoint < scanInput.minRange)
//                scanInput.minRange = rangeOfPoint;

//            averageRange += rangeOfPoint;

        }

//        averageRange /= scanInput.pointCount;
//                cout<<"A:  "<<averageRange<<endl;

        scanInput.minRange = this->minRange;
        scanInput.maxRange = this->maxRange;
//        cout<<"maxRange:  "<<scanInput.maxRange<<"  minRange:  "<<scanInput.minRange<<endl;
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

//            if(intensityOfPoint > scanInput.maxIntensity)
//                scanInput.maxIntensity = intensityOfPoint;
//            if(intensityOfPoint < scanInput.minIntensity)
//                scanInput.minIntensity = intensityOfPoint;

        }

//        cout<<"maxIntensity:  "<<scanInput.maxIntensity<<"  minIntensity:  "<<scanInput.minIntensity<<endl;
        scanInput.minIntensity = this->minIntensity;
        scanInput.maxIntensity = this->maxIntensity;
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

void Similarity::getEMDRings()
{
    /*
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
    ///NO
    if(1)
    {
        ofstream recordEMDRange;
        stringstream ssEMDRange;

        ssEMDRange <<"/home/yh/temp/EMDRange.txt";
        recordEMDRange.open(ssEMDRange.str(), ios::app);
        recordEMDRange << std::fixed << EMDRange << endl;

        ofstream recordEMDIntensity;
        stringstream ssEMDIntensity;

        ssEMDIntensity <<"/home/yh/temp/EMDIntensity.txt";
        recordEMDIntensity.open(ssEMDIntensity.str(), ios::app);
        recordEMDIntensity << std::fixed << EMDIntensity << endl;
    }
    */

    //save the origin Bins data to draw the Similarity Matrix
    if(1)
    {
        ///record the range bins
        ofstream recordBinRange;
        stringstream ssBinRange;

        ssBinRange << "/home/yh/temp/BinRangeRings.txt";
        recordBinRange.open(ssBinRange.str(), ios::app);

        for(int r = this->upStart; r < this->ringsCount; r++)
        {
            int at = r - upStart;

            for(int i = 0; i < sectionNumOfR; i++)
            {
                if(i < sectionNumOfR - 1)
                    recordBinRange << std::fixed << rings1Vector.at(at).sectionRatioVector.at(i)<<" ";
                else
                    recordBinRange << std::fixed << rings1Vector.at(at).sectionRatioVector.at(i);
            }

            recordBinRange <<endl;
        }

        ///record the intensity bins
        ofstream recordBinIntensity;
        stringstream ssBinIntensity;

        ssBinIntensity << "/home/yh/temp/BinIntensityRings.txt";
        recordBinIntensity.open(ssBinIntensity.str(), ios::app);

        for(int r = this->upStart; r < this->ringsCount; r++)
        {
            int at = r - upStart;

            for(int i = 0; i < sectionNumOfI; i++)
            {
                if(i < sectionNumOfI - 1)
                    recordBinIntensity << std::fixed << rings1Vector.at(at).sectionRatioVectorOfI.at(i)<<" ";
                else
                    recordBinIntensity << std::fixed << rings1Vector.at(at).sectionRatioVectorOfI.at(i);
            }

            recordBinIntensity <<endl;

        }

    }

    //save the averageRange, testing
    if(0)
    {
        ofstream recordAverageRange;
        stringstream ssAverageRange;

        ssAverageRange << "/home/yh/temp/averageRange.txt";
        recordAverageRange.open(ssAverageRange.str(), ios::app);

        recordAverageRange << std::fixed << this->averageRange;

        recordAverageRange <<endl;
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

int Similarity::getRingOfPoint(Eigen::Vector3f inputXYZ)
{
//    float range = this->calculateRange(inputXYZ);

    float range = inputXYZ.norm();

    float theta = asin(inputXYZ(2) / range );

    theta = 180 / M_PI * theta;


    ///TODO: WRONG CALCULATATION OF RING ???
    if(1)
    {
        ofstream recordRing;
        stringstream ssRing;

        ssRing << "/home/yh/temp/ring.txt";
        recordRing.open(ssRing.str(), ios::app);

        recordRing << std::fixed << theta;

        recordRing <<endl;
    }

    return 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserScan_Similarity_kitti_Rings");
    ros::NodeHandle n;

    Similarity similarity(n);

    ros::spin();
    return 0;
}
