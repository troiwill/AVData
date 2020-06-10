#include <boost/filesystem.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <mutex>
#include <signal.h>
#include <string>

using namespace std;
namespace boostfs = boost::filesystem;


long scanId = 0;
boostfs::path savedir;
std::mutex mtx;
ofstream ofs;
bool recvdmsg = false;
const string lidarScanDir("lidar_scan");

void signalShutdown(int sig)
{
    cout << "\nClosing info file and shutting down program.\n" << flush;
    ofs.close();
    ros::shutdown();
}

inline
void writeBinToStream(fstream& os, const pcl::PointXYZI& kPoint)
{
    float x = kPoint.x, y = kPoint.y, z = kPoint.z, intensity = kPoint.intensity;

    os.write((char*)&x, sizeof(float));
    os.write((char*)&y, sizeof(float));
    os.write((char*)&z, sizeof(float));
    os.write((char*)&intensity, sizeof(float));
}

void writePCDbin(const string& kPcdSavePath, const pcl::PointCloud<pcl::PointXYZI>& kCloud)
{
    // Open the file.
    fstream pcdf(kPcdSavePath, ios::out | ios::binary);
    if (!pcdf.good())
    {
        cerr << "Cannot open or create PCD binary file: " << kPcdSavePath << endl;
        exit(EXIT_FAILURE);
    }

    for (auto p_it = kCloud.begin(); p_it != kCloud.end(); ++p_it)
        writeBinToStream(pcdf, *p_it);
    pcdf.close();
}

string timestampToString(const std_msgs::Header& hdr)
{
    return to_string(hdr.stamp.sec) + string(".") + to_string(hdr.stamp.nsec);
}

void scanCallback(const geometry_msgs::PoseStampedConstPtr& gtPose,
    const sensor_msgs::PointCloud2ConstPtr& redScan,
    const sensor_msgs::PointCloud2ConstPtr& greenScan,
    const sensor_msgs::PointCloud2ConstPtr& blueScan,
    const sensor_msgs::PointCloud2ConstPtr& yellowScan)//,
//    const sensor_msgs::ImageConstPtr& img)
{
    // Extract the timestamp and ground truth pose information.
    string gtTime(timestampToString(gtPose->header));
    string redTime(timestampToString(redScan->header));
    string greenTime(timestampToString(greenScan->header));
    string blueTime(timestampToString(blueScan->header));
    string yellowTime(timestampToString(yellowScan->header));

    stringstream gtPoseStr;
    const geometry_msgs::Point& position = gtPose->pose.position;
    const geometry_msgs::Quaternion& orient = gtPose->pose.orientation;
    gtPoseStr << position.x << ',' << position.y << ',' << position.z << ',';
    gtPoseStr << orient.x << ',' << orient.y << ',' << orient.z << ',' << orient.w;

    mtx.lock();
    long currScanId = scanId++;
    string idStr(to_string(currScanId));
    idStr = string(7 - idStr.length(), '0') + idStr;
    ofs << idStr << ',' << gtPoseStr.str() << ',' << gtTime << ',' << redTime << ',' << greenTime
	<< ',' << blueTime << ',' << yellowTime << '\n';
    if (!recvdmsg)
    {
        cout << "Receiving messages!\n";
	recvdmsg = true;
    }
    mtx.unlock();

    string scanName = string("scan_") + idStr + string(".bin");
    //imgName = string("img_") + idStr + string(".png");

    // Combine all four scans into one point cloud.
    pcl::PointCloud<pcl::PointXYZI> cloud, tmpCloud;
    for (const auto& pc2scan: { redScan, greenScan, blueScan, yellowScan })
    {
	tmpCloud.clear();
	pcl::fromROSMsg(*pc2scan, tmpCloud);
	cloud += tmpCloud;
    }

    // Write the point cloud to disk. 
    writePCDbin((savedir / lidarScanDir / scanName).string(), cloud);
}

int main(int argc, char** argv)
{
    const boostfs::path saveRoot(argv[1]);
    savedir = saveRoot;
    const string nodeName("gtpose_lidar_listener");
    recvdmsg = false;

    cout << "Node parameters:\n  Save dir: " << savedir.string() << "\n  Node name: "
	<< nodeName << endl;

    // Create the save path directory if necessary.
    if (!boostfs::exists(savedir / lidarScanDir) && !boostfs::create_directories(savedir / lidarScanDir))
    {
        cerr << "Could not find save directory: " << (savedir / lidarScanDir).string() << endl;
        exit(EXIT_FAILURE);
    }

    // Init the ROS node.
    cout << "Init-ing ROS.\n" << flush;
    ros::init(argc, argv, nodeName.c_str(), ros::init_options::NoSigintHandler);
    cout << "Starting listener: " << nodeName << endl << endl << flush;

    // Create a ROS node and subscribe to a topic.
    cout << "Constructing the Node Handle..." << flush;
    ros::NodeHandle n("~");
    
    cout << "Done!\nConstructing subscribers..." << flush;
    const size_t gtPoseLen = 40, lidarLen = 40, policyQueueLen = 100;
    message_filters::Subscriber<geometry_msgs::PoseStamped> gtPoseSub(n, "/pose_ground_truth", gtPoseLen);
    message_filters::Subscriber<sensor_msgs::PointCloud2> redScanSub(n, "/lidar_red_pointcloud", lidarLen);
    message_filters::Subscriber<sensor_msgs::PointCloud2> blueScanSub(n, "/lidar_blue_pointcloud", lidarLen);
    message_filters::Subscriber<sensor_msgs::PointCloud2> greenScanSub(n, "/lidar_green_pointcloud", lidarLen);
    message_filters::Subscriber<sensor_msgs::PointCloud2> yellowScanSub(n, "/lidar_yellow_pointcloud", lidarLen);
    
    cout << "Done!\nConstructing ApproximateTime policy..." << flush;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
	sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
	sensor_msgs::PointCloud2> FordLiDARSyncPolicy;

    message_filters::Synchronizer<FordLiDARSyncPolicy> sync(FordLiDARSyncPolicy(policyQueueLen),
	gtPoseSub, redScanSub, blueScanSub, greenScanSub, yellowScanSub);
    sync.registerCallback(boost::bind(&scanCallback, _1, _2, _3, _4, _5));

    // Set up the signal handler.
    cout << "Done!\nSetting up signal handler..." << flush;
    signal(SIGINT, signalShutdown);

    // Create info file.
    const string infofile((savedir / "gtpose_lidar_info.csv").string());
    cout << "Done!\nInfo file: " << infofile << "\nWaiting for ROS sensor messages.\n" << flush;
    ofs.open(infofile, ofstream::out);
    ofs << "mid,px,py,pz,ox,oy,oz,ow,gttime,redtime,greentime,bluetime,yellowtime\n";

    // Spin until there are no more messages.
    ros::spin();
    cout << "Exiting " << nodeName << ".\n";

    exit(EXIT_SUCCESS);
}

