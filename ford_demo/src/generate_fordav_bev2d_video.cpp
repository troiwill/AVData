#include "bev2d_helpers.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace boostfs = boost::filesystem;
namespace boostpo = boost::program_options;

using namespace bev2d;
using namespace Eigen;
using namespace pcl;
using namespace std;

mutex mtx;
int scansDone = 0;
int nScansToDo = 0;

struct ScanInfo
{
    string scanName;
    union
    {
        double tq[7];
        double tx, ty, tz, qx, qy, qz, qw;
    };    
};

inline Matrix4d convertGTPoseToRotMat(const ScanInfo& kInfo)
{
    // Create the rotation matrix.
    Quaterniond q(kInfo.qw, kInfo.qx, kInfo.qy, kInfo.qz);
    Matrix4d rot4d = Matrix4d::Identity();
    rot4d.block<3,3>(0,0) = q.toRotationMatrix();
    return rot4d;
}

void extractQuaternionComps(const string& kScanLine, ScanInfo& info)
{
    // Split the scan line into tokens.
    vector<string> tokens;
    boost::split(tokens, kScanLine, boost::is_any_of(","));

    // Extract the scan ID from the scan line.
    info.scanName = "scan_" + tokens[0] + ".bin";

    // Extract the translation values.
    for (size_t i = 0; i < 7; i++)
        info.tq[i] = stod(tokens[i + 1]);
}

void createFordAvBevImages(queue<string>& scanQueue, const boostfs::path& kScanDir,
    const boostfs::path& kSaveDir, const double kBevRes, const double kScanLenX,
    const double kScanLenY, const bool useIntensity)
{
    cout << fixed;

    // Extract the front information from the queue.
    const int kPixHeight = static_cast<int>(kScanLenX / kBevRes);
    const int kPixWidth = static_cast<int>(kScanLenY / kBevRes);
    const double kHalfScanLenX = kScanLenX / 2.0;
    const double kHalfScanLenY = kScanLenY / 2.0;

    while (true)
    {
        // Get the next line if possible.
        string scanline;
        mtx.lock();
        if (scanQueue.empty())
        {
            mtx.unlock();
            return;
        }
        scanline = scanQueue.front();
        scanQueue.pop();
        cout << "Progress: " << setw(10) << setprecision(3) << (++scansDone * 100.0f / nScansToDo)
            << "%\r" << flush;
        mtx.unlock();

        // Extract the scan information from the line.
        ScanInfo info;
        extractQuaternionComps(scanline, info);

        // Read the scan binary from disk.
        PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);
        string scanpath((kScanDir / info.scanName).string());
        readXYZIBin(scanpath, *cloud);

        // Use a filter to remove points within a certain range.
        Vector2d min2d{-kHalfScanLenX, -kHalfScanLenY};
        Vector2d max2d{ kHalfScanLenX,  kHalfScanLenY};
        vector<int> filteredPoints;
        filterPointsXY(cloud, min2d, max2d, filteredPoints);

        // Create a BEV image from the point cloud.
        Vector2d cloudCenter{0.0,0.0};
        vector<uint8_t> scanBev(kPixHeight * kPixWidth, 0);
        createBEVimage(*cloud, filteredPoints, cloudCenter, kScanLenX, kScanLenY, kPixHeight,
            kPixWidth, kBevRes, useIntensity, scanBev);

        // Write the BEV image to disk.
        size_t extIdx = info.scanName.find_last_of('.');
        string bevfilename(info.scanName);
        bevfilename.replace(extIdx, string::npos, ".pgm");
        writeBEVimage((kSaveDir / bevfilename).string(), kPixHeight, kPixWidth, scanBev);
    }
}

int main(int argc, char** argv)
{
    // Parse the command-line arguments.
    boostfs::path infofiledir, savedir;
    double bevres = 0.0;
    size_t nworkers = 0;
    double scanlenx = 0.0, scanleny = 0.0;

    boostpo::options_description desc("Build Kitti 2D BEV Data Program Options");
    desc.add_options()
        ("help,h", "Print help message.")
        ("infodir", boostpo::value<boostfs::path>(&infofiledir)->required(),
            "Directory containing info file.")
        ("savedir", boostpo::value<boostfs::path>(&savedir)->required(),
            "Save directory.")
        ("bevres,b", boostpo::value<double>(&bevres)->required(),
            "BEV resolution (meters / pixel).")
        ("sx", boostpo::value<double>(&scanlenx)->required(),
            "Scan length X from sensor (in meters).")
        ("sy", boostpo::value<double>(&scanleny)->required(),
            "Scan length X from sensor (in meters).")
        ("nworkers,w", boostpo::value<size_t>(&nworkers)->required(),
            "Number of workers to create data.")
        ("intensity", boostpo::bool_switch()->default_value(false),
            "Use intensity values in BEV images?")
        ;
    boostpo::variables_map vm;
    boostpo::store(boostpo::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
        cout << desc << endl;
        exit(EXIT_SUCCESS);
    }
    boostpo::notify(vm);
    bool useIntensity = vm.count("intensity");

    // Sanity checks.
    if (bevres <= 0.0)
        throw runtime_error("'bevres' must be positive. Value given: " + to_string(bevres));
    
    if (scanlenx <= 5.0)
        throw runtime_error("'sx' must be > 5 meters. Value given: " + to_string(scanlenx));
    
    if (scanleny <= 5.0)
        throw runtime_error("'sy' must be > 5 meters. Value given: " + to_string(scanleny));
    
    if (nworkers < 1)
        throw runtime_error("'nworkers' must be positive int. Value given:" + to_string(nworkers));

    // Read in the information file for the LiDAR scans.
    queue<string> scanInfoQ;
    string infofile((infofiledir / "gtpose_lidar_info.csv").string());
    ifstream inf(infofile);
    string line;
    size_t lineIdx = -1;
    
    if (!inf.is_open()) throw runtime_error("Cannot open information file.");
    cout << "Reading file: " << infofile << endl;
    while (getline(inf, line))
    {
        // Skip the line with the header information.
        lineIdx++;
        if (lineIdx == 0) continue;
        if (line.empty()) continue;

        // Read the rest of the lines into memory.
        scanInfoQ.push(line);
    }
    cout << "Read " << lineIdx << " lines from info file.\n";

    // Run separate threads to create the BEV images.
    nScansToDo = scanInfoQ.size();
    thread* workers = new thread[nworkers];
    const boostfs::path kLidarScanDir(infofiledir / "lidar_scan");
    cout << "Creating threads for creating BEV images.\n" << flush;
    for (size_t i = 0; i < nworkers; i++)
    {
        workers[i] = thread(createFordAvBevImages, ref(scanInfoQ),
            ref(kLidarScanDir), ref(savedir), bevres, scanlenx, scanleny,
            useIntensity);
    }
    cout << "Waiting on threads to complete.\n" << flush;
    for (size_t i = 0; i < nworkers; i++) workers[i].join();
    cout << "Complete!\n";

    exit(EXIT_SUCCESS);
}
