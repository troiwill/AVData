#include "bev2d_helpers.hpp"
#include <fstream>
using namespace Eigen;
using namespace pcl;
using namespace std;


void bev2d::createBEVimage(const PointCloud<PointXYZI>& kCloud, const vector<int>& kCloudIdx,
    const Vector2d& kCenterPt, const double kScanLenX, const double kScanLenY,
    const int kPixHeight, const int kPixWidth, const double kBevRes, const bool useIntensity,
    vector<uint8_t>& bevImg)
{
    // Sanity checks.
    int x_idx = 0, y_idx = 0, bev_idx = 0;
    double x_min = kCenterPt.x() - (kScanLenX / 2);
    double y_min = kCenterPt.y() - (kScanLenY / 2);
    const uint8_t kMaxPixelVal = 255, kOne = 1;
    for (const int idx : kCloudIdx)
    {
        const PointXYZI& kPt = kCloud[idx];
        x_idx = std::min(kPixHeight - static_cast<int>((kPt.x - x_min) / kBevRes), kPixHeight - 1);
        y_idx = std::min(kPixWidth - static_cast<int>((kPt.y - y_min) / kBevRes), kPixWidth - 1);

        bev_idx = (y_idx * kPixWidth) + x_idx;
        if (useIntensity)
        {
	    uint8_t prevInten = bevImg[bev_idx];
	    uint8_t newInten = kPt.intensity > prevInten ? kPt.intensity : prevInten;
            bevImg[bev_idx] = std::min(static_cast<uint8_t>(255), newInten);
        }
        else
        {
            if (bevImg[bev_idx] < kMaxPixelVal)
                bevImg[bev_idx] += kOne;
        }
    }
}

void bev2d::filterPointsXY(const PointCloud<PointXYZI>::Ptr& kCloud, const Vector2d& kMinPt,
    const Vector2d& kMaxPt, vector<int>& filteredPoints)
{
    // Create the PassThrough filter to get the points of interest.
    PassThrough<PointXYZI> ptfilter;
    ptfilter.setInputCloud(kCloud);

    // Filter along the x-axis.
    vector<int> x_indices;
    ptfilter.setFilterFieldName("x");
    ptfilter.setFilterLimits(kMinPt.x(), kMaxPt.x());
    ptfilter.filter(x_indices);

    // Filter along the y-axis and place the indices in the output vector.
    IndicesPtr xptr(new vector<int>(x_indices));
    // for (const int& xval : x_indices) xptr->push_back(xval);
    ptfilter.setIndices(xptr);
    ptfilter.setFilterFieldName("y");
    ptfilter.setFilterLimits(kMinPt.y(), kMaxPt.y());
    ptfilter.filter(filteredPoints);
}

void bev2d::readXYZIBin(const std::string& kBinPath, pcl::PointCloud<PointXYZI>& cloud)
{
    // Load the point cloud binary file.
    std::ifstream binfile(kBinPath.c_str(), ios::in | ios::binary);
    if (!binfile.good())
    {
        cerr << "Cannot load binary file: " << kBinPath << endl;
        exit(EXIT_FAILURE);
    }

    // Iteratively load all the points from the binary file.
    binfile.seekg(0, ios::beg);
    for (std::size_t i = 0; binfile.good() && !binfile.eof(); i++)
    {
        pcl::PointXYZI pt;
        binfile.read((char*) &pt.x, 3 * sizeof(float));
        binfile.read((char*) &pt.intensity, sizeof(float));
        cloud.push_back(pt);
    }
    binfile.close();
}

void bev2d::writeBEVimage(const string& kBevFilepath, const int kPixHeight,
    const int kPixWidth, const vector<uint8_t>& kBevImg)
{
    // Open the file.
    fstream bevfile(kBevFilepath, ios::out | ios::binary);
    if (!bevfile.good())
    {
        cerr << "Cannot open or create PCD binary file: " << kBevFilepath << endl;
        exit(EXIT_FAILURE);
    }
    bevfile << "P5\n" << kPixWidth << " " << kPixHeight << "\n255\n";
    for (const uint8_t& kVal : kBevImg)
        bevfile << kVal;
    bevfile.close();
}

