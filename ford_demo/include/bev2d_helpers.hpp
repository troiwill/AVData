#pragma once

#include <Eigen/Core>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
using namespace Eigen;
using namespace pcl;
using namespace std;


namespace bev2d
{
    void createBEVimage(const PointCloud<PointXYZI>& kCloud, const vector<int>& kCloudIdx,
	const Vector2d& kCenterPt, const double kScanLenX, const double kScanLenY,
	const int kPixHeight, const int kPixWidth, const double kBevRes, const bool useIntensity,
	vector<uint8_t>& bevImg);

    void filterPointsXY(const PointCloud<PointXYZI>::Ptr& kCloud, const Vector2d& kMinPt,
	const Vector2d& kMaxPt, vector<int>& filteredPoints);

    void readXYZIBin(const string& kBinPath, PointCloud<PointXYZI>& cloud);

    void writeBEVimage(const string& kBevFilepath, const int kPixHeight, const int kPixWidth,
        const vector<uint8_t>& kBevImg);
}

