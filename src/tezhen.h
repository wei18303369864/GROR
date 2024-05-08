#pragma once
#include <iostream>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/shot_omp.h>
#include "pcl/features/fpfh.h"
#include <pcl/io/ply_io.h>
#include<pcl/kdtree/impl/kdtree_flann.hpp>
#include<pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;
using namespace Eigen;

void tez(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	pcl::PointIndicesPtr tez_Idx);