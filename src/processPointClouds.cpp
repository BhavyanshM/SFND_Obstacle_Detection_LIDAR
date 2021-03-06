// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set> 
 

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	pcl::VoxelGrid<PointT> vg;
  	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
  	vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices) inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  	typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
  
  	
  
  	for(int index: inliers->indices){
    	planeCloud->points.push_back(cloud->points[index]);
    }
  
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(cloud);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*obstacleCloud);
  
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

  	pcl::SACSegmentation<PointT> segmenter;
    pcl::PointIndices::Ptr  inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    segmenter.setOptimizeCoefficients(true);
    segmenter.setModelType(pcl::SACMODEL_PLANE);
    segmenter.setMethodType(pcl::SAC_RANSAC);
    segmenter.setMaxIterations(maxIterations);
    segmenter.setDistanceThreshold(distanceThreshold);

    segmenter.setInputCloud(cloud);
    segmenter.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    auto cloudPoints = cloud->points;
    
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for(int i = 0; i<maxIterations; i++){
    
        std::unordered_set<int> inliers;
            // Randomly sample subset and fit line
        int a = rand() % cloud->points.size();
        int b = rand() % cloud->points.size();
        int c = rand() % cloud->points.size();
        inliers.insert(a);
        inliers.insert(b);
        inliers.insert(c);
        PointT pa = cloud->points[a];
        PointT pb = cloud->points[b];
        PointT pc = cloud->points[c];
        
        double x1 = pa.x,x2 = pb.x,x3 = pc.x;
        double y1 = pa.y,y2 = pb.y,y3 = pc.y;
        double z1 = pa.z,z2 = pb.z,z3 = pc.z;
      
        double A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        double B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        double C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        double D = -(A*x1 + B*y1 + C*z1);
        
        // Measure distance between every point and fitted line
        for(int j = 0; j<cloud->points.size(); j++){
          
            if(inliers.count(j)>0)continue;
            PointT p = cloud->points[j];
            double d = fabs(A*p.x + B*p.y + C*p.z + D)/sqrt(A*A + B*B + C*C);
            if(d<distanceThreshold)inliers.insert(j);
        }
      
        if(inliers.size()>inliersResult.size())inliersResult = inliers;      
      
    }


    if(inliersResult.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outlierCloud(new pcl::PointCloud<PointT>());
    for (int i = 0; i<cloud->points.size();i++){
        PointT point = cloud->points[i];
        if (inliersResult.count(i) == 0){
            outlierCloud->points.push_back(point);
        } else {
            inlierCloud->points.push_back(point);
        }
    }

    std::cout << "POINTS: " << cloud->points.size() << " INDICES: " << inliersResult.size() << " INLIERS: " << inlierCloud->points.size() << " OUTLIERS: " << outlierCloud->points.size() << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(outlierCloud,inlierCloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol){
  
    processed[index] = true;
    cluster.push_back(index); 
  
    PointT cloudPoint = cloud->points[index];
    std::vector<float> point = {cloudPoint.x,cloudPoint.y,cloudPoint.z};
    std::vector<int> nearest = tree->search(point,distanceTol);
  
  
  
    for (int id: nearest){
        if(!processed[id]){
            Proximity(id, cloud, cluster, processed, tree, distanceTol);
        }
    }
    
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster
    
    std::vector<bool> processed(cloud->points.size(),false);
  
    std::vector<std::vector<int>> clusters;
    for (int i = 0; i<cloud->points.size(); i++){
        if(processed[i]){
            continue;
        }
      
        std::vector<int> cluster;
        Proximity(i,cloud,cluster,processed, tree, distanceTol);
        clusters.push_back(cluster);
        
    }
 
    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KdTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> pclClusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    KdTree* tree = new KdTree;

    for (int i=0; i<cloud->points.size(); i++){
        PointT cloudPoint = cloud->points[i];
        std::vector<float> point = {cloudPoint.x,cloudPoint.y,cloudPoint.z};
        tree->insert(point,i); 
    } 

    std::vector<std::vector<int>> clusters = euclideanCluster(cloud,tree,clusterTolerance);
  
    for(std::vector<int> cluster: clusters){
        
        if(cluster.size() > minSize && cluster.size() < maxSize){
             
              typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

              for(int index : cluster){
                    cloudCluster->points.push_back(cloud->points[index]);
              }

              cloudCluster->width = cloudCluster->points.size();
              cloudCluster->height = 1;
              cloudCluster->is_dense = true; 

              pclClusters.push_back(cloudCluster);

        }


    }
  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return pclClusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  	tree->setInputCloud(cloud);
  
  	std::vector<pcl::PointIndices> clusterIndices;
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
  
    for(pcl::PointIndices getIndices: clusterIndices){

      typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

      for(int index : getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
      }

      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true; 

      clusters.push_back(cloudCluster);

    }
  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}