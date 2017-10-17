#ifndef MAPPING_H
#define MAPPING_H

#include "slamBase.h"
#include "pose_graph.hpp"
class Mapping
{
public:
    Mapping(PoseGraph& graph)
        : posegraph(graph)
    {

        gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //分辨图可以在parameters.txt里调
        camera = getDefaultCamera();
        //viewerThread = std::make_shared<std::thread> ( std::bind( &Mapping::viewer, this ));
    }
    //typedef pcl::PointCloud<PointT> PointCloud;
    PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T);
    void viewer();
private:
       // std::shared_ptr<std::thread>	viewerThread = nullptr;
    double gridsize=0.01;
     CAMERA_INTRINSIC_PARAMETERS camera;
     PoseGraph&  posegraph;
     ParameterReader pd;
};
#endif
