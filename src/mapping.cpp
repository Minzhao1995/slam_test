#include "mapping.hpp"
PointCloud::Ptr Mapping::joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T )
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
}
void Mapping::viewer()
{
    ros::Rate loop_rate(1);
    ros::NodeHandle nh;

     ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
     pcl::PointCloud<pcl::PointXYZRGB> cloud_read_from_file;
     sensor_msgs::PointCloud2 ros_output;

     image_transport::ImageTransport it(nh);
     image_transport::Publisher apriltags_pub = it.advertise("camera/apriltags", 1);
     sensor_msgs::ImagePtr findtf;
     cout<<"mapping thread start"<<endl;
    while (ros::ok())
    {

        if(posegraph.keyframes.size()<5)
        {
            // SetAcceptKeyFrames(true);
               ros::spinOnce();
           // %EndTag(SPINONCE)%
               loop_rate.sleep();
               continue;
        }


        cout<<"keyframe size:"<<posegraph.keyframes.size()<<endl;
        posegraph.SetAcceptKeyFrames(false);
        posegraph.mainloop();


        octomap::OcTree tree( 0.05 ); //全局map

        // 拼接点云地图
        cout<<"saving the point cloud map..."<<endl;
        PointCloud::Ptr output ( new PointCloud() ); //全局地图
        PointCloud::Ptr tmp ( new PointCloud() );

        pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
        pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
        pass.setFilterFieldName("z");
        pass.setFilterLimits( 0.0, 20.0 ); //20m以上就不要了


        cout<<"gridsize=  "<<gridsize<<endl;
        voxel.setLeafSize( gridsize, gridsize, gridsize );

        for (size_t i=0; i<posegraph.keyframes.size(); i++)//i++)
        {
            // 从g2o里取出一帧
            //g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
            //Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿
            PointCloud::Ptr newCloud = image2PointCloud( posegraph.keyframes[i].rgb, posegraph.keyframes[i].depth, camera ); //转成点云
            // 以下是滤波
            voxel.setInputCloud( newCloud );
            voxel.filter( *tmp );
            pass.setInputCloud( tmp );
            pass.filter( *newCloud );
            // 把点云变换后加入全局地图中
            pcl::transformPointCloud( *newCloud, *tmp, posegraph.keyframes[i].T_global.matrix() );
             *output += *tmp;

            cout<<"octomap is going to add a new cloud :"<<i<<endl;
            //octomap
            octomap::Pointcloud cloud_octo;
             for (auto p:tmp->points)
              cloud_octo.push_back( p.x, p.y, p.z );

              tree.insertPointCloud( cloud_octo,
            octomap::point3d(posegraph.keyframes[i].T_global(0,3), posegraph.keyframes[i].T_global(1,3), posegraph.keyframes[i].T_global(2,3) ) );


             tmp->clear();
             newCloud->clear();
        }
        tree.updateInnerOccupancy();
        tree.write( "map.ot" );
        posegraph.SetAcceptKeyFrames(true);
        voxel.setInputCloud( output );
        voxel.filter( *tmp );
        //存储
        pcl::io::savePCDFile( "./result.pcd", *tmp );

        cout<<"Final map is saved."<<endl;

        pcl::io::loadPCDFile ("./result.pcd", cloud_read_from_file);
        //Convert the cloud to ROS message

        pcl::toROSMsg(cloud_read_from_file, ros_output);
        ros_output.header.frame_id = "zed";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
        pcl_pub.publish(ros_output);
        // SetAcceptKeyFrames(true);
           ros::spinOnce();
       // %EndTag(SPINONCE)%
           loop_rate.sleep();
    }

}
