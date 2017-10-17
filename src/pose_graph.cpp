#include "pose_graph.hpp"
bool PoseGraph::tryInsertKeyFrame(FRAME& newframe)
{
        if ( keyframes.size() == 0 )
         {
            cout<<BLUE"initialing g2o"<<endl;
            // 新增:有关g2o的初始化
            //***************************
            // 初始化求解器
            linearSolver = new SlamLinearSolver();
            linearSolver->setBlockOrdering( false );
            blockSolver = new SlamBlockSolver( linearSolver );
            solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

            globalOptimizer.setAlgorithm( solver );
            // 不要输出调试信息
            globalOptimizer.setVerbose( false );

            // 向globalOptimizer增加第一个顶点
            g2o::VertexSE3* v = new g2o::VertexSE3();

            v->setId(0);
            v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
            v->setFixed( true ); //第一个顶点固定，不用优化
            globalOptimizer.addVertex( v );
            cout<<BLUE"vertices: "<<globalOptimizer.vertices().size()<<endl;
             keyframes.push_back( newframe );
             return true;
        }
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( newframe.frameID );
        globalOptimizer.addVertex(v);

        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->setVertex( 0, globalOptimizer.vertex(keyframes.back().frameID ));
        edge->setVertex( 1, globalOptimizer.vertex(newframe.frameID ));
        cout<<RED"last id : "<<keyframes.back().frameID<<"   ,this id : "<<newframe.frameID<<endl;
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        Eigen::Isometry3d T = newframe.T_lastKF;


 ///////////////
         edge->setMeasurement( T );
       // edge->setMeasurement( T.inverse() );

///////////////////////////

        // 将此边加入图中
        globalOptimizer.addEdge(edge);
       cout<<BLUE"vertices: "<<globalOptimizer.vertices().size()<<endl;
        keyframes.push_back( newframe );
        return true;

}
void PoseGraph::mainloop()
{

    // 优化
    cout<<BLUE"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./result_after.g2o" );
    cout<<BLUE"Optimization done."<<endl;
    for (size_t i=0; i<keyframes.size(); i++)
    {
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //该帧优化后的位姿


        keyframes[i].T_global=pose;
    }

}
