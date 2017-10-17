#ifndef POSEGRAPH_H
#define POSEGRAPH_H
#include "slamBase.h"

class PoseGraph
{
    public:
      vector< FRAME > keyframes;    //keyframes
      bool AcceptKeyFrames()
      {
          boost::mutex::scoped_lock unlock(mMutexAccept);
          return mbAcceptKeyFrames;
      }

      void SetAcceptKeyFrames(bool flag)
      {
          if(flag==false)
          {
              boost::mutex::scoped_lock lock(mMutexAccept);
          }
          else
          {
          boost::mutex::scoped_lock unlock(mMutexAccept);
          }
          mbAcceptKeyFrames=flag;
      }
      bool tryInsertKeyFrame(FRAME&);
      void mainloop();
private:
      boost::mutex mMutexAccept;
      bool mbAcceptKeyFrames=true;
      SlamLinearSolver* linearSolver;
      g2o::OptimizationAlgorithmLevenberg* solver;
      SlamBlockSolver* blockSolver;
      g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
};
#endif
