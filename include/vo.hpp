#include "slamBase.h"
namespace zmz
{//声明命名空间ns1
    class VisualOdometry {
    public:
    VisualOdometry();
    RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

      VisualOdometryStereo::parameters param;
    };
}
