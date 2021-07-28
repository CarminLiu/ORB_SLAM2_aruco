/**
 * Add by liujiamin 2021-01-03
 * TODO: in g2o
 */

#include "EdgeMarker.h"

namespace g2o
{
typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>    Vector8D;
bool EdgeMarker::read(std::istream& is)
{
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
        is >> information()(i,j);
        if (i!=j)
            information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeMarker::write(std::ostream& os) const
{
    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
        os << " " <<  information()(i,j);
        }
    return os.good();
}
    
} // namespace g2o
