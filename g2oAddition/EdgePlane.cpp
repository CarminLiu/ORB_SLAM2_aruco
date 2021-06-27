/**
 * Add by liujiamin
 * TODO: in g2o
 */

#include "EdgePlane.h"

namespace g2o
{

bool EdgePlaneMarker::read(std::istream& is)
{
    Vector4D v;
    is >> v(0) >> v(1) >> v(2) >> v(3);
    setMeasurement(Plane3D(v));
    for (int i = 0; i < information().rows(); ++i)
        for (int j = i; j < information().cols(); ++j) {
            is >> information()(i, j);
            if (i != j)
                information()(j, i) = information()(i, j);
        }
    return true;
}

bool EdgePlaneMarker::write(std::ostream& os) const
{
    Vector4D v = _measurement.toVector();
    os << v(0) << " " << v(1) << " " << v(2) << " " << v(3) << " ";
    for (int i = 0; i < information().rows(); ++i)
        for (int j = i; j < information().cols(); ++j)
            os << " " << information()(i, j);
    return os.good();
}

bool EdgePlanePoint::read(std::istream& is)
{
    for (int i=0; i<3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
            information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgePlanePoint::write(std::ostream& os) const
{
    for (int i=0; i<3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
        }
    return os.good();
}
    
} // namespace g2o
