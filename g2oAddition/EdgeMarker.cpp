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

// void EdgeMarker::linearizeOplus()
// {
//     VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
//     VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//     SE3Quat Tc2g(vj->estimate());
//     SE3Quat Tg2m(vi->estimate());

//     auto Tc2m = Tc2g * Tg2m;
//     g2o::Vector3D p_c2m = Tc2m.map(point);
//     g2o::Vector3D p_w = Tg2m.map(point);

//     Eigen::Matrix<double, 3, 6> pm;
//     pm<< 0,       p_w(2), -p_w(1), 1, 0, 0,
//         -p_w(2),  0,       p_w(0), 0, 1, 0,
//         p_w(1),  -p_w(0),   0,     0, 0, 1;


//     double x0=p_c2m(0), y0=p_c2m(1), z0=p_c2m(2);
//     double z0_2=z0*z0;
//     Eigen::Matrix<double,2,3> tmp0;
//     tmp0(0,0) = fx;
//     tmp0(0,1) = 0;
//     tmp0(0,2) = -x0/z0*fx;
//     tmp0(1,0) = 0;
//     tmp0(1,1) = fy;
//     tmp0(1,2) = -y0/z0*fy;
//     _jacobianOplusXi= -1./z0 * tmp0 * Tc2g.rotation().toRotationMatrix()*pm;
//     // 0
//     _jacobianOplusXj(0,0) =  x0*y0/z0_2 *fx;
//     _jacobianOplusXj(0,1) = -(1+(x0*x0/z0_2)) *fx;
//     _jacobianOplusXj(0,2) = y0/z0 *fx;
//     _jacobianOplusXj(0,3) = -1./z0 *fx;
//     _jacobianOplusXj(0,4) = 0;
//     _jacobianOplusXj(0,5) = x0/z0_2 *fx;
//     _jacobianOplusXj(1,0) = (1+y0*y0/z0_2) *fy;
//     _jacobianOplusXj(1,1) = -x0*y0/z0_2 *fy;
//     _jacobianOplusXj(1,2) = -x0/z0 *fy;
//     _jacobianOplusXj(1,3) = 0;
//     _jacobianOplusXj(1,4) = -1./z0 *fy;
//     _jacobianOplusXj(1,5) = y0/z0_2 *fy;

// }

bool EdgeMapPointInAruco::read(std::istream& is)
{
    is >> _measurement;
    
    is >> information()(0,0);
}

bool EdgeMapPointInAruco::write(std::ostream& os) const
{
    os << measurement();
    os << " " << information()(0,0);
    return os.good();
}
    
} // namespace g2o
