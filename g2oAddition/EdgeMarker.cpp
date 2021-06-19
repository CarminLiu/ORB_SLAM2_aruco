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
// //     for(int i=0; i<4; i++)
// //     {
// //         points2[i]=Tc2m.map(points[i]);
// //     }
//     Eigen::Matrix<double, 3, 6> pm0;
//     pm0<< 0,         -p_c2m(2), p_c2m(1), -1, 0, 0,
//           p_c2m(2),  0,        -p_c2m(0), 0, -1, 0,
//           -p_c2m(1), p_c2m(0),  0,        0, 0, -1;
// //     Eigen::Matrix<double, 3, 6> pm1;
// //     pm1<< 0, -points2[1](2), points2[1](1), -1, 0, 0,
// //           points2[1](2), 0, -points2[1](0), 0, -1, 0,
// //           -points2[1](1), points2[1](0), 0, 0, 0, -1;
// //     Eigen::Matrix<double, 3, 6> pm2;
// //     pm2<< 0, -points2[2](2), points2[2](1), -1, 0, 0,
// //           points2[2](2), 0, -points2[2](0), 0, -1, 0,
// //           -points2[2](1), points2[2](0), 0, 0, 0, -1;
// //     Eigen::Matrix<double, 3, 6> pm3;
// //     pm3<< 0, -points2[3](2), points2[3](1), -1, 0, 0,
// //           points2[3](2), 0, -points2[3](0), 0, -1, 0,
// //           -points2[3](1), points2[3](0), 0, 0, 0, -1;

//     double x0=p_c2m(0), y0=p_c2m(1), z0=p_c2m(2);
//     double z0_2=z0*z0;
//     Eigen::Matrix<double,2,3> tmp0;
//     tmp0(0,0) = fx;
//     tmp0(0,1) = 0;
//     tmp0(0,2) = -x0/z0*fx;
//     tmp0(1,0) = 0;
//     tmp0(1,1) = fy;
//     tmp0(1,2) = -y0/z0*fy;
//     _jacobianOplusXi= -1./z0 * tmp0 * Tc2g.rotation().toRotationMatrix()*pm0;
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
// //     // 1
//     double x1=points2[1](0), y1=points2[1](1), z1=points2[1](2);
//     double z1_2=z1*z1;
//     Eigen::Matrix<double,2,3> tmp1;
//     tmp1(0,0) = fx;
//     tmp1(0,1) = 0;
//     tmp1(0,2) = -x1/z1*fx;
//     tmp1(1,0) = 0;
//     tmp1(1,1) = fy;
//     tmp1(1,2) = -y1/z1*fy;
//     _jacobianOplusXi.block(2,0,2,6) = -1./z1 * tmp1 * Tc2g.rotation().toRotationMatrix()*pm1;
//     _jacobianOplusXj(2,0) =  x1*y1/z1_2 *fx;
//     _jacobianOplusXj(2,1) = -(1+(x1*x1/z1_2)) *fx;
//     _jacobianOplusXj(2,2) = y1/z1 *fx;
//     _jacobianOplusXj(2,3) = -1./z1 *fx;
//     _jacobianOplusXj(2,4) = 0;
//     _jacobianOplusXj(2,5) = x1/z1_2 *fx;
//     _jacobianOplusXj(3,0) = (1+y1*y1/z1_2) *fy;
//     _jacobianOplusXj(3,1) = -x1*y1/z1_2 *fy;
//     _jacobianOplusXj(3,2) = -x1/z1 *fy;
//     _jacobianOplusXj(3,3) = 0;
//     _jacobianOplusXj(3,4) = -1./z1 *fy;
//     _jacobianOplusXj(3,5) = y1/z1_2 *fy;
//     // 2
//     double x2=points2[2](0), y2=points2[2](1), z2=points2[2](2);
//     double z2_2=z2*z2;
//     Eigen::Matrix<double,2,3> tmp2;
//     tmp2(0,0) = fx;
//     tmp2(0,1) = 0;
//     tmp2(0,2) = -x2/z2*fx;
//     tmp2(1,0) = 0;
//     tmp2(1,1) = fy;
//     tmp2(1,2) = -y2/z2*fy;
//     _jacobianOplusXi.block(4,0,2,6) = -1./z2 * tmp2 * Tc2g.rotation().toRotationMatrix()*pm2;
//     _jacobianOplusXj(4,0) =  x2*y2/z2_2 *fx;
//     _jacobianOplusXj(4,1) = -(1+(x2*x2/z2_2)) *fx;
//     _jacobianOplusXj(4,2) = y2/z2 *fx;
//     _jacobianOplusXj(4,3) = -1./z2 *fx;
//     _jacobianOplusXj(4,4) = 0;
//     _jacobianOplusXj(4,5) = x2/z2_2 *fx;
//     _jacobianOplusXj(5,0) = (1+y2*y2/z2_2) *fy;
//     _jacobianOplusXj(5,1) = -x2*y2/z2_2 *fy;
//     _jacobianOplusXj(5,2) = -x2/z2 *fy;
//     _jacobianOplusXj(5,3) = 0;
//     _jacobianOplusXj(5,4) = -1./z2 *fy;
//     _jacobianOplusXj(5,5) = y2/z2_2 *fy;
//     // 3
//     double x3=points2[3](0), y3=points2[3](1), z3=points2[3](2);
//     double z3_2=z3*z3;
//     Eigen::Matrix<double,2,3> tmp3;
//     tmp3(0,0) = fx;
//     tmp3(0,1) = 0;
//     tmp3(0,2) = -x3/z3*fx;
//     tmp3(1,0) = 0;
//     tmp3(1,1) = fy;
//     tmp3(1,2) = -y3/z3*fy;
//     _jacobianOplusXi.block(6,0,2,6) = -1./z3 * tmp3 * Tc2g.rotation().toRotationMatrix()*pm3;
//     _jacobianOplusXj(6,0) =  x3*y3/z3_2 *fx;
//     _jacobianOplusXj(6,1) = -(1+(x3*x3/z3_2)) *fx;
//     _jacobianOplusXj(6,2) = y3/z3 *fx;
//     _jacobianOplusXj(6,3) = -1./z3 *fx;
//     _jacobianOplusXj(6,4) = 0;
//     _jacobianOplusXj(6,5) = x3/z3_2 *fx;
//     _jacobianOplusXj(7,0) = (1+y3*y3/z3_2) *fy;
//     _jacobianOplusXj(7,1) = -x3*y3/z3_2 *fy;
//     _jacobianOplusXj(7,2) = -x3/z3 *fy;
//     _jacobianOplusXj(7,3) = 0;
//     _jacobianOplusXj(7,4) = -1./z3 *fy;
//     _jacobianOplusXj(7,5) = y3/z3_2 *fy;
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
