/**
 * Add by liujiamin 
 * TODO: in g2o
 */

#ifndef ORB_SLAM2_EDGEPLANE_H
#define ORB_SLAM2_EDGEPLANE_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/stuff/misc.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "plane3d.h"
#include "vertex_plane.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace g2o
{

class EdgePlaneMarker: public g2o::BaseBinaryEdge<3, Plane3D, VertexPlane, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgePlaneMarker(){}

    virtual bool read(std::istream& is);

    virtual bool write(std::ostream& os) const;

    // virtual void linearizeOplus();

    void computeError() {
        //plane
        const VertexPlane* pl = static_cast<const VertexPlane*>(_vertices[0]);
        //marker
        const VertexSE3Expmap* Tg2m = static_cast<const VertexSE3Expmap*>(_vertices[1]);

        Plane3D plane = pl->estimate();
        Eigen::Matrix3d mR = Tg2m->estimate().rotation().toRotationMatrix();
        Eigen::Vector3d mt = Tg2m->estimate().translation();
        Vector3D mNormal(mR(0,2), mR(1,2), mR(2,2));
        double dis = mNormal.dot(mt);
        Plane3D MarkerPlane(Vector4D(mR(0,2), mR(1,2), mR(2,2),-dis));
        _error = plane.ominus(MarkerPlane);
    }

    // g2o::Vector3D point;
    // g2o::Vector3D pInPlane;

};

class EdgePlanePoint: public g2o::BaseBinaryEdge<3, Vector3D, VertexPlane, VertexSBAPointXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgePlanePoint(){}

    virtual bool read(std::istream& is);

    virtual bool write(std::ostream& os) const;

    void computeError() {
        const VertexPlane* pl = static_cast<const VertexPlane*>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        
        g2o::Vector3D point = v2->estimate();
        g2o::Vector3D pNor = pl->estimate().normal();
        double pDis = pl->estimate().distance();
        double t = pNor.dot(point) - pDis;
        pInPlane[0] = point[0] - pNor[0]*t;
        pInPlane[1] = point[1] - pNor[1]*t;
        pInPlane[2] = point[2] - pNor[2]*t;
        
        _error = point - pInPlane;
    }
    g2o::Vector3D pInPlane;
};

// class EdgePlaneParallel: public g2o::BaseBinaryEdge<2, Plane3D, VertexPlane, VertexPlane>
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

//     EdgePlaneParallel(){}

//     virtual bool read(std::istream& is);

//     virtual bool write(std::ostream& os) const;

//     void computeError() {

//     }

// };

} // namespace g2o


#endif //ORB_SLAM2_EDGEMARKER_H