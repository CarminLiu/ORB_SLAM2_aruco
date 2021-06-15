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

class EdgePlaneMarker: public g2o::BaseBinaryEdge<1, double, VertexPlane, VertexSE3Expmap>
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
        //camera
        const VertexSE3Expmap* c2g = static_cast<const VertexSE3Expmap*>(_vertices[1]);

        
        // auto T_c2m = c2g->estimate() * g2m->estimate();
        // g2o::Vector3D p_c2m = T_c2m.map(point);
        // Vector2d obs(_measurement);
        // double projx=( p_c2m(0)/p_c2m(2)) *fx +cx;
        // double projy=( p_c2m(1)/p_c2m(2)) *fy +cy;
        // _error(0) = obs(0) - projx;
        // _error(1) = obs(1) - projy;
    }

    g2o::Vector3D point;
    double fx, fy, cx, cy;

};

class EdgePlanePoint: public g2o::BaseBinaryEdge<1, double, Plane3D, VertexSBAPointXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgePlanePoint(){}

    virtual bool read(std::istream& is);

    virtual bool write(std::ostream& os) const;

};

} // namespace g2o


#endif //ORB_SLAM2_EDGEMARKER_H