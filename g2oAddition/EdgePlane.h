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
        //marker
        const VertexSE3Expmap* g2m = static_cast<const VertexSE3Expmap*>(_vertices[1]);

        point = g2m->estimate().map(point);
        // pl->estimate().coeffs();
        g2o::Vector3D pNor = pl->estimate().normal();
        double pDis = pl->estimate().distance();
        g2o::Vector3D pPL;
        pPL[0] = pInPlane[0];
        pPL[1] = pInPlane[1];
        pPL[2] = -(pPL[0]*pNor[0]+pPL[1]*pNor[1]-pDis)/pNor[2];

        g2o::Vector3D p = point - pPL;
        
        _error(0) = p.dot( pl->estimate().normal() ); 
    }

    g2o::Vector3D point;
    g2o::Vector3D pInPlane;

};

class EdgePlanePoint: public g2o::BaseBinaryEdge<1, double, VertexPlane, VertexSBAPointXYZ>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgePlanePoint(){}

    virtual bool read(std::istream& is);

    virtual bool write(std::ostream& os) const;

    void computeError() {
        const VertexPlane* pl = static_cast<const VertexPlane*>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[1]);
        
        g2o::Vector3D pNor = pl->estimate().normal();
        double pDis = pl->estimate().distance();
        g2o::Vector3D pPL;
        pPL[0] = pInPlane[0];
        pPL[1] = pInPlane[1];
        pPL[2] = -(pPL[0]*pNor[0]+pPL[1]*pNor[1]+pDis)/pNor[2];
        
        g2o::Vector3D p = v2->estimate() - pPL;
        _error(0) = p.dot( pl->estimate().normal() );
    }
    g2o::Vector3D pInPlane;
};

} // namespace g2o


#endif //ORB_SLAM2_EDGEMARKER_H