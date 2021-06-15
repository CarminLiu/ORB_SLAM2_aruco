/**
 * Add by liujiamin
 * TODO: in g2o
 */

#include "EdgePlane.h"

namespace g2o
{
bool EdgePlaneMarker::read(std::istream& is)
{
    is >> _measurement >> information()(0,0);
    return true;
}

bool EdgePlaneMarker::write(std::ostream& os) const
  {
    os << measurement() << " " << information()(0,0);
    return os.good();
  }
    
} // namespace g2o
