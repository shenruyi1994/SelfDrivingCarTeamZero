#ifndef _sdcBoundingShape_hh
#define _sdcBoundingShape_hh

namespace gazebo {
  class sdcBoundingShape {
  public:
    virtual bool Intersects(const sdcBoundingShape* shape) const = 0;
  };
}

#endif
