#ifndef _NBODY_BODY_H
#define _NBODY_BODY_H

#include <nbody/Vector3.h>

#include <iosfwd>

namespace nbody {

  class Body {
    Vector3f _position;
    Vector3f _oldPosition;

    Vector3f _velocity;
    Vector3f _oldVelocity;
    Vector3f _accel;
    Vector3f _oldAccel;
    Vector3f _jerk;
    Vector3f _oldJerk;
    Vector3f _force;
    float _mass;
  public:
    Body() : _position{}, _oldPosition{}, _velocity{}, _oldVelocity{}, _accel{}, _oldAccel{}, _jerk{}, _oldJerk{}, _force{}, _mass{} {}
    inline Vector3f position() const { return _position; }
    inline Vector3f& position() { return _position; }
    inline Vector3f oldPosition() const { return _oldPosition;}
    inline Vector3f& oldPositon() {return _oldPosition;}
    inline Vector3f velocity() const { return _velocity; }
    inline Vector3f& velocity() { return _velocity; }
    inline Vector3f oldVelocity() const { return _oldVelocity;}
    inline Vector3f& oldVelocity() { return _oldVelocity;}
    inline Vector3f accel() const { return _accel;}
    inline Vector3f& accel() {return _accel;}
    inline Vector3f oldAccel() const { return _oldAccel;}
    inline Vector3f& oldAccel() {return _oldAccel;}
    inline Vector3f jerk() const { return _jerk;}
    inline Vector3f& jerk() {return _jerk;}
    inline Vector3f oldJerk() const { return _oldJerk;}
    inline Vector3f& oldJerk() { return _oldJerk;}
    inline Vector3f force() const { return _force; }
    inline Vector3f& force() { return _force; }
    inline float mass() const { return _mass; }
    friend std::istream& operator>>( std::istream &is, Body &body );
    friend std::ostream& operator<<( std::ostream &os, const Body &body );
  };

} // namespace nbody

#endif // _NBODY_BODY_H
