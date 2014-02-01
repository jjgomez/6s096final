#include <nbody/constants.h>
#include <nbody/System.h>
#include <nbody/Vector3.h>

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace nbody {

  inline void System::interactBodies( size_t i, size_t j, float softFactor, Vector3f &acc ) const {
    Vector3f r = _body[j].position() - _body[i].position();
    float distance = r.norm() + softFactor;
    float invDist = 1.0f / distance;
    float invDistCubed = cube( invDist );
    //printf("inv = %f, inv3 = %f, dist = %f\n", invDist, invDistCubed, distance);
    //printf("G = %f\n", NEWTON_G);
    acc = acc + NEWTON_G * _body[j].mass() * invDistCubed * r;
   // printf("X: pos = %f, accel = %f\n", r.x(), acc.x());
   // printf("Y: pos = %f, accel = %f\n", r.y(), acc.y());
    //printf("Z: pos = %f, accel = %f\n", r.z(), acc.z());
  }

  void System::computeGravitation() {
    for( size_t i = 0; i < _nBodies; ++i ) {
      Vector3f acc{ 0.0f, 0.0f, 0.0f };
      for( size_t j = 0; j < _nBodies; ++j ) {
        if( i != j ) {
          interactBodies( i, j, _softFactor, acc );
        }
      }
      _body[i].force() = acc;
    }
  }

  void System::integrateSystem( float dt ) {
    Vector3f r, v, a;
    for( size_t i = 0; i < _nBodies; ++i ) {
      r = _body[i].position();
      v = _body[i].velocity();
      a = _body[i].force();

      v = v + ( a * dt );
      v = v * _dampingFactor;
      r = r + v * dt + a * dt *dt / 2.0f;

      _body[i].position() = r;
      _body[i].velocity() = v;
    }
  }

  void System::average(){
      Vector3f old_r, r, old_v, v;
      for( size_t i = 0; i < _nBodies; ++i ){

           old_r = _body[i].oldPosition();
           r = _body[i].position();
           old_v = _body[i].oldVelocity();
           v = _body[i].velocity();
       //    printf("X: oldP = %g, P = %g, oldV = %g, V = %g\n", old_r.x(), r.x(), old_v.x(), v.x());

           r = (r+old_r)/2.0f;
           v = (v+old_v)/2.0f;

           _body[i].position() = r;
           _body[i].velocity() = v;
        //   printf("X: average pos = %g average vel = %g\n", r.x(),v.x());

       }



  }

  void System::update( float dt ) {
    computeGravitation();
    integrateSystem( dt );
    Vector3f r, v;
    for( size_t i = 0; i < _nBodies; ++i ){
       // printf("start of for\n");
       // printf("X: pos = %g, vel = %g \n", _body[i].position().x(), _body[i].velocity().x());
        r = _body[i].position();
        v = _body[i].velocity();
        _body[i].oldPosition() = r;
        _body[i].oldVelocity() = v;
       // printf("X old: pos = %g, vel = %g \n", _body[i].oldPosition().x(), _body[i].oldVelocity().x());
      //  printf("Y old: pos = %g \n", _body[i].oldPosition().y());
       // printf("Z old: pos = %g \n", _body[i].oldPosition().z());
       // printf("end of for\n");
    }
    integrateSystem( dt );
    average();
  }

  void System::readState( std::istream &input ) {
    input >> _nBodies;
    if( _nBodies > MAX_BODIES_RECOMMENDED ) {
      throw std::runtime_error( "Too many input bodies" );
    }
    _body = new Body[_nBodies];
    for( size_t i = 0; i < _nBodies; ++i ) {
      input >> _body[i];
    }
  }

  void System::writeState( std::ostream &output ) const {
    output << _nBodies << "\n";
    for( size_t i = 0; i < _nBodies; ++i ) {
      output << _body[i] << "\n";
    }
  }

} // namespace nbody
