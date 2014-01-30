#include <nbody/constants.h>
#include <nbody/System.h>
#include <nbody/Vector3.h>

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace nbody {

  inline void System::interactBodies( size_t i, size_t j, float softFactor, Vector3f &acc, Vector3f &jerk ) const {
      Vector3f r, v, r2, v_r2;
    r = _body[j].position() - _body[i].position();
    v = _body[j].velocity() - _body[i].velocity();
    float distance = r.norm() + softFactor;
    float invDist = 1.0f / distance;
    float invDistCubed = cube( invDist );
    r2 = r*r;
    v_r2 = v*r2;
    acc = acc + NEWTON_G * _body[j].mass() * invDistCubed * r;
    jerk = jerk + NEWTON_G* _body[j].mass() * invDistCubed * ( v - ( 3.0f * ((v_r2)*distance *invDistCubed))); // equation for jerk
  }

  void System::computeGravitation() {
    for( size_t i = 0; i < _nBodies; ++i ) {
      Vector3f acc{ 0.0f, 0.0f, 0.0f };
      Vector3f jerk{ 0.0f, 0.0f, 0.0f };
      for( size_t j = 0; j < _nBodies; ++j ) {
        if( i != j ) {
          interactBodies( i, j, _softFactor, acc, jerk );
        }
      }
      _body[i].force() = acc;
      _body[i].jerk() = jerk;
    }
  }

  void System::integrateSystem( float dt ) {
    Vector3f r, v, a, j;
    for( size_t i = 0; i < _nBodies; ++i ) {
      r = _body[i].position();
      v = _body[i].velocity();
      a = _body[i].force();
      j = _body[i].jerk();


      v = v + ( a * dt ) + j*(dt*dt)/2.0f;
      //v = v * _dampingFactor;
      r = r + v * dt + a*dt*dt/2.0f + j*dt*dt*dt/6.0f;

      _body[i].position() = r;
      _body[i].velocity() = v;
    }
  }

  void System::corrector( float dt){
      Vector3f v, old_v, r, old_r, a, old_a, j, old_j;
   for ( size_t i = 0; i < _nBodies ; ++i){
     //  Vector3f v, old_v, r, old_r, a, old_a, j, old_j;
       v = _body[i].velocity();
       old_v = _body[i].oldVelocity();
       r = _body[i].position();
       old_r = _body[i].oldPosition();
       a = _body[i].accel();
       old_a = _body[i].oldAccel();
       j = _body[i].jerk();
       old_j = _body[i].oldJerk();
       v = old_v + (old_a + a) * dt /2.0f + (old_j - j)*dt*dt/12.0f;
       r = old_r + (old_v + v) * dt /2.0f + (old_a - a)*dt*dt/12.0f;
       _body[i].position() = r;
       _body[i].velocity() = v;


   }


  }

  void System::update( float dt ) {
    computeGravitation();
    integrateSystem( dt );
    corrector( dt );
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
