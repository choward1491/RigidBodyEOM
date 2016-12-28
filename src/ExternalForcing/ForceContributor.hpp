//
//  ForceContributor.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef ForceContributor_h
#define ForceContributor_h

#include "math3d_define.hpp"
class EquationsOfMotion;

class ForceContributor {
public:
    ForceContributor();
    void setEOM( EquationsOfMotion* refb_ );
    virtual void getForce( double time, vec3 & outForceBody ) = 0;
    virtual void getLocation( double time, vec3 & locBody ) = 0;
private:
    EquationsOfMotion* ref_body;
};

#endif /* ForceContributor_h */
