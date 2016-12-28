//
//  MassProperties.hpp
//  MissileSim
//
//  Created by Christian J Howard on 4/30/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//

#ifndef MassProperties_hpp
#define MassProperties_hpp

#include "math3d_define.hpp"

class MassProperties {
public:
    virtual void initialize()           = 0;
    virtual void update( double time )  = 0;
    
protected:
    friend class EquationsOfMotion;
    mat3 I, Iinv;
    double mass;
};


#endif /* MassProperties_hpp */
