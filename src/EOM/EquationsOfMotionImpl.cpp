//
//  EOM.cpp
//  MissileSim
//
//  Created by Christian J Howard on 4/24/16.
//  Copyright © 2016 Christian Howard. All rights reserved.
//


#ifndef _EOM_IMPL_HPP_
#define _EOM_IMPL_HPP_

#include "EquationsOfMotion.hpp"
#include "math3d_define.hpp"
#include "ForceContributor.hpp"
#include "MomentContributor.hpp"


#define HEADER
#define EOM EquationsOfMotion


HEADER
void EOM::addForceContributor( ForceContributor & force ){
    forces.push_back(&force);
}

HEADER
void EOM::addMomentContributor( MomentContributor & moment ){
    moments.push_back(&moment);
}

HEADER
EOM::EquationsOfMotion(){
    numDims_ = 4 + 3*3;
}

HEADER
void EOM::setInertia( mat3 & I_, mat3 & Iinv_ ){
    I = &I_;
    Iinv = &Iinv_;
}

HEADER
void EOM::setMass( double & mass_ ){
    mass = &mass_;
}

HEADER
void EOM::setEOM_State( double* eom_state ){
    state = eom_state;
}

const vec3 & EOM::getPos() const {
    return pos;
}
const vec3 & EOM::getVel() const {
    return vel;
}
const vec3 & EOM::getAccel() const {
    return accel;
}
const quat & EOM::getAttitude() const {
    return q;
}
const vec3 & EOM::getAngularVel() const {
    return omega;
}

HEADER
void EOM::updateComponents(){
    double* s = state;
    pos[0] = s[0];
    pos[1] = s[1];
    pos[2] = s[2];
    vel[0] = s[3];
    vel[1] = s[4];
    vel[2] = s[5];
    q[0] = s[6];
    q[1] = s[7];
    q[2] = s[8];
    q[3] = s[9];
    q.normalize();
    omega[0] = s[10];
    omega[1] = s[11];
    omega[2] = s[12];
}

HEADER
void EOM::operator()(double t, double* dudt ){
    
    vec3 totAccel, totMomentBody;
    
    // get total external moments and forces
    getTotalForceAndMoments( t, totAccel, totMomentBody );
    
    // velocity derivative
    double m = (*mass);
    totAccel[0] /= m; totAccel[1] /= m; totAccel[2] /= m;
    accel = totAccel; // used for sensors/external models

    // set position derivative
    dudt[0] = vel[0];
    dudt[1] = vel[1];
    dudt[2] = vel[2];
    
    // set derivative of velocity
    dudt[3] = accel[0];
    dudt[4] = accel[1];
    dudt[5] = accel[2];
    
    // set attitude derivative
    quat dqdt = q.getDerivative(omega);
    dudt[6] = dqdt[0];
    dudt[7] = dqdt[1];
    dudt[8] = dqdt[2];
    dudt[9] = dqdt[3];
    
    // set angular velocity derivative
    vec3 dwdt = (*Iinv)*(totMomentBody - omega.cross((*I)*omega));
    dudt[10] = dwdt[0];
    dudt[11] = dwdt[1];
    dudt[12] = dwdt[2];
    
}

HEADER
void EOM::getTotalForceAndMoments(  double time, vec3 & totalForce, vec3 & totalMomentBody ){
    vec3 force, moment, r, tmp;
    
    for(int i = 0; i < forces.size(); ++i){
        forces[i]->getForce( time, force );
        totalForce[0] += force[0];
        totalForce[1] += force[1];
        totalForce[2] += force[2];
        
        forces[i]->getLocation(time, r);
        
        tmp = r.cross(force);
        totalMomentBody[0] += tmp[0];
        totalMomentBody[1] += tmp[1];
        totalMomentBody[2] += tmp[2];
    }
    
    for(int i = 0; i < moments.size(); ++i ){
        moments[i]->getMoment(time, moment);
        totalMomentBody[0] += moment[0];
        totalMomentBody[1] += moment[1];
        totalMomentBody[2] += moment[2];
    }
}

HEADER
int EOM::numDims() const {
    return numDims_;
}

#undef HEADER
#undef EOM

#endif
