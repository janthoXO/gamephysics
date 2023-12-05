#pragma once
#include "vectorbase.h"

class mass_point
{
public:
    mass_point(GamePhysics::Vec3 position, GamePhysics::Vec3 velocity, Vec3 force, bool isFixed)
    {
        this->position = position;
        this->velocity = velocity;
        this->force = force;
        this->is_fixed = isFixed;
    }
    
    Vec3 position;
    Vec3 velocity;
    Vec3 force;
    bool is_fixed;
};
