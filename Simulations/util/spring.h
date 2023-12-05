#pragma once

class spring
{
public:
    spring(float stiffness, float initialLength, mass_point mp1, mass_point mp2)
    {
        this->stiffness = stiffness;
        this->initial_length = initialLength;
        this->mp1 = mp1;
        this->mp2 = mp2;
    }

    float stiffness;
    float initial_length;
    mass_point mp1;
    mass_point mp2;
};
