#pragma once
#include "mat3.h"
#include "vectorbase.h"

class rigidbody
{
public:
    rigidbody(Vec3 size, int mass,
    Vec3 position, Quat orientation,
    Vec3 linearVelocity, Vec3 angularVelocity)
    {
        this->size = size;
        this->mass = mass;
        this->position = position;
        this->orientation = orientation;
        this->linearVelocity = linearVelocity;
        this->angularVelocity = angularVelocity;
    }

    Vec3 size;
    int mass;
    Vec3 position;
    Quat orientation;
    Vec3 force;
    Vec3 torque;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 angularMomentum;
    mat3<double> invInertia;

    void initializeRigidbody()
    {
        double one = this->mass * (this->size.y * this->size.y + this->size.z *
            this->size.z) / 12;
        double two = this->mass * (this->size.x * this->size.x + this->size.y *
            this->size.y) / 12;
        double three = this->mass * (this->size.x * this->size.x + this->size.z *
            this->size.z) / 12;

        this->invInertia = mat3<double>(
            1/one, 0, 0,
            0, 1/two, 0,
            0, 0, 1/three);
        calcInvInertia();

        calcAngularVelocity();

        cout << "invInertia " << this->invInertia <<
                " angularvelocity " << this->angularVelocity <<
                    std::endl;
    }

    void applyForce(Vec3 position, Vec3 force)
    {
        this->force += force;
        this->torque = cross(position - this->position, force);
    }
    void calcInvInertia()
    {
        this->invInertia = this->orientation.getRotMat3() * this->invInertia * this->
            orientation.getRotMat3();
    }

    void calcAngularVelocity()
    {
        this->angularVelocity = this->invInertia * this->angularMomentum;
    }
    
};
