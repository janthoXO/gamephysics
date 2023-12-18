#include "RigidBodySystemSimulator.h"

#include "util/rigidbody.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    m_externalForce = Vec3();

    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
    demo_num = 1;
}

vector<rigidbody> rigidbodies;

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
    rigidbodies.clear();
    m_externalForce = Vec3();

    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();

    switch (demo_num)
    {
    case 1:
    case 2:
        {
            rigidbody r = rigidbody(Vec3(1, 0.6, 0.5), 2,
                                    Vec3(0, 0, 0), Quat(0,0,0.707, 0.707),
                                    Vec3(0, 0, 0), Vec3(0, 0, 0));

            
            r.initializeRigidbody();
            r.applyForce(Vec3(0.3, 0.5, 0.25), Vec3(1,1,0));

            rigidbodies.push_back(r);
        }
        break;
    case 3:
        {
        }
            break;
    case 4:
        break;
    default:
        cout << "No valid demo";
    }
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    for (int i = 0; i < getNumberOfRigidBodies(); ++i)
    {
        DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

        matrix4x4<double> scaleMat = matrix4x4<double>(rigidbodies[i].size.x, 0,0,0,
            0, rigidbodies[i].size.y, 0,0,
            0,0,rigidbodies[i].size.z, 0,
            0,0,0,1);
        matrix4x4<double> rotMat = rigidbodies[i].orientation.getRotMat();
        matrix4x4<double> translatMat;
        translatMat.initTranslation(rigidbodies[i].position.x, rigidbodies[i].position.y, rigidbodies[i].position.z);

        matrix4x4<double> r = scaleMat * rotMat * translatMat;
        DUC->drawRigidBody(r);
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    demo_num = testCase + 1;
    reset();
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    for (int i = 0; i < getNumberOfRigidBodies(); ++i)
    {
        rigidbodies[i].position = rigidbodies[i].position + timeStep * rigidbodies[i].linearVelocity;
        rigidbodies[i].linearVelocity = rigidbodies[i].linearVelocity + timeStep * rigidbodies[i].force / rigidbodies[i].mass;

        rigidbodies[i].orientation = rigidbodies[i].orientation + (timeStep / 2) * Quat(
            rigidbodies[i].angularVelocity.x, rigidbodies[i].angularVelocity.y, rigidbodies[i].angularVelocity.z, 0) * rigidbodies[i].orientation;
        if (rigidbodies[i].orientation.norm() != 0)
        {
            rigidbodies[i].orientation = rigidbodies[i].orientation.unit();
        }

        rigidbodies[i].angularMomentum = rigidbodies[i].angularMomentum + timeStep * rigidbodies[i].torque;

        rigidbodies[i].calcInvInertia();

        rigidbodies[i].calcAngularVelocity();

        // cout << "Rigidbody " << i <<
        //     " linear velocity " << rigidbodies[i].linearVelocity <<
        //         " angular veloctity " << rigidbodies[i].angularVelocity <<
        //             " torque " << rigidbodies[i].torque <<
        //                 std::endl;
    }
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = m_trackmouse.x;
    m_oldtrackmouse.y = m_trackmouse.y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return rigidbodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return rigidbodies[i].linearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return rigidbodies[i].angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    rigidbody r = rigidbody(size, mass, position, Quat(), Vec3(), Vec3());
    rigidbodies.push_back(r);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    rigidbodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    rigidbodies[i].linearVelocity = velocity;
}
