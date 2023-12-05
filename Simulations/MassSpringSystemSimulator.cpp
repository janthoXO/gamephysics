#include "MassSpringSystemSimulator.h"

#include "util/MassPoint.h"
#include "util/spring.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_fMass = 10.1;
    m_fStiffness = 0.1;
    m_fDamping = 0.1;
    m_iIntegrator = 1;

    // UI Attributes
    m_externalForce = Vec3();
    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
}

void MassSpringSystemSimulator::reset()
{
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    switch (m_iIntegrator)
    {
    case EULER:
        euler_step(timeStep);
        break;
    case LEAPFROG:
        cout << "NOT IMPLEMENTED YET" << endl;
        break;
    case MIDPOINT:
        mid_point(timeStep);
        break;
    default:
        cout << "UNDEFINED INTEGRATOR" << endl;
    }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

// Specific Functions
vector<mass_point> masspoints;
vector<spring> springs;

void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    masspoints.push_back(mass_point(position, Velocity, Vec3(), isFixed));
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    if (masspoint1 < masspoint2)
    {
        spring s = spring(m_fStiffness, initialLength, masspoints[masspoint1], masspoints[masspoint2]);
        springs.push_back(s);
    }
    else
    {
        spring s = spring(m_fStiffness, initialLength, masspoints[masspoint2], masspoints[masspoint1]);
        springs.push_back(s);
    }
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return masspoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return masspoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

void MassSpringSystemSimulator::euler_step(float h)
{
    // reset forces
    for (auto& i : masspoints)
    {
        i.force = Vec3();
        i.force += Vec3(0, -9.81, 0);
    }

    for (auto& i : springs)
    {
        // compute elastic forces
        Vec3 internal_force = -i.stiffness * (i.mp1.position - i.mp2.position - i.initial_length) * (i.mp1.position - i.
            mp2.position / i.initial_length);
        i.mp1.force += internal_force;
        i.mp2.force -= internal_force;
    }

    for (auto& i : masspoints)
    {
        // integrate Position x[t+1] = x[t] + h * v
        i.position += h * i.velocity;

        // integrate velocity v[t+1] = F[x[t]] - damp*v[t]/ m
        i.velocity = (i.force - i.velocity * m_fDamping) / m_fMass;
    }
}

void MassSpringSystemSimulator::mid_point(float h)
{
    
}
