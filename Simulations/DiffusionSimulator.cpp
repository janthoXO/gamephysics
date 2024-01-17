#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
    m_iTestCase = 0;
    m_vfMovableObjectPos = Vec3();
    m_vfMovableObjectFinalPos = Vec3();
    m_vfRotate = Vec3();
    // rest to be implemented
    m_mouse = Point2D();
    m_trackmouse = Point2D();
    m_oldtrackmouse = Point2D();

    m = 10;
    n = 10;
    T = Grid(m, n);
    
    size = 1;
    alpha = 0.3;
}

const char* DiffusionSimulator::getTestCasesStr()
{
    return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
    
    T = Grid(m, n);
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            // generate number between -1 and 1
            T.arr[i][j] = -1 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(1+1)));
        }
    }
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    // to be implemented
    TwAddVarRW(DUC->g_pTweakBar, "m", TW_TYPE_INT32, &m, "min=1");
    TwAddVarRW(DUC->g_pTweakBar, "n", TW_TYPE_INT32, &n, "min=1");
    TwAddVarRW(DUC->g_pTweakBar, "size", TW_TYPE_FLOAT, &size, "min=0 step=0.1");
    TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_FLOAT, &alpha, "min=0 max=1 step=0.1");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    m_vfMovableObjectPos = Vec3(0, 0, 0);
    m_vfRotate = Vec3(0, 0, 0);
    //
    // to be implemented
    //
    switch (m_iTestCase)
    {
    case 0:
        cout << "Explicit solver!\n";
        break;
    case 1:
        cout << "Implicit solver!\n";
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }

    reset();
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep)
{
    Grid newGrid = Grid(m, n);
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if(i == 0 || i == m - 1 || j == 0 || j == n - 1)
            {
                newGrid.arr[i][j] = 0;
                continue;
            }
            float xcomponent, ycomponent;
            float plus2 = i+2 >= m ? 0 : T.arr[i+2][j];
            float minus2 = i-2 < 0 ? 0 : T.arr[i-2][j];
            xcomponent = (plus2 - 2*T.arr[i][j] + minus2) / (4 * size * size);
            
            plus2 = j+2 >= n ? 0 : T.arr[i][j+2];
            minus2 = j-2 < 0 ? 0 : T.arr[i][j-2];
            ycomponent = (plus2 - 2*T.arr[i][j] + minus2) / (4 * size * size);
            
            newGrid.arr[i][j] = T.arr[i][j] + alpha * timeStep * (xcomponent + ycomponent);
        }
    }

    T = newGrid;
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep)
{
    // solve A T = b

    // This is just an example to show how to work with the PCG solver,
    const int nx = m-2;
    const int ny = n-2;
    const int nz = 1;
    const int N = nx * ny * nz;

    SparseMatrix<Real> A(N);
    std::vector<Real> b(N);

    // This is the part where you have to assemble the system matrix A and the right-hand side b!
    float gamma = -alpha * timeStep / (4 * size * size);
    for (int i = 0; i < m-2; ++i)
    {
        for (int j = 0; j < n-2; ++j)
        {
            vector<int> indices;
            vector<double> values;
            if(i != 0 && i != 1)
            {
                // add i-2
                indices.push_back(i-2+j*(m-2));
                values.push_back(gamma);
            }
            if (i != m - 3 && i != m - 4)
            {
                // add i+2
                indices.push_back(i+2+j*(m-2));
                values.push_back(gamma);
            }
            if (j != 0 && j != 1)
            {
                // add j-2
                indices.push_back(i+(j-2)*(m-2));
                values.push_back(gamma);
            }
            if (j != n - 3 && j != n - 4)
            {
                // add j+2
                indices.push_back(i+(j+2)*(m-2));
                values.push_back(gamma);
            }

            indices.push_back(i+j*(m-2));
            values.push_back(-4*gamma+1);
            
            A.add_sparse_row(i+j*(m-2), indices, values);
        }
        }

    // perform solve
    Real pcg_target_residual = 1e-05;
    Real pcg_max_iterations = 1000;
    Real ret_pcg_residual = 1e10;
    int ret_pcg_iterations = -1;

    SparsePCGSolver<Real> solver;
    solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

    std::vector<Real> x(N);
    for (int i = 0; i < m-2; ++i)
    {
        for (int j = 0; j < n-2; ++j)
        {
            x[i+j*(m-2)] = T.arr[i+1][j+1];
        }
    }
    

    // preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
    solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

    // Final step is to extract the grid temperatures from the solution vector x
    // TODO to be implemented
    for (int i = 0; i < m-2; ++i)
    {
        for (int j = 0; j < n-2; ++j)
        {
            T.arr[i+1][j+1] = x[i+j*(m-2)];
        }
    }
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
    // update current setup for each frame
    switch (m_iTestCase)
    {
    case 0:
        // feel free to change the signature of this function
        diffuseTemperatureExplicit(timeStep);
        break;
    case 1:
        // feel free to change the signature of this function
        diffuseTemperatureImplicit(timeStep);
        break;
    }
}

void DiffusionSimulator::drawObjects()
{
    // TODO to be implemented
    //visualization
    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            float p = T.arr[i][j];
            Vec3 color = T.arr[i][j] < 0 ? Vec3(p,0,0) : Vec3(p,p,p);
            DUC->setUpLighting(Vec3(0,0,0), Vec3(1,1,1), 1000, color);
            
            float x = (i - (int)m / 2) * size;
            float y = j * size;
            DUC->drawSphere(Vec3(x, y, 0), Vec3(size, size, size));
        }
    }
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}
