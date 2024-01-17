#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
	// to be implemented
public:
	Grid(int m, int n)
	{
		arr = new float*[m];
		for (int i = 0; i < m; i++)
		{
			arr[i] = new float[n];
		}
	}
	float** arr;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	
	Grid T = Grid(1, 1);
	int m;
	int n;
	float size;
	float alpha;
};

#endif