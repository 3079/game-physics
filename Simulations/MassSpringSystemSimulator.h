#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


struct MassPoint
{
	float m_mass;
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_acceleration;
	Vec3 m_force;
	bool m_isFixed;

	MassPoint(float mass, Vec3 position, Vec3 velocity, bool isFixed) : m_mass(mass), m_position(position), m_velocity(velocity),
		m_acceleration(Vec3()), m_force(Vec3()), m_isFixed(isFixed) {}
};

struct Spring
{
	MassPoint* m_p1;
	MassPoint* m_p2;
	float m_stiffness;
	float m_baseLength;

	Spring(MassPoint* p1, MassPoint* p2, float stiffness, float baseLength) : m_p1(p1), m_p2(p2), m_stiffness(stiffness), m_baseLength(baseLength) {}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	// Added Functions
	void calculateForces();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	// Added Attributes
	std::vector<MassPoint> m_massPoints;
	std::vector<Spring> m_springs;

	// UI Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	float m_fSphereSize;
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};

#endif