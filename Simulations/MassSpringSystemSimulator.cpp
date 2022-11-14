#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = EULER;
	//m_massPoints = {};
	//m_springs = {};
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	m_fSphereSize = 0.05f;

	// DEMO 1 & 2
	MassPoint p1 = MassPoint(m_fMass, Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	MassPoint p2 = MassPoint(m_fMass, Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	m_massPoints = { p1, p2 };
	Spring s = Spring(&m_massPoints.at(0), &m_massPoints.at(1), m_fStiffness, 1);
	m_springs = { s };

	//// DEMO 3
	//m_iIntegrator = MIDPOINT;
	//MassPoint p1 = MassPoint(m_fMass, Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	//MassPoint p2 = MassPoint(m_fMass, Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	//m_massPoints = { p1, p2 };
	//Spring s = Spring(&m_massPoints.at(0), &m_massPoints.at(1), m_fStiffness, 1);
	//m_springs = { s };
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Explicit Euler, Midpoint, Leap-Frog";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));

	for (int i = 0; i < m_massPoints.size(); i++)
	{
		MassPoint* p = &m_massPoints.at(i);
		DUC->drawSphere(p->m_position, Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
	}
	
	for (int i = 0; i < m_springs.size(); i++)
	{
		Spring s = m_springs.at(i);
		MassPoint* p1 = s.m_p1;
		MassPoint* p2 = s.m_p2;

		// FOR WHATEVER REASON DRAWLINE() METHOD CAUSES EVERYTHING TO CRASH !!!!!!
		
		//DUC->drawLine(p1->m_position, Vec3(1, 1, 1), p2->m_position, Vec3(1, 1, 1));
		//DUC->drawLine(p1->m_position.toDirectXVector(), Colors::LightGray, p2->m_position.toDirectXVector(), Colors::LightGray);
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit Euler !\n";
		break;
	case 1:
		cout << "Midpoint !\n";
		break;
	case 2:
		cout << "Leap-Frog !\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	m_massPoints.push_back(MassPoint(m_fMass, position, velocity, isFixed));
	cout << "Added point: Mass: " << m_fMass << "; Position: (" << position.x << ", " << position.y << ", " << position.z << "); Velocity: ("
		<< velocity.x << ", " << velocity.y << ", " << velocity.z << "); ifFixed: " << isFixed << endl;
	cout << "Total number of points: " << m_massPoints.size() - 1 << endl;
	return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.push_back(Spring(&m_massPoints.at(masspoint1), &m_massPoints.at(masspoint2), m_fStiffness, initialLength));
	cout << "Added spring: Point 1: " << masspoint1 << "; Point 2: " << masspoint2 << "; Stiffness: " << m_fStiffness << "; Initial Length: " << initialLength << endl;
	cout << "Total number of springs: " << m_springs.size() - 1 << endl;
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_massPoints.at(index).m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_massPoints.at(index).m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}

void MassSpringSystemSimulator::calculateForces()
{
	// Clear each point's forces
	for (int i = 0; i < m_massPoints.size(); i++)
		m_massPoints.at(i).m_force = m_externalForce;

	// Calculate spring forces for every spring and accumulate them in mass points
	for (int i = 0; i < m_springs.size(); i++)
	{
		Spring spring = m_springs.at(i);
		MassPoint* p1 = spring.m_p1;
		MassPoint* p2 = spring.m_p2;

		Vec3 d = p1->m_position - p2->m_position;
		float l = sqrt(d.x * d.x + d.y * d.y);
		d /= l;

		Vec3 F = spring.m_stiffness * (spring.m_baseLength - l) *  d;

		p1->m_force += F;
		p2->m_force -= F;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	calculateForces();

	// Calculate each point's acceleration, velocity and position according to the m_iIntegrator value
	switch (m_iIntegrator)
	{
	// Explicit Euler
	case EULER:
		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint* p = &m_massPoints.at(i);

			if (p->m_isFixed)
				continue;

			p->m_acceleration = p->m_force / p->m_mass;
			p->m_position += p->m_velocity * timeStep;
			p->m_velocity += p->m_acceleration * timeStep;
			cout << "Point " << i << ": Position: (" << p->m_position.x << ", " << p->m_position.y << ", " << p->m_position.z << "); Velocity: ("
				<< p->m_velocity.x << ", " << p->m_velocity.y << ", " << p->m_velocity.z << ")" << endl;
		}
		break;
	// Midpoint
	case MIDPOINT:
	{
		// Temporarily store points' initial positions and velocities at the beginning of the time step
		std::vector<Vec3> original_positions;
		std::vector<Vec3> original_velocitiess;

		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint* p = &m_massPoints.at(i);

			if (p->m_isFixed)
				continue;

			p->m_acceleration = p->m_force / p->m_mass;
			original_positions.push_back(p->m_position);
			original_velocitiess.push_back(p->m_velocity);
			p->m_position += p->m_velocity * timeStep / 2;
			p->m_velocity += p->m_acceleration * timeStep / 2;

			cout << "Point " << i << ": Midpoint Position: (" << p->m_position.x << ", " << p->m_position.y << ", " << p->m_position.z << "); Midpoint Velocity: ("
				<< p->m_velocity.x << ", " << p->m_velocity.y << ", " << p->m_velocity.z << endl;
		}

		// Recalculating forces at the midpoint
		calculateForces();

		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint* p = &m_massPoints.at(i);

			if (p->m_isFixed)
				continue;

			p->m_acceleration = p->m_force / p->m_mass;
			p->m_position = original_positions.at(i) + p->m_velocity * timeStep;
			p->m_velocity = original_velocitiess.at(i) + p->m_acceleration * timeStep;

			cout << "Point " << i << ": Position: (" << p->m_position.x << ", " << p->m_position.y << ", " << p->m_position.z << "); Velocity: ("
				<< p->m_velocity.x << ", " << p->m_velocity.y << ", " << p->m_velocity.z << endl;
		}
		break;
	}
	// Leap-Frog
	case LEAPFROG:
		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint* p = &m_massPoints.at(i);

			if (p->m_isFixed)
				continue;

			p->m_acceleration = p->m_force / p->m_mass;
			p->m_velocity += p->m_acceleration * timeStep;
			p->m_position += p->m_velocity * timeStep;

			cout << "Point " << i << ": Position: (" << p->m_position.x << ", " << p->m_position.y << ", " << p->m_position.z << "); Velocity: ("
				<< p->m_velocity.x << ", " << p->m_velocity.y << ", " << p->m_velocity.z << endl;
		}
		break;
	default:
		break;
	}
}