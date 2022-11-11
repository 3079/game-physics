#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
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
	return m_massPoints.size();
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.push_back(Spring(&m_massPoints.at(masspoint1), &m_massPoints.at(masspoint2), m_fStiffness, initialLength));
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
	// Calculate spring forces for every spring and accumulate them in mass points
	for (int i = 0; i < m_springs.size(); i++)
	{
		Spring spring = m_springs.at(i);
		MassPoint p1 = *spring.m_p1;
		MassPoint p2 = *spring.m_p2;

		Vec3 d = p1.m_position - p2.m_position;
		float l = sqrt(d.x * d.x + d.y * d.y);
		d /= l;

		Vec3 F = spring.m_stiffness * (spring.m_baseLength - l) * d;

		p1.m_force += F;
		p2.m_force -= F;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// Clear each point's forces
	for (int i = 0; i < m_massPoints.size(); i++)
		m_massPoints.at(i).m_force = m_externalForce;

	calculateForces();

	// Calculate each point's acceleration, velocity and position according to the m_iIntegrator value
	switch (m_iIntegrator)
	{
	// Explicit Euler
	case EULER:
		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint p = m_massPoints.at(i);

			p.m_acceleration = p.m_force / p.m_mass;
			p.m_position += p.m_velocity * timeStep;
			p.m_velocity += p.m_acceleration * timeStep;
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
			MassPoint p = m_massPoints.at(i);

			p.m_acceleration = p.m_force / p.m_mass;
			original_positions.push_back(p.m_position);
			original_velocitiess.push_back(p.m_velocity);
			p.m_position += p.m_velocity * timeStep / 2;
			p.m_velocity += p.m_acceleration * timeStep / 2;
		}

		// Recalculating forces at the midpoint
		calculateForces;

		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint p = m_massPoints.at(i);

			p.m_acceleration = p.m_force / p.m_mass;
			p.m_position = original_positions.at(i) + p.m_velocity * timeStep;
			p.m_velocity = original_velocitiess.at(i) + p.m_acceleration * timeStep;
		}
		break;
	}
	// Leap-Frog
	case LEAPFROG:
		for (int i = 0; i < m_massPoints.size(); i++)
		{
			MassPoint p = m_massPoints.at(i);

			p.m_acceleration = p.m_force / p.m_mass;
			p.m_velocity += p.m_acceleration * timeStep;
			p.m_position += p.m_velocity * timeStep;
		}
		break;
	default:
		break;
	}
}