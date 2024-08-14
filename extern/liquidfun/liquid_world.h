#include "liquid_stack_allocator.h"
#include "particle_system.h"

#ifndef B2_LIQUID_WORLD_H
#define B2_LIQUID_WORLD_H

// B2_API void b2LiquidWorld_Step( b2WorldId worldId, float timeStep, int subStepCount );
void b2Shape_ComputeDistance( b2ShapeId shapeId, b2Vec2 target, float32* distance, b2Vec2* normal );
bool operator<( b2ShapeId a, b2ShapeId b );

/// Callback class for AABB queries.
/// See b2World::Query
class b2LiquidQueryCallback
{
public:
	virtual ~b2LiquidQueryCallback() {}

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	// virtual bool ReportFixture(b2Fixture* fixture) = 0;

	/// Called for each particle found in the query AABB.
	/// @return false to terminate the query.
	virtual bool ReportParticle(const b2ParticleSystem* particleSystem,
								int32 index)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
		return false;
	}

	/// Cull an entire particle system from b2World::QueryAABB. Ignored for
	/// b2ParticleSystem::QueryAABB.
	/// @return true if you want to include particleSystem in the AABB query,
	/// or false to cull particleSystem from the AABB query.
	virtual bool ShouldQueryParticleSystem(
		const b2ParticleSystem* particleSystem)
	{
		B2_NOT_USED(particleSystem);
		return true;
	}
};

class b2LiquidWorld {
public:
	// b2LiquidWorld( const b2Vec2& gravity );

	b2LiquidWorld( b2WorldId worldId );

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b2LiquidWorld();

	/// Create a particle system given a definition. No reference to the
	/// definition is retained.
	/// @warning This function is locked during callbacks.
	b2ParticleSystem* CreateParticleSystem(const b2ParticleSystemDef* def);

	/// Destroy a particle system.
	/// @warning This function is locked during callbacks.
	void DestroyParticleSystem(b2ParticleSystem* p);

	void QueryAABB( b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context );

	void Step(float timeStep, int subStepCount );

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const;

	/// Get the world particle-system list. With the returned body, use
	/// b2ParticleSystem::GetNext to get the next particle-system in the world
	/// list. A NULL particle-system indicates the end of the list.
	/// @return the head of the world particle-system list.
	b2ParticleSystem* GetParticleSystemList();
	const b2ParticleSystem* GetParticleSystemList() const;

	b2WorldId GetWorldId() {
		return m_worldId;
	}

	/// Get API version.
	const b2Version* GetVersion() const {
		return m_liquidFunVersion;
	}

	/// Get API version string.
	const char* GetVersionString() const {
		return m_liquidFunVersionString;
	}

private:

	friend class b2ParticleSystem;

	b2WorldId m_worldId;

	b2ParticleSystem* m_particleSystemList;

	b2BlockAllocator m_blockAllocator;
	b2LiquidStackAllocator m_stackAllocator;

	/// Used to reference b2_LiquidFunVersion so that it's not stripped from
	/// the static library.
	const b2Version *m_liquidFunVersion;
	const char *m_liquidFunVersionString;
};

#endif
