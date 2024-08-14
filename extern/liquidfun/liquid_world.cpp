#include "Rtt_Assert.h"
#include "liquid_world.h"
extern "C" {
#include "src/world.h"

#include "src/aabb.h"
#include "src/allocate.h"
#include "src/array.h"
#include "src/bitset.h"
#include "src/block_array.h"
#include "src/body.h"
#include "src/broad_phase.h"
#include "src/constraint_graph.h"
#include "src/contact.h"
#include "src/core.h"
#include "src/ctz.h"
#include "src/island.h"
#include "src/joint.h"
#include "src/shape.h"
#include "src/solver.h"
#include "src/solver_set.h"

#include <float.h>
#include <stdio.h>
#include <string.h>
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_MAYBE_UNUSED( proxyId );

	WorldQueryContext* worldContext = (WorldQueryContext*)context;
	b2World* world = worldContext->world;

	b2CheckId( world->shapeArray, shapeId );
	b2Shape* shape = world->shapeArray + shapeId;

	b2Filter shapeFilter = shape->filter;
	b2QueryFilter queryFilter = worldContext->filter;

	if ( ( shapeFilter.categoryBits & queryFilter.maskBits ) == 0 || ( shapeFilter.maskBits & queryFilter.categoryBits ) == 0 )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world->worldId, shape->revision };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

static void b2CollideTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( collide_task, "Collide Task", b2_colorDodgerBlue, true );

	b2StepContext* stepContext = (b2StepContext*)context;
	b2World* world = stepContext->world;
	B2_ASSERT( threadIndex < world->workerCount );
	b2TaskContext* taskContext = world->taskContextArray + threadIndex;
	b2ContactSim** contactSims = stepContext->contacts;
	b2Shape* shapes = world->shapeArray;
	b2Body* bodies = world->bodyArray;

	B2_ASSERT( startIndex < endIndex );

	for ( int i = startIndex; i < endIndex; ++i )
	{
		b2ContactSim* contactSim = contactSims[i];

		int contactId = contactSim->contactId;

		b2Shape* shapeA = shapes + contactSim->shapeIdA;
		b2Shape* shapeB = shapes + contactSim->shapeIdB;

		// Do proxies still overlap?
		bool overlap = b2AABB_Overlaps( shapeA->fatAABB, shapeB->fatAABB );
		if ( overlap == false )
		{
			contactSim->simFlags |= b2_simDisjoint;
			contactSim->simFlags &= ~b2_simTouchingFlag;
			b2SetBit( &taskContext->contactStateBitSet, contactId );
		}
		else
		{
			bool wasTouching = ( contactSim->simFlags & b2_simTouchingFlag );

			// Update contact respecting shape/body order (A,B)
			b2Body* bodyA = bodies + shapeA->bodyId;
			b2Body* bodyB = bodies + shapeB->bodyId;
			b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
			b2BodySim* bodySimB = b2GetBodySim( world, bodyB );

			// avoid cache misses in b2PrepareContactsTask
			contactSim->bodySimIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
			contactSim->invMassA = bodySimA->invMass;
			contactSim->invIA = bodySimA->invInertia;

			contactSim->bodySimIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;
			contactSim->invMassB = bodySimB->invMass;
			contactSim->invIB = bodySimB->invInertia;

			b2Transform transformA = bodySimA->transform;
			b2Transform transformB = bodySimB->transform;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA->localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB->localCenter );

			// This updates solid contacts and sensors
			bool touching =
				b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			// State changes that affect island connectivity. Also contact and sensor events.
			if ( touching == true && wasTouching == false )
			{
				contactSim->simFlags |= b2_simStartedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim->simFlags |= b2_simStoppedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}
		}
	}

	b2TracyCZoneEnd( collide_task );
}

static void b2UpdateTreesTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	B2_MAYBE_UNUSED( startIndex );
	B2_MAYBE_UNUSED( endIndex );
	B2_MAYBE_UNUSED( threadIndex );

	b2TracyCZoneNC( tree_task, "Rebuild Trees", b2_colorSnow, true );

	b2World* world = (b2World*)context;
	b2BroadPhase_RebuildTrees( &world->broadPhase );

	b2TracyCZoneEnd( tree_task );
}

static void b2AddNonTouchingContact( b2World* world, b2Contact* contact, b2ContactSim* contactSim )
{
	B2_ASSERT( contact->setIndex == b2_awakeSet );
	b2SolverSet* set = world->solverSetArray + b2_awakeSet;
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contacts.count;

	b2ContactSim* newContactSim = b2AddContact( &set->contacts );
	memcpy( newContactSim, contactSim, sizeof( b2ContactSim ) );
}

static void b2RemoveNonTouchingContact( b2World* world, int setIndex, int localIndex )
{
	b2CheckIndex( world->solverSetArray, setIndex );
	b2SolverSet* set = world->solverSetArray + setIndex;
	int movedIndex = b2RemoveContact( &set->contacts, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = set->contacts.data + localIndex;
		b2CheckIndex( world->contactArray, movedContactSim->contactId );
		b2Contact* movedContact = world->contactArray + movedContactSim->contactId;
		B2_ASSERT( movedContact->setIndex == setIndex );
		B2_ASSERT( movedContact->localIndex == movedIndex );
		B2_ASSERT( movedContact->colorIndex == B2_NULL_INDEX );
		movedContact->localIndex = localIndex;
	}
}

// Narrow-phase collision
static void b2Collide( b2StepContext* context )
{
	b2World* world = context->world;

	B2_ASSERT( world->workerCount > 0 );

	b2TracyCZoneNC( collide, "Collide", b2_colorDarkOrchid, true );

	// Tasks that can be done in parallel with the narrow-phase
	// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	world->userTreeTask = world->enqueueTaskFcn( &b2UpdateTreesTask, 1, 1, world, world->userTaskContext );
	world->taskCount += 1;
	world->activeTaskCount += world->userTreeTask == NULL ? 0 : 1;

	// gather contacts into a single array for easier parallel-for
	int contactCount = 0;
	b2GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		contactCount += graphColors[i].contacts.count;
	}

	int nonTouchingCount = world->solverSetArray[b2_awakeSet].contacts.count;
	contactCount += nonTouchingCount;

	if ( contactCount == 0 )
	{
		b2TracyCZoneEnd( collide );
		return;
	}

	b2ContactSim** contactSims = (b2ContactSim**)b2AllocateStackItem( &world->stackAllocator, contactCount * sizeof( b2ContactSim ), "contacts" );

	int contactIndex = 0;
	for ( int i = 0; i < b2_graphColorCount; ++i )
	{
		b2GraphColor* color = graphColors + i;
		int count = color->contacts.count;
		b2ContactSim* base = color->contacts.data;
		for ( int j = 0; j < count; ++j )
		{
			contactSims[contactIndex] = base + j;
			contactIndex += 1;
		}
	}

	{
		b2ContactSim* base = world->solverSetArray[b2_awakeSet].contacts.data;
		for ( int i = 0; i < nonTouchingCount; ++i )
		{
			contactSims[contactIndex] = base + i;
			contactIndex += 1;
		}
	}

	B2_ASSERT( contactIndex == contactCount );

	context->contacts = contactSims;

	// Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	int contactIdCapacity = b2GetIdCapacity( &world->contactIdPool );
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->taskContextArray[i].contactStateBitSet, contactIdCapacity );
	}

	// Task should take at least 40us on a 4GHz CPU (10K cycles)
	int minRange = 64;
	void* userCollideTask = world->enqueueTaskFcn( &b2CollideTask, contactCount, minRange, context, world->userTaskContext );
	world->taskCount += 1;
	if ( userCollideTask != NULL )
	{
		world->finishTaskFcn( userCollideTask, world->userTaskContext );
	}

	b2FreeStackItem( &world->stackAllocator, contactSims );
	context->contacts = NULL;
	contactSims = NULL;

	// Serially update contact state
	b2TracyCZoneNC( contact_state, "Contact State", b2_colorCoral, true );

	// Bitwise OR all contact bits
	b2BitSet* bitSet = &world->taskContextArray[0].contactStateBitSet;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->taskContextArray[i].contactStateBitSet );
	}

	b2Contact* contacts = world->contactArray;
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;

	const b2Shape* shapes = world->shapeArray;
	int16_t worldId = world->worldId;

	// Process contact state changes. Iterate over set bits
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = b2CTZ64( bits );
			int contactId = (int)( 64 * k + ctz );

			b2CheckIndex( contacts, contactId );

			b2Contact* contact = contacts + contactId;
			B2_ASSERT( contact->setIndex == b2_awakeSet );

			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;

			b2ContactSim* contactSim = NULL;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// contact lives in constraint graph
				B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );
				b2GraphColor* color = graphColors + colorIndex;
				B2_ASSERT( 0 <= localIndex && localIndex < color->contacts.count );
				contactSim = color->contacts.data + localIndex;
			}
			else
			{
				B2_ASSERT( 0 <= localIndex && localIndex < awakeSet->contacts.count );
				contactSim = awakeSet->contacts.data + localIndex;
			}

			const b2Shape* shapeA = shapes + contact->shapeIdA;
			const b2Shape* shapeB = shapes + contact->shapeIdB;
			b2ShapeId shapeIdA = { shapeA->id + 1, (uint16_t)worldId, shapeA->revision };
			b2ShapeId shapeIdB = { shapeB->id + 1, (uint16_t)worldId, shapeB->revision };
			uint32_t flags = contact->flags;
			uint32_t simFlags = contactSim->simFlags;

			if ( simFlags & b2_simDisjoint )
			{
				// Was touching?
				if ( ( flags & b2_contactTouchingFlag ) != 0 && ( flags & b2_contactEnableContactEvents ) != 0 )
				{
					b2ContactEndTouchEvent event = { shapeIdA, shapeIdB };
					b2Array_Push( world->contactEndArray, event );
				}

				// Bounding boxes no longer overlap
				contact->flags &= ~b2_contactTouchingFlag;
				b2DestroyContact( world, contact, false );
				contact = NULL;
				contactSim = NULL;
			}
			else if ( simFlags & b2_simStartedTouching )
			{
				B2_ASSERT( contact->islandId == B2_NULL_INDEX );
				if ( ( flags & b2_contactSensorFlag ) != 0 )
				{
					// Contact is a sensor
					if ( ( flags & b2_contactEnableSensorEvents ) != 0 )
					{
						if ( shapeA->isSensor )
						{
							b2SensorBeginTouchEvent event = { shapeIdA, shapeIdB };
							b2Array_Push( world->sensorBeginEventArray, event );
						}

						if ( shapeB->isSensor )
						{
							b2SensorBeginTouchEvent event = { shapeIdB, shapeIdA };
							b2Array_Push( world->sensorBeginEventArray, event );
						}
					}

					contactSim->simFlags &= ~b2_simStartedTouching;
					contact->flags |= b2_contactSensorTouchingFlag;
				}
				else
				{
					// Contact is solid
					if ( flags & b2_contactEnableContactEvents )
					{
						b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB };
						b2Array_Push( world->contactBeginArray, event );
					}

					B2_ASSERT( contactSim->manifold.pointCount > 0 );
					B2_ASSERT( contact->setIndex == b2_awakeSet );

					// Link first because this wakes colliding bodies and ensures the body sims
					// are in the correct place.
					contact->flags |= b2_contactTouchingFlag;
					b2LinkContact( world, contact );

					// Make sure these didn't change
					B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
					B2_ASSERT( contact->localIndex == localIndex );

					// Contact sim pointer may have become orphaned due to awake set growth,
					// so I just need to refresh it.
					B2_ASSERT( 0 <= localIndex && localIndex < awakeSet->contacts.count );
					contactSim = awakeSet->contacts.data + localIndex;

					contactSim->simFlags &= ~b2_simStartedTouching;

					b2AddContactToGraph( world, contactSim, contact );
					b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );
					contactSim = NULL;
				}
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim->simFlags &= ~b2_simStoppedTouching;

				if ( ( flags & b2_contactSensorFlag ) != 0 )
				{
					// Contact is a sensor
					contact->flags &= ~b2_contactSensorTouchingFlag;

					if ( ( flags & b2_contactEnableSensorEvents ) != 0 )
					{
						if ( shapeA->isSensor )
						{
							b2SensorEndTouchEvent event = { shapeIdA, shapeIdB };
							b2Array_Push( world->sensorEndEventArray, event );
						}

						if ( shapeB->isSensor )
						{
							b2SensorEndTouchEvent event = { shapeIdB, shapeIdA };
							b2Array_Push( world->sensorEndEventArray, event );
						}
					}
				}
				else
				{
					// Contact is solid
					contact->flags &= ~b2_contactTouchingFlag;

					if ( contact->flags & b2_contactEnableContactEvents )
					{
						b2ContactEndTouchEvent event = { shapeIdA, shapeIdB };
						b2Array_Push( world->contactEndArray, event );
					}

					B2_ASSERT( contactSim->manifold.pointCount == 0 );

					b2UnlinkContact( world, contact );
					int bodyIdA = contact->edges[0].bodyId;
					int bodyIdB = contact->edges[1].bodyId;

					b2AddNonTouchingContact( world, contact, contactSim );
					b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
					contact = NULL;
					contactSim = NULL;
				}
			}

			// Clear the smallest set bit
			bits = bits & ( bits - 1 );
		}
	}

	b2ValidateSolverSets( world );
	b2ValidateContacts( world );

	b2TracyCZoneEnd( contact_state );
	b2TracyCZoneEnd( collide );
}

static b2Shape* b2GetShape( b2World* world, b2ShapeId shapeId )
{
	int id = shapeId.index1 - 1;
	b2CheckIdAndRevision( world->shapeArray, id, shapeId.revision );
	b2Shape* shape = world->shapeArray + id;
	return shape;
}

void b2Shape_ComputeDistance( b2ShapeId shapeId, b2Vec2 target, float32* distance, b2Vec2* normal ) {
	b2World* world = b2GetWorld( shapeId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b2Shape* shape = b2GetShape( world, shapeId );
	b2Body* body = b2GetBody( world, shape->bodyId );
	b2Transform transform = b2GetBodyTransformQuick( world, body );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &target, 1, 0.0f );
	input.transformA = transform;
	input.transformB = b2Transform_identity;
	input.useRadii = true;

	b2DistanceCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	*distance = output.distance;
	*normal = b2Normalize( output.pointB - output.pointA );
}

bool operator<( b2ShapeId a, b2ShapeId b ) {
	return a.index1 < b.index1;
}

b2LiquidWorld::b2LiquidWorld( b2WorldId worldId )
{
	m_worldId = worldId;
	m_particleSystemList = NULL;
}

b2LiquidWorld::~b2LiquidWorld()
{
	m_worldId = b2_nullWorldId;

	while (m_particleSystemList)
	{
		DestroyParticleSystem(m_particleSystemList);
	}

	// Even though the block allocator frees them for us, for safety,
	// we should ensure that all buffers have been freed.
	// b2Assert(m_blockAllocator.GetNumGiantAllocations() == 0);
}

b2ParticleSystem* b2LiquidWorld::CreateParticleSystem(const b2ParticleSystemDef* def)
{
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return NULL;
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b2ParticleSystem));
	b2ParticleSystem* p = new (mem) b2ParticleSystem(def, this);

	// Add to world doubly linked list.
	p->m_prev = NULL;
	p->m_next = m_particleSystemList;
	if (m_particleSystemList)
	{
		m_particleSystemList->m_prev = p;
	}
	m_particleSystemList = p;

	return p;
}


void b2LiquidWorld::DestroyParticleSystem(b2ParticleSystem* p)
{
	b2Assert(m_particleSystemList != NULL);
	b2Assert(IsLocked() == false);
	if (IsLocked())
	{
		return;
	}

	// Remove world particleSystem list.
	if (p->m_prev)
	{
		p->m_prev->m_next = p->m_next;
	}

	if (p->m_next)
	{
		p->m_next->m_prev = p->m_prev;
	}

	if (p == m_particleSystemList)
	{
		m_particleSystemList = p->m_next;
	}

	p->~b2ParticleSystem();
	m_blockAllocator.Free(p, sizeof(b2ParticleSystem));
}

bool b2LiquidWorld::IsLocked() const
{
	b2World* world = b2GetWorldFromId( m_worldId );
	return world->locked;
}

void b2LiquidWorld::QueryAABB( b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( m_worldId );
	// B2_ASSERT( world->locked == false );
	// if ( world->locked )
	// {
	// 	Rtt_Log("b2LiquidWorld::QueryAABB locked!");
	// 	return;
	// }

	B2_ASSERT( b2AABB_IsValid( aabb ) );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Query( world->broadPhase.trees + i, aabb, filter.maskBits, TreeQueryCallback, &worldContext );
	}
}

void b2LiquidWorld::Step(float timeStep, int subStepCount )
{
	b2World* world = b2GetWorldFromId( m_worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	// Prepare to capture events
	// Ensure user does not access stale data if there is an early return
	b2Array_Clear( world->bodyMoveEventArray );
	b2Array_Clear( world->sensorBeginEventArray );
	b2Array_Clear( world->sensorEndEventArray );
	b2Array_Clear( world->contactBeginArray );
	b2Array_Clear( world->contactEndArray );
	b2Array_Clear( world->contactHitArray );

	world->profile = ( b2Profile ){ 0 };

	if ( timeStep == 0.0f )
	{
		// todo would be useful to still process collision while paused
		return;
	}

	b2TracyCZoneNC( world_step, "Step", b2_colorChartreuse, true );

	world->locked = true;
	world->activeTaskCount = 0;
	world->taskCount = 0;

	b2Timer stepTimer = b2CreateTimer();

	// Update collision pairs and create contacts
	{
		b2Timer timer = b2CreateTimer();
		b2UpdateBroadPhasePairs( world );
		world->profile.pairs = b2GetMilliseconds( &timer );
	}

	b2StepContext context = { 0 };
	context.world = world;
	context.dt = timeStep;
	context.subStepCount = b2MaxInt( 1, subStepCount );

	if ( timeStep > 0.0f )
	{
		context.inv_dt = 1.0f / timeStep;
		context.h = timeStep / context.subStepCount;
		context.inv_h = context.subStepCount * context.inv_dt;
	}
	else
	{
		context.inv_dt = 0.0f;
		context.h = 0.0f;
		context.inv_h = 0.0f;
	}

	world->inv_h = context.inv_h;

	// Hertz values get reduced for large time steps
	float contactHertz = b2MinFloat( world->contactHertz, 0.25f * context.inv_h );
	float jointHertz = b2MinFloat( world->jointHertz, 0.125f * context.inv_h );

	context.contactSoftness = b2MakeSoft( contactHertz, world->contactDampingRatio, context.h );
	context.staticSoftness = b2MakeSoft( 2.0f * contactHertz, world->contactDampingRatio, context.h );
	context.jointSoftness = b2MakeSoft( jointHertz, world->jointDampingRatio, context.h );

	context.restitutionThreshold = world->restitutionThreshold;
	context.enableWarmStarting = world->enableWarmStarting;

	// Update contacts
	{
		b2Timer timer = b2CreateTimer();
		b2Collide( &context );
		world->profile.collide = b2GetMilliseconds( &timer );
	}

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if ( context.dt > 0.0f )
	{
		b2Timer timer = b2CreateTimer();

		b2TimeStep step = {
			context.dt,
			context.inv_dt,
			0.0f,
			1,
			false
		};
		for (b2ParticleSystem* p = m_particleSystemList; p; p = p->GetNext())
		{
			p->Solve(step); // Particle Simulation
		}
		b2Solve( world, &context );
		world->profile.solve = b2GetMilliseconds( &timer );
	}

	world->locked = false;

	world->profile.step = b2GetMilliseconds( &stepTimer );

	B2_ASSERT( b2GetStackAllocation( &world->stackAllocator ) == 0 );

	// Ensure stack is large enough
	b2GrowStack( &world->stackAllocator );

	// Make sure all tasks that were started were also finished
	B2_ASSERT( world->activeTaskCount == 0 );

	b2TracyCZoneEnd( world_step );
}

inline b2ParticleSystem* b2LiquidWorld::GetParticleSystemList()
{
	return m_particleSystemList;
}

inline const b2ParticleSystem* b2LiquidWorld::GetParticleSystemList() const
{
	return m_particleSystemList;
}
