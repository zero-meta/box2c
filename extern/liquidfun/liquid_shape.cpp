#include "liquid_shape.h"

void b2LiquidShape::SetAsBox( float32 hx, float32 hy ) {
	polygon = b2MakeBox(hx, hy);
	type = b2_polygonShape;
}

void b2LiquidShape::SetAsBox( float32 hx, float32 hy, const b2Vec2 &center, float32 angle ) {
	polygon = b2MakeOffsetBox( hx, hy, center, angle );
	type = b2_polygonShape;
}

void b2LiquidShape::SetAsCircle( const b2Vec2 &center, float32 radius ) {
	circle = { center, radius };
	type = b2_circleShape;
}

bool b2LiquidShape::Set( const b2Vec2 *points, int32 count ) {
	b2Hull hull = b2ComputeHull( points, count );
	bool result = b2ValidateHull( &hull );
	if (result) {
		polygon = b2MakePolygon( &hull, 0.0f );
	}
	type = b2_polygonShape;
	return result;
}

b2AABB b2LiquidShape::ComputeAABB( b2Transform transform ) const {
	switch ( type )
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleAABB( &capsule, transform );

		case b2_circleShape:
			return b2ComputeCircleAABB( &circle, transform );

		case b2_polygonShape:
			return b2ComputePolygonAABB( &polygon, transform );

		default:
			return {b2Vec2_zero, b2Vec2_zero};
	}
}

bool b2LiquidShape::TestPoint( b2Transform transform, b2Vec2 point ) const {
	b2Vec2 localPoint = b2InvTransformPoint( transform, point );
	switch ( type )
	{
		case b2_capsuleShape:
			return b2PointInCapsule( localPoint, &capsule );

		case b2_circleShape:
			return b2PointInCircle( localPoint, &circle );

		case b2_polygonShape:
			return b2PointInPolygon( localPoint, &polygon );

		default:
			return false;
	}
}
