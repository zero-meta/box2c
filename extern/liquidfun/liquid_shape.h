#ifndef B2_LIQUID_SHAPE
#define B2_LIQUID_SHAPE

#include "box2d/box2d.h"
#include "settings.h"

class b2LiquidShape {
public:
	// b2LiquidShape();
	// ~b2LiquidShape() {};
	b2ShapeType GetType() const;
	void SetAsBox(float32 hx, float32 hy);
	void SetAsBox(float32 hx, float32 hy, const b2Vec2& center, float32 angle);
	void SetAsCircle(const b2Vec2 &center, float32 radius);
	bool Set(const b2Vec2* points, int32 count);
	b2AABB ComputeAABB(b2Transform transform) const;
	bool TestPoint(b2Transform transform, b2Vec2 point) const;

	union
	{
		b2Capsule capsule;
		b2Circle circle;
		b2Polygon polygon;
		b2Segment segment;
		b2SmoothSegment smoothSegment;
	};
	b2ShapeType type;
};

inline b2ShapeType b2LiquidShape::GetType() const
{
	return type;
}


#endif
