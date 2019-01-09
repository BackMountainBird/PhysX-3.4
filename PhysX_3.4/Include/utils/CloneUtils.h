#pragma once

#include "PxPhysicsAPI.h"

namespace physx
{

	/// Clone the geometry. Copied from GrbEvents.cpp of PhysX
	void clonePhysXGeometry(PxGeometryHolder & clone, const PxGeometryHolder & orig);

} // namespace physx