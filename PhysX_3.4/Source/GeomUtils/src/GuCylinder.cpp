//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<GuCylinder.h>
///	@path	~/GeomUtils
///	@date	2016/05/19
///	@brief	.

#include "PsIntrinsics.h"
#include "PsMathUtils.h"
#include "GuInternal.h"
#include "GuBox.h"
#include "GuCylinder.h"

using namespace physx;

/**
*	Computes an OBB surrounding the cylinder.
*	\param		box		[out] the OBB
*/
void Gu::computeBoxAroundCylinder(const Gu::Cylinder& cylinder, Gu::Box& box)
{
	// Box center = center of the two capsule's endpoints
	box.center = cylinder.computeCenter();

	// Box extents
	const PxF32 d = (cylinder.p0 - cylinder.p1).magnitude();
	box.extents.x = d * 0.5f;
	box.extents.y = cylinder.radius;
	box.extents.z = cylinder.radius;

	// Box orientation
	if(d==0.0f)
	{
		box.rot = PxMat33(PxIdentity);
	}
	else
	{
		PxVec3 dir, right, up;
		Ps::computeBasis(cylinder.p0, cylinder.p1, dir, right, up);
		box.setAxes(dir, right, up);
	}
}
