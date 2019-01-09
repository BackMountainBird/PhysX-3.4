//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<GuSweepCylinderCylinder.h>
///	@path	~/GeomUtils/Sweep
///	@date	2016/05/19
///	@brief	.

#pragma once

#include "CmPhysXCommon.h"
#include "foundation/PxVec3.h"

namespace physx
{
	namespace Gu
	{
		class Cylinder;

		bool sweepCylinderCylinder_ParallelSpecial(const Cylinder& lss0, const Cylinder& lss1, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& ip, PxVec3& normal, PxU32 inHintFlags, PxU16& outHintFlags);

	} // namespace Gu

} // namespace physx
