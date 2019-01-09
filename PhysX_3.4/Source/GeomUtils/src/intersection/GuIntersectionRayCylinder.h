//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<GuIntersectionRayCylinder.h>
///	@path	~/GeomUtils/intersection
///	@date	2016/05/19
///	@brief	.

#pragma once

#include "CmPhysXCommon.h"

namespace physx
{
	namespace Gu
	{
		class Cylinder;
		PxU32 intersectRayCylinder(const PxVec3& origin, const PxVec3& dir, const Gu::Cylinder& cylinder, PxReal s[2]);
	} // namespace Gu

} // namespace physx