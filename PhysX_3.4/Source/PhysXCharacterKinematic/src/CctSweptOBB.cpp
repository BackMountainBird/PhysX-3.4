//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<CctSweptOBB.cpp>
///	@path	~/PhysXSDk/Source/PhysXCharacterKinematic
///	@date	2018/03/05
///	@brief	.

#include "CctSweptOBB.h"
#include "CctCharacterController.h"
#include "CctUtils.h"

using namespace physx;
using namespace Cct;

SweptOBB::SweptOBB()
{
	mType = SweptVolumeType::eOBB;
}

SweptOBB::~SweptOBB()
{
}

void SweptOBB::computeTemporalBox(const SweepTest& test, PxExtendedBounds3& box, const PxExtendedVec3& center, const PxVec3& direction) const
{
	const float radius = PxMax(mExtents.y, mExtents.z);
	const float height = 2.0f * mExtents.x;
	Cct::computeTemporalBox(box, radius, height, test.mUserParams.mContactOffset, test.mUserParams.mMaxJumpHeight, test.mUserParams.mUpDirection, center, direction);
}
