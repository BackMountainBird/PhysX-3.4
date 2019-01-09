//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<CctSweptOBB.h>
///	@path	~/PhysXSDk/Source/PhysXCharacterKinematic
///	@date	2018/03/05
///	@brief	.

#pragma once

#include "CctSweptVolume.h"

namespace physx
{
namespace Cct
{

	class SweptOBB : public SweptVolume
	{
	public:
						SweptOBB();
		virtual			~SweptOBB();

		virtual	void	computeTemporalBox(const SweepTest&, PxExtendedBounds3& box, const PxExtendedVec3& center, const PxVec3& direction) const override;

				PxVec3	mExtents;
	};

} // namespace Cct

}