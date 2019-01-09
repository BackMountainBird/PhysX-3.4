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

#pragma once

/** \addtogroup geomutils
@{
*/

#include "GuSegment.h"

namespace physx
{
	namespace Gu
	{

		/**
		\brief Represents a cylinder.
		*/
		class Cylinder : public Segment
		{
		public:
			/**
			\brief Constructor
			*/
			PX_INLINE Cylinder()
			{
			}

			/**
			\brief Constructor

			\param seg Line segment to create cylinder from.
			\param _radius Radius of the cylinder.
			*/
			PX_INLINE Cylinder(const Segment& seg, PxF32 _radius) : Segment(seg), radius(_radius)
			{
			}

			/**
			\brief Constructor

			\param p0 First segment point
			\param p1 Second segment point
			\param _radius Radius of the cylinder.
			*/
			PX_INLINE Cylinder(const PxVec3& _p0, const PxVec3& _p1, PxF32 _radius) : Segment(_p0, _p1), radius(_radius)
			{
			}

			/**
			\brief Destructor
			*/
			PX_INLINE ~Cylinder()
			{
			}

			PxF32	radius;
		};
	} // namespace Gu

} // namespace physx

/** @} */