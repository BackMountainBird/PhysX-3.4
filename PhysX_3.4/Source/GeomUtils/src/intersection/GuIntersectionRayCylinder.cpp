//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<GuIntersectionRayCylinder.cpp>
///	@path	~/GeomUtils/intersection
///	@date	2016/05/19
///	@brief	.

#include "GuIntersectionRayCylinder.h"
#include "GuCylinder.h"
#include "GuIntersectionRayPlane.h"
#include "PsMathUtils.h"
#include "PxPlane.h"

#include <cstdio>
#include "PsFoundation.h"

using namespace physx;

PxU32 Gu::intersectRayCylinder(const PxVec3& origin, const PxVec3& dir, const Gu::Cylinder& cylinder, PxReal s[2])
{
	PxU32 nIntersection = 0;
	auto cylinderDir = cylinder.computeDirection();
	auto cylinderDirLength = cylinderDir.normalize();
	if (Ps::isAlmostZero(cylinderDirLength))
	{
		// Invalid cylinder.
		return nIntersection;
	}
	PxReal radius2 = cylinder.radius * cylinder.radius;
	// Find the intersection of line and cylinder side surface
	{
		PxVec3 ndir, tdir;
		Ps::decomposeVector(ndir, tdir, dir, cylinderDir);
		PxReal a = tdir.dot(tdir);
		if (a > PX_EPS_REAL)
		{
			PxReal halfInvA = 0.5f / a;
			PxVec3 nDeltaP, tDeltaP, DeltaP;
			DeltaP = origin - cylinder.p0;
			Ps::decomposeVector(nDeltaP, tDeltaP, DeltaP, cylinderDir);
			// The ray not parallel the cylinder direction there should have to intersection
			PxReal b = 2.0f * tDeltaP.dot(tdir);
			PxReal c = tDeltaP.magnitudeSquared() - radius2;
			PxReal d = b * b - 4.0f * a * c;
			PxReal DeltaPDotCylinderD = DeltaP.dot(cylinderDir);
			PxReal dirDotCylinderD = dir.dot(cylinderDir);
			if (d <= PX_EPS_REAL && d >= 0.0f)
			{
				PxReal t = -b * halfInvA;
				if (t >= 0.0f)
				{
					PxReal distToBottom = DeltaPDotCylinderD + t * dirDotCylinderD;
					if (distToBottom >= 0.0f || distToBottom <= cylinderDirLength)
					{
						s[nIntersection++] = t;
					}
				}
			}
			else if (d > PX_EPS_REAL)
			{
				PxReal sqrtd = sqrtf(d);
				PxReal t = (-b + sqrtd) * halfInvA;
				if (t >= 0.0f)
				{
					PxReal distToBottom = DeltaPDotCylinderD + t * dirDotCylinderD;
					if (distToBottom >= 0.0f && distToBottom <= cylinderDirLength)
					{
						s[nIntersection++] = t;
					}
				}
				t = (-b - sqrtd) *halfInvA;
				if (t >= 0.0f)
				{
					PxReal distToBottom = DeltaPDotCylinderD + t * dirDotCylinderD;
					if (distToBottom >= 0.0f || distToBottom <= cylinderDirLength)
					{
						s[nIntersection++] = t;
					}
				}
				if (nIntersection == 2)
				{
					// We already get all intersections
					return nIntersection;
				}
			}
		}
	} // Find the intersection of line and cylinder side surface
	// Find the intersection of line and cylinder up/buttom surface
	{
		PxPlane plane0(cylinder.p0, cylinderDir);
		float t = -1.0f;
		PxVec3 pointOnPlane;
		intersectRayPlane(origin, dir, plane0, t, &pointOnPlane);
		if (t >= 0.0f)
		{
			PxReal tmp = (pointOnPlane - cylinder.p0).magnitudeSquared();
			if (tmp < radius2)
			{
				s[nIntersection++] = t;
				if (nIntersection == 2)
					return nIntersection;
			}
		}
		PxPlane plane1(cylinder.p1, cylinderDir);
		t = -1.0f;
		intersectRayPlane(origin, dir, plane1, t, &pointOnPlane);
		if (t >= 0.0f)
		{
			PxReal tmp = (pointOnPlane - cylinder.p1).magnitudeSquared();
			if (tmp < radius2)
			{
				s[nIntersection++] = t;
			}
		}
	}
	return nIntersection;
}
