//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<GuSweepCylinderCylinder.cpp>
///	@path	~/GeomUtils/Sweep
///	@date	2016/05/19
///	@brief	.


#include "GuSweepCylinderCylinder.h"
#include "GuCylinder.h"
#include "GuDistancePointSegment.h"
#include "GuDistanceSegmentSegment.h"
#include "GuIntersectionRayCylinder.h"
#include "PxQueryReport.h"
#include "PxTriangle.h"

#include <cstdio>
#include "PsFoundation.h"
#include "PsMathUtils.h"

using namespace physx;
using namespace Gu;

#define LOCAL_EPSILON 0.00001f	// PT: this value makes the 'basicAngleTest' pass. Fails because of a ray almost parallel to a triangle

static void edgeEdgeDist(PxVec3& x, PxVec3& y,				// closest points
						 const PxVec3& p, const PxVec3& a,	// seg 1 origin, vector
						 const PxVec3& q, const PxVec3& b)	// seg 2 origin, vector
{
	const PxVec3 T = q - p;
	const PxReal ADotA = a.dot(a);
	const PxReal BDotB = b.dot(b);
	const PxReal ADotB = a.dot(b);
	const PxReal ADotT = a.dot(T);
	const PxReal BDotT = b.dot(T);

	// t parameterizes ray (p, a)
	// u parameterizes ray (q, b)

	// Compute t for the closest point on ray (p, a) to ray (q, b)
	const PxReal Denom = ADotA*BDotB - ADotB*ADotB;

	PxReal t;
	if(Denom!=0.0f)	
	{
		t = (ADotT*BDotB - BDotT*ADotB) / Denom;

		// Clamp result so t is on the segment (p, a)
				if(t<0.0f)	t = 0.0f;
		else	if(t>1.0f)	t = 1.0f;
	}
	else
	{
		t = 0.0f;
	}

	// find u for point on ray (q, b) closest to point at t
	PxReal u;
	if(BDotB!=0.0f)
	{
		u = (t*ADotB - BDotT) / BDotB;

		// if u is on segment (q, b), t and u correspond to closest points, otherwise, clamp u, recompute and clamp t
		if(u<0.0f)
		{
			u = 0.0f;
			if(ADotA!=0.0f)
			{
				t = ADotT / ADotA;

						if(t<0.0f)	t = 0.0f;
				else	if(t>1.0f)	t = 1.0f;
			}
			else
			{
				t = 0.0f;
			}
		}
		else if(u > 1.0f)
		{
			u = 1.0f;
			if(ADotA!=0.0f)
			{
				t = (ADotB + ADotT) / ADotA;

						if(t<0.0f)	t = 0.0f;
				else	if(t>1.0f)	t = 1.0f;
			}
			else
			{
				t = 0.0f;
			}
		}
	}
	else
	{
		u = 0.0f;

		if(ADotA!=0.0f)
		{
			t = ADotT / ADotA;

					if(t<0.0f)	t = 0.0f;
			else	if(t>1.0f)	t = 1.0f;
		}
		else
		{
			t = 0.0f;
		}
	}

	x = p + a * t;
	y = q + b * u;
}

static bool rayQuad(const PxVec3& orig, const PxVec3& dir, const PxVec3& vert0, const PxVec3& vert1, const PxVec3& vert2, PxReal& t, PxReal& u, PxReal& v, bool cull)
{
	// Find vectors for two edges sharing vert0
	const PxVec3 edge1 = vert1 - vert0;
	const PxVec3 edge2 = vert2 - vert0;

	// Begin calculating determinant - also used to calculate U parameter
	const PxVec3 pvec = dir.cross(edge2);

	// If determinant is near zero, ray lies in plane of triangle
	const PxReal det = edge1.dot(pvec);

	if(cull)
	{
		if(det<LOCAL_EPSILON)
			return false;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = orig - vert0;

		// Calculate U parameter and test bounds
		u = tvec.dot(pvec);
		if(u<0.0f || u>det)
			return false;

		// Prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		v = dir.dot(qvec);
		if(v<0.0f || v>det)
			return false;

		// Calculate t, scale parameters, ray intersects triangle
		t = edge2.dot(qvec);
		const PxReal oneOverDet = 1.0f / det;
		t *= oneOverDet;
		u *= oneOverDet;
		v *= oneOverDet;
	}
	else
	{
		// the non-culling branch
		if(det>-LOCAL_EPSILON && det<LOCAL_EPSILON)
			return false;
		const PxReal oneOverDet = 1.0f / det;

		// Calculate distance from vert0 to ray origin
		const PxVec3 tvec = orig - vert0;

		// Calculate U parameter and test bounds
		u = (tvec.dot(pvec)) * oneOverDet;
		if(u<0.0f || u>1.0f)
			return false;

		// prepare to test V parameter
		const PxVec3 qvec = tvec.cross(edge1);

		// Calculate V parameter and test bounds
		v = (dir.dot(qvec)) * oneOverDet;
		if(v<0.0f || v>1.0f)
			return false;

		// Calculate t, ray intersects triangle
		t = (edge2.dot(qvec)) * oneOverDet;
	}
	return true;
}

bool Gu::sweepCylinderCylinder_ParallelSpecial(const Cylinder& lss0, const Cylinder& lss1, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& ip, PxVec3& normal, PxU32 inHintFlags, PxU16& outHintFlags)
{
	auto cylinderDir0 = lss0.computeDirection();
	auto cylinderDir1 = lss1.computeDirection();
	PX_ASSERT(Ps::parallelDirection(cylinderDir0, cylinderDir1));
	const PxReal halfMag = (cylinderDir0.magnitude() + cylinderDir1.magnitude()) * 0.5f;
	const PxVec3 center0 = lss0.computeCenter();
	const PxVec3 center1 = lss1.computeCenter();
	const PxVec3 normDir = cylinderDir1.getNormalized();
	Cylinder tmpCylinder(center1 - normDir*halfMag, center1 + normDir*halfMag, lss0.radius + lss1.radius);

	if(!(inHintFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
	{
		// PT: test if shapes initially overlap

		// PT: It would be better not to use the same code path for spheres and capsules. The segment-segment distance
		// function doesn't work for degenerate capsules so we need to test all combinations here anyway.
		bool initialOverlapStatus = false;
		if (distancePointSegmentSquared(tmpCylinder, center0) < tmpCylinder.radius*tmpCylinder.radius)
		{
			if (fabsf((center0 - center1).dot(normDir)) < halfMag)
				initialOverlapStatus = true;
		}

		if(initialOverlapStatus)
		{
			min_dist	= 0.0f;
			normal		= -dir;
			outHintFlags = PxHitFlag::eDISTANCE | PxHitFlag::eNORMAL;
			return true;
		}
	}

	// Do ray intersection
	bool Status;
	{
		PxReal s[2];
		auto n = intersectRayCylinder(center0, dir, tmpCylinder, s);
		if (n == 0)
		{
			Status = false;
		}
		else
		{
			PxReal w = s[0];
			if (n == 2)
			{
				w = w < s[1] ? w : s[1];
			}
			if (w > length)
			{
				Status = false;
			}
			else
			{
				min_dist = w;
				Status = true;
			}
		}
	}

	if(Status)
	{
		outHintFlags = PxHitFlag::eDISTANCE;
		if(inHintFlags & (PxU32)(PxHitFlag::ePOSITION|PxHitFlag::eNORMAL))
		{
			const PxVec3 p00 = lss0.p0 - min_dist * dir;
			const PxVec3 p01 = lss0.p1 - min_dist * dir;

			const PxVec3 edge0 = p01 - p00;
			const PxVec3 edge1 = lss1.computeDirection();

			PxVec3 x, y;
			edgeEdgeDist(x, y, p00, edge0, lss1.p0, edge1);

			PxReal xyLen = (x - y).magnitude();
			if(inHintFlags & PxHitFlag::eNORMAL)
			{
				if (xyLen < tmpCylinder.radius - PX_EPS_REAL)
				{
					// Bottom face collide.
					normal = y - center1;
				}
				else
				{
					// Side face collide.
					normal = x - y;
				}
				if (normal.normalize() < PX_EPS_REAL)
				{
					normal = -dir;
					normal.normalize();
				}
				outHintFlags |= PxHitFlag::eNORMAL;
			}

			if(inHintFlags & PxHitFlag::ePOSITION)
			{
				if (xyLen < tmpCylinder.radius - PX_EPS_REAL)
				{
					// Bottom face collide.
					PxReal halfXYLen = 0.5f * xyLen;
					ip = ((lss1.radius - halfXYLen) * x + (lss0.radius - halfXYLen) * y) / (tmpCylinder.radius - xyLen);
				}
				else
				{
					// Side face collide.
					ip = (lss1.radius*x + lss0.radius*y) / tmpCylinder.radius;
				}
				outHintFlags |= PxHitFlag::ePOSITION;
			}
		}
	}
	return Status;
}
