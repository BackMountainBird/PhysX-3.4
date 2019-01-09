//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<CctRotatableController.h>
///	@path	~/PhysXSDk/Source/PhysXCharacterKinematic
///	@date	2018/03/01
///	@brief	.


#include "PxController.h"
#include "CctRotatableController.h"
#include "CctCharacterControllerManager.h"
#include "PxCapsuleGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PxScene.h"
#include "PxPhysics.h"
#include "PxExtensionsAPI.h"

#include <cmath>

using namespace physx;
using namespace Cct;

static PX_FORCE_INLINE float CCTtoProxyRadius(float r, PxF32 coeff)	{ return r * coeff;			}
static PX_FORCE_INLINE float CCTtoProxyHeight(float h, PxF32 coeff)	{ return 0.5f * h * coeff;	}
static PX_FORCE_INLINE PxVec3 CCTtoProxyExtents(PxF32 halfHeight, PxF32 halfSideExtent, PxF32 halfForwardExtent, PxF32 coeff)
{
	// PT: because we now orient the box CCT using the same quat as for capsules...
	// i.e. the identity quat corresponds to a up dir = 1,0,0 (which is like the worst choice we could have made, of course)
	return PxVec3(halfHeight * coeff, halfSideExtent * coeff, halfForwardExtent * coeff);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RotatableController::RotatableController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* s) : Controller(desc, s)
{
	const PxRotatableControllerDesc& cc = static_cast<const PxRotatableControllerDesc&>(desc);

	mOrientation		= cc.orientation;
	mRadius				= cc.radius;
	mHeight				= cc.height;
	mHalfBoxHeight		= cc.halfBoxHeight;
	mHalfSideExtent		= cc.halfSideExtent;
	mHalfForwardExtent	= cc.halfForwardExtent;
	mClimbingMode		= cc.climbingMode;
	mShapeType			= cc.shapeType;
	mType				= PxControllerShapeType::eROTATABLE;

	// Create kinematic actor under the hood
	switch (mShapeType)
	{
	case PxRotatableShapeType::eCapsule:
		{
			PxCapsuleGeometry capsGeom;
			capsGeom.radius		= CCTtoProxyRadius(mRadius, mProxyScaleCoeff);
			capsGeom.halfHeight	= CCTtoProxyHeight(mHeight, mProxyScaleCoeff);
			createRotatableProxyActor(sdk, capsGeom, *desc.material);
		}
		break;
	case PxRotatableShapeType::eOBB:
		{
			PxBoxGeometry boxGeom;
			boxGeom.halfExtents = CCTtoProxyExtents(mHalfBoxHeight, mHalfSideExtent, mHalfForwardExtent, mProxyScaleCoeff);
			createRotatableProxyActor(sdk, boxGeom, *desc.material);
		}
		break;
	default:
		PX_ASSERT(false);
		break;
	}
}

RotatableController::~RotatableController()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RotatableController::invalidateCache()
{
	if(mManager->mLockingEnabled)
	{
		mWriteLock.lock();
		mCctModule.voidTestCache();
		mWriteLock.unlock();
	}
	else
	{
		mCctModule.voidTestCache();
	}	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::getWorldBox(PxExtendedBounds3& box) const
{
	switch (mShapeType)
	{
	case PxRotatableShapeType::eCapsule:
		setCenterExtents(box, mPosition, PxVec3(mRadius, mRadius + mHeight * 0.5f, mRadius));
		break;
	case PxRotatableShapeType::eOBB:
		{
			float cosO = PxCos(mOrientation);
			float sinO = PxSin(mOrientation);
			float maxX = PxMax(PxAbs(mHalfSideExtent * cosO + mHalfForwardExtent * sinO), PxAbs(mHalfSideExtent * cosO - mHalfForwardExtent * sinO));
			float maxZ = PxMax(PxAbs(mHalfSideExtent * sinO + mHalfForwardExtent * cosO), PxAbs(mHalfSideExtent * sinO - mHalfForwardExtent * cosO));
			setCenterExtents(box, mPosition, PxVec3(maxX, mHalfBoxHeight, maxZ));
		}
		break;
	default:
		PX_ASSERT(false);
		break;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setPosition(const PxExtendedVec3& position)
{
	// Set position for CCT
	mPosition = position;
	// Update kinematic proxy global pose
	return updateKinematicPose();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setOrientation(PxF32 orientation)
{
	// Set orientation for CCT
	mOrientation = std::remainder(orientation, PxTwoPi);

	if (mShapeType == PxRotatableShapeType::eCapsule)
		return true;

	// Update kinematic proxy global pose
	return updateKinematicPose();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setRadius(PxF32 r)
{
	// Set radius for CCT volume
	mRadius = r;

	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setHeight(PxF32 h)
{
	// Set height for CCT volume
	mHeight = h;

	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setHalfBoxHeight(PxF32 h)
{
	// Set half height for CCT volume
	mHalfBoxHeight = h;

	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setHalfSideExtent(PxF32 s)
{
	// Set half side extent for CCT volume
	mHalfSideExtent = s;

	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setHalfForwardExtent(PxF32 f)
{
	// Set half forward extent for CCT volume
	mHalfForwardExtent = f;

	return updateKinematicProxy();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setClimbingMode(PxCapsuleClimbingMode::Enum mode)
{
	if(mode>=PxCapsuleClimbingMode::eLAST)
		return false;
	mClimbingMode = mode;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::setShapeType(PxRotatableShapeType::Enum shapeType)
{
	if (shapeType == mShapeType)
		return true;
	if (mKineActor->getNbShapes() != 1)
		return false;
	PxShape* shapeBuffer[1] = { nullptr };
	if (mKineActor->getShapes(shapeBuffer, 1, 0) != 1)
		return false;
	if (!shapeBuffer[0])
		return false;
	if (shapeBuffer[0]->getNbMaterials() != 1)
		return false;
	PxMaterial* shapeMaterials[1] = { nullptr };
	if (shapeBuffer[0]->getMaterials(shapeMaterials, 1) != 1)
		return false;
	if (!shapeMaterials[0])
		return false;
	PxShape* newShape{ nullptr };
	switch (shapeType)
	{
		case PxRotatableShapeType::eCapsule:
			// Change from OBB to Capsule
			{
				PxCapsuleGeometry capsGeom;
				capsGeom.radius = CCTtoProxyRadius(mRadius, mProxyScaleCoeff);
				capsGeom.halfHeight = CCTtoProxyHeight(mHeight, mProxyScaleCoeff);
				newShape = mKineActor->createShape(capsGeom, *(shapeMaterials[0]));
				if (newShape == nullptr)
					return false;
				mPosition += mUserParams.mUpDirection * (mRadius + mHeight * 0.5f - mHalfBoxHeight);
			}
			break;
		case PxRotatableShapeType::eOBB:
			// Change from Capsule to OBB 
			{
				PxBoxGeometry boxGeom;
				boxGeom.halfExtents = CCTtoProxyExtents(mHalfBoxHeight, mHalfSideExtent, mHalfForwardExtent, mProxyScaleCoeff);
				newShape = mKineActor->createShape(boxGeom, *(shapeMaterials[0]));
				if (newShape == nullptr)
					return false;
				mPosition += mUserParams.mUpDirection * (mHalfBoxHeight - mRadius - mHeight * 0.5f);
			}
			break;
		default:
			PX_ASSERT(false);
			break;
	}
	newShape->setSimulationFilterData(shapeBuffer[0]->getSimulationFilterData());
	newShape->setQueryFilterData(shapeBuffer[0]->getQueryFilterData());
	mManager->mCCTShapes.erase(shapeBuffer[0]);
	mManager->mCCTShapes.insert(newShape);
	mKineActor->detachShape(*(shapeBuffer[0]));
	mShapeType = shapeType;
	updateKinematicPose();
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxExtendedVec3 RotatableController::getFootPosition() const
{
	PxExtendedVec3 groundPosition = mPosition;														// Middle of the CCT
	switch (mShapeType)
	{
	case PxRotatableShapeType::eCapsule:
		groundPosition -= mUserParams.mUpDirection * (mUserParams.mContactOffset + mRadius + mHeight * 0.5f);	// Ground
		break;
	case PxRotatableShapeType::eOBB:
		groundPosition -= mUserParams.mUpDirection * (mHalfBoxHeight + mUserParams.mContactOffset);	// Ground
		break;
	default:
		PX_ASSERT(false);
		break;
	}
	return groundPosition;
}

bool RotatableController::setFootPosition(const PxExtendedVec3& position)
{
	PxExtendedVec3 centerPosition = position;
	switch (mShapeType)
	{
	case PxRotatableShapeType::eCapsule:
		centerPosition += mUserParams.mUpDirection * (mUserParams.mContactOffset + mRadius + mHeight * 0.5f);
		break;
	case PxRotatableShapeType::eOBB:
		centerPosition += mUserParams.mUpDirection * (mHalfBoxHeight + mUserParams.mContactOffset);
		break;
	default:
		PX_ASSERT(false);
		break;
	}
	return setPosition(centerPosition);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RotatableController::getCapsule(PxExtendedCapsule& capsule) const
{
	PX_ASSERT(PxRotatableShapeType::eCapsule == mShapeType);
	// PT: TODO: optimize this
	PxExtendedVec3 p0 = mPosition;
	PxExtendedVec3 p1 = mPosition;
	const PxVec3 extents = mUserParams.mUpDirection*mHeight*0.5f;
	p0 -= extents;
	p1 += extents;

	capsule.p0		= p0;
	capsule.p1		= p1;
	capsule.radius	= mRadius;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RotatableController::getOBB(PxExtendedBox& obb) const
{
	PX_ASSERT(PxRotatableShapeType::eOBB == mShapeType);

	// PT: TODO: optimize this
	obb.center = mPosition;
	obb.extents = CCTtoProxyExtents(mHalfBoxHeight, mHalfSideExtent, mHalfForwardExtent, mProxyScaleCoeff);
	obb.rot = GetRotateQuat(mOrientation, mUserParams.mUpDirection, mUserParams.mQuatFromUp);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::updateKinematicProxy()
{
	if (mKineActor)
	{
		switch (mShapeType)
		{
		case PxRotatableShapeType::eCapsule:
			{
				PxShape* shape = getKineShape();

				PX_ASSERT(shape->getGeometryType() == PxGeometryType::eCAPSULE);
				PxCapsuleGeometry cg;
				shape->getCapsuleGeometry(cg);

				cg.halfHeight = CCTtoProxyHeight(mHeight, mProxyScaleCoeff);
				cg.radius = CCTtoProxyRadius(mRadius, mProxyScaleCoeff);
				shape->setGeometry(cg);
			}
			break;
		case PxRotatableShapeType::eOBB:
			{
				PxShape* shape = getKineShape();

				PX_ASSERT(shape->getGeometryType() == PxGeometryType::eBOX);
				PxBoxGeometry bg;
				shape->getBoxGeometry(bg);

				bg.halfExtents = CCTtoProxyExtents(mHalfBoxHeight, mHalfSideExtent, mHalfForwardExtent, mProxyScaleCoeff);
				shape->setGeometry(bg);
			}
			break;
		default:
			PX_ASSERT(false);
			break;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::updateKinematicPose()
{
	if (mKineActor)
	{
		switch (mShapeType)
		{
		case PxRotatableShapeType::eCapsule:
			{
				PxTransform targetPose = mKineActor->getGlobalPose();
				targetPose.p = toVec3(mPosition);  // LOSS OF ACCURACY
				targetPose.q = mUserParams.mQuatFromUp;
				mKineActor->setKinematicTarget(targetPose);
			}
			break;
		case PxRotatableShapeType::eOBB:
			{
				PxTransform targetPose = mKineActor->getGlobalPose();
				targetPose.p = toVec3(mPosition);  // LOSS OF ACCURACY
				targetPose.q = GetRotateQuat(mOrientation, mUserParams.mUpDirection, mUserParams.mQuatFromUp);
				mKineActor->setKinematicTarget(targetPose);
			}
			break;
		default:
			PX_ASSERT(false);
			break;
		}
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RotatableController::createRotatableProxyActor(PxPhysics& sdk, const PxGeometry& geometry, const PxMaterial& material)
{
	// PT: we don't disable raycasting or CD because:
	// - raycasting is needed for visibility queries (the SDK otherwise doesn't know about the CCTS)
	// - collision is needed because the only reason we create actors there is to handle collisions with dynamic shapes
	// So it's actually wrong to disable any of those.

	PxTransform globalPose;
	globalPose.p = toVec3(mPosition);	// LOSS OF ACCURACY
	globalPose.q = mUserParams.mQuatFromUp;

	if (mShapeType == PxRotatableShapeType::eOBB)
	{
		globalPose.q = PxQuat(mOrientation, mUserParams.mUpDirection) * globalPose.q;
	}

	mKineActor = sdk.createRigidDynamic(globalPose);
	if (!mKineActor)
		return false;

	mKineActor->createShape(geometry, material);
	mKineActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	PxRigidBodyExt::updateMassAndInertia(*mKineActor, mProxyDensity);
	mScene->addActor(*mKineActor);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RotatableController::resize(PxReal height)
{
	if (mShapeType != PxRotatableShapeType::eCapsule)
		return;

	const float oldHeight = getHeight();
	setHeight(height);

	const float delta = height - oldHeight;
	PxExtendedVec3 pos = getPosition();
	pos += mUserParams.mUpDirection * delta * 0.5f;
	setPosition(pos);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
