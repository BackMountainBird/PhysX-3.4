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


#pragma once

/* Exclude from documentation */
/** \cond */

#include "CctController.h"
#include "PxRotatableController.h"

namespace physx
{

class PxPhysics;

namespace Cct
{

	class RotatableController : public PxRotatableController, public Controller
	{
	public:
													RotatableController(const PxControllerDesc& desc, PxPhysics& sdk, PxScene* scene);
		virtual										~RotatableController();

		// Controller
		virtual	PxF32								getHalfHeightInternal()				const override					{ return (mShapeType == PxRotatableShapeType::eCapsule) ? mRadius + mHeight * 0.5f : mHalfBoxHeight; };
		virtual	bool								getWorldBox(PxExtendedBounds3& box) const override;
		virtual	PxController*						getPxController()					override						{ return this;							}
		//~Controller

		// PxController
		virtual	PxControllerShapeType::Enum			getType()							const override					{ return mType;							}
		virtual void								release()							override						{ releaseInternal();					}
		virtual	PxControllerCollisionFlags			move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles) override;
		virtual	PxControllerCollisionFlags			moveWithRotate(const PxVec3& disp, PxF32 minDist, PxF32 rotAngle, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles=NULL) override;
		virtual	bool								setPosition(const PxExtendedVec3& position) override;
		virtual	const PxExtendedVec3&				getPosition()						const override					{ return mPosition;						}
		virtual	bool								setFootPosition(const PxExtendedVec3& position);
		virtual	PxExtendedVec3						getFootPosition()					const override;
		virtual	PxRigidDynamic*						getActor()							const override					{ return mKineActor;					}
		virtual	void								setStepOffset(const float offset)	override						{ if(offset>0.0f)
																															mUserParams.mStepOffset = offset;	}
		virtual	PxF32								getStepOffset()						const override					{ return mUserParams.mStepOffset;		}
		virtual	void								setNonWalkableMode(PxControllerNonWalkableMode::Enum flag) override	{ mUserParams.mNonWalkableMode = flag;	}
		virtual	PxControllerNonWalkableMode::Enum	getNonWalkableMode()				const override					{ return mUserParams.mNonWalkableMode;	}
		virtual PxF32								getContactOffset()					const override					{ return mUserParams.mContactOffset;	}
		virtual	void								setContactOffset(PxF32 offset)		override						{ if(offset>0.0f)
																															mUserParams.mContactOffset = offset;}
		virtual PxVec3								getUpDirection()					const override					{ return mUserParams.mUpDirection;		}
		virtual	void								setUpDirection(const PxVec3& up)	override						{ setUpDirectionInternal(up);			}
		virtual PxF32								getSlopeLimit()						const override					{ return mUserParams.mSlopeLimit;		}
		virtual void								setSlopeLimit(PxF32 slopeLimit)		override						{ if(slopeLimit>0.0f)
																															mUserParams.mSlopeLimit = slopeLimit;}
		virtual PxControllerOptimizeFlags			getOptimizeFlag()					const override					{ return mUserParams.mOptimizeFlags;	}
		virtual void								setOptimizeFlag(PxControllerOptimizeFlags flags) override			{ mUserParams.mOptimizeFlags = flags;	}
		virtual	void								invalidateCache()					override;
		virtual	PxScene*							getScene()							override						{ return mScene;						}
		virtual	void*								getUserData()						const override					{ return mUserData;						}
		virtual	void								setUserData(void* userData)			override						{ mUserData = userData;					}
		virtual	void								getState(PxControllerState& state)	const override					{ return getInternalState(state);		}
		virtual	void								getStats(PxControllerStats& stats)	const override					{ return getInternalStats(stats);		}
		virtual	void								resize(PxReal height)				override;

		virtual		void							setNewCollision(bool val) override { mUserParams.mNewCollision = val; };
		virtual		bool							getNewCollision(bool val) const override {PX_UNUSED(val); return mUserParams.mNewCollision; };
		virtual		void							setNewCollisionFriction(PxReal val) override { mUserParams.mNewCollisionFricition = val; };
		virtual		PxReal							getNewCollisionFriction() const override { return mUserParams.mNewCollisionFricition; };
		//~PxController

		// PxRotatableController
		virtual	PxF32						getOrientation()							const override					{ return mOrientation;					}
		virtual	PxF32						getRadius()									const override					{ return mRadius;						}
		virtual	PxF32						getHeight()									const override					{ return mHeight;						}
		virtual	PxF32						getHalfBoxHeight()							const override					{ return mHalfBoxHeight;				}
		virtual	PxF32						getHalfSideExtent()							const override					{ return mHalfSideExtent;				}
		virtual	PxF32						getHalfForwardExtent()						const override					{ return mHalfForwardExtent;			}
		virtual	PxCapsuleClimbingMode::Enum	getClimbingMode()							const override					{ return mClimbingMode;					}
		virtual	PxRotatableShapeType::Enum	getShapeType()								const override					{ return mShapeType;					}
		virtual	bool						setOrientation(PxF32 orientation)			override;
		virtual	bool						setRadius(PxF32 radius)						override;
		virtual	bool						setHeight(PxF32 height)						override;
		virtual	bool						setHalfBoxHeight(PxF32 height)				override;
		virtual	bool						setHalfSideExtent(PxF32 halfSideExtent)		override;
		virtual	bool						setHalfForwardExtent(PxF32 halfForwardExtent) override;
		virtual	bool						setClimbingMode(PxCapsuleClimbingMode::Enum) override;
		virtual	bool						setShapeType(PxRotatableShapeType::Enum shapeType) override;
		//~ PxRotatableController

				void						getCapsule(PxExtendedCapsule& capsule)		const;
				void						getOBB(PxExtendedBox& obb)					const;

				bool						updateKinematicProxy();
				bool						updateKinematicPose();

				bool						createRotatableProxyActor(PxPhysics& sdk, const PxGeometry& geometry, const PxMaterial& material);

				PxF32						mOrientation{ 0.0f };
				PxF32						mRadius;
				PxF32						mHeight;
				PxF32						mHalfBoxHeight;
				PxF32						mHalfSideExtent;
				PxF32						mHalfForwardExtent;
				PxCapsuleClimbingMode::Enum	mClimbingMode;
				PxRotatableShapeType::Enum	mShapeType{ PxRotatableShapeType::eCapsule };
	};

	PX_INLINE PxQuat GetRotateQuat(const PxF32 orient, const PxVec3& up, const PxQuat& quatFromUp)
	{
		return PxQuat(orient, up) * quatFromUp;
	}

} // namespace Cct

}

/** \endcond */
