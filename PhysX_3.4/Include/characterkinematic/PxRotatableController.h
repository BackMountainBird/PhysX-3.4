//	--------------------------------------------------------------------
//	Copyright(C) 2006~2018 NetEase. All rights reserved.
//	This file is a part of the project Messiah.
//	Project Messiah
//	Contact : gzzhengyi@corp.netease.com
//	Author: Bleston
//	$(reservedDeclaration)
//	--------------------------------------------------------------------
///	@file	<PxRotatableController.cpp>
///	@path	~/PhysXSDk/Include/characterkinematic
///	@date	2018/03/01
///	@brief	.


#pragma once
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"
#include "characterkinematic/PxController.h"
#include "characterkinematic/PxBoxController.h"
#include "characterkinematic/PxCapsuleController.h"

#if !PX_DOXYGEN
namespace physx
{
#endif


struct PxRotatableShapeType
{
	enum Enum
	{
		eCapsule,
		eOBB,

		eLAST
	};
};


/**
\brief A descriptor for a rotatable character controller.

*/
class PxRotatableControllerDesc : public PxControllerDesc
{
public:
	/**
	\brief constructor sets to default.
	*/
	PX_INLINE									PxRotatableControllerDesc ();
	PX_INLINE virtual							~PxRotatableControllerDesc () {}

	/**
	\brief copy constructor.
	*/
	PX_INLINE									PxRotatableControllerDesc(const PxRotatableControllerDesc&);

	/**
	\brief assignment operator.
	*/
	PX_INLINE PxRotatableControllerDesc&			operator=(const PxRotatableControllerDesc&);

	/**
	\brief (re)sets the structure to the default.
	*/
	PX_INLINE virtual	void				setToDefault();
	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	PX_INLINE virtual	bool				isValid()		const;

	/**
	\brief The orientation of the character

	\note The character's initial position must be such that it does not overlap the static geometry.

	<b>Default:</b> Zero
	*/
	PxF32				orientation{ 0.0f };

	/**
	\brief The radius of the capsule

	<b>Default:</b> 0.0

	@see PxRotatableController
	*/
	PxF32				radius{ 0.0f };

	/**
	\brief The height of the controller when it has a capsule shape

	<b>Default:</b> 0.0

	@see PxRotatableController
	*/
	PxF32				height{ 0.0f };

	/**
	\brief Half height when controller has a box shape

	<b>Default:</b> 1.0
	*/
	PxF32				halfBoxHeight{ 1.0f };		// Half-height in the "up" direction
	/**
	\brief Half side extent

	<b>Default:</b> 0.5
	*/
	PxF32				halfSideExtent{ 0.5f };		// Half-extent in the "side" direction

	/**
	\brief Half forward extent

	<b>Default:</b> 0.5
	*/
	PxF32				halfForwardExtent{ 0.5f };	// Half-extent in the "forward" direction

	/**
	\brief The climbing mode

	<b>Default:</b> PxCapsuleClimbingMode::eEASY

	@see PxRotatableController
	*/
	PxCapsuleClimbingMode::Enum		climbingMode{ PxCapsuleClimbingMode::eEASY };

	/**
	\brief The shape type

	<b>Default:</b> PxRotatableShapeType::eCapsule

	@see PxRotatableController
	*/
	PxRotatableShapeType::Enum		shapeType{ PxRotatableShapeType::eCapsule };
	
protected:
	PX_INLINE void								copy(const PxRotatableControllerDesc&);
};

PX_INLINE PxRotatableControllerDesc::PxRotatableControllerDesc() : PxControllerDesc(PxControllerShapeType::eROTATABLE)
{
}

PX_INLINE PxRotatableControllerDesc::PxRotatableControllerDesc(const PxRotatableControllerDesc& other) : PxControllerDesc(other)
{
	copy(other);
}

PX_INLINE PxRotatableControllerDesc& PxRotatableControllerDesc::operator=(const PxRotatableControllerDesc& other)
{
	PxControllerDesc::operator=(other);
	copy(other);
	return *this;
}

PX_INLINE void PxRotatableControllerDesc::copy(const PxRotatableControllerDesc& other)
{
	orientation			= other.orientation;
	radius				= other.radius;
	height				= other.height;
	halfBoxHeight		= other.halfBoxHeight;
	halfSideExtent		= other.halfSideExtent;
	halfForwardExtent	= other.halfForwardExtent;
	climbingMode	= other.climbingMode;
}

PX_INLINE void PxRotatableControllerDesc::setToDefault()
{
	*this = PxRotatableControllerDesc();
}

PX_INLINE bool PxRotatableControllerDesc::isValid() const
{
	if(!PxControllerDesc::isValid())	return false;
	if(radius<=0.0f)					return false;
	if(height<=0.0f)					return false;
	if(halfSideExtent <= 0.0f)			return false;
	if(halfForwardExtent <= 0.0f)		return false;
	if((shapeType == PxRotatableShapeType::eOBB) && (stepOffset > halfBoxHeight*2.0f)) return false;	// Prevents obvious mistakes
	if((shapeType == PxRotatableShapeType::eCapsule) && (stepOffset>height+radius*2.0f)) return false;	// Prevents obvious mistakes
	return true;
}
/**
\brief A rotatable character controller.

This controller can rotate around the up axis. Its shape can change between capsule and box.

The capsule is defined as a position, a vertical height, and a radius.
The height is the distance between the two sphere centers at the end of the capsule.
In other words:

p = pos (returned by controller)<br>
h = height<br>
r = radius<br>

p = center of capsule<br>
top sphere center = p.y + h*0.5<br>
bottom sphere center = p.y - h*0.5<br>
top capsule point = p.y + h*0.5 + r<br>
bottom capsule point = p.y - h*0.5 - r<br>
*/
class PxRotatableController : public PxController
{
public:

	/**
	\brief Gets controller's orientation.

	\return The orientation of the controller.

	@see PxRotatableControllerDesc.orientation setOrientation()
	*/
	virtual		PxF32			getOrientation() const = 0;

	/**
	\brief Sets controller's orientation.

	\warning this doesn't check for collisions.

	\param[in] orientation The new orientation for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.orientation getOrientation()
	*/
	virtual		bool			setOrientation(PxF32 orientation) = 0;

	/**
	\brief Gets controller's radius.

	\return The radius of the controller.

	@see PxRotatableControllerDesc.radius setRadius()
	*/
	virtual		PxF32			getRadius() const = 0;

	/**
	\brief Sets controller's radius.

	\warning this doesn't check for collisions.

	\param[in] radius The new radius for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.radius getRadius()
	*/
	virtual		bool			setRadius(PxF32 radius) = 0;

	/**
	\brief Gets controller's height.

	\return The half box height of the controller.

	@see PxRotatableControllerDesc.height setHeight()
	*/
	virtual		PxF32			getHeight() const = 0;

	/**
	\brief Resets controller's height.

	\warning this doesn't check for collisions.

	\param[in] height The new height for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.height getHeight()
	*/
	virtual		bool			setHeight(PxF32 height) = 0;

	/**
	\brief Gets controller's half box height.

	\return The half box height of the controller.

	@see PxRotatableControllerDesc.halfBoxHeight setHalfBoxHeight()
	*/
	virtual		PxF32			getHalfBoxHeight() const = 0;

	/**
	\brief Resets controller's half box height.

	\warning this doesn't check for collisions.

	\param[in] height The new half box height for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.halfBoxHeight getHalfBoxHeight()
	*/
	virtual		bool			setHalfBoxHeight(PxF32 height) = 0;

	/**
	\brief Gets controller's half side extent.

	\return The half side extent of the controller.

	@see PxRotatableControllerDesc.halfSideExtent setHalfSideExtent()
	*/
	virtual		PxF32			getHalfSideExtent()		const = 0;

	/**
	\brief Sets controller's half side extent.

	\warning this doesn't check for collisions.

	\param[in] halfSideExtent The new half side extent for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.halfSideExtent getHalfSideExtent()
	*/
	virtual		bool			setHalfSideExtent(PxF32 halfSideExtent) = 0;

	/**
	\brief Gets controller's half forward extent.

	\return The half forward extent of the controller.

	@see PxRotatableControllerDesc.halfForwardExtent setHalfForwardExtent()
	*/
	virtual		PxF32			getHalfForwardExtent()	const = 0;

	/**
	\brief Sets controller's half forward extent.

	\warning this doesn't check for collisions.

	\param[in] halfForwardExtent The new half forward extent for the controller.
	\return Currently always true.

	@see PxRotatableControllerDesc.halfForwardExtent getHalfForwardExtent()
	*/
	virtual		bool			setHalfForwardExtent(PxF32 halfForwardExtent) = 0;

	/**
	\brief Gets controller's climbing mode.

	\return The capsule controller's climbing mode.

	@see PxCapsuleControllerDesc.climbingMode setClimbingMode()
	*/
	virtual		PxCapsuleClimbingMode::Enum		getClimbingMode()	const	= 0;

	/**
	\brief Sets controller's climbing mode.

	\param[in] mode The capsule controller's climbing mode.

	@see PxCapsuleControllerDesc.climbingMode getClimbingMode()
	*/
	virtual		bool			setClimbingMode(PxCapsuleClimbingMode::Enum mode)	= 0;
	
	/**
	\brief Sets controller's shape type

	\param[in] shapeType The rotatable controller's shape type.

	@see PxCapsuleControllerDesc.shapeType setShapeType()
	*/
	virtual		PxRotatableShapeType::Enum		getShapeType()	const	= 0;

	/**
	\brief Sets controller's shape type

	\param[in] shapeType The rotatable controller's shape type.

	@see PxCapsuleControllerDesc.shapeType getShapeType()
	*/
	virtual		bool			setShapeType(PxRotatableShapeType::Enum shapeType)	= 0;

protected:
	PX_INLINE					PxRotatableController()		{}
	virtual						~PxRotatableController()	{}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
