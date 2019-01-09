#include "utils/CloneUtils.h"

#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"
#include "GuTriangleMeshRTree.h"
#include "GuConvexMesh.h"
#include "GuBigConvexData2.h"
#include "GuHeightField.h"
#include "CmUtils.h"
#include "NpFactory.h"
#include "PsAllocator.h"

namespace physx
{
	
	void cloneRTree(Gu::RTree& clone, const Gu::RTree& orig);

	void cloneBV4Tree(Gu::BV4Tree& clone, const Gu::BV4Tree& orig);

	void cloneBV4TriangleMesh(Gu::BV4TriangleMesh* clone, Gu::BV4TriangleMesh* orig);

	void cloneRTreeTriangleMesh(Gu::RTreeTriangleMesh* clone, Gu::RTreeTriangleMesh* orig);

	void cloneTriangleMesh(Gu::TriangleMesh* clone, Gu::TriangleMesh* orig);

	void cloneConvexMesh(Gu::ConvexMesh* clone, Gu::ConvexMesh* orig);

	void cloneHeightField(Gu::HeightField* clone, Gu::HeightField* orig);

	//-----------------------------------------------------------------------------
	void clonePhysXGeometry(PxGeometryHolder & clone, const PxGeometryHolder & orig)
	{
		clone = orig;

		//Some geometries have pointers that need to be cloned
		switch (orig.getType())
		{
			//Basic geometries don't have any pointers
		case PxGeometryType::eSPHERE:
		case PxGeometryType::ePLANE:
		case PxGeometryType::eBOX:
		case PxGeometryType::eCAPSULE:
			break;
		case PxGeometryType::eTRIANGLEMESH:
			{
				const PxTriangleMeshGeometry & origGeom = orig.triangleMesh();
				PxTriangleMeshGeometry & clonedGeom = clone.triangleMesh();
				clonedGeom.triangleMesh = nullptr;
	
				Gu::TriangleMesh * origTriMesh = static_cast<Gu::TriangleMesh *>(origGeom.triangleMesh);
	
				Gu::TriangleMesh * clonedTriMesh = nullptr;
				Gu::BV4TriangleMesh* bv4TriMesh = nullptr;
				Gu::RTreeTriangleMesh* rTreeTriMesh = nullptr;
				
				if (origTriMesh->getMidphaseID() == PxMeshMidPhase::eBVH34)
				{
					PX_NEW_SERIALIZED(bv4TriMesh, Gu::BV4TriangleMesh)(PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE);
					clonedTriMesh = bv4TriMesh;
				}
				else if (origTriMesh->getMidphaseID() == PxMeshMidPhase::eBVH33)
				{
					PX_NEW_SERIALIZED(rTreeTriMesh, Gu::RTreeTriangleMesh)(PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE);
					clonedTriMesh = rTreeTriMesh;
				}
				else
				{
					PX_ASSERT(false);
					return;
				}
				if (!clonedTriMesh)
					return;

				// Clone the mesh
				cloneTriangleMesh(clonedTriMesh, origTriMesh);
	
				clonedTriMesh->setMeshFactory(&NpFactory::getInstance());
				NpFactory::getInstance().addTriangleMesh(clonedTriMesh);
	
				PX_ASSERT(clonedTriMesh->getConcreteType() == origTriMesh->getConcreteType());

				clonedGeom.triangleMesh = static_cast<PxTriangleMesh *>(clonedTriMesh);
			}
			break;
		case PxGeometryType::eCONVEXMESH:
			{
				const PxConvexMeshGeometry& origGeom = orig.convexMesh();
				PxConvexMeshGeometry& clonedGeom = clone.convexMesh();
				clonedGeom.convexMesh = nullptr;

				Gu::ConvexMesh* clonedConvexMesh = nullptr;
				PX_NEW_SERIALIZED(clonedConvexMesh, Gu::ConvexMesh)(PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE);
				if (!clonedConvexMesh)
					return;

				cloneConvexMesh(clonedConvexMesh, dynamic_cast<Gu::ConvexMesh*>(origGeom.convexMesh));

				clonedConvexMesh->setMeshFactory(&NpFactory::getInstance());
				NpFactory::getInstance().addConvexMesh(clonedConvexMesh);

				PX_ASSERT(clonedConvexMesh->getConcreteType() == origGeom.convexMesh->getConcreteType());

				clonedGeom.convexMesh = static_cast<PxConvexMesh*>(clonedConvexMesh);
			}
			break;
		case PxGeometryType::eHEIGHTFIELD:
			{
				const PxHeightFieldGeometry & origGeom = orig.heightField();
				PxHeightFieldGeometry & clonedGeom = clone.heightField();
				clonedGeom.heightField = nullptr;

				clonedGeom.columnScale = origGeom.columnScale;
				clonedGeom.rowScale = origGeom.rowScale;
				clonedGeom.heightScale = origGeom.heightScale;
				clonedGeom.heightFieldFlags = origGeom.heightFieldFlags;

				Gu::HeightField* clonedHeightField = nullptr;;
				PX_NEW_SERIALIZED(clonedHeightField, Gu::HeightField)(PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE);
				if (!clonedHeightField)
					return;

				cloneHeightField(clonedHeightField, dynamic_cast<Gu::HeightField*>(origGeom.heightField));

				clonedHeightField->setMeshFactory(&NpFactory::getInstance());
				NpFactory::getInstance().addHeightField(clonedHeightField);

				PX_ASSERT(clonedHeightField->getConcreteType() == origGeom.heightField->getConcreteType());

				clonedGeom.heightField = static_cast<PxHeightField*>(clonedHeightField);
			}
			break;
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
		default:
			PX_ASSERT(0);
		};
	}

	void cloneBV4Tree(Gu::BV4Tree& clone, const Gu::BV4Tree& orig)
	{
		clone.mLocalBounds = orig.mLocalBounds;
		clone.mNbNodes = orig.mNbNodes;
		clone.mInitData = orig.mInitData;
#ifdef GU_BV4_QUANTIZED_TREE
		clone.mCenterOrMinCoeff = orig.mCenterOrMinCoeff;
		clone.mExtentsOrMaxCoeff = orig.mExtentsOrMaxCoeff;
#endif
		clone.mUserAllocated = false;
#ifdef GU_BV4_USE_SLABS
		clone.mNodes = reinterpret_cast<Gu::BVDataPacked*>(PX_ALLOC(sizeof(Gu::BVDataPacked)*clone.mNbNodes, "BV4 nodes"));
#else
		clone.mNodes = PX_NEW(Gu::BVDataPacked)[clone.mNbNodes];
#endif
		*clone.mNodes = *orig.mNodes;
	}

	void cloneRTree(Gu::RTree& clone, const Gu::RTree& orig)
	{
		clone = orig;
		PxU32 numBytes = sizeof(Gu::RTreePage)*clone.mTotalPages;
		clone.mPages = static_cast<Gu::RTreePage*>(physx::shdfnd::AlignedAllocator<128>().allocate(numBytes, __FILE__, __LINE__));
		clone.mFlags &= (~(PxU32)Gu::RTree::USER_ALLOCATED);
		memcpy(clone.mPages, orig.mPages, numBytes);
	}

	void cloneBV4TriangleMesh(Gu::BV4TriangleMesh* clone, Gu::BV4TriangleMesh* orig)
	{
		cloneBV4Tree(clone->mBV4Tree, orig->mBV4Tree);
		if (clone->has16BitIndices())
			clone->mMeshInterface.setPointers(NULL, const_cast<Gu::IndTri16*>(reinterpret_cast<const Gu::IndTri16*>(clone->getTrianglesFast())), clone->getVerticesFast());
		else
			clone->mMeshInterface.setPointers(const_cast<Gu::IndTri32*>(reinterpret_cast<const Gu::IndTri32*>(clone->getTrianglesFast())), NULL, clone->getVerticesFast());
		clone->mBV4Tree.mMeshInterface = &clone->mMeshInterface;

	}

	void cloneRTreeTriangleMesh(Gu::RTreeTriangleMesh* clone, Gu::RTreeTriangleMesh* orig)
	{
		cloneRTree(clone->mRTree, orig->mRTree);
	}

	void cloneTriangleMesh(Gu::TriangleMesh* clone, Gu::TriangleMesh* orig)
	{
		// copy data
		clone->mNbVertices = orig->mNbVertices;
		clone->mNbTriangles = orig->mNbTriangles;
		clone->mVertices = orig->mVertices;
		clone->mTriangles = orig->mTriangles;
		clone->mAABB = orig->mAABB;
		clone->mExtraTrigData = orig->mExtraTrigData;
		clone->mGeomEpsilon = orig->mGeomEpsilon;
		clone->mFlags = orig->mFlags;
		clone->mMaterialIndices = orig->mMaterialIndices;
		clone->mFaceRemap = orig->mFaceRemap;
		clone->mAdjacencies = orig->mAdjacencies;
		clone->mGRB_triIndices = orig->mGRB_triIndices;
		clone->mGRB_triAdjacencies = orig->mGRB_triAdjacencies;
		clone->mGRB_faceRemap = orig->mGRB_faceRemap;
		clone->mGRB_BV32Tree = orig->mGRB_BV32Tree;
		clone->mConcreteType = orig->mConcreteType;

		// Copy vertices
		if (clone->mVertices)
		{
			//PX_ALIGNED16_ALLOC;
			clone->mVertices = (PxVec3*)PX_ALLOC(sizeof(PxVec3)*clone->mNbVertices, PX_DEBUG_EXP("PxVec3"));
			memcpy(clone->mVertices, orig->mVertices, sizeof(PxVec3)*clone->mNbVertices);
		}
		// Copy Triangles
		if (clone->mTriangles)
		{
			if (clone->mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
			{
				clone->mTriangles = (void*)PX_ALLOC(sizeof(PxU16)*clone->mNbTriangles * 3, PX_DEBUG_EXP(""));
				memcpy(clone->mTriangles, orig->mTriangles, sizeof(PxU16)*clone->mNbTriangles * 3);
			}
			else
			{
				clone->mTriangles = (void*)PX_ALLOC(sizeof(PxU32)*clone->mNbTriangles * 3, PX_DEBUG_EXP(""));
				memcpy(clone->mTriangles, orig->mTriangles, sizeof(PxU32)*clone->mNbTriangles * 3);
			}
		}
		// Copy mExtraTrigData
		if (clone->mExtraTrigData)
		{
			clone->mExtraTrigData = (PxU8*)PX_ALLOC(sizeof(PxU8)*clone->mNbTriangles, PX_DEBUG_EXP(""));
			memcpy(clone->mExtraTrigData, orig->mExtraTrigData, sizeof(PxU8)*clone->mNbTriangles);
		}
		// Copy mMaterialIndices
		if (clone->mMaterialIndices)
		{
			clone->mMaterialIndices = (PxU16*)PX_ALLOC(sizeof(PxU16)*clone->mNbTriangles, PX_DEBUG_EXP(""));
			memcpy(clone->mMaterialIndices, orig->mMaterialIndices, sizeof(PxU16)*clone->mNbTriangles);
		}
		// Copy mFaceRemap
		if (clone->mFaceRemap)
		{
			clone->mFaceRemap = (PxU32*)PX_ALLOC(sizeof(PxU32)*clone->mNbTriangles, PX_DEBUG_EXP(""));
			memcpy(clone->mFaceRemap, orig->mFaceRemap, sizeof(PxU32)*clone->mNbTriangles);
		}
		// Copy mAdjacencies
		if (clone->mAdjacencies)
		{
			clone->mAdjacencies = (PxU32*)PX_ALLOC(sizeof(PxU32)*clone->mNbTriangles*3, PX_DEBUG_EXP(""));
			memcpy(clone->mAdjacencies, orig->mAdjacencies, sizeof(PxU32)*clone->mNbTriangles*3);
		}

		clone->mGRB_triIndices = nullptr;
		clone->mGRB_triAdjacencies = nullptr;
		clone->mGRB_faceRemap = nullptr;
		clone->mGRB_BV32Tree = nullptr;

		if (clone->getMidphaseID() == PxMeshMidPhase::eBVH34)
		{
			cloneBV4TriangleMesh(dynamic_cast<Gu::BV4TriangleMesh*>(clone), dynamic_cast<Gu::BV4TriangleMesh*>(orig));
		}
		else if (clone->getMidphaseID() == PxMeshMidPhase::eBVH33)
		{
			cloneRTreeTriangleMesh(dynamic_cast<Gu::RTreeTriangleMesh*>(clone), dynamic_cast<Gu::RTreeTriangleMesh*>(orig));
		}
		else
		{
			PX_ASSERT(false);
			return;
		}
	}

	void cloneBigConvexData(BigConvexData* clone, BigConvexData* orig)
	{
		//*clone = *orig; // may lead mVBuffer not nullptr;
		clone->mData = orig->mData;

		if (clone->mData.mSamples)
		{
			clone->mData.mSamples = (PxU8*)PX_ALLOC(PxU32(clone->mData.mNbSamples * 2), PX_DEBUG_EXP(""));
			memcpy(clone->mData.mSamples, orig->mData.mSamples, PxU32(clone->mData.mNbSamples * 2));
		}

		if (clone->mData.mValencies)
		{
			PxU32 numVerts = (clone->mData.mNbVerts + 3)&~3;
			clone->mData.mValencies = (Gu::Valency*)PX_ALLOC(sizeof(Gu::Valency) * numVerts, PX_DEBUG_EXP(""));
			memcpy(clone->mData.mValencies, orig->mData.mValencies, sizeof(Gu::Valency) * numVerts);

			clone->mData.mAdjacentVerts = (PxU8*)PX_ALLOC(sizeof(PxU8) * clone->mData.mNbAdjVerts, PX_DEBUG_EXP(""));
			memcpy(clone->mData.mAdjacentVerts, orig->mData.mAdjacentVerts, sizeof(PxU8) * clone->mData.mNbAdjVerts);

		}
	}

	void cloneConvexMesh(Gu::ConvexMesh* clone, Gu::ConvexMesh* orig)
	{
		clone->mHullData = orig->mHullData;
		clone->mNb = orig->mNb;
		clone->mBigConvexData = orig->mBigConvexData;
		clone->mMass = orig->mMass;
		clone->mInertia = orig->mInertia;
		clone->mConcreteType = orig->mConcreteType;

		const PxU32 bufferSize = computeBufferSize(clone->mHullData, clone->getNb());
		clone->mHullData.mPolygons = (Gu::HullPolygonData*)PX_ALLOC(sizeof(PxU8)*bufferSize, PX_DEBUG_EXP("HullData polygons"));
		//*clone->mHullData.mPolygons = *orig->mHullData.mPolygons;
		PxMemCopy(clone->mHullData.mPolygons, orig->mHullData.mPolygons, bufferSize);

		if (clone->mBigConvexData)
		{
			clone->mBigConvexData = (BigConvexData*)PX_ALLOC(sizeof(BigConvexData), PX_DEBUG_EXP("ConvexMesh BigConvexData"));
			memset(clone->mBigConvexData, 0, sizeof(BigConvexData));
			cloneBigConvexData(clone->mBigConvexData, orig->mBigConvexData);
			clone->mHullData.mBigConvexRawData = &clone->mBigConvexData->mData;
		}
	}

	void cloneHeightField(Gu::HeightField* clone, Gu::HeightField* orig)
	{
		if (!orig)
			return;

		clone->mSampleStride = orig->mSampleStride;
		clone->mNbSamples = orig->mNbSamples;
		clone->mMinHeight = orig->mMinHeight;
		clone->mMaxHeight = orig->mMaxHeight;

		clone->mData = orig->mData;

		clone->mConcreteType = orig->mConcreteType;

		if (!orig->mData.samples)
			return;
		clone->mData.samples = nullptr;
		PxU32 numBytes = clone->mData.rows * clone->mData.columns * sizeof(PxHeightFieldSample);
		clone->mData.samples = (PxHeightFieldSample*)PX_ALLOC(numBytes, PX_DEBUG_EXP("PxHeightFieldSample"));
		if (!clone->mData.samples)
			return;

		memcpy(clone->mData.samples, orig->mData.samples, numBytes);
	}

} // end physx namespace

