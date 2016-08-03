' Copyright (c) 2015-2016 Bruce A Henderson
'
' This software is provided 'as-is', without any express or implied
' warranty. In no event will the authors be held liable for any damages
' arising from the use of this software.
'
' Permission is granted to anyone to use this software for any purpose,
' including commercial applications, and to alter it and redistribute it
' freely, subject to the following restrictions:
'
'    1. The origin of this software must not be misrepresented; you must not
'    claim that you wrote the original software. If you use this software
'    in a product, an acknowledgment in the product documentation would be
'    appreciated but is not required.
'
'    2. Altered source versions must be plainly marked as such, and must not be
'    misrepresented as being the original software.
'
'    3. This notice may not be removed or altered from any source
'    distribution.
'
SuperStrict


Import "NewtonDynamics/source/core/*.h"
Import "NewtonDynamics/source/physics/*.h"
Import "NewtonDynamics/source/meshUtil/*.h"
Import "NewtonDynamics/source/newton/*.h"
Import "NewtonDynamics/packages/dMath/*.h"

Import "NewtonDynamics/source/core/dgAABBPolygonSoup.cpp"
Import "NewtonDynamics/source/core/dgAsyncThread.cpp"
Import "NewtonDynamics/source/core/dgConvexHull3d.cpp"
Import "NewtonDynamics/source/core/dgConvexHull4d.cpp"
Import "NewtonDynamics/source/core/dg.cpp"
Import "NewtonDynamics/source/core/dgCRC.cpp"
Import "NewtonDynamics/source/core/dgDebug.cpp"
Import "NewtonDynamics/source/core/dgDelaunayTetrahedralization.cpp"
Import "NewtonDynamics/source/core/dgGeneralMatrix.cpp"
Import "NewtonDynamics/source/core/dgGeneralVector.cpp"
Import "NewtonDynamics/source/core/dgGoogol.cpp"
Import "NewtonDynamics/source/core/dgIntersections.cpp"
Import "NewtonDynamics/source/core/dgMatrix.cpp"
Import "NewtonDynamics/source/core/dgMemory.cpp"
Import "NewtonDynamics/source/core/dgMutexThread.cpp"
Import "NewtonDynamics/source/core/dgNode.cpp"
Import "NewtonDynamics/source/core/dgPolygonSoupBuilder.cpp"
Import "NewtonDynamics/source/core/dgPolyhedra.cpp"
Import "NewtonDynamics/source/core/dgPolyhedraMassProperties.cpp"
Import "NewtonDynamics/source/core/dgQuaternion.cpp"
Import "NewtonDynamics/source/core/dgRandom.cpp"
Import "NewtonDynamics/source/core/dgRefCounter.cpp"
Import "NewtonDynamics/source/core/dgRef.cpp"
Import "NewtonDynamics/source/core/dgSmallDeterminant.cpp"
Import "NewtonDynamics/source/core/dgSPDMatrix.cpp"
Import "NewtonDynamics/source/core/dgObb.cpp"
Import "NewtonDynamics/source/core/dgThread.cpp"
Import "NewtonDynamics/source/core/dgThreadHive.cpp"
Import "NewtonDynamics/source/core/dgTree.cpp"
Import "NewtonDynamics/source/core/dgTypes.cpp"
Import "NewtonDynamics/source/physics/dgBody.cpp"
Import "NewtonDynamics/source/physics/dgDynamicBody.cpp"
Import "NewtonDynamics/source/physics/dgKinematicBody.cpp"
Import "NewtonDynamics/source/physics/dgBallConstraint.cpp"
Import "NewtonDynamics/source/physics/dgBilateralConstraint.cpp"
Import "NewtonDynamics/source/physics/dgBody.cpp"
Import "NewtonDynamics/source/physics/dgDynamicBody.cpp"
Import "NewtonDynamics/source/physics/dgKinematicBody.cpp"
Import "NewtonDynamics/source/physics/dgBodyMasterList.cpp"
Import "NewtonDynamics/source/physics/dgBroadPhase.cpp"
Import "NewtonDynamics/source/physics/dgCollisionBox.cpp"
Import "NewtonDynamics/source/physics/dgCollisionBVH.cpp"
Import "NewtonDynamics/source/physics/dgCollisionCapsule.cpp"
Import "NewtonDynamics/source/physics/dgCollisionChamferCylinder.cpp"
Import "NewtonDynamics/source/physics/dgCollisionCompoundFractured.cpp"
Import "NewtonDynamics/source/physics/dgCollisionCompound.cpp"
Import "NewtonDynamics/source/physics/dgCollisionCone.cpp"
Import "NewtonDynamics/source/physics/dgCollisionConvex.cpp"
Import "NewtonDynamics/source/physics/dgCollisionConvexHull.cpp"
Import "NewtonDynamics/source/physics/dgCollisionConvexPolygon.cpp"
Import "NewtonDynamics/source/physics/dgCollision.cpp"
Import "NewtonDynamics/source/physics/dgCollisionCylinder.cpp"
Import "NewtonDynamics/source/physics/dgCollisionDeformableClothPatch.cpp"
Import "NewtonDynamics/source/physics/dgCollisionDeformableSolidMesh.cpp"
Import "NewtonDynamics/source/physics/dgCollisionDeformableMesh.cpp"
Import "NewtonDynamics/source/physics/dgCollisionHeightField.cpp"
Import "NewtonDynamics/source/physics/dgCollisionInstance.cpp"
Import "NewtonDynamics/source/physics/dgCollisionMesh.cpp"
Import "NewtonDynamics/source/physics/dgCollisionNull.cpp"
Import "NewtonDynamics/source/physics/dgCollisionScene.cpp"
Import "NewtonDynamics/source/physics/dgCollisionSphere.cpp"
Import "NewtonDynamics/source/physics/dgCollisionTaperedCapsule.cpp"
Import "NewtonDynamics/source/physics/dgCollisionTaperedCylinder.cpp"
Import "NewtonDynamics/source/physics/dgCollisionUserMesh.cpp"
Import "NewtonDynamics/source/physics/dgConstraint.cpp"
Import "NewtonDynamics/source/physics/dgContact.cpp"
Import "NewtonDynamics/source/physics/dgCorkscrewConstraint.cpp"
Import "NewtonDynamics/source/physics/dgDeformableBody.cpp"
Import "NewtonDynamics/source/physics/dgDeformableContact.cpp"
Import "NewtonDynamics/source/physics/dgHingeConstraint.cpp"
Import "NewtonDynamics/source/physics/dgNarrowPhaseCollision.cpp"
Import "NewtonDynamics/source/physics/dgSlidingConstraint.cpp"
Import "NewtonDynamics/source/physics/dgUniversalConstraint.cpp"
Import "NewtonDynamics/source/physics/dgUpVectorConstraint.cpp"
Import "NewtonDynamics/source/physics/dgUserConstraint.cpp"
Import "NewtonDynamics/source/physics/dgWorld.cpp"
Import "NewtonDynamics/source/physics/dgDeformableBodiesUpdate.cpp"
Import "NewtonDynamics/source/physics/dgWorldDynamicsParallelSolver.cpp"
Import "NewtonDynamics/source/physics/dgWorldDynamicsSimpleSolver.cpp"
Import "NewtonDynamics/source/physics/dgWorldDynamicUpdate.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect1.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect2.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect3.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect4.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect5.cpp"
Import "NewtonDynamics/source/meshUtil/dgMeshEffect6.cpp"
Import "NewtonDynamics/source/newton/Newton.cpp"
Import "NewtonDynamics/source/newton/NewtonClass.cpp"

Import "NewtonDynamics/packages/dMath/dLinearAlgebra.cpp"
Import "NewtonDynamics/packages/dMath/dMathDefines.cpp"
Import "NewtonDynamics/packages/dMath/dQuaternion.cpp"
Import "NewtonDynamics/packages/dMath/dVector.cpp"
Import "NewtonDynamics/packages/dMath/dMatrix.cpp"

Import "glue.cpp"

