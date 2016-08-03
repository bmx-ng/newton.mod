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

Import "source.bmx"



Extern

	Function bmx_newtondynamics_NewtonCreate:Byte Ptr(obj:Object)
	Function NewtonDestroy(world:Byte Ptr)
	Function NewtonDestroyAllBodies(world:Byte Ptr)
	
	Function NewtonInvalidateCache(world:Byte Ptr)
	Function NewtonUpdate(world:Byte Ptr, timestep:Float)
	Function NewtonSetFrictionModel(world:Byte Ptr, model:Int)
	Function NewtonSetMinimumFrameRate(world:Byte Ptr, framerate:Float)
	Function NewtonSetContactMergeTolerance(world:Byte Ptr, tolerance:Float)
	
	Function bmx_newtondynamics_NewtonCreateDynamicBody:Byte Ptr(obj:Object, world:Byte Ptr, coll:Byte Ptr, matrix:Float Ptr)
	Function bmx_newtondynamics_NewtonCreateSphere:Byte Ptr(obj:Object, world:Byte Ptr, radius:Float, shapeID:Int, offsetMatrix:Float Ptr)
	Function bmx_newtondynamics_NewtonCreateBox:Byte Ptr(obj:Object, world:Byte Ptr, dx:Float, dy:Float, dz:Float, shapeID:Int, offsetMatrix:Float Ptr)
	Function bmx_newtondynamics_NewtonCreateTreeCollision:Byte Ptr(obj:Object, world:Byte Ptr, shapeID:Int)
	Function bmx_newtondynamics_NewtonCreateCylinder:Byte Ptr(obj:Object, world:Byte Ptr, radius:Float, height:Float, shapeID:Int, offsetMatrix:Float Ptr)

	Function bmx_newtondynamics_NewtonWorldRayCast(world:Byte Ptr, p0x:Float, p0y:Float, p0z:Float, p1x:Float, p1y:Float, p1z:Float, ..
		fcb:Float(bodyPtr:Byte Ptr, collPtr:Byte Ptr, hitContact:Float Ptr, hitNormal:Float Ptr, delPtr:Byte Ptr, intersectParam:Float), ..
		delegate:Object, _pfcb:Int(bodyPtr:Byte Ptr, collPtr:Byte Ptr, delPtr:Byte Ptr), threadIndex:Int)

	Function NewtonWorldGetBodyCount:Int(world:Byte Ptr)
	Function NewtonWorldGetConstraintCount:Int(world:Byte Ptr)

	Function NewtonDestroyCollision(coll:Byte Ptr)

	Function NewtonTreeCollisionBeginBuild(coll:Byte Ptr)
	Function NewtonTreeCollisionEndBuild(coll:Byte Ptr, optimize:Int)
	Function NewtonTreeCollisionAddFace(coll:Byte Ptr, vertexCount:Int, vertexPtr:Float Ptr, strideInBytes:Int, faceAttribute:Int)
	
	Function NewtonDestroyBody(body:Byte Ptr)
	Function NewtonBodyGetMatrix(body:Byte Ptr, matrix:Float Ptr)
	Function NewtonBodySetForceAndTorqueCallback(body:Byte Ptr, callback(body:Byte Ptr, timestamp:Float, threadIndex:Int))
	Function NewtonBodySetSimulationState(body:Byte Ptr, state:Int)
	Function NewtonBodyGetSimulationState:Int(body:Byte Ptr)
	Function NewtonBodyGetType:Int(body:Byte Ptr)
	Function NewtonBodyGetCollidable:Int(body:Byte Ptr)
	Function NewtonBodySetCollidable(body:Byte Ptr, collidableState:Int)
	Function NewtonBodyGetMassMatrix(body:Byte Ptr, mass:Float Ptr, Ixx:Float Ptr, Iyy:Float Ptr, Izz:Float Ptr)
	Function NewtonBodySetMassProperties(body:Byte Ptr, mass:Float, coll:Byte Ptr)
	Function NewtonBodySetLinearDamping(body:Byte Ptr, linearDamp:Float)
	Function NewtonBodyGetInvMass(body:Byte Ptr, mass:Float Ptr, Ixx:Float Ptr, Iyy:Float Ptr, Izz:Float Ptr)
	Function NewtonBodyGetInertiaMatrix(body:Byte Ptr, matrix:Float Ptr)
	Function NewtonBodyGetInvInertiaMatrix(body:Byte Ptr, matrix:Float Ptr)
	Function NewtonBodySetMatrix(body:Byte Ptr, matrix:Float Ptr)
	Function NewtonBodySetTransformCallback(body:Byte Ptr, callback(body:Byte Ptr, matrix:Float Ptr, threadIndex:Int))
	
	Function bmx_newtondynamics_NewtonBodyGetRotation(body:Byte Ptr, q0:Float Ptr, q1:Float Ptr, q2:Float Ptr, q3:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetOmega(body:Byte Ptr, ox:Float Ptr, oy:Float Ptr, oz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetVelocity(body:Byte Ptr, vx:Float Ptr, vy:Float Ptr, vz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetForce(body:Byte Ptr, fx:Float Ptr, fy:Float Ptr, fz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetTorque(body:Byte Ptr, tx:Float Ptr, ty:Float Ptr, tz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetForceAcc(body:Byte Ptr, fx:Float Ptr, fy:Float Ptr, fz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetTorqueAcc(body:Byte Ptr, tx:Float Ptr, ty:Float Ptr, tz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodyGetCentreOfMass(body:Byte Ptr, cx:Float Ptr, cy:Float Ptr, cz:Float Ptr)
	Function bmx_newtondynamics_NewtonBodySetAngularDamping(body:Byte Ptr, aX:Float, aY:Float, aZ:Float)
	Function bmx_newtondynamics_NewtonBodyGetAngularDamping(body:Byte Ptr, aX:Float Ptr, aY:Float Ptr, aZ:Float Ptr)
	Function bmx_newtondynamics_NewtonBodySetOmega(body:Byte Ptr, ox:Float, oy:Float, oz:Float, ow:Float)
	Function bmx_newtondynamics_NewtonBodySetVelocity(body:Byte Ptr, vx:Float, vy:Float, vz:Float, vw:Float)
	Function bmx_newtondynamics_NewtonBodySetForce(body:Byte Ptr, fx:Float, fy:Float, yz:Float, fw:Float)
	Function bmx_newtondynamics_NewtonBodySetTorque(body:Byte Ptr, tx:Float, ty:Float, tz:Float, tw:Float)
	Function bmx_newtondynamics_NewtonBodyGetAABB(body:Byte Ptr, p0x:Float Ptr, p0y:Float Ptr, p0z:Float Ptr, p1x:Float Ptr, p1y:Float Ptr, p1z:Float Ptr)

	Function bmx_newtondynamics_matrix_GetEulerAngles(frontX:Float Ptr, pitch0:Float Ptr, yaw0:Float Ptr, roll0:Float Ptr, pitch1:Float Ptr, yaw1:Float Ptr, roll1:Float Ptr)

End Extern
