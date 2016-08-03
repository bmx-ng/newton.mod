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

Rem
bbdoc: Newton Dynamics
End Rem
Module Newton.Dynamics

ModuleInfo "Version: 1.00"
ModuleInfo "License: zlib"
ModuleInfo "Copyright: Newton Dynamics - 2003-2011 Julio Jerez and Alain Suero"
ModuleInfo "Copyright: Wrapper - 2015-2016 Bruce A Henderson"

ModuleInfo "History: 1.00"
ModuleInfo "History: Initial Release."

ModuleInfo "CC_OPTS: -D_NEWTON_STATIC_LIB"

?win32x86
ModuleInfo "CC_OPTS: -DPTW32_BUILD -DPTW32_STATIC_LIB -D_MINGW_32_VER"
ModuleInfo "CC_OPTS: -msse -msse3 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant"
?win32x64
ModuleInfo "CC_OPTS: -DPTW32_BUILD -DPTW32_STATIC_LIB -D_MINGW_64_VER"
ModuleInfo "CC_OPTS: -msse -msse3 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant"
?macos
ModuleInfo "CC_OPTS: -D_MACOSX_VER -msse4.1"
?linuxx86
ModuleInfo "CC_OPTS: -O2 -msse -msse3"
ModuleInfo "CC_OPTS: -D_POSIX_VER -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant"
?linuxx64
ModuleInfo "CC_OPTS: -O2 -msse -msse3"
ModuleInfo "CC_OPTS: -D_POSIX_VER_64 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant"
?raspberrypi
ModuleInfo "CC_OPTS: -D_POSIX_VER -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant"
?

Import "common.bmx"

Rem
bbdoc: 
End Rem
Type TNWorld

	Field worldPtr:Byte Ptr

	Rem
	bbdoc: Creates an instance of the Newton world.
	End Rem
	Function Create:TNWorld()
		Local this:TNWorld = New TNWorld
		this.worldPtr = bmx_newtondynamics_NewtonCreate(this)
		Return this
	End Function

	Rem
	bbdoc: Creates a rigid body.
	about: Creates a Newton rigid body and assigns a the @collision geometry representing the rigid body.
	Optionally pass a subclassed TNBody object for direct access to force/torque callbacks.
	End Rem
	Method CreateDynamicBody:TNBody(collision:TNCollision, matrix:TNMatrix, custom:TNBody = Null)
		Local body:TNBody
		If Not custom Then
			body = New TNBody
		Else
			body = custom
		End If
		body.bodyPtr = bmx_newtondynamics_NewtonCreateDynamicBody(body, worldPtr, collision.collisionPtr, Varptr matrix.frontX)
		NewtonBodySetForceAndTorqueCallback(body.bodyPtr, TNBody._defaultForceAndTorqueCallback)
		NewtonBodySetTransformCallback(body.bodyPtr, TNBody._defaultTransformCallback)
		Return body
	End Method
	
	Rem
	bbdoc: Creates an empty complex collision geometry tree. 
	End Rem
	Method CreateTreeCollision:TNTreeCollision(shapeID:Int = 0, custom:TNTreeCollision = Null)
		Local tree:TNTreeCollision
		If Not custom Then
			tree = New TNTreeCollision
		Else
			tree = custom
		End If
		tree.collisionPtr = bmx_newtondynamics_NewtonCreateTreeCollision(tree, worldPtr, shapeID)
		Return tree
	End Method
	
	Rem
	bbdoc: Creates a generalized ellipsoid primitive.
	about: Sphere collision are generalized ellipsoids, the application can create many different kind of objects by just playing with dimensions of the radius.
	for example to make a sphere set all tree radius to the same value, to make a ellipse of revolution just set two of the tree radius to the same value.
	End Rem
	Method CreateSphere:TNCollision(radius:Float, shapeID:Int = 0, offsetMatrix:TNMatrix = Null, custom:TNCollision = Null)
		Local sphere:TNCollision
		If Not custom Then
			sphere = New TNCollision
		Else
			sphere = custom
		End If
		If Not offsetMatrix Then
			sphere.collisionPtr = bmx_newtondynamics_NewtonCreateSphere(sphere, worldPtr, radius, shapeID, Null)
		Else
			sphere.collisionPtr = bmx_newtondynamics_NewtonCreateSphere(sphere, worldPtr, radius, shapeID, Varptr offsetMatrix.frontX)
		End If
		Return sphere
	End Method

	Rem
	bbdoc: Creates a box primitive for collision.
	End Rem
	Method CreateBox:TNCollision(dx:Float, dy:Float, dz:Float, shapeID:Int = 0, offsetMatrix:TNMatrix = Null, custom:TNCollision = Null)
		Local box:TNCollision
		If Not custom Then
			box = New TNCollision
		Else
			box = custom
		End If
		If Not offsetMatrix Then
			box.collisionPtr = bmx_newtondynamics_NewtonCreateBox(box, worldPtr, dx, dy, dz, shapeID, Null)
		Else
			box.collisionPtr = bmx_newtondynamics_NewtonCreateBox(box, worldPtr, dx, dy, dz, shapeID, Varptr offsetMatrix.frontX)
		End If
		Return box
	End Method

	Rem
	bbdoc: Creates a cylinder primitive for collision.
	End Rem
	Method CreateCylinder:TNCollision(radius:Float, height:Float, shapeID:Int = 0, offsetMatrix:TNMatrix = Null, custom:TNCollision = Null)
		Local cylinder:TNCollision
		If Not custom Then
			cylinder = New TNCollision
		Else
			cylinder = custom
		End If
		If Not offsetMatrix Then
			cylinder.collisionPtr = bmx_newtondynamics_NewtonCreateCylinder(cylinder, worldPtr, radius, height, shapeID, Null)
		Else
			cylinder.collisionPtr = bmx_newtondynamics_NewtonCreateCylinder(cylinder, worldPtr, radius, height, shapeID, Varptr offsetMatrix.frontX)
		End If
		Return cylinder
	End Method
	
	Rem
	bbdoc: Sets coulomb model of friction.
	about: Allows the application to chose between and exact or an adaptive coulomb friction model.
 
	 0: Is the exact model. Friction forces are calculated in each frame. 
	This model is good for applications where precision is more important than speed, ex: realistic simulation.

	1: Is the adaptive model. Here values from previous frames are used to determine the maximum friction values of the current frame. 
	This is about 10% faster than the exact model however it may introduce strange friction behaviors. For example a 
	bouncing object tumbling down a ramp will act as a friction less object because the contacts do not have continuity. 
	In general each time a new contact is generated the friction value is zero, only if the contact persist a non zero 
	friction values is used. The second effect is that if a normal force is very strong, and if the contact is suddenly 
	destroyed, a very strong friction force will be generated at the contact point making the object react in a non-familiar way. 
	End Rem
	Method SetFrictionModel(model:Int)
		NewtonSetFrictionModel(worldPtr, model)
	End Method
	
	Rem
	bbdoc: Sets the minimum frame rate at which the simulation can run.
	about: The default minimum frame rate of the simulation is 60 frame per second.
	When the simulation falls below the specified minimum frame, Newton will perform sub steps in order to meet the desired minimum FPS. 
	End Rem
	Method SetMinimumFrameRate(framerate:Float)
		NewtonSetMinimumFrameRate(worldPtr, framerate)
	End Method
	
	Rem
	bbdoc: Sets the contact merge tolerance.
	End Rem
	Method SetContactMergeTolerance(tolerance:Float)
		NewtonSetContactMergeTolerance(worldPtr, tolerance)
	End Method
	
	Rem
	bbdoc: Resets all internal states of the engine.
	about: When an application wants to reset the state of all the objects in the world to a predefined initial condition, 
	just setting the initial position and velocity is not sufficient to reproduce equal runs since the engine maintain 
	there are internal states that in order to take advantage of frame to frame coherence.
	In this cases this method will reset all of the internal states.
	End Rem
	Method InvalidateCache()
		NewtonInvalidateCache(worldPtr)
	End Method
	
	Rem
	bbdoc: Advances the simulation by an amount of time.
	about: The Newton Engine does not perform sub-steps, and does not need tuning parameters. It is the responsibility of the application to 
	ensure that @timestep is small enough to guarantee physics stability. 
	End Rem
	Method Update(timestep:Float)
		NewtonUpdate(worldPtr, timestep)
	End Method

	Rem
	bbdoc: Returns the total number of rigid bodies in the world.
	End Rem
	Method GetBodyCount:Int()
		Return NewtonWorldGetBodyCount(worldPtr)
	End Method
	
	Rem
	bbdoc: Returns the total number of constraints in the world.
	End Rem
	Method GetConstraintCount:Int()
		Return NewtonWorldGetConstraintCount(worldPtr)
	End Method

	Rem
	bbdoc: Shoots a ray from p0 to p1 and calling the delegate's OnFilter/OnPreFilter with each ray intersection.
	about: The ray cast will call the application with each body intersecting the line segment. 
	By writing the OnFilter() in different ways the application can implement different flavors of ray casting. 
	For example an all body ray cast can be easily implemented by having the OnFilter() always returning 1.0, and copying each 
	rigid body into an array/list; a closest hit ray cast can be implemented by saving the body with the smaller intersection 
	parameter and returning the parameter t; and a report the first body hit can be implemented by having the filter returning 
	zero after the first call and saving the rigid body.
	End Rem
	Method RayCast(p0x:Float, p0y:Float, p0z:Float, p1x:Float, p1y:Float, p1z:Float, delegate:TNRayCastDelegate, threadIndex:Int = 0)
		If delegate.prefilter Then
			bmx_newtondynamics_NewtonWorldRayCast(worldPtr, p0x, p0y, p0z, p1x, p1y, p1z, TNRayCastDelegate._filterCallback, delegate, TNRayCastDelegate._prefilterCallback, threadIndex)
		Else
			bmx_newtondynamics_NewtonWorldRayCast(worldPtr, p0x, p0y, p0z, p1x, p1y, p1z, TNRayCastDelegate._filterCallback, delegate, Null, threadIndex)
		End If
	End Method

	Rem
	bbdoc: 
	End Rem
	Method Destroy()
		If worldPtr Then
			NewtonDestroy(worldPtr)
			worldPtr = Null
		End If
	End Method
	
End Type

Rem
bbdoc: A helper delegate for handling ray casts.
End Rem
Type TNRayCastDelegate

	Rem
	bbdoc: Whether or not to enable the prefilter delegation.
	about: If @prefilter is True, Newton will call OnPreFilter() right before executing the intersections between the ray and the primitive.
	If the method returns zero, Newton will not ray cast the primitive.
	The application can use this implement faster or smarter filters when implementing complex logic, otherwise for normal all ray cast
	this setting can be False.
	End Rem
	Field prefilter:Int = False

	Rem
	bbdoc: 
	End Rem
	Method OnPreFilter:Int(body:TNBody, collision:TNCollision)
	End Method
	
	Rem
	bbdoc: 
	about: The most common use for the ray cast function is the closest body hit. In this case it is important, for performance reasons, 
	that the OnFilter() method returns the intersection parameter. If the OnFilter() method returns a value of zero the ray cast will terminate 
	immediately.
	End Rem
	Method OnFilter:Float(body:TNBody, shapeHit:TNCollision, hitContact:Float Ptr, hitNormal:Float Ptr, intersectParam:Float)
	End Method


	Function _prefilterCallback:Int(bodyPtr:Byte Ptr, collPtr:Byte Ptr, delPtr:Byte Ptr)
		Local delegate:TNRayCastDelegate = bmx_newtondynamics_RayCastDelegateFromPtr(delPtr)
		Return delegate.OnPreFilter(NewtonBodyGetUserData(bodyPtr), NewtonCollisionGetUserData(collPtr))
	End Function
	
	Function _filterCallback:Float(bodyPtr:Byte Ptr, collPtr:Byte Ptr, hitContact:Float Ptr, hitNormal:Float Ptr, delPtr:Byte Ptr, intersectParam:Float)
		Local delegate:TNRayCastDelegate = bmx_newtondynamics_RayCastDelegateFromPtr(delPtr)
		Return delegate.OnFilter(NewtonBodyGetUserData(bodyPtr), NewtonCollisionGetUserData(collPtr), hitContact, hitNormal, intersectParam)
	End Function
	
End Type


Rem
bbdoc: 
End Rem
Type TNCollision

	Field collisionPtr:Byte Ptr

	Method Destroy()
		If collisionPtr Then
			NewtonDestroyCollision(collisionPtr)
			collisionPtr = Null
		End If
	End Method

End Type

Rem
bbdoc: A complex collision geometry tree.
about: TNTreeCollision is the preferred method within Newton for collision with polygonal meshes of arbitrary complexity.
The mesh must be made of flat non-intersecting polygons, but they do not explicitly need to be triangles.
End Rem
Type TNTreeCollision Extends TNCollision

	Method BeginBuild()
		NewtonTreeCollisionBeginBuild(collisionPtr)
	End Method
	
	Rem
	bbdoc: Finalizes the construction of the polygonal mesh.
	about: After the application has finished adding polygons to the TNTreeCollision, it must call this method to finalize the construction of the collision mesh.
	If concave polygons are added to the *TreeCollision*, the application must call this function with the parameter *optimize* set to 1.
	With the *optimize* parameter set to 1, Newton will optimize the collision mesh by removing non essential edges from adjacent flat polygons.
	Newton will not change the topology of the mesh but significantly reduces the number of polygons in the mesh. The reduction factor of the number of polygons in the mesh depends upon the irregularity of the mesh topology.
	A reduction factor of 1.5 to 2.0 is common. 
	Calling this method with the parameter *optimize* set to zero, will leave the mesh geometry unaltered.
	End Rem
	Method EndBuild(optimize:Int)
		NewtonTreeCollisionEndBuild(collisionPtr, optimize)
	End Method
	
	Rem
	bbdoc: Adds an individual polygon to the TNTreeCollision.
	about: 
	End Rem
	Method AddFace(vertexCount:Int, vertexPtr:Float Ptr, strideInBytes:Int, faceAttribute:Int)
		NewtonTreeCollisionAddFace(collisionPtr, vertexCount, vertexPtr, strideInBytes, faceAttribute)
	End Method
	
End Type

Rem
bbdoc: Creates a rigid body.
about: Can be subclassed in order to override callbacks such as OnForceAndTorque().
End Rem
Type TNBody

	Field bodyPtr:Byte Ptr
	
	' the user callback function
	Field _fatCallback(body:TNBody, timestamp:Float, threadIndex:Int)
	Field _transCallback(body:TNBody, matrix:Float Ptr, threadIndex:Int)
	
	Rem
	bbdoc: 
	End Rem
	Method OnForceAndTorque(timestamp:Float, threadIndex:Int)
	End Method
	
	Method OnTransform(matrix:Float Ptr, threadIndex:Int)
	End Method
	
	Rem
	bbdoc: Sets the transformation matrix of a rigid body.
	about: The application should make sure the transformation matrix has not scale, otherwise unpredictable result will occur.
	End Rem
	Method SetMatrix(matrix:TNMatrix)
		NewtonBodySetMatrix(bodyPtr, Varptr matrix.frontX)
	End Method
	
	Rem
	bbdoc: Gets the transformation matrix of a rigid body, populating @matrix.
	End Rem
	Method GetMatrix(matrix:TNMatrix)
		NewtonBodyGetMatrix(bodyPtr, Varptr matrix.frontX)
	End Method
	
	Rem
	bbdoc: Gets the mass matrix of a rigid body.
	End Rem
	Method GetMassMatrix(mass:Float Var, Ixx:Float Var, Iyy:Float Var, Izz:Float Var)
		NewtonBodyGetMassMatrix(bodyPtr, Varptr mass, Varptr Ixx, Varptr Iyy, Varptr Izz)
	End Method
	
	Rem
	bbdoc: Gets the rotation part of the transformation matrix of a body, in form of a unit quaternion.
	about: The rotation quaternion is the same as what the application would get by using at function to extract a quaternion form a matrix.
	however since the rigid body already contained the rotation in it, it is more efficient to just call this method avoiding expensive conversion. 
	End Rem
	Method GetRotation(q0:Float Var, q1:Float Var, q2:Float Var, q3:Float Var)
		bmx_newtondynamics_NewtonBodyGetRotation(bodyPtr, Varptr q0, Varptr q1, Varptr q2, Varptr q3)
	End Method
	
	Rem
	bbdoc: 
	End Rem
	Method GetInvMass(mass:Float Var, Ixx:Float Var, Iyy:Float Var, Izz:Float Var)
		NewtonBodyGetInvMass(bodyPtr, Varptr mass, Varptr Ixx, Varptr Iyy, Varptr Izz)
	End Method
	
	Rem
	bbdoc: 
	End Rem
	Method GetInertiaMatrix(matrix:TNMatrix)
		NewtonBodyGetInertiaMatrix(bodyPtr, Varptr matrix.frontX)
	End Method
	
	Rem
	bbdoc: 
	End Rem
	Method GetInvInertiaMatrix(matrix:TNMatrix)
		NewtonBodyGetInvInertiaMatrix(bodyPtr, Varptr matrix.frontX)
	End Method
	
	Rem
	bbdoc: Gets the global angular velocity of the body.
	End Rem
	Method GetOmega(ox:Float Var, oy:Float Var, oz:Float Var)
		bmx_newtondynamics_NewtonBodyGetOmega(bodyPtr, Varptr ox, Varptr oy, Varptr oz)
	End Method
	
	Rem
	bbdoc: Gets the global linear velocity of the body.
	End Rem
	Method GetVelocity(vx:Float Var, vy:Float Var, vz:Float Var)
		bmx_newtondynamics_NewtonBodyGetVelocity(bodyPtr, Varptr vx, Varptr vy, Varptr vz)
	End Method
	
	Rem
	bbdoc: Gets the net force applied to a rigid body after the last Newton Update.
	End Rem
	Method GetForce(fx:Float Var, fy:Float Var, fz:Float Var)
		bmx_newtondynamics_NewtonBodyGetForce(bodyPtr, Varptr fx, Varptr fy, Varptr fz)
	End Method
	
	Rem
	bbdoc: Gets the net torque applied to a rigid body after the last Newton Update.
	End Rem
	Method GetTorque(tx:Float Var, ty:Float Var, tz:Float Var)
		bmx_newtondynamics_NewtonBodyGetTorque(bodyPtr, Varptr tx, Varptr ty, Varptr tz)
	End Method
	
	Rem
	bbdoc: Gets the force applied on the last call to OnForceAndTorque/apply force and torque callback.
	about: This method can be useful to modify force from joint callback
	End Rem
	Method GetForceAcc(fx:Float Var, fy:Float Var, fz:Float Var)
		bmx_newtondynamics_NewtonBodyGetForceAcc(bodyPtr, Varptr fx, Varptr fy, Varptr fz)
	End Method
	
	Rem
	bbdoc: Gets the torque applied on the last call to OnForceAndTorque/apply force and torque callback.
	about: This method can be useful to modify torque from joint callback
	End Rem
	Method GetTorqueAcc(tx:Float Var, ty:Float Var, tz:Float Var)
		bmx_newtondynamics_NewtonBodyGetTorqueAcc(bodyPtr, Varptr tx, Varptr ty, Varptr tz)
	End Method
	
	Rem
	bbdoc: Gets the relative position of the center of mass of a rigid body.
	about: This method can be used to set the relative offset of the center of mass of a rigid body.
	when a rigid body is created the center of mass is set the the point c(0, 0, 0), and normally this is 
	the best setting for a rigid body. However there are situations in which and object does not have symmetry or
	simple some kind of special effect is desired, and this origin needs to be changed.
	End Rem
	Method GetCentreOfMass(cx:Float Var, cy:Float Var, cz:Float Var)
		bmx_newtondynamics_NewtonBodyGetCentreOfMass(bodyPtr, Varptr cx, Varptr cy, Varptr cz)
	End Method
	
	Rem
	bbdoc: Gets the linear viscous damping of the body.
	End Rem
	Method GetAngularDampling(aX:Float Var, aY:Float Var, aZ:Float Var)
		bmx_newtondynamics_NewtonBodyGetAngularDamping(bodyPtr, Varptr aX, Varptr aY, Varptr aZ)
	End Method

	Rem
	bbdoc: Gets the world axis aligned bounding box (AABB) of the body.
	End Rem
	Method NewtonBodyGetAABB(p0x:Float Var, p0y:Float Var, p0z:Float Var, p1x:Float Var, p1y:Float Var, p1z:Float Var)
		bmx_newtondynamics_NewtonBodyGetAABB(bodyPtr, Varptr p0x, Varptr p0y, Varptr p0z, Varptr p1x, Varptr p1y, Varptr p1z)
	End Method
	
	Rem
	bbdoc: Sets the linear viscous damping of the body.
	about: The default value of *angularDamp* is clamped to a value between 0.0 and 1.0; the default value is 0.1,
	There is a non zero implicit attenuation value of 0.0001 assumed by the integrator.
	End Rem
	Method SetAngularDampling(aX:Float, aY:Float, aZ:Float)
		bmx_newtondynamics_NewtonBodySetAngularDamping(bodyPtr, aX, aY, aZ)
	End Method

	Rem
	bbdoc: Assigns an event function for applying external force and torque to a rigid body.
	about: The default is to call the OnForceAndTorque() method, which can be overriden by subclassing TNBody and
	re-implementing the method.
	The callback is called by the Newton Engine every time an active body is going to be simulated. 
	The Newton Engine does not call the callback for bodies that are inactive or have reached a state of stable equilibrium.
	End Rem
	Method SetForceAndTorqueCallback(callback(body:TNBody, timestamp:Float, threadIndex:Int))
		_fatCallback = callback
		NewtonBodySetForceAndTorqueCallback(bodyPtr, _forceAndTorqueCallback)
	End Method
	
	Rem
	bbdoc: Assign a transformation event function to the body.
	about: The default is to call the OnTransform() method, which can be overriden by subclassing TNBody and
	re-implementing the method.
	The callback is called by the Newton engine every time a visual object that represents the rigid body has changed.
	The Newton engine does not call the callback for bodies that are inactive or have reached a state of stable equilibrium.
	End Rem
	Method SetTransformCallback(callback(body:TNBody, matrix:Float Ptr, threadIndex:Int))
		_transCallback = callback
		NewtonBodySetTransformCallback(bodyPtr, _transformCallback)
	End Method
	
	Rem
	bbdoc: Sets the global angular velocity of the body.
	End Rem
	Method SetOmega(ox:Float, oy:Float, oz:Float, ow:Float)
		bmx_newtondynamics_NewtonBodySetOmega(bodyPtr, ox, oy, oz, ow)
	End Method
	
	Rem
	bbdoc: Sets the net force applied to a rigid body.
	about: This method is only effective when called from OnForceAndTorque/apply force and torque callback.
	End Rem
	Method SetForce(fx:Float, fy:Float, fz:Float, fw:Float)
		bmx_newtondynamics_NewtonBodySetForce(bodyPtr, fx, fy, fz, fw)
	End Method
	
	Rem
	bbdoc: Sets the global linear velocity of the body.
	End Rem
	Method SetVelocity(vx:Float, vy:Float, vz:Float, vw:Float)
		bmx_newtondynamics_NewtonBodySetVelocity(bodyPtr, vx, vy, vz, vw)
	End Method

	Rem
	bbdoc: Sets the net torque applied to a rigid body.
	about: This method is only effective when called from OnForceAndTorque/apply force and torque callback.
	End Rem
	Method SetTorque(tx:Float, ty:Float, tz:Float, tw:Float)
		bmx_newtondynamics_NewtonBodySetTorque(bodyPtr, tx, ty, tz, tw)
	End Method
	
	Rem
	bbdoc: 
	End Rem
	Method SetMassProperties(mass:Float, collision:TNCollision)
		NewtonBodySetMassProperties(bodyPtr, mass, collision.collisionPtr)
	End Method
	
	Rem
	bbdoc: Applies the linear viscous damping coefficient to the body. 
	about: The default value of @linearDamp is clamped to a value between 0.0 and 1.0; the default value is 0.1,
	There is a non zero implicit attenuation value of 0.0001 assume by the integrator.
	End Rem
	Method SetLinearDamping(linearDamp:Float)
		NewtonBodySetLinearDamping(bodyPtr, linearDamp)
	End Method
	
	' internal
	Function _forceAndTorqueCallback(bodyPtr:Byte Ptr, timestamp:Float, threadIndex:Int)
		Local body:TNBody = NewtonBodyGetUserData(bodyPtr)
		body._fatCallback(body, timestamp, threadIndex)
	End Function

	Function _defaultForceAndTorqueCallback(bodyPtr:Byte Ptr, timestamp:Float, threadIndex:Int)
		Local body:TNBody = NewtonBodyGetUserData(bodyPtr)
		body.OnForceAndTorque(timestamp, threadIndex)
	End Function

	Function _transformCallback(bodyPtr:Byte Ptr, matrix:Float Ptr, threadIndex:Int)
		Local body:TNBody = NewtonBodyGetUserData(bodyPtr)
		body._transCallback(body, matrix, threadIndex)
	End Function

	Function _defaultTransformCallback(bodyPtr:Byte Ptr, matrix:Float Ptr, threadIndex:Int)
		Local body:TNBody = NewtonBodyGetUserData(bodyPtr)
		body.OnTransform(matrix, threadIndex)
	End Function

	Rem
	bbdoc: 
	End Rem
	Method Destroy()
		If bodyPtr Then
			NewtonDestroyBody(bodyPtr)
			bodyPtr = Null
		End If
	End Method

End Type

Rem
bbdoc: 
End Rem
Type TNMatrix

	Field frontX:Float
	Field frontY:Float
	Field frontZ:Float
	Field frontW:Float

	Field upX:Float
	Field upY:Float
	Field upZ:Float
	Field upW:Float

	Field rightX:Float
	Field rightY:Float
	Field rightZ:Float
	Field rightW:Float

	Field positX:Float
	Field positY:Float
	Field positZ:Float
	Field positW:Float

	Rem
	bbdoc: 
	End Rem
	Function GetIdentityMatrix:TNMatrix()
		Local matrix:TNMatrix = New TNMatrix
		matrix.frontX = 1
		matrix.upY = 1
		matrix.rightZ = 1
		matrix.positW = 1
		Return matrix
	End Function

	Rem
	bbdoc: 
	End Rem
	Method GetEulerAngles(pitch0:Float Var, yaw0:Float Var, roll0:Float Var, pitch1:Float Var, yaw1:Float Var, roll1:Float Var)
		bmx_newtondynamics_matrix_GetEulerAngles(Varptr frontX, Varptr pitch0, Varptr yaw0, Varptr roll0, Varptr pitch1, Varptr yaw1, Varptr roll1)
	End Method
	
End Type

Extern
	Function NewtonBodyGetUserData:TNBody(body:Byte Ptr)
	Function NewtonCollisionGetUserData:TNCollision(coll:Byte Ptr)
	Function bmx_newtondynamics_RayCastDelegateFromPtr:TNRayCastDelegate(del:Byte Ptr)
End Extern
