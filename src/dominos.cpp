/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

/*
Ball(b2World* world, uint16 categoryBits, uint16 maskBits) {
    myFixtureDef.filter.categoryBits = categoryBits;
    myFixtureDef.filter.maskBits = maskBits;
 }
 */

namespace cs296
{
  /** The is the constructor for the physical system to be simulated. <br> 
   * What follows, is the documentation for various blocks of the system. <br><br>
   * The 10 blocks have been listed first along with their documentation. <br>
   * This follows with the 3 blocks added by us and their documentation. <br>
   * All the values assigned to physical parameters, follow MKS system(metre-kilogram-second), wherever meaningful.<br>
   * All bodies have been created by calling member function 'CreateBody' of class b2World on variable 'm_world', <br>
   * which is a pointer variable to our b2World class instance. <br>
   * Similarly, fixtures and joints have been created using 'CreateFixture' and 'CreateJoint' member functions respectively. <br>
   */ 
  
  dominos_t::dominos_t()
  {

    //!**********Ground************************************************************************************************<br><br>
    //! Variable name: b1 , Datatype: b2Body* <br>
    //! Details: Pointer to an instance of rigid body(ground here).
    b2Body* b1;  
    {
      //! Variable name: shape , Datatype: b2EdgeShape <br>
      //! Details: Represents an edge with end-points as b2Vec2(-90.0f, 0.0f) and b2Vec2(90.0f, 0.0f).
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds the data to construct the rigid body (ground).
      b2BodyDef bd; 

      //! Details: Body is created in 'm_world' using CreateBody function and pointer returned is stored in 'b1'.<br>
      //! A fixture is attached to the body using CreateFixture function, passing address of 'shape' and density=0.0f.
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
          /*
    //!***********The top horizontal shelf******************************************************************************* 
    {
      //! Variable name: shape , Datatype: b2PolyginShape <br>
      //! Details: The polygon is set to a rectangular box with x-extent= 6.0f and y-extent= 0.25f.
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct rigid body(the shelf). Position coordinates set to (-31.0f,30.0f).
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);

      //! Variable name: ground , Datatype: b2Body* <br>
      //! Details: Assigned with a pointer to the body created for the shelf. Fixture attached as before.
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //!***********Dominos***********************************************************************************************
    {
      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangular box with x-extent= 0.1f and y-extent= 1.0f. Represents domino shape.
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      //! Variable name: fd , Datatype: b2FixtureDef <br>
      //! Details: Stores info about fixtures of the body. <br>
      //! Fixture attributes: shape= &shape, density= 20.0f, friction= 0.1f.
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      //! Variable name: bd, Datatype: b2BodyDef <br>
      //! Details: Holds data needed to create 10 instances of a movable rigid body(domino). <br>
      //! Attributes: Iteratively position is set to (-35.5f + 1.0f*i, 31.25f) for 'i' from 0-9, type=b2_dynamicBody<br>
      //! Variable name: body , Datatype: b2Body* <br>
      //! Details: Assigned with a pointer to the body created for a domino. Fixture attached by passing address of 'fd'.
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      
    //!***********Another horizontal shelf*******************************************************************************
    {
      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=7.0f, y-extent=0.25f, center=(-20.0f,20.0f), angle=0.0f.
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct rigid body(the shelf). Position set to (1.0f, 6.0f).
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);

      //! Variable name: ground , Datatype: b2Body* <br>
      //! Details: Assigned with a pointer to the body created for the shelf. <br>
      //! Fixture attached to the body by passing address of 'shape', density=0.0f.
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //!***********Pendulum that knocks the dominos off*****************************************************************
    {
      //! Variable name: b2 , Datatype: b2Body* <br>
      //! Details: Pointer to an instance of rigid body(the static block supporting the pendulum bob).
      b2Body* b2;
      { 
        //! Variable name: shape , Datatype: b2PolygonShape <br>
        //! Details: The polygon is set to a rectangular box with x-extent=0.25f, y-extent=1.5f. 
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	//! Variable name: bd , Datatype: b2BodyDef <br>
	//! Details: Holds data to construct the rigid body(supporting static block). Position set to (-36.5f,28.0f).<br>
	//! Fixture is then created by passing address of 'shape' and density= 10.0f.
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

      //! Variable name: b4 , Datatype: b2Body* <br>
      //! Details: Pointer to an instance of rigid body(pendulum bob).	
      b2Body* b4;
      {
	//! Variable name: shape , Datatype: b2PolygonShape <br>
	//! Details: The polygon is set to a rectangle with x-extent=0.25f, y-extent=0.25f. 
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	//! Variable name: bd , Datatype: b2BodyDef <br>
	//! Details: Holds data to construct the pendulum bob. Position set to (-40.0f, 33.0f), type=b2_dynamicBody. <br>
        //! 'b4' is assigned with the pointer to the body created using 'bd' in m_world. <br>
	//! Fixture is then created by passing address of 'shape' and density= 2.0f;
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      //! Variable name: jd , Datatype: b2RevoluteJointDef <br>
      //! Details: Holds data to construct a revolute joint between bodies 'b2' and 'b4'.<br>
      //! Variable name: anchor , Datatype: b2Vec2 <br>
      //! Position of 'anchor' set to (-37.0f, 40.0f). Joint initialized with 'b2','b4' and the anchor point. <br>
      //! Joint created by passing address of 'jd' to CreateJoint function, called on 'm_world'.
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    //!***********The train of small spheres**************************************************************************
    {
      //! Variable name: spherebody , Datatype: b2Body* <br>
      //! Details: Points to an instance of a rigid body(sphere).
      b2Body* spherebody;
	
      //! Variable name: circle , Datatype: b2CircleShape <br>
      //! Details: A circle shape is initialized, and its radius set to 0.5f.
      b2CircleShape circle;
      circle.m_radius = 0.5f;
	
      //! Variable name: ballfd , Datatype: b2FixtureDef <br>
      //! Details: Stores info about fixtures of the body(the small sphere). <br>
      //! Fixture attributes: shape= &circle, density= 1.0f, friction= 0.0f, restitution= 0.0f.
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      //! Variable name: ballbd, Datatype: b2BodyDef <br>
      //! Details: Holds data needed to create 10 instances of a movable rigid body(small sphere). <br>
      //! Body attributes: Iteratively position is set to (-22.2f + i*1.0f, 26.6f) for 'i' from 0-9, type=b2_dynamicBody<br>
      //! Details: 'spherebody' is assigned with a pointer to the body created for each small sphere, <br>
      //! and a fixture is attached to the body by passing address of 'ballfd'.
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0f, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //! ***********The pulley system*********************************************************************************
    {
      //! Variable name: bd , Datatype: b2BodyDef* <br>
      //! Details: Initialized to point to a new instance of a b2BodyDef class. Body attributes are set as: <br>
      //! type=b2_dynamicBody, position= (-10, 15), fixedRotation= true.
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //! ----The open box-----------------------
  
      //! Variable name: fd1 , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture attributes are set as: <br>
      //! density= 10.0, friction= 0.5f, restitution= 0.0f <br>
      //! Variable name: bs1 , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=2, y-extent=0.2, center=(0.f,-1.9f), angle=0 <br>
      //! Finally, shape attribute of '*fd1' is assigned '&bs1'.
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;

      //! Variable name: fd2 , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture attributes are set as: <br>
      //! density= 10.0, friction= 0.5f, restitution= 0.0f <br>
      //! Variable name: bs2 , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=0.2, y-extent=2, center=(2.0f,0.f), angle=0 <br>
      //! Finally, shape attribute of '*fd2' is assigned '&bs2'.
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;

      //! Variable name: fd3 , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture attributes are set as: <br>
      //! density= 10.0, friction= 0.5f, restitution= 0.0f <br>
      //! Variable name: bs3 , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=0.2, y-extent=2, center=(-2.0f,0.f), angle=0 <br>
      //! Finally, shape attribute of '*fd3' is assigned '&bs3'.
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      //! Variable name: box1 , Datatype: b2Body* <br>
      //! Details: 'box1' is assigned with a pointer to the body created for the open box using 'bd'. <br>
      //! Fixtures are then added to the body using 'fd1', 'fd2' and 'fd3'.
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //! ----The bar---------------------------- 

      //! Details: position of 'bd' reset to (10,15), and density to 34.0 <br>
      //! Variable name: box2 , Datatype: b2Body* <br>
      //! Details: 'box2' is assigned with a pointer to the body created in m_world using 'bd'. <br>
      //! Fixture is then attached to it, using 'fd1'.
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      //! ----The pulley joint-------------------

      //! Variable name: myjoint , Datatype: b2PulleyJointDef* <br>
      //! Details: Points to a new instance of b2PulleyJointDef, created using constructor for the class. 
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();

      //! Variable name: worldAnchorOnBody1 , Datatype: b2Vec2 <br>
      //! Details: Anchor point on body 1 in world axis. Set to (-10,15).
      b2Vec2 worldAnchorOnBody1(-10, 15);
      //! Variable name: worldAnchorOnBody2 , Datatype: b2Vec2 <br>
      //! Details: Anchor point on body 2 in world axis. Set to (10,15).
      b2Vec2 worldAnchorOnBody2(10, 15);
      //! Variable name: worldAnchorGround1 , Datatype: b2Vec2 <br>
      //! Details: Anchor point for ground 1 in world axis. Set to (-10,20).
      b2Vec2 worldAnchorGround1(-10, 20);
      //! Variable name: worldAnchorGround2 , Datatype: b2Vec2 <br>
      //! Details: Anchor point for ground 2 in world axis. Set to (10,20).
      b2Vec2 worldAnchorGround2(10, 20);

      //! Variable name: ratio , Datatype: float32 <br>
      //! Details: Serves as the pulley ratio, value set to 1.0f. <br>
      //! 'myjoint' is initialized with the bodies, ground anchor positions in world coordinates, body anchor <br>
      //! positions in body coordinates, and the pulley ratio. 
      //! Finally, the pulley joint is created in m_world using 'myjoint'.
      float32 ratio = 1.0f;
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //! ***********The revolving horizontal platform*****************************************************************
    {
      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=2.2f, y-extent=0.2f <br>
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the horizontal platform. Position set to (14.0f,14.0f), type=b2_dynamicBody.<br>
      //! Variable name: body , Datatype: b2Body* <br>
      //! Details: Assigned with pointer to body created in 'm_world' for the platform using 'bd'. <br>
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      //! Variable name: fd , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture density set to 1.f <br>
      //! The 'shape' parameter of 'fd' is then initialized, and assigned with '&shape'<br>
      //! Finally, a fixture is then added to 'body' using 'fd'.
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangle with x-extent=0.2f, y-extent=2.0f. <br>
      //! Note that this variable remains unused, as the body created below is not given any fixture. 
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      
      //! Variable name: bd2 , Datatype: b2BodyDef <br>
      //! Details: This holds data for the other body needed for the revolute joint. <br>
      //! Variable name: body2 , Datatype: b2Body* <br>
      //! Details: Assigned pointer to the (dummy)body created in 'm_world' using 'bd2'. <br>  
      //! The body has been created, but without a fixture and hence doesn't show physical interference with the system.
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);
      
      //! Variable name: jointdef , Datatype: b2RevoluteJointDef <br>
      //! Details: Holds data to construct a revolute joint between bodies 'body' and 'body2'.<br>
      //! Joint attributes set as: localAnchorA=(0,0) , localAnchorB=(0,0) and collideConnected=false. <br>
      //! Joint created by passing address of 'jointdef' to CreateJoint function, called on 'm_world'.
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }  
	
    //! ***********The heavy sphere on the platform******************************************************************
    {
      //! Variable name: sbody , Datatype: b2Body* <br>
      //! Variable name: circle , Datatype: b2CircleShape <br>
      //! Details: A circle shape is initialized, and its radius set to 1.0.
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      //! Variable name: ballfd , Datatype: b2FixtureDef <br>
      //! Details: Stores info about fixtures of the body(the sphere). <br>
      //! Fixture attributes: shape= &circle, density= 50.0f, friction= 0.0f, restitution= 0.0f.
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      //! Variable name: ballbd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the heavy sphere. Position set to (14.0f,18.0f) and type=b2_dynamicBody. <br>
      //! 'sbody' is assigned with pointer to the body created for the sphere in 'm_world' using 'ballbd'. <br> 
      //! Fixture is then created by passing address of 'ballfd'.       
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //! ***********The see-saw system at the bottom******************************************************************
    {
      //! ----The triangle wedge----------------- 

      //! Variable name: sbody , Datatype: b2Body* <br>
      //! Variable name: poly , Datatype: b2PolygonShape <br>
      //! Details: 'poly' is set to a triangle with the 3 vertices as (-1,0), (1,0) and (0,1.5)
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      
      //! Variable name: wedgefd , Datatype: b2FixtureDef <br>
      //! Details: Holds the data for the body fixture. Its attributes are set as: <br>
      //! shape= &poly, density= 10.0f, friction= 0.0f, restitution= 0.0f 
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;

      //! Variable name: wedgebd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the body for triangular wedge. Position set to (30.0f,0.0f). <br>
      //! 'sbody' is assigned with pointer to the body created for the wedge in 'm_world' using 'wedgebd'. <br> 
      //! Fixture is then created by passing address of 'wedgefd'.      
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //! ----The plank on top of the wedge------

      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! The polygon is set to a rectangle with x-extent= 15.0f, y-extent= 0.2f .
      //! Variable name: bd2 , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the plank. Position set to (30.0f,1.5f) and type=b2_dynamicBody. <br>
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);

      //! Variable name: fd2 , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture density set to 1.f <br>
      //! The 'shape' parameter of 'fd2' is then initialized, and assigned with '&shape'<br>
      //! Finally, a fixture is then added to 'body' using 'fd2'.
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      //! Variable name: jd , Datatype: b2RevoluteJointDef <br>
      //! Details: Holds data to construct a revolute joint between bodies 'sbody' and 'body'.<br>
      //! Variable name: anchor , Datatype: b2Vec2 <br>
      //! Position of 'anchor' set to (30.0f, 1.5f). Joint initialized with 'sbody','body' and the anchor point. <br>
      //! Joint created by passing address of 'jd' to CreateJoint function, called on 'm_world'.
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //! ----Light box on rightside of seesaw---

      //! Variable name: shape2 , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangular box(light box on seesaw) with x-extent= 2.0f, y-extent= 2.0f.<br>
      //! Variable name: bd3 , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the box. Position set to (40.0f, 2.0f) and type=b2_dynamicBody. <br>
      //! Variable name: body3 , Datatype: b2Body* <br>
      //! Details: Assigned with pointer to the body created for the light box in 'm_world' using 'bd3'. 
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);

      //! Variable name: fd3 , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture density set to 0.01f <br>
      //! The 'shape' parameter of 'fd2' is then initialized, and assigned with '&shape'<br>
      //! Finally, a fixture is then added to 'body3' using 'fd3'.
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }

    //! ***********New items Added***********************************************************************************
    
    //! ***********ITEM-1 : Lower-left sphere***************************************************************************
    {
      //! Variable name: sbody , Datatype: b2Body* <br>
      //! Variable name: circle , Datatype: b2CircleShape <br>
      //! Details: A circle shape is initialized, and its radius set to 1.3.
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.3;
	
      //! Variable name: ballfd , Datatype: b2FixtureDef <br>
      //! Details: Stores info about fixtures of the body(the sphere). <br>
      //! Fixture attributes: shape= &circle, density= 10.0f, friction= 0.0f, restitution= 0.9f.
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 10.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.9f;

      //! Variable name: ballbd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the heavy sphere body. Position set to (-27.0f,13.0f) and type=b2_dynamicBody. <br>
      //! 'sbody' is assigned with pointer to the body created for the sphere in 'm_world' using 'ballbd'. <br> 
      //! Fixture is then created by passing address of 'ballfd'.       
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-27.0f, 13.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
    
    //! --------Horizontal shelf holding the lower-left sphere---------
    {
      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangular box with x-extent=1.0f, y-extent=0.2f.
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 0.2f);
	
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct rigid body(the shelf). Position set to (-28.0f, 10.0f).
      b2BodyDef bd;
      bd.position.Set(-28.0f, 10.0f);

      //! Variable name: ground , Datatype: b2Body* <br>
      //! Details: Assigned with a pointer to the body created for the shelf.<br> 
      //! Fixture attached to the body by passing address of 'shape', density=0.0f.
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //! ***********ITEM-2 : Triangular wedge*************************************************************************
    {
      //! Variable name: sbody , Datatype: b2Body* <br>
      //! Variable name: poly , Datatype: b2PolygonShape <br>
      //! Details: 'poly' is set to a triangle with the 3 vertices as (0,0), (4,0) and (0,4)
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(0,0);
      vertices[1].Set(4,0);
      vertices[2].Set(0,4);
      poly.Set(vertices, 3);
      
      //! Variable name: wedgefd , Datatype: b2FixtureDef <br>
      //! Details: Holds the data for the body fixture. Its attributes are set as: <br>
      //! shape= &poly, density= 50.0f, friction= 0.001f, restitution= 0.9f <br>
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 50.0f;
      wedgefd.friction = 0.001f;
      wedgefd.restitution = 0.9f;

      //! Variable name: wedgebd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the body for triangular wedge. Position set to (-30.0f,0.0f). <br>
      //! 'sbody' is assigned with pointer to the body created for the wedge in 'm_world' using 'wedgebd'. <br> 
      //! Fixture is then created by passing address of 'wedgefd'.      
      b2BodyDef wedgebd;
      wedgebd.type = b2_dynamicBody;
      wedgebd.position.Set(-30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
    }

     //! ***********ITEM-3 : Rotating Rod to the left of pulley**********************************************************
    {
      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangular box with x-extent=0.2f, y-extent=6.7f 
      b2PolygonShape shape;
      shape.SetAsBox(0.2f, 6.7f);
	
      //! Variable name: bd , Datatype: b2BodyDef <br>
      //! Details: Holds data to construct the vertical rod. Position set to (-20.0f, 7.0f), type=b2_dynamicBody. <br>
      //! Variable name: body , Datatype: b2Body* <br>
      //! Details: Assigned with pointer to body created in 'm_world' for the platform using 'bd'.
      b2BodyDef bd;
      bd.position.Set(-20.0f, 7.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      
      //! Variable name: fd , Datatype: b2FixtureDef* <br>
      //! Details: Initialized to point to a new instance of b2FixtureDef class. Fixture density=1.f restitution=1.0f <br>
      //! The 'shape' parameter of 'fd' is then initialized, and assigned with '&shape'<br>
      //! Finally, a fixture is then added to 'body' using 'fd'.
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->restitution = 1.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      //! Variable name: shape , Datatype: b2PolygonShape <br>
      //! Details: The polygon is set to a rectangular box with x-extent=2.0f, y-extent=0.2f.<br>
      //! Note that this variable remains unused, as the body created below is not given any fixture. 
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 0.2f);

      //! Variable name: bd2 , Datatype: b2BodyDef <br>
      //! Details: This holds data for the other body needed for the revolute joint. <br>
      //! Variable name: body2 , Datatype: b2Body* <br>
      //! Details: Assigned pointer to the (dummy)body created in 'm_world' using 'bd2'. <br>  
      //! The body has been created, but without a fixture and hence doesn't interfere physically with the system.
      b2BodyDef bd2;
      bd2.position.Set(-20.0f, 8.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      //! Variable name: jointdef , Datatype: b2RevoluteJointDef <br>
      //! Details: Holds data to construct a revolute joint between bodies 'body' and 'body2'.<br>
      //! Joint attributes set as: localAnchorA=(0,0) , localAnchorB=(0,0) and collideConnected=false. <br>
      //! Joint created by passing address of 'jointdef' to CreateJoint function, called on 'm_world'.
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


*/
    // New Items in the project
    {
      b2BodyDef *bd0 = new b2BodyDef;
      bd0->type = b2_dynamicBody;
      bd0->position.Set(0,50);
      bd0->fixedRotation = false;
      b2Body* body0 = m_world->CreateBody(bd0);
      b2FixtureDef *fd0;

    {
      b2PolygonShape poly0;             // big square main
      b2Vec2 vertices[4];
      vertices[0].Set(-10.0,-10.0);
      vertices[1].Set(-10.0,10.0); 
      vertices[2].Set(10.0,10.0);
      vertices[3].Set(10.0,-10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;              // small window(left) inside big square
      b2Vec2 vertices[4];
      vertices[0].Set(-7.0,5.5);
      vertices[1].Set(-1.0,5.5); 
      vertices[2].Set(-1.0,0.0);
      vertices[3].Set(-7.0,0.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;              // above window's shutter
      b2Vec2 vertices[4];
      vertices[0].Set(-7.0,6.5);
      vertices[1].Set(-7.0,7.5); 
      vertices[2].Set(-1.0,7.5);
      vertices[3].Set(-1.0,6.5);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;               // small window(right) inside big square
      b2Vec2 vertices[4];
      vertices[0].Set(2.0,-7.0);
      vertices[1].Set(2.0,7.0); 
      vertices[2].Set(8.0,7.0);
      vertices[3].Set(8.0,-7.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;               // above window's outer border
      b2Vec2 vertices[4];
      vertices[0].Set(1.5,-7.5);
      vertices[1].Set(1.5,7.5); 
      vertices[2].Set(8.5,7.5);
      vertices[3].Set(8.5,-7.5);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                // thin long vertical bar to the left of big square
      b2Vec2 vertices[4];
      vertices[0].Set(-12.0,-10.0);
      vertices[1].Set(-12.0,10.0); 
      vertices[2].Set(10.0,10.0);
      vertices[3].Set(10.0,-10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                // thin long horizontal bar above the big square
      b2Vec2 vertices[4];
      vertices[0].Set(-14.0,10.0);
      vertices[1].Set(-14.0,12.0); 
      vertices[2].Set(14.0,12.0);
      vertices[3].Set(12.0,10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                 // leftmost block just below big square
      b2Vec2 vertices[4];
      vertices[0].Set(-12.0,-10.0);
      vertices[1].Set(-16.0,-14.0);
      vertices[2].Set(-6.0,-12.0);
      vertices[3].Set(-6.0,-10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                  // middle block just below big square
      b2Vec2 vertices[4];
      vertices[0].Set(-6.0,-12.0);
      vertices[1].Set(-6.0,-10.0); 
      vertices[2].Set(6.0,-10.0);
      vertices[3].Set(6.0,-12.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                  // rightmost block just below big square
      b2Vec2 vertices[5];
      vertices[0].Set(6.0,-12.0);
      vertices[1].Set(6.0,-10.0); 
      vertices[2].Set(11.0,-10.0);
      vertices[3].Set(11.0,-13.0);
      vertices[4].Set(9.0,-13.0);
      poly0.Set(vertices, 5);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                 // block for the lower body of the bulldozer
      b2Vec2 vertices[7];
      vertices[0].Set(-16.0,-14.0);
      vertices[1].Set(-16.0,-23.0);
      vertices[2].Set(-14.0,-25.0);
      vertices[3].Set(9.0,-25.0);
      vertices[4].Set(9.0,-13.0);
      vertices[5].Set(6.0,-12.0);
      vertices[6].Set(-6.0,-12.0);
      poly0.Set(vertices, 7);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // thin vertical bar(lower) to the right of big square
      b2Vec2 vertices[4];
      vertices[0].Set(10.0,-10.0);
      vertices[1].Set(10.0,-1.0); 
      vertices[2].Set(11.0,-1.0);
      vertices[3].Set(11.0,-10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // small block to the right of the above bar
      b2Vec2 vertices[4];
      vertices[0].Set(11.0,-6.0);
      vertices[1].Set(11.0,-8.0); 
      vertices[2].Set(13.0,-8.0);
      vertices[3].Set(13.0,-6.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // block attached to the right of the above part
      b2Vec2 vertices[4];
      vertices[0].Set(13.0,-10.0);
      vertices[1].Set(13.0,-4.0); 
      vertices[2].Set(23.0,-4.0);
      vertices[3].Set(23.0,-10.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                // block projecting out from above the thin vertical bar(right)
      b2Vec2 vertices[4];
      vertices[0].Set(10.0,-1.0);
      vertices[1].Set(10.0,2.5); 
      vertices[2].Set(25.0,1.5);
      vertices[3].Set(25.0,-1.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // block for the siren (to the right of big square)
      b2Vec2 vertices[4];
      vertices[0].Set(15.0,-4.0);
      vertices[1].Set(15.0,4.0); 
      vertices[2].Set(18.0,4.0);
      vertices[3].Set(18.0,-4.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // triangle on top of the siren
      b2Vec2 vertices[3];
      vertices[0].Set(14.0,4.0);
      vertices[1].Set(16.5,7.0); 
      vertices[2].Set(19.0,4.0);
      poly0.Set(vertices, 3);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                   // block for chimney (to the right of siren)
      b2Vec2 vertices[4];
      vertices[0].Set(20.5,-4.0);
      vertices[1].Set(20.5,8.0); 
      vertices[2].Set(22.5,8.0);
      vertices[3].Set(22.5,-4.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                    // outer trapezium to the lower right of big square
      b2Vec2 vertices[4];
      vertices[0].Set(9.0,-13.0);
      vertices[1].Set(25.0,-13.0); 
      vertices[2].Set(25.0,-25.0);
      vertices[3].Set(9.0,-20.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                    // inner trapezium to the lower right of big square
      b2Vec2 vertices[4];
      vertices[0].Set(11.0,-15.0);
      vertices[1].Set(23.0,-15.0); 
      vertices[2].Set(23.0,-23.0);
      vertices[3].Set(11.0,-19.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    {
      b2PolygonShape poly0;                    // block attached above the right end of the trapezium
      b2Vec2 vertices[4];
      vertices[0].Set(23.0,-13.0);
      vertices[1].Set(23.0,-3.0); 
      vertices[2].Set(25.0,-3.0);
      vertices[3].Set(25.0,-13.0);
      poly0.Set(vertices, 4);
      fd0 = new b2FixtureDef();
      fd0->shape = &poly0;
      body0->CreateFixture(fd0);
    }

    }
 }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
