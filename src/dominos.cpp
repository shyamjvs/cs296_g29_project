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

#include "stdio.h"
#include "iostream"
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

	
	//! Variable name: body0, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the bulldozer
	b2Body* body0;
	
	//! Variable name: lwheel, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the left wheel of the bulldozer
	b2Body* lwheel;
	//! Variable name: rwheel, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the right wheel of the bulldozer
	b2Body* rwheel;
	
	//! Variable name: body1, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the claw at the rear end
	b2Body* body1;
	
	//! Variable name: body2, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the arm at the rear end
	b2Body* body2;
	
	//! Variable name: body3, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the horizontal part which holds the push frame of the bulldozer
	b2Body* body3;
	
	//! Variable name: body4, Datatype: b2Body* <br>
	//! Details: Pointer to an instance of the body for the front blade
	b2Body* body4;
	
	//! Variable name: joint1 , Datatype: b2RevoluteJointDef <br>
	//! Details: Holds data to construct a revolute joint between bodies
	b2RevoluteJointDef joint1;

	//! Function name: keyboard, Datatype: void <br>
	//! Details: Detects the inputs through keyboard and moves the parts based on the inputs
	void dominos_t::keyboard(unsigned char key)
	{
		switch(key){
			//movement of the body
			case('d'):
				rwheel->ApplyAngularImpulse( -500,0 );
				lwheel->ApplyAngularImpulse( -500,0 );
				break;
			case('a'):
				rwheel->ApplyAngularImpulse( 500,0 );
				lwheel->ApplyAngularImpulse( 500,0 );
				break;
			case('f'):
				if(rwheel->GetAngularVelocity() > 0){
					rwheel->ApplyAngularImpulse( -700,0 );
					lwheel->ApplyAngularImpulse( -700,0 );
				}
				else if(rwheel->GetAngularVelocity() < 0){
					rwheel->ApplyAngularImpulse( 700,0 );
					lwheel->ApplyAngularImpulse( 700,0 );
				}
				break;
			
			// movement of the back part
			case('y'):
				body1->SetActive(true);
				//body1->ApplyForce( b2Vec2(-100,0), b2Vec2(-49,4), true );
				body1->ApplyAngularImpulse( -5,0 );
				break;
			case('h'):
				body1->SetActive(true);
				//body1->ApplyForce( b2Vec2(100,0), b2Vec2(-49,4), true );
				body1->ApplyAngularImpulse( 5,0 );
				break;
				
			// movement of the second part
			case('u'):
				body2->SetActive(true);
				//body2->ApplyForce( b2Vec2(-100,0), b2Vec2(-23,-9), true );
				body2->ApplyAngularImpulse( -200,0 );
				break;
			case('j'):
				body2->SetActive(true);
				//body2->ApplyForce( b2Vec2(100,0), b2Vec2(-23,-9), true );
				body2->ApplyAngularImpulse( 200,0 );
				break;
				
			// movement of the horizontal rod
			case('i'):
				body3->SetActive(true);
				//body3->ApplyForce( b2Vec2(0,1200), b2Vec2(25,0), true );
				body3->ApplyAngularImpulse( 500,0 );
				break;
			case('k'):
				body3->SetActive(true);
				//body3->ApplyForce( b2Vec2(0,-1200), b2Vec2(25,0), true );
				body3->ApplyAngularImpulse( -500,0 );
				break;
				
			// movement of the front part
			case('o'):
				body4->SetActive(true);
				//body4->ApplyForce( b2Vec2(1000,0), b2Vec2(6.5,18), true );
				body4->ApplyAngularImpulse( 120,0 );
				break;
			case('l'):
				body4->SetActive(true);
				//body4->ApplyForce( b2Vec2(-1000,0), b2Vec2(6.5,18), true );
				body4->ApplyAngularImpulse( -120,0 );
				break;
		}
	};


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
			b2FixtureDef groundfd;
			groundfd.shape = &shape;
			groundfd.filter.groupIndex = -1;
			groundfd.filter.categoryBits = 0x0002;
			groundfd.filter.maskBits = 0x0004;

			//! Details: Body is created in 'm_world' using CreateBody function and pointer returned is stored in 'b1'.<br>
			//! A fixture is attached to the body using CreateFixture function, passing address of 'shape' and density=0.0f.
			b1 = m_world->CreateBody(&bd); 
			b1->CreateFixture(&groundfd);
		}

		// New Items in the project

		{

			//first part
			
			//! Variable names: firstx, firsty, Datatype: float <br>
			//! Details: Stores the relative origin for the rear claw.
			float firstx = -44, firsty = 5;
			
			//! The below defined three polygon shapes 'poly1', poly2', 'poly3' are used for creation of fixtures for the body 'body1' <br>
			//! The below defined fixtures are then provided to the body 'body1' <br>
			//! Fixture attributes: density=0.1f, friction=1.0f, restitution=0.2f <br>
			b2BodyDef *bd = new b2BodyDef;
			bd->gravityScale=0;
			bd->type = b2_dynamicBody;
			bd->position.Set(firstx,firsty);
			bd->fixedRotation = false;


			b2PolygonShape poly1;
			b2Vec2 vertices1[4];
			vertices1[0].Set(1.5,6);
			vertices1[1].Set(3,5);
			vertices1[2].Set(1.5,3);
			vertices1[3].Set(0,3);
			poly1.Set(vertices1, 4);
			b2FixtureDef *fd1 = new b2FixtureDef;
			fd1->filter.categoryBits = 0x0004;
			fd1->filter.maskBits = 0x0002;
			fd1->density = 0.1;
			fd1->friction = 1.0;
			fd1->restitution = 0.2f;
			fd1->shape = &poly1;

			b2PolygonShape poly2;
			b2Vec2 vertices2[4];
			vertices2[0].Set(0,3);
			vertices2[1].Set(0,0);
			vertices2[2].Set(1.5,0);
			vertices2[3].Set(1.5,3);
			poly2.Set(vertices2, 4);
			b2FixtureDef *fd2 = new b2FixtureDef;
			fd2->filter.categoryBits = 0x0004;
			fd2->filter.maskBits = 0x0002;
			fd2->density = 0.1;
			fd2->friction = 1.0;
			fd2->restitution = 0.2f;
			fd2->shape = &poly2;

			b2PolygonShape poly3;
			b2Vec2 vertices3[6];
			vertices3[0].Set(0,0);
			vertices3[1].Set(1,-1.5);
			vertices3[2].Set(4,-3);
			vertices3[3].Set(1.5,0);
			poly3.Set(vertices3, 4);
			b2FixtureDef *fd3 = new b2FixtureDef;
			fd3->filter.categoryBits = 0x0004;
			fd3->filter.maskBits = 0x0002;
			fd3->density = 0.1;
			fd3->friction = 1.0;
			fd3->restitution = 0.2f;
			fd3->shape = &poly3;

			body1 = m_world->CreateBody(bd);
			body1->CreateFixture(fd1);
			body1->CreateFixture(fd2);
			body1->CreateFixture(fd3);


			// 2nd part
			
			//! The below defined polygon shape 'poly' is used for creation of fixture for the body 'body2' <br>
			//! The below defined fixtures are then provided to the body 'body2' <br>
			//! Fixture attributes: density=0.1f, friction=1.0f, restitution=0.2f <br>

			b2PolygonShape poly;
			b2Vec2 vertices[4];
			vertices[0].Set(-0.3,0);
			vertices[1].Set(1.2,-1);
			vertices[2].Set(7.5,4);
			vertices[3].Set(6.5,5);
			poly.Set(vertices, 4);

			b2FixtureDef fd;
			fd.filter.groupIndex = -1;
			fd.shape = &poly;
			fd.density = 0.1f;
			fd.friction = 1.0f;
			fd.restitution = 0.2f;

			b2BodyDef db;
			db.type = b2_dynamicBody;
			db.position.Set(firstx+1, firsty+5);
			body2 = m_world->CreateBody(&db);
			body2->CreateFixture(&fd);


			//! The b2RevoluteJointDef 'joint1' here is used to create joint between the bodies 'body1' and 'body2' <br>
			//! The range of movement for this joint is (-0.6*pi, 0.3*pi) <br>
			joint1.lowerAngle = -0.6f * b2_pi;
			joint1.upperAngle = 0.3f * b2_pi;
			joint1.enableLimit = true;
			joint1.maxMotorTorque =1e2f;
			joint1.enableMotor = true;
			
			//! Variable name: anchor , Datatype: b2Vec2 <br>
			//! Position of 'anchor' set to (-42.5f, 10.0f). Joint initialized with 'body1','body2' and the anchor point. <br>
			//! Joint created by passing address of 'joint1' to CreateJoint function, called on 'm_world'.
			b2Vec2 anchor;
			anchor.Set(firstx+1.5, firsty+5);
			joint1.Initialize(body2, body1, anchor);
			m_world->CreateJoint(&joint1);


			// 3rd part
			//! Variable name: b3_poly, Datatype: b2PolygonShape <br>
			//! Details: This is used for the creating the fixture for holding
			b2PolygonShape b3_poly;
			b2Vec2 b3_ver[4];

			b3_ver[0].Set(firstx+22.5, firsty-18);
			b3_ver[1].Set(firstx+22, firsty-19);
			b3_ver[2].Set(firstx+34, firsty-26);
			b3_ver[3].Set(firstx+34.5, firsty-25);
			b3_poly.Set(b3_ver, 4);

			b2FixtureDef b3_fd;
			b3_fd.filter.groupIndex = -1;
			b3_fd.shape = &b3_poly;
			b3_fd.density = 0.1f;
			b3_fd.friction = 1.0;
			b3_fd.restitution = 0.2f;



			// 4th part; the below horizontal rod
			
			//! Variable names: floatx, floaty; Datatype: float <br>
			//! frontx=-3, fronty=8 are the values set to the variables <br>
			//! Details: These are used as reference points for the body 'body3'
			float frontx = -3, fronty = 8;

			b2PolygonShape b4_poly;
			b2Vec2 b4_ver[5];
			b4_ver[0].Set(-1.2,0);
			b4_ver[1].Set(-0.7,-1);
			b4_ver[2].Set(25,-1);
			b4_ver[3].Set(25,1);
			b4_ver[4].Set(-0.7,1);
			b4_poly.Set(b4_ver, 5);

			//! Variable name: b4_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body3'
			b2FixtureDef b4_fd;
			b4_fd.filter.groupIndex = -1;
			b4_fd.shape = &b4_poly;
			b4_fd.density = 0.1f;
			b4_fd.friction = 1.0f;
			b4_fd.restitution = 0.2f;

			//! Variable name: b4_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body3'
			b2BodyDef b4_bd;
			b4_bd.type = b2_dynamicBody;
			b4_bd.position.Set(frontx, fronty);
			body3 = m_world->CreateBody(&b4_bd);
			body3->CreateFixture(&b4_fd);


			// the front part using three bodies
			
			//! Variable name: front_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body4' <br>
			//! Three fixtures are combined to create the fixture for this body
			b2BodyDef *front_bd = new b2BodyDef;
			front_bd->type = b2_dynamicBody;
			front_bd->position.Set(frontx+20,fronty-1);

			vertices1[0].Set(5,12);
			vertices1[1].Set(8,12);
			vertices1[2].Set(8,3);
			vertices1[3].Set(5,3);
			poly1.Set(vertices1, 4);
			fd1->density = 0.1;
			fd1->friction = 1.0f;
			fd1->restitution = 0.2f;
			fd1->shape = &poly1;

			vertices2[0].Set(5,3);
			vertices2[1].Set(8,3);
			vertices2[2].Set(8,-1);
			vertices2[3].Set(5,-1);
			vertices2[4].Set(3,0);
			vertices2[5].Set(3,2);
			poly2.Set(vertices2, 6);
			fd2->density = 0.1;
			fd2->friction = 1.0f;
			fd2->restitution = 0.2f;
			fd2->shape = &poly2;

			vertices3[0].Set(5,-1);
			vertices3[1].Set(8,-1);
			vertices3[2].Set(12,-6);
			vertices3[3].Set(5,-2);
			poly3.Set(vertices3, 4);
			fd3->density = 0.1;
			fd3->friction = 1.0f;
			fd3->restitution = 0.2f;
			fd3->shape = &poly3;

			body4 = m_world->CreateBody(front_bd);
			body4->CreateFixture(fd1);
			body4->CreateFixture(fd2);
			body4->CreateFixture(fd3);

			// joint between the two bodies
			
			//! Variable name: front_jd, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the bodies 'body3' and 'body4' <br>
			//! at the anchor point (21.2, 7.8) and the angle limits are (-0.1*pi, 0.1*pi)
			b2RevoluteJointDef front_jd;
			front_jd.lowerAngle = -0.1f * b2_pi;
			front_jd.upperAngle = 0.1f * b2_pi;
			front_jd.enableLimit = true;
			front_jd.maxMotorTorque =1e3f;
			front_jd.motorSpeed = 0.0f;
			front_jd.enableMotor = true;
			b2Vec2 front_anchor;
			front_anchor.Set(frontx+24.2, fronty-0.2);
			front_jd.Initialize(body3, body4, front_anchor);
			m_world->CreateJoint(&front_jd);



			// top compression

			//! Variable names: firstcomx, firstcomy; Datatype: float <br>
			//! frontx=12, fronty=32 are the values set to the variables <br>
			//! Details: These are used as reference points for both the cylinders
			float firstcomx = 12, firstcomy = 32;
			// first rod
			
			//! Variable name: body5, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for upper part of the blade lift cylinder
			b2Body* body5;
			b2PolygonShape body5_poly;
			b2Vec2 body5_ver[4];

			body5_ver[0].Set(-1.5,-0.6);
			body5_ver[1].Set(0,0);
			body5_ver[2].Set(5.5,-12.4);
			body5_ver[3].Set(4,-13);
			body5_poly.Set(body5_ver, 4);
			
			//! Variable name: body5_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body5'
			b2FixtureDef body5_fd;
			body5_fd.filter.groupIndex = -1;
			body5_fd.shape = &body5_poly;
			body5_fd.density = 0.1f;
			body5_fd.friction = 1.0f;
			body5_fd.restitution = 0.2f;
			
			//! Variable name: body5_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body5'
			b2BodyDef body5_bd;
			body5_bd.type = b2_dynamicBody;
			body5_bd.position.Set(firstcomx, firstcomy);
			body5 = m_world->CreateBody(&body5_bd);
			body5->CreateFixture(&body5_fd);

			//second rod
			//! Variable name: body6, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for the lower part of the blade lift cylinder
			b2Body* body6;
			b2PolygonShape body6_poly;
			b2Vec2 body6_ver[4];

			body6_ver[0].Set(0,0);
			body6_ver[1].Set(1,0.5);
			body6_ver[2].Set(4.5,-8);
			body6_ver[3].Set(3.5,-8.5);
			body6_poly.Set(body6_ver, 4);
			
			//! Variable name: body6_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body6'
			b2FixtureDef body6_fd;
			body6_fd.filter.groupIndex = -1;
			body6_fd.shape = &body6_poly;
			body6_fd.density = 0.1f;
			body6_fd.friction = 1.0f;
			body6_fd.restitution = 0.2f;
			
			//! Variable name: body6_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body6'
			b2BodyDef body6_bd;
			body6_bd.type = b2_dynamicBody;
			body6_bd.position.Set(firstcomx+5, firstcomy-15);
			body6 = m_world->CreateBody(&body6_bd);
			body6->CreateFixture(&body6_fd);
			
			//! Variable name: jointDef, Datatype: b2PrismaticJointDef <br>
			//! Details: This is a prismatic joint used to connect the bodies 'body5' and 'body6' <br>
			//! along the axis (-1,2.3) and the movement limits are (1,4)
			// the prismatic joint
			b2PrismaticJointDef jointDef;
			b2Vec2 worldAxis(-1.0f, 2.3f);
			jointDef.Initialize(body5, body6, body5->GetWorldCenter(), worldAxis);
			jointDef.lowerTranslation = 1.0f;
			jointDef.upperTranslation = 4.0f;
			jointDef.enableLimit = true;
			jointDef.maxMotorForce = 1.0f;
			jointDef.motorSpeed = 0.0f;
			jointDef.enableMotor = true;
			m_world->CreateJoint(&jointDef);


			// the part which joins body9 with the bulldozer
			//! Variable name: body9, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body which is used to join the lift cylinder and <br>
			//! the main body of the bulldozer and is positioned at (0,23)
			b2Body* body9;
			b2PolygonShape body9_poly;
			b2Vec2 body9_ver[6];
			body9_ver[0].Set(0.4+firstcomx-3.5,0.4);
			body9_ver[1].Set(1.5+firstcomx-3.5,0);
			body9_ver[2].Set(4+firstcomx-3.5,5.5);
			body9_ver[3].Set(5.1+firstcomx-3.5,5.1);
			body9_ver[4].Set(5.5+firstcomx-3.5,4);
			body9_ver[5].Set(0+firstcomx-3.5,1.5);
			body9_poly.Set(body9_ver, 6);
			
			//! Variable name: body9_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body9'
			//! Fixture Attributes: density: 0.1f, friction:1.0f, restitution: 0.2f
			b2FixtureDef body9_fd;
			body9_fd.filter.groupIndex = -1;
			body9_fd.shape = &body9_poly;
			body9_fd.density = 0.1f;
			body9_fd.friction = 1.0f;
			body9_fd.restitution = 0.2f;
			
			//! Variable name: body9_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body9'
			b2BodyDef body9_bd;
			body9_bd.type = b2_dynamicBody;
			body9_bd.position.Set(0, firstcomy-9);
			body9 = m_world->CreateBody(&body9_bd);
			body9->CreateFixture(&body9_fd);

			// revolute joint b/w the body9 and body5
			//! Variable name: top_joint, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the bodies 'body9' and 'body5' <br>
			//! at the anchor point (13.5, 28) and the angle limits are (-0.2*pi, 0.0*pi)
			b2RevoluteJointDef top_joint;
			top_joint.lowerAngle = -0.2f * b2_pi;
			top_joint.upperAngle = 0.0f * b2_pi;
			top_joint.enableLimit = true;
			b2Vec2 top_anchor;
			top_anchor.Set(firstcomx+1.5, firstcomy-4);
			top_joint.Initialize(body9, body5, top_anchor);
			m_world->CreateJoint(&top_joint);


			// bottom compression
			
			// first part
			b2PolygonShape shape1;
			shape1.SetAsBox(5, 0.5f);
			
			//! Variable name: body7, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for lower part of the lower cylinder <br>
			//! and is located at (4, 11) at an angle of 20deg
			b2Body* body7;
			//! Variable name: body7_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body7' <br>
			//! Fixture Attributes: density: 0.1f, friction: 1.0f, restitution: 0.2f
			b2FixtureDef body7_fd;
			body7_fd.filter.groupIndex = -1;
			body7_fd.shape = &shape1;
			body7_fd.density = 0.1f;
			body7_fd.friction = 1.0f;
			body7_fd.restitution = 0.2f;
			
			//! Variable name: body7_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body7'
			b2BodyDef body7_bd;
			body7_bd.type = b2_dynamicBody;
			body7 = m_world->CreateBody(&body7_bd);
			body7->CreateFixture(&body7_fd);
			shape1.SetAsBox(3.8, 1.5,b2Vec2(7,0),0);
			body7->CreateFixture(&body7_fd);
			
			body7->SetTransform( b2Vec2(4,11), 20 * DEGTORAD);
			
			
			// second part
			b2PolygonShape shape2;
			shape2.SetAsBox(6.5f, 0.5f);
			
			//! Variable name: body8, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for lower part of the lower cylinder <br>
			//! and is located at (8, 16) at an angle of 20deg
			b2Body* body8;
			//! Variable name: body8_fd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body8' <br>
			//! Fixture Attributes: density: 0.1f, friction: 1.0f, restitution: 0.2f
			b2FixtureDef body8_fd;
			body8_fd.filter.groupIndex = -1;
			body8_fd.shape = &shape2;
			body8_fd.density = 0.1f;
			body8_fd.friction = 1.0f;
			body8_fd.restitution = 0.2f;
			
			//! Variable name: body8_bd, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body8'
			b2BodyDef body8_bd;
			body8_bd.type = b2_dynamicBody;
			body8 = m_world->CreateBody(&body8_bd);
			body8->CreateFixture(&body8_fd);
			
			body8->SetTransform( b2Vec2(18,16), 20 * DEGTORAD);



			//************  the wheels

			// left wheel
			
			//! Variable names: xcenter, ycenter; Datatype: float <br>
			//! frontx=-23, fronty=3.6 are the values set to the variables <br>
			//! Details: These are used as reference points for the wheels
			float xcenter = -23.0, ycenter = 3.6;
			b2CircleShape circle1;
			circle1.m_radius = 3.0;

			//! Variable name: ballfd1, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'lwheel' <br>
			//! Fixture Attributes: density: 0.1f, friction: 1.0f, restitution: 0.2f
			b2FixtureDef ballfd1;
			ballfd1.filter.categoryBits = 0x0002;//
			ballfd1.filter.maskBits = 0x0004;//
			ballfd1.filter.groupIndex = -1;
			ballfd1.shape = &circle1;
			ballfd1.density = 0.1f;
			ballfd1.friction = 1.0f;
			ballfd1.restitution = 0.2f;
			//ballfd.filter.categoryBits = 0;
			//ballfd.filter.maskBits = 0;

			//! Variable name: ballbd1, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'lwheel'
			b2BodyDef ballbd1;
			ballbd1.type = b2_dynamicBody;
			ballbd1.position.Set(xcenter, ycenter);
			lwheel = m_world->CreateBody(&ballbd1);
			lwheel->CreateFixture(&ballfd1);


			// right wheel
			b2CircleShape circle2;
			circle2.m_radius = 3.0;
			
			//! Variable name: ballfd2, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'rwheel' <br>
			//! Fixture Attributes: density: 0.1f, friction: 1.0f, restitution: 0.2f
			b2FixtureDef ballfd2;
			ballfd2.filter.categoryBits = 0x0002;//
			ballfd2.filter.maskBits = 0x0004;//
			ballfd2.filter.groupIndex = -1;
			ballfd2.shape = &circle2;
			ballfd2.density = 0.1f;
			ballfd2.friction = 1.0f;
			ballfd2.restitution = 0.2f;
			//ballfd.filter.categoryBits = 0;
			//ballfd.filter.maskBits = 0;

			//! Variable name: ballbd2, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'rwheel'
			b2BodyDef ballbd2;
			ballbd2.type = b2_dynamicBody;
			ballbd2.position.Set(xcenter+31, ycenter);
			rwheel = m_world->CreateBody(&ballbd2);
			rwheel->CreateFixture(&ballfd2);

/*
			// distance joint
			b2DistanceJointDef disjointDef;
			disjointDef.Initialize(lwheel, rwheel, b2Vec2(xcenter,ycenter), b2Vec2(xcenter+31,ycenter));
			disjointDef.collideConnected = true;
			m_world->CreateJoint(&disjointDef);

*/
			// Chain
			//! Variable name: chain, Datatype: b2Body* <br>
			//! Details: Array of pointers to instances of the chain parts
			b2Body* chain[76];
			
			//! Variable name: jointPoint, Datatype: b2Body* <br>
			//! Details: Array of pointers to instances of the points where the joints will be made between the chain parts
			b2Vec2 jointPoint[76];
			
			//! Variable names: chainx, chainy; Datatype: float <br>
			//! frontx=-23.5, fronty=6.7 are the values set to the variables <br>
			//! Details: These are used as reference points for the chain
			float chainx = -23.5f, chainy = 6.7;
			
			//! Variable name: chainfd, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'chain' <br>
			//! Fixture Attributes: density: 10.0f, friction: 100.0f
			b2FixtureDef chainfd;
			chainfd.filter.categoryBits = 0x0004;
			chainfd.filter.maskBits = 0x0002;
			b2PolygonShape chainshape;
			chainshape.SetAsBox(0.5f, 0.25f);
			chainfd.shape = &chainshape;
			chainfd.density=10.0f;
			chainfd.friction=100.0f;
			b2BodyDef chainDef;
			chainDef.type = b2_dynamicBody;

			// Top chain part
			for (int i = 0; i < 32; ++i)
			{
				jointPoint[i].Set(chainx+1.0f*i,chainy);
				chainDef.position.Set(chainx+0.5+1.0f*i,chainy);
				chain[i]=m_world->CreateBody(&chainDef);
				chain[i]->CreateFixture(&chainfd);
			}

			// Right chain part
			chainshape.SetAsBox(0.25f, 0.5f);
			for (int i = 0; i < 6; ++i)
			{
				jointPoint[i+32].Set(chainx+32,chainy-i*1.0f);
				chainDef.position.Set(chainx+32,chainy-0.5-i*1.0f);
				chain[i+32]=m_world->CreateBody(&chainDef);
				chain[i+32]->CreateFixture(&chainfd);
			}

			// Bottom chain part
			chainshape.SetAsBox(0.5f, 0.25f);
			for (int i = 0; i < 32; ++i)
			{
				jointPoint[i+38].Set(chainx+32-1.0f*i,chainy-6);
				chainDef.position.Set(chainx+32-0.5-1.0f*i,chainy-6);
				chain[i+38]=m_world->CreateBody(&chainDef);
				chain[i+38]->CreateFixture(&chainfd);	
			}

			// Left chain part
			chainshape.SetAsBox(0.25f, 0.5f);
			for (int i = 0; i < 6; ++i)
			{
				jointPoint[i+70].Set(chainx,chainy-6+1.0f*i);
				chainDef.position.Set(chainx,chainy-6+1.0f*i+.5f);
				chain[i+70] = m_world->CreateBody(&chainDef);
				chain[i+70]->CreateFixture(&chainfd);
			}
			//////////////////////////////////////////////////////////////////////////////
			// Revolute joint between chain parts
			//! Variable name: chainjointdef, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the two chain parts.
			b2RevoluteJointDef chainjointdef;
			for(int i=1;i<76;i++)
			{
				chainjointdef.Initialize(chain[i-1], chain[i],jointPoint[i]);
				m_world->CreateJoint(&chainjointdef);
			}

			chainjointdef.Initialize(chain[0], chain[75],jointPoint[0]);
			m_world->CreateJoint(&chainjointdef);



			////////////////////////////////// MAIN BODY ///////////////////////////////////
			
			//! Variable name: bd0, Datatype: b2BodyDef <br>
			//! Details: Used for creating the body 'body0' positioned at (-15,28) <br>
			//! Many fixtures are created and combined to the fixture of this body
			b2BodyDef *bd0 = new b2BodyDef;
			bd0->type = b2_dynamicBody;
			bd0->position.Set(-15,28);
			body0 = m_world->CreateBody(bd0);
			
			//! Variable name: fd0, Datatype: b2FixtureDef <br>
			//! Details: Used for creating the fixture for body 'body0' <br>
			//! Fixture Attributes: density: 0.1f
			b2FixtureDef *fd0 = new b2FixtureDef;
			fd0->filter.groupIndex = -1;
			fd0->density = 0.1f;

			{
				b2PolygonShape poly0;             // big square main
				b2Vec2 vertices[4];
				vertices[0].Set(-10.0,-10.0);
				vertices[1].Set(-10.0,10.0); 
				vertices[2].Set(10.0,10.0);
				vertices[3].Set(10.0,-10.0);
				poly0.Set(vertices, 4);
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
				fd0->shape = &poly0;
				body0->CreateFixture(fd0);
			}
			
			
			// joint between left wheel and body
			
			//! Variable name: lwheeljoint, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the bodies 'lwheel' and 'body0' <br>
			//! at the anchor point (-23, 3.6)
			b2RevoluteJointDef lwheeljoint;
			lwheeljoint.maxMotorTorque =1e4f;
			lwheeljoint.motorSpeed = 0.0f;
			lwheeljoint.enableMotor = true;
			b2Vec2 left_anchor;
			left_anchor.Set(xcenter, ycenter);
			lwheeljoint.Initialize(body0, lwheel, left_anchor);
			m_world->CreateJoint(&lwheeljoint);


			// joint between right wheel and body
			
			//! Variable name: rwheeljoint, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the bodies 'rwheel' and 'body0' <br>
			//! at the anchor point (8, 3.6)
			b2RevoluteJointDef rwheeljoint;
			rwheeljoint.maxMotorTorque =1e4f;
			rwheeljoint.motorSpeed = 0.0f;
			rwheeljoint.enableMotor = true;
			b2Vec2 right_anchor;
			right_anchor.Set(xcenter+31, ycenter);
			rwheeljoint.Initialize(body0, rwheel, right_anchor);
			m_world->CreateJoint(&rwheeljoint);
			
			
			// fixture to stabilize the chain
			b2PolygonShape shape;
			shape.SetAsBox(11.0f, 2.1f);

			//! Variable name: bd , Datatype: b2BodyDef <br>
			//! Details: Holds data to construct rigid body(the shelf). Position set to (1.0f, 6.0f).
			bd->position.Set(-8.0f, 4.3);
			bd->type=b2_dynamicBody;
			fd.shape=&shape;
			fd.filter.categoryBits = 0x0002;//
			fd.filter.maskBits = 0x0004;//
			fd.filter.groupIndex = -1;
			fd.friction=0;


			//below support for chain
			//! Variable name: chain_below, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for supporting the chain from below
			b2Body* chain_below = m_world->CreateBody(bd);
			chain_below->CreateFixture(&fd);

			top_anchor.Set(-19.0f, 6.3);
			top_joint.Initialize(chain_below, body0, top_anchor);
			m_world->CreateJoint(&top_joint);

			top_anchor.Set(3.0f, 6.3);
			top_joint.Initialize(chain_below, body0, top_anchor);
			m_world->CreateJoint(&top_joint);

			//above support for chain
			shape.SetAsBox(12.0f, 0.1f);
			bd->position.Set(-8.0, 7.0);
			//! Variable name: chain_above, Datatype: b2Body* <br>
			//! Details: Pointer to an instance of the body for supporting the chain from above
			b2Body* chain_above = m_world->CreateBody(bd);
			chain_above->CreateFixture(&fd);

			top_anchor.Set(-19.0f, 7.0);
			top_joint.Initialize(chain_above, body0, top_anchor);
			m_world->CreateJoint(&top_joint);

			top_anchor.Set(3.0f, 7.0);
			top_joint.Initialize(chain_above, body0, top_anchor);
			m_world->CreateJoint(&top_joint);
			
			
			// creating fixture for holding the backpack
			body0->CreateFixture(&b3_fd);
			
			//! Variable name: jd23, Datatype: b2RevoluteJointDef <br>
			//! Details: This is a revolute joint used to connect the bodies 'lwheel' and 'body0' <br>
			//! //! at the anchor point (-36.5, 14.5)
			b2RevoluteJointDef jd23;
			jd23.lowerAngle = -0.2f * b2_pi;
			jd23.upperAngle = 0.25f * b2_pi;
			jd23.enableLimit = true;
			jd23.maxMotorTorque =1e4f;
			jd23.motorSpeed = 0.0f;
			jd23.enableMotor = true;
			b2Vec2 anchor23;
			anchor23.Set(firstx+7.5, firsty+9.5);
			jd23.Initialize(body2, body0, anchor23);
			m_world->CreateJoint(&jd23);

			//joining horizontal rod to main body

			jd23.lowerAngle = -0.15 * b2_pi;
			jd23.upperAngle = 0.05f * b2_pi;
			jd23.enableLimit = true;
			jd23.maxMotorTorque =1e4f;
			jd23.motorSpeed = 0.0f;
			jd23.enableMotor = true;
			anchor23.Set(frontx, fronty);
			jd23.Initialize(body3, body0, anchor23);
			m_world->CreateJoint(&jd23);
			
			// revolute joint b/w the body9 and mainbody
			
			top_joint.lowerAngle = -0.2f * b2_pi;
			top_joint.upperAngle = 0.2f * b2_pi;
			top_joint.enableLimit = true;
			top_joint.maxMotorTorque =1e3f;
			top_joint.motorSpeed = 0.0f;
			top_joint.enableMotor = true;
			top_anchor.Set(firstcomx-2.5, firstcomy-8);
			top_joint.Initialize(body9, body0, top_anchor);
			m_world->CreateJoint(&top_joint);
			
			// revolute joint b/w the front part and top compression
			
			top_anchor.Set(firstcomx+9, firstcomy-24);
			top_joint.Initialize(body6, body4, top_anchor);
			m_world->CreateJoint(&top_joint);
			
			
			// joints for second compression
			
			//leftmost joint
			front_jd.lowerAngle = -0.5f * b2_pi;
			front_jd.upperAngle = 0.5f * b2_pi;
			front_jd.enableLimit = true;
			//front_jd.maxMotorTorque =1e3f;
			front_jd.motorSpeed = 0.0f;
			front_jd.enableMotor = true;
			front_anchor.Set(0,9.5);
			front_jd.Initialize(body7, body0, front_anchor);
			m_world->CreateJoint(&front_jd);
			
			//rightmost joint
			front_jd.lowerAngle = -0.5f * b2_pi;
			front_jd.upperAngle = 0.5f * b2_pi;
			front_jd.enableLimit = true;
			//front_jd.maxMotorTorque =1e3f;
			front_jd.motorSpeed = 0.0f;
			front_jd.enableMotor = true;
			front_anchor.Set(23.5,18);
			front_jd.Initialize(body8, body4, front_anchor);
			m_world->CreateJoint(&front_jd);
			
			// the prismatic joint
			b2Vec2 worldAxis1(2.7f, 1.0f);
			jointDef.Initialize(body7, body8, body7->GetWorldCenter(), worldAxis1);
			jointDef.lowerTranslation = -4.0f;
			jointDef.upperTranslation = 4.0f;
			jointDef.enableLimit = true;
			jointDef.maxMotorForce = 1.0f;
			jointDef.motorSpeed = 0.0f;
			jointDef.enableMotor = true;
			m_world->CreateJoint(&jointDef);
			
		}

	}

	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
