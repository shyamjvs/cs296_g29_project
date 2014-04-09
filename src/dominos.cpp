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
	  
		b2Body* body1;
		b2Body* body2;
		b2Body* body4;
		b2Body* front_body;
		b2Body* lwheel;
		b2Body* rwheel;
		b2RevoluteJointDef joint1;
	  
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

			float frontx = -3, fronty = 8;

			b2PolygonShape b4_poly;
			b2Vec2 b4_ver[5];
			b4_ver[0].Set(-1.2,0);
			b4_ver[1].Set(-0.7,-1);
			b4_ver[2].Set(25,-1);
			b4_ver[3].Set(25,1);
			b4_ver[4].Set(-0.7,1);
			b4_poly.Set(b4_ver, 5);

			b2FixtureDef b4_fd;
			b4_fd.filter.groupIndex = -1;
			b4_fd.shape = &b4_poly;
			b4_fd.density = 0.1f;
			b4_fd.friction = 1.0f;
			b4_fd.restitution = 0.2f;

			b2BodyDef b4_bd;
			b4_bd.type = b2_dynamicBody;
			b4_bd.position.Set(frontx, fronty);
			body4 = m_world->CreateBody(&b4_bd);
			body4->CreateFixture(&b4_fd);


			// the front part using three bodies
			
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

			front_body = m_world->CreateBody(front_bd);
			front_body->CreateFixture(fd1);
			front_body->CreateFixture(fd2);
			front_body->CreateFixture(fd3);

			// joint between the two bodies
			b2RevoluteJointDef front_jd;
			front_jd.lowerAngle = -0.06f * b2_pi;
			front_jd.upperAngle = 0.1f * b2_pi;
			front_jd.enableLimit = true;
			front_jd.maxMotorTorque =1e3f;
			front_jd.motorSpeed = 0.0f;
			front_jd.enableMotor = true;
			b2Vec2 front_anchor;
			front_anchor.Set(frontx+24.2, fronty-0.2);
			front_jd.Initialize(body4, front_body, front_anchor);
			m_world->CreateJoint(&front_jd);



			// top compression


			float firstcomx = 12, firstcomy = 32;
			// first rod
			b2Body* firstCom1;
			b2PolygonShape firstCom1_poly;
			b2Vec2 firstCom1_ver[4];

			firstCom1_ver[0].Set(-1.5,-0.6);
			firstCom1_ver[1].Set(0,0);
			firstCom1_ver[2].Set(5.5,-12.4);
			firstCom1_ver[3].Set(4,-13);
			firstCom1_poly.Set(firstCom1_ver, 4);

			b2FixtureDef firstCom1_fd;
			firstCom1_fd.filter.groupIndex = -1;
			firstCom1_fd.shape = &firstCom1_poly;
			firstCom1_fd.density = 0.1f;
			firstCom1_fd.friction = 1.0f;
			firstCom1_fd.restitution = 0.2f;

			b2BodyDef firstCom1_bd;
			firstCom1_bd.type = b2_dynamicBody;
			firstCom1_bd.position.Set(firstcomx, firstcomy);
			firstCom1 = m_world->CreateBody(&firstCom1_bd);
			firstCom1->CreateFixture(&firstCom1_fd);

			//second rod
			b2Body* firstCom2;
			b2PolygonShape firstCom2_poly;
			b2Vec2 firstCom2_ver[4];

			firstCom2_ver[0].Set(0,0);
			firstCom2_ver[1].Set(1,0.5);
			firstCom2_ver[2].Set(4.5,-8);
			firstCom2_ver[3].Set(3.5,-8.5);
			firstCom2_poly.Set(firstCom2_ver, 4);

			b2FixtureDef firstCom2_fd;
			firstCom2_fd.filter.groupIndex = -1;
			firstCom2_fd.shape = &firstCom2_poly;
			firstCom2_fd.density = 0.1f;
			firstCom2_fd.friction = 1.0f;
			firstCom2_fd.restitution = 0.2f;

			b2BodyDef firstCom2_bd;
			firstCom2_bd.type = b2_dynamicBody;
			firstCom2_bd.position.Set(firstcomx+5, firstcomy-15);
			firstCom2 = m_world->CreateBody(&firstCom2_bd);
			firstCom2->CreateFixture(&firstCom2_fd);

			// the prismatic joint
			b2PrismaticJointDef jointDef;
			b2Vec2 worldAxis(-1.0f, 2.3f);
			jointDef.Initialize(firstCom1, firstCom2, firstCom1->GetWorldCenter(), worldAxis);
			jointDef.lowerTranslation = 1.0f;
			jointDef.upperTranslation = 4.0f;
			jointDef.enableLimit = true;
			jointDef.maxMotorForce = 1.0f;
			jointDef.motorSpeed = 0.0f;
			jointDef.enableMotor = true;
			m_world->CreateJoint(&jointDef);


			// the part which joins firstCom1 with the bulldozer
			b2Body* jbody;
			b2PolygonShape jbody_poly;
			b2Vec2 jbody_ver[6];
			jbody_ver[0].Set(0.4+firstcomx-3.5,0.4);
			jbody_ver[1].Set(1.5+firstcomx-3.5,0);
			jbody_ver[2].Set(4+firstcomx-3.5,5.5);
			jbody_ver[3].Set(5.1+firstcomx-3.5,5.1);
			jbody_ver[4].Set(5.5+firstcomx-3.5,4);
			jbody_ver[5].Set(0+firstcomx-3.5,1.5);
			jbody_poly.Set(jbody_ver, 6);

			b2FixtureDef jbody_fd;
			jbody_fd.filter.groupIndex = -1;
			jbody_fd.shape = &jbody_poly;
			jbody_fd.density = 0.1f;
			jbody_fd.friction = 1.0f;
			jbody_fd.restitution = 0.2f;

			b2BodyDef jbody_bd;
			jbody_bd.type = b2_dynamicBody;
			jbody_bd.position.Set(0, firstcomy-9);
			jbody = m_world->CreateBody(&jbody_bd);
			jbody->CreateFixture(&jbody_fd);

			// revolute joint b/w the jbody and firstCom1
			b2RevoluteJointDef top_joint;
			top_joint.lowerAngle = -0.2f * b2_pi;
			top_joint.upperAngle = 0.0f * b2_pi;
			top_joint.enableLimit = true;
			b2Vec2 top_anchor;
			top_anchor.Set(firstcomx+1.5, firstcomy-4);
			top_joint.Initialize(jbody, firstCom1, top_anchor);
			m_world->CreateJoint(&top_joint);


			// bottom compression
			
			// first part
			b2PolygonShape shape1;
			shape1.SetAsBox(5, 0.5f);
			
			b2Body* secondCom1;
			b2FixtureDef secondCom1_fd;
			secondCom1_fd.filter.groupIndex = -1;
			secondCom1_fd.shape = &shape1;
			secondCom1_fd.density = 0.1f;
			secondCom1_fd.friction = 1.0f;
			secondCom1_fd.restitution = 0.2f;

			b2BodyDef secondCom1_bd;
			secondCom1_bd.type = b2_dynamicBody;
			secondCom1 = m_world->CreateBody(&secondCom1_bd);
			secondCom1->CreateFixture(&secondCom1_fd);
			
			shape1.SetAsBox(3, 1.5,b2Vec2(6.8,0),0);
			secondCom1->CreateFixture(&secondCom1_fd);
			
			secondCom1->SetTransform( b2Vec2(4,11), 20 * DEGTORAD);
			
			
			// second part
			b2PolygonShape shape2;
			shape2.SetAsBox(6.5f, 0.5f);
			
			b2Body* secondCom2;
			b2FixtureDef secondCom2_fd;
			secondCom2_fd.filter.groupIndex = -1;
			secondCom2_fd.shape = &shape2;
			secondCom2_fd.density = 0.1f;
			secondCom2_fd.friction = 1.0f;
			secondCom2_fd.restitution = 0.2f;

			b2BodyDef secondCom2_bd;
			secondCom2_bd.type = b2_dynamicBody;
			secondCom2 = m_world->CreateBody(&secondCom2_bd);
			secondCom2->CreateFixture(&secondCom2_fd);
			
			secondCom2->SetTransform( b2Vec2(18,16), 20 * DEGTORAD);
			
			
			//************  the wheels

			// left wheel

			float xcenter = -23.0, ycenter = 3.6;
			float angular_vel = 10;
			b2CircleShape circle1;
			circle1.m_radius = 3.0;


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


			b2BodyDef ballbd1;
			ballbd1.type = b2_dynamicBody;
			ballbd1.position.Set(xcenter, ycenter);
			lwheel = m_world->CreateBody(&ballbd1);
			lwheel->CreateFixture(&ballfd1);
			lwheel->SetAngularVelocity(angular_vel);


			// right wheel
			b2CircleShape circle2;
			circle2.m_radius = 3.0;


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


			b2BodyDef ballbd2;
			ballbd2.type = b2_dynamicBody;
			ballbd2.position.Set(xcenter+31, ycenter);
			rwheel = m_world->CreateBody(&ballbd2);
			rwheel->CreateFixture(&ballfd2);
			rwheel->SetAngularVelocity(angular_vel);


			// distance joint
			b2DistanceJointDef disjointDef;
			disjointDef.Initialize(lwheel, rwheel, b2Vec2(xcenter,ycenter), b2Vec2(xcenter+31,ycenter));
			disjointDef.collideConnected = true;
			m_world->CreateJoint(&disjointDef);


			///Adding the Chain
			b2Vec2 vs[76];
			b2Body* conveyer[76];
			float chainx = -23.5f, chainy = 6.7;

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

			///The top chain units
			for (int i = 0; i < 32; ++i)
			{
				vs[i].Set(chainx+1.0f*i,chainy);
				chainDef.position.Set(chainx+0.5+1.0f*i,chainy);
				conveyer[i]=m_world->CreateBody(&chainDef);
				conveyer[i]->CreateFixture(&chainfd);
			}

			///The right chain units
			chainshape.SetAsBox(0.25f, 0.5f);
			for (int i = 0; i < 6; ++i)
			{
				vs[i+32].Set(chainx+32,chainy-i*1.0f);
				chainDef.position.Set(chainx+32,chainy-0.5-i*1.0f);
				conveyer[i+32]=m_world->CreateBody(&chainDef);
				conveyer[i+32]->CreateFixture(&chainfd);
			}

			///The Bottom chain units
			chainshape.SetAsBox(0.5f, 0.25f);
			for (int i = 0; i < 32; ++i)
			{
				vs[i+38].Set(chainx+32-1.0f*i,chainy-6);
				chainDef.position.Set(chainx+32-0.5-1.0f*i,chainy-6);
				conveyer[i+38]=m_world->CreateBody(&chainDef);
				conveyer[i+38]->CreateFixture(&chainfd);	
			}

			///The left chain units
			chainshape.SetAsBox(0.25f, 0.5f);
			for (int i = 0; i < 6; ++i)
			{
				vs[i+70].Set(chainx,chainy-6+1.0f*i);
				chainDef.position.Set(chainx,chainy-6+1.0f*i+.5f);
				conveyer[i+70] = m_world->CreateBody(&chainDef);
				conveyer[i+70]->CreateFixture(&chainfd);
			}
			//////////////////////////////////////////////////////////////////////////////
			///Adding Revolute joint between chain units
			b2RevoluteJointDef jointDef3;
			for(int i=1;i<76;i++)
			{
				jointDef3.Initialize(conveyer[i-1], conveyer[i],vs[i]);
				m_world->CreateJoint(&jointDef3);
			}

			jointDef3.Initialize(conveyer[0], conveyer[75],vs[0]);
			m_world->CreateJoint(&jointDef3);
	  }

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
