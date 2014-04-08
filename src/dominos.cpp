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
