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
