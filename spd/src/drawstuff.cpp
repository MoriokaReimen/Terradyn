//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : start(), command(), loop()
//            Functions for Drawstuff
//
// y.sato [2004.5]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include <ode/ode.h>

//#include <drawstuff/drawstuff.h>
#include "../../drawstuff/drawstuff.h"
#include "../include/draw.h"

// state set by keyboard commands
static int occasional_error = 0;

// start simulation - set viewpoint
void start()
{
    // view point for JEMRMS
    //static float xyz[3] = {10, 4, -2.0};//{1.0382f*10,-1.0811f*10,1.4700f*3};
    //static float hpr[3] = {-130.0000f, 0.000f, 180.0000f};
    // ordinary view point
    static float xyz[3] = {0.75,0.9, 0.5};
    static float hpr[3] = {-125.0000f,-20.0000f,0.0000f};

    dsSetViewpoint (xyz,hpr);
}

// called when a key pressed
void command(int cmd)
{
    static float xyz[3];
    static float hpr[3];

    switch (cmd) {
    case 'e':
        occasional_error ^= 1;
        break;
    case 'v':
        dsGetViewpoint (xyz, hpr);
        xyz[0] += 0.1;
        xyz[1] += 0.1;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'r':
        xyz[0] = 0.75;
        xyz[1] = 0.90;
        xyz[2] = 0.5;
        hpr[0] = -125.0;
        hpr[1] = -20.00;
        hpr[2] = 0.0;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'x':
        dsGetViewpoint(xyz, hpr);
        hpr[0] = 180.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'y':
        dsGetViewpoint(xyz, hpr);
        hpr[0] = -90.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'a':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = 0.0;
        hpr[1] = -90.0;
        hpr[2] = 0.0;
        xyz[0] = 0.5;
        xyz[1] = 0.0;
        xyz[2] = 2.0;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'z':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = 0.0;
        hpr[1] = -90.0;
        hpr[2] = 0.0;
        xyz[0] = 0.0;
        xyz[1] = 0.0;
        xyz[2] = 2.5;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'q':
        xyz[0] = 0.00;
        xyz[1] = 0.10;
        xyz[2] = 0.5;
        hpr[0] = 0.0;
        hpr[1] = 0.00;
        hpr[2] = -90.0;
        dsSetViewpoint (xyz,hpr);
        break;

    //new
    case 'p':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = -90.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        xyz[0] = .0;
        xyz[1] = 1.0;
        xyz[2] = 0.4;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'o':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = 0.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        xyz[0] = -0.9;
        xyz[1] = 0.0;
        xyz[2] = 0.2;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'i':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = 90.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        xyz[0] = 0.0;
        xyz[1] = -1.0;
        xyz[2] = 0.3;
        dsSetViewpoint (xyz,hpr);
        break;
    case 'u':
        dsGetViewpoint (xyz, hpr);
        hpr[0] = 0.0;
        hpr[1] = 0.0;
        hpr[2] = 0.0;
        xyz[0] = -0.9;
        xyz[1] = 0.0;
        xyz[2] = 0.02;
        dsSetViewpoint (xyz,hpr);
        break;
    }

}
