//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//
//
// Function : start(), command(), loop()
//            Functions for Drawstuff
//
// y.sato [2004.5]
//
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//

#include <ode/ode.h>

#include <drawstuff/drawstuff.h>
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
    static float xyz[3] = {1.0382f*3,-1.0811f*3,1.4700f*2.5};
    static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
    
    dsSetViewpoint (xyz,hpr);
    printf ("Press 'e' to start/stop occasional error.\n");
}

// called when a key pressed
void command(int cmd)
{
    if (cmd == 'e' || cmd == 'E') {
	occasional_error ^= 1;
    }
}
