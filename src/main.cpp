// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and
// ends there.
//

#include <iostream>
#include <vector>
#include <algorithm>
#include <cstddef>
/*#include <Eigen/SVD>
#include <Eigen/Dense>*/
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ctime>
//-----------------------------------------------------------------------------
// Some sample code for slvs.dll. We draw some geometric entities, provide
// initial guesses for their positions, and then constrain them. The solver
// calculates their new positions, in order to satisfy the constraints.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------

#include <windows.h>
#include <stdio.h>

#include "slvs.h"

Slvs_System sys;


void* CheckMalloc(size_t n) {
    void* r = malloc(n);
    if(!r) {
        printf("out of memory!\n");
        exit(-1);
    }
    return r;
}

//-----------------------------------------------------------------------------
// An example of a constraint in 3d. We create a single group, with some
// entities and constraints.
//-----------------------------------------------------------------------------
void Example3d(void) {
    // This will contain a single group, which will arbitrarily number 1.
    int g = 1;

    // A point, initially at (x y z) = (10 10 10)
    sys.param[sys.params++]    = Slvs_MakeParam(1, g, 10.0);
    sys.param[sys.params++]    = Slvs_MakeParam(2, g, 10.0);
    sys.param[sys.params++]    = Slvs_MakeParam(3, g, 10.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(101, g, 1, 2, 3);
    // and a second point at (20 20 20)
    sys.param[sys.params++]    = Slvs_MakeParam(4, g, 20.0);
    sys.param[sys.params++]    = Slvs_MakeParam(5, g, 20.0);
    sys.param[sys.params++]    = Slvs_MakeParam(6, g, 20.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(102, g, 4, 5, 6);
    // and a line segment connecting them.
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(200, g, SLVS_FREE_IN_3D, 101, 102);

    // The distance between the points should be 30.0 units.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(1, g, SLVS_C_PT_PT_DISTANCE, SLVS_FREE_IN_3D, 30.0, 101, 102, 0, 0);

    // Let's tell the solver to keep the second point as close to constant
    // as possible, instead moving the first point.
    sys.dragged[0] = 1;
    sys.dragged[1] = 2;
    sys.dragged[2] = 3;

    // Now that we have written our system, we solve.
    Slvs_Solve(&sys, g);

    if(sys.result == SLVS_RESULT_OKAY) {
        printf("okay; now at (%.3f %.3f %.3f)\n"
               "             (%.3f %.3f %.3f)\n",
               sys.param[0].val, sys.param[1].val, sys.param[2].val, sys.param[3].val,
               sys.param[4].val, sys.param[5].val);
        printf("%d DOF\n", sys.dof);
    } else {
        printf("solve failed");
    }
}



void Example3d_NEW2() {
    double qw, qx, qy, qz;
    int g = 1;
    // First, we create our workplane. Its origin corresponds to the origin
    // of our base frame (x y z) = (0 0 0)
    sys.param[sys.params++]    = Slvs_MakeParam(1, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(2, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(3, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(101, g, 1, 2, 3);
    // and it is parallel to the xy plane, so it has basis vectors (1 0 0)
    // and (0 1 0).
    Slvs_MakeQuaternion(1, 0, 0, 0, 1, 0, &qw, &qx, &qy, &qz);
    sys.param[sys.params++]    = Slvs_MakeParam(4, g, qw);
    sys.param[sys.params++]    = Slvs_MakeParam(5, g, qx);
    sys.param[sys.params++]    = Slvs_MakeParam(6, g, qy);
    sys.param[sys.params++]    = Slvs_MakeParam(7, g, qz);
    sys.entity[sys.entities++] = Slvs_MakeNormal3d(102, g, 4, 5, 6, 7);

    sys.entity[sys.entities++] = Slvs_MakeWorkplane(200, g, 101, 102);

    g                          = 2;
    sys.param[sys.params++]    = Slvs_MakeParam(8, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(9, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(301, g, 200, 8, 9);
    sys.param[sys.params++]    = Slvs_MakeParam(10, g, 10.0);
    sys.param[sys.params++]    = Slvs_MakeParam(11, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(302, g, 200, 10, 11);
    sys.param[sys.params++]    = Slvs_MakeParam(12, g, 9.0);
    sys.param[sys.params++]    = Slvs_MakeParam(13, g, 5.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(303, g, 200, 12, 13);
    sys.param[sys.params++]    = Slvs_MakeParam(14, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(15, g, 5.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(304, g, 200, 14, 15);

    sys.entity[sys.entities++] = Slvs_MakeLineSegment(401, g, 200, 301, 302);
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(402, g, 200, 302, 303);
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(403, g, 200, 303, 304);
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(404, g, 200, 304, 301);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(1, g, SLVS_C_PARALLEL, 200, 0.0, 0, 0, 401, 403);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(2, g, SLVS_C_HORIZONTAL, 200, 0.0, 0, 0, 401, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(3, g, SLVS_C_ANGLE, 200, 60.0, 0, 0, 401, 402);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(4, g, SLVS_C_ANGLE, 200, 60.0, 0, 0, 401, 404);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(5, g, SLVS_C_PT_LINE_DISTANCE, 200, 5.0, 303, 0, 401, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(6, g, SLVS_C_WHERE_DRAGGED, 200, 0, 301, 0, 0, 0);

    Slvs_Solve(&sys, g);

    std::cerr << "//////////////////////" << std::endl;
    if(sys.result == SLVS_RESULT_OKAY) {

        for(int i = 0; i < sys.params; i++) {
            std::cerr << i << " " << sys.param[i].val << std::endl;
        }


        printf("%d DOF\n", sys.dof);
    } else {
        int i;
        printf("solve failed: problematic constraints are:");
        for(i = 0; i < sys.faileds; i++) {
            printf(" %d", sys.failed[i]);
        }
        printf("\n");
        if(sys.result == SLVS_RESULT_INCONSISTENT) {
            printf("system inconsistent\n");
        } else {
            printf("system nonconvergent\n");
        }
    }
}

void Example3d_NEW1() {
    Slvs_hGroup g              = 1;
    sys.param[sys.params++]    = Slvs_MakeParam(1, g, 2.0);
    sys.param[sys.params++]    = Slvs_MakeParam(2, g, 2.0);
    sys.param[sys.params++]    = Slvs_MakeParam(3, g, 2.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(1, g, 1, 2, 3);

    sys.param[sys.params++] = Slvs_MakeParam(4, g, 21.0);
    sys.param[sys.params++] = Slvs_MakeParam(5, g, 4.0);
    sys.param[sys.params++] = Slvs_MakeParam(6, g, 5.0);
    // sys.param[sys.params++] = Slvs_MakeParam(7, g, 5.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(2, g, 4, 5, 6);

   // sys.constraint[sys.constraints++] = Slvs_MakeConstraint(1, g, SLVS_C_PT_PT_DISTANCE, SLVS_FREE_IN_3D, 1.0, 1, 2, 0, 0);
    //sys.constraint[sys.constraints++] = Slvs_MakeConstraint(2, g, SLVS_C_WHERE_DRAGGED, SLVS_FREE_IN_3D, 0, 1, 0, 0, 0);
 
    // sys.constraint[sys.constraints++] = Slvs_MakeConstraint(2, g, SLVS_C_WHERE_DRAGGED,
    // SLVS_FREE_IN_3D, 0, 1, 0, 0, 0); sys.constraint[sys.constraints++] = Slvs_MakeConstraint(3,
    // g, SLVS_C_WHERE_DRAGGED, SLVS_FREE_IN_3D, 0, 2, 0, 0, 0);

    Slvs_Solve(&sys, g);

    std::cerr << "//////////////////////" << std::endl;
    if(sys.result == SLVS_RESULT_OKAY) {

        for(int i = 0; i < sys.params; i++) {
            std::cerr << i << " " << sys.param[i].val << std::endl;
        }


        printf("%d DOF\n", sys.dof);
    } else {
        int i;
        printf("solve failed: problematic constraints are:");
        for(i = 0; i < sys.faileds; i++) {
            printf(" %d", sys.failed[i]);
        }
        printf("\n");
        if(sys.result == SLVS_RESULT_INCONSISTENT) {
            printf("system inconsistent\n");
        } else {
            printf("system nonconvergent\n");
        }
    }
}

void Example3d_NEW(void) {
    Slvs_hGroup g = 1;

    // origin
    sys.param[sys.params++]    = Slvs_MakeParam(1, g, 2.0);
    sys.param[sys.params++]    = Slvs_MakeParam(2, g, 2.0);
    sys.param[sys.params++]    = Slvs_MakeParam(3, g, 2.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(101, g, 1, 2, 3);

    // normal
    double qw, qx, qy, qz;
    Slvs_MakeQuaternion(1, 0, 0, 0, 1, 0, &qw, &qx, &qy, &qz);
    sys.param[sys.params++]    = Slvs_MakeParam(4, g, qw);
    sys.param[sys.params++]    = Slvs_MakeParam(5, g, qx);
    sys.param[sys.params++]    = Slvs_MakeParam(6, g, qy);
    sys.param[sys.params++]    = Slvs_MakeParam(7, g, qz);
    sys.entity[sys.entities++] = Slvs_MakeNormal3d(102, g, 4, 5, 6, 7);

    // Work plane
    sys.entity[sys.entities++] = Slvs_MakeWorkplane(200, g, 101, 102);


    // 2d points
    g                          = 2;
    sys.param[sys.params++]    = Slvs_MakeParam(8, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(9, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(300, g, 200, 8, 9);

    sys.param[sys.params++]    = Slvs_MakeParam(10, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(11, g, 1.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(301, g, 200, 10, 11);

    sys.param[sys.params++]    = Slvs_MakeParam(12, g, -1.0);
    sys.param[sys.params++]    = Slvs_MakeParam(13, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(302, g, 200, 12, 13);

    sys.entity[sys.entities++] = Slvs_MakeLineSegment(400, g, 200, 300, 301);
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(401, g, 200, 300, 302);


    sys.param[sys.params++]    = Slvs_MakeParam(14, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(15, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(16, g, 1.1);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(303, g, 14, 15, 16);
    sys.param[sys.params++]    = Slvs_MakeParam(17, g, 11.1);
    sys.param[sys.params++]    = Slvs_MakeParam(18, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(19, g, 1.1);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(304, g, 17, 18, 19);
    sys.param[sys.params++]    = Slvs_MakeParam(20, g, 11.1);
    sys.param[sys.params++]    = Slvs_MakeParam(21, g, 11.1);
    sys.param[sys.params++]    = Slvs_MakeParam(22, g, 11.1);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(305, g, 20, 21, 22);
    sys.param[sys.params++]    = Slvs_MakeParam(23, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(24, g, 11.1);
    sys.param[sys.params++]    = Slvs_MakeParam(25, g, 11.1);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(306, g, 23, 24, 25);

    sys.param[sys.params++]    = Slvs_MakeParam(26, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(27, g, 1.1);
    sys.param[sys.params++]    = Slvs_MakeParam(28, g, 1.1);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(307, g, 26, 27, 28);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(1, g, SLVS_C_PT_PT_DISTANCE, 200, 1.0, 301, 300, 0, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(2, g, SLVS_C_PT_PT_DISTANCE, 200, 1.0, 302, 300, 0, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(3, g, SLVS_C_ANGLE, 200, 90, 0, 0, 400, 401);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(4, g, SLVS_C_VERTICAL, 200, 0.0, 0, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(5, g, SLVS_C_POINTS_COINCIDENT, 200, 0.0, 101, 300, 0, 0);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(6, g, SLVS_C_PT_PLANE_DISTANCE, 0, 0.0, 303, 0, 200, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(7, g, SLVS_C_PT_LINE_DISTANCE, 200, 1.0, 303, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(8, g, SLVS_C_PT_LINE_DISTANCE, 200, 1.0, 303, 0, 401, 0);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(9, g, SLVS_C_PT_PLANE_DISTANCE, 0, 0.0, 304, 0, 200, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(10, g, SLVS_C_PT_LINE_DISTANCE, 200, 10.0, 304, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(11, g, SLVS_C_PT_LINE_DISTANCE, 200, 1.0, 304, 0, 401, 0);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(12, g, SLVS_C_PT_PLANE_DISTANCE, 0, 0.0, 305, 0, 200, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(13, g, SLVS_C_PT_LINE_DISTANCE, 200, 10.0, 305, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(14, g, SLVS_C_PT_LINE_DISTANCE, 200, 5.0, 305, 0, 401, 0);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(15, g, SLVS_C_PT_PLANE_DISTANCE, 0, 0.0, 306, 0, 200, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(16, g, SLVS_C_PT_LINE_DISTANCE, 200, 1.0, 306, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(17, g, SLVS_C_PT_LINE_DISTANCE, 200, 5.0, 306, 0, 401, 0);

    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(18, g, SLVS_C_PT_PLANE_DISTANCE, 0, 3.0, 307, 0, 200, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(19, g, SLVS_C_PT_LINE_DISTANCE, 200, 2.5, 307, 0, 400, 0);
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(20, g, SLVS_C_PT_LINE_DISTANCE, 200, 2.5, 307, 0, 401, 0);

    // sys.constraint[sys.constraints++] = Slvs_MakeConstraint(14, g, SLVS_C_PT_IN_PLANE, 200, 0,
    // 307, 0, 200, 0);

    sys.dragged[7] = 2;
    sys.dragged[8] = 2;

    // Now that we have written our system, we solve.
    Slvs_Solve(&sys, g);

    if(sys.result == SLVS_RESULT_OKAY) {

        // 3 4 5 6
        double UX, UY, UZ, VX, VY, VZ, NX, NY, NZ;
        Slvs_QuaternionU(sys.param[3].val, sys.param[4].val, sys.param[5].val, sys.param[6].val,
                         &UX, &UY, &UZ);
        Slvs_QuaternionV(sys.param[3].val, sys.param[4].val, sys.param[5].val, sys.param[6].val,
                         &VX, &VY, &VZ);
        Slvs_QuaternionN(sys.param[3].val, sys.param[4].val, sys.param[5].val, sys.param[6].val,
                         &NX, &NY, &NZ);

        printf("Origin: (%.3f %.3f %.3f)\n", sys.param[0].val, sys.param[1].val, sys.param[2].val);
        printf("U: (%.3f %.3f %.3f)\n", UX, UY, UZ);
        printf("V: (%.3f %.3f %.3f)\n", VX, VY, VZ);
        printf("N: (%.3f %.3f %.3f)\n", NX, NY, NZ);

        printf("Local Frames:  (%.3f %.3f)\n", sys.param[7].val, sys.param[8].val);
        printf("Local Frames:  (%.3f %.3f)\n", sys.param[9].val, sys.param[10].val);
        printf("Local Frames:  (%.3f %.3f)\n", sys.param[11].val, sys.param[12].val);

        printf("okay; now at \n(%.3f %.3f %.3f)\n (%.3f %.3f %.3f)\n (%.3f %.3f %.3f)\n (%.3f %.3f "
               "%.3f)\n",
               sys.param[13].val, sys.param[14].val, sys.param[15].val, sys.param[16].val,
               sys.param[17].val, sys.param[18].val, sys.param[19].val, sys.param[20].val,
               sys.param[21].val, sys.param[22].val, sys.param[23].val, sys.param[24].val);

        // 25 26 27
        printf("okay; now at \n(%.3f %.3f %.3f)\n", sys.param[25].val, sys.param[26].val,
               sys.param[27].val);

        for(int i = 0; i <= 27; i++) {
            // std::cerr << sys.param[i].val << std::endl;
        }


        printf("%d DOF\n", sys.dof);
    } else {
        int i;
        printf("solve failed: problematic constraints are:");
        for(i = 0; i < sys.faileds; i++) {
            printf(" %d", sys.failed[i]);
        }
        printf("\n");
        if(sys.result == SLVS_RESULT_INCONSISTENT) {
            printf("system inconsistent\n");
        } else {
            printf("system nonconvergent\n");
        }
    }
}


//-----------------------------------------------------------------------------
// An example of a constraint in 2d. In our first group, we create a workplane
// along the reference frame's xy plane. In a second group, we create some
// entities in that group and dimension them.
//-----------------------------------------------------------------------------
void Example2d(void) {
    int g;
    double qw, qx, qy, qz;

    g = 1;
    // First, we create our workplane. Its origin corresponds to the origin
    // of our base frame (x y z) = (0 0 0)
    sys.param[sys.params++]    = Slvs_MakeParam(1, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(2, g, 0.0);
    sys.param[sys.params++]    = Slvs_MakeParam(3, g, 0.0);
    sys.entity[sys.entities++] = Slvs_MakePoint3d(101, g, 1, 2, 3);
    // and it is parallel to the xy plane, so it has basis vectors (1 0 0)
    // and (0 1 0).
    Slvs_MakeQuaternion(1, 0, 0, 0, 1, 0, &qw, &qx, &qy, &qz);
    sys.param[sys.params++]    = Slvs_MakeParam(4, g, qw);
    sys.param[sys.params++]    = Slvs_MakeParam(5, g, qx);
    sys.param[sys.params++]    = Slvs_MakeParam(6, g, qy);
    sys.param[sys.params++]    = Slvs_MakeParam(7, g, qz);
    sys.entity[sys.entities++] = Slvs_MakeNormal3d(102, g, 4, 5, 6, 7);

    sys.entity[sys.entities++] = Slvs_MakeWorkplane(200, g, 101, 102);

    // Now create a second group. We'll solve group 2, while leaving group 1
    // constant; so the workplane that we've created will be locked down,
    // and the solver can't move it.
    g = 2;
    // These points are represented by their coordinates (u v) within the
    // workplane, so they need only two parameters each.
    sys.param[sys.params++]    = Slvs_MakeParam(11, g, 10.0);
    sys.param[sys.params++]    = Slvs_MakeParam(12, g, 20.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(301, g, 200, 11, 12);

    sys.param[sys.params++]    = Slvs_MakeParam(13, g, 20.0);
    sys.param[sys.params++]    = Slvs_MakeParam(14, g, 10.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(302, g, 200, 13, 14);

    // And we create a line segment with those endpoints.
    sys.entity[sys.entities++] = Slvs_MakeLineSegment(400, g, 200, 301, 302);

    // Now three more points.
    sys.param[sys.params++]    = Slvs_MakeParam(15, g, 100.0);
    sys.param[sys.params++]    = Slvs_MakeParam(16, g, 120.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(303, g, 200, 15, 16);

    sys.param[sys.params++]    = Slvs_MakeParam(17, g, 120.0);
    sys.param[sys.params++]    = Slvs_MakeParam(18, g, 110.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(304, g, 200, 17, 18);

    sys.param[sys.params++]    = Slvs_MakeParam(19, g, 115.0);
    sys.param[sys.params++]    = Slvs_MakeParam(20, g, 115.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(305, g, 200, 19, 20);

    // And arc, centered at point 303, starting at point 304, ending at
    // point 305.
    sys.entity[sys.entities++] = Slvs_MakeArcOfCircle(401, g, 200, 102, 303, 304, 305);

    // Now one more point, and a distance
    sys.param[sys.params++]    = Slvs_MakeParam(21, g, 200.0);
    sys.param[sys.params++]    = Slvs_MakeParam(22, g, 200.0);
    sys.entity[sys.entities++] = Slvs_MakePoint2d(306, g, 200, 21, 22);

    sys.param[sys.params++]    = Slvs_MakeParam(23, g, 30.0);
    sys.entity[sys.entities++] = Slvs_MakeDistance(307, g, 200, 23);

    // And a complete circle, centered at point 306 with radius equal to
    // distance 307. The normal is 102, the same as our workplane.
    sys.entity[sys.entities++] = Slvs_MakeCircle(402, g, 200, 306, 102, 307);


    // The length of our line segment is 30.0 units.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(1, g, SLVS_C_PT_PT_DISTANCE, 200, 30.0, 301, 302, 0, 0);

    // And the distance from our line segment to the origin is 10.0 units.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(2, g, SLVS_C_PT_LINE_DISTANCE, 200, 10.0, 101, 0, 400, 0);
    // And the line segment is vertical.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(3, g, SLVS_C_VERTICAL, 200, 0.0, 0, 0, 400, 0);
    // And the distance from one endpoint to the origin is 15.0 units.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(4, g, SLVS_C_PT_PT_DISTANCE, 200, 15.0, 301, 101, 0, 0);
    /*
        // And same for the other endpoint; so if you add this constraint then
        // the sketch is overconstrained and will signal an error.
        sys.constraint[sys.constraints++] = Slvs_MakeConstraint(
                                                5, g,
                                                SLVS_C_PT_PT_DISTANCE,
                                                200,
                                                18.0,
                                                302, 101, 0, 0); */

    // The arc and the circle have equal radius.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(6, g, SLVS_C_EQUAL_RADIUS, 200, 0.0, 0, 0, 401, 402);
    // The arc has radius 17.0 units.
    sys.constraint[sys.constraints++] =
        Slvs_MakeConstraint(7, g, SLVS_C_DIAMETER, 200, 17.0 * 2, 0, 0, 401, 0);

    // If the solver fails, then ask it to report which constraints caused
    // the problem.
    sys.calculateFaileds = 1;

    // And solve.
    Slvs_Solve(&sys, g);

    if(sys.result == SLVS_RESULT_OKAY) {
        printf("solved okay\n");
        printf("line from (%.3f %.3f) to (%.3f %.3f)\n", sys.param[7].val, sys.param[8].val,
               sys.param[9].val, sys.param[10].val);

        printf("arc center (%.3f %.3f) start (%.3f %.3f) finish (%.3f %.3f)\n", sys.param[11].val,
               sys.param[12].val, sys.param[13].val, sys.param[14].val, sys.param[15].val,
               sys.param[16].val);

        printf("circle center (%.3f %.3f) radius %.3f\n", sys.param[17].val, sys.param[18].val,
               sys.param[19].val);
        printf("%d DOF\n", sys.dof);
    } else {
        int i;
        printf("solve failed: problematic constraints are:");
        for(i = 0; i < sys.faileds; i++) {
            printf(" %d", sys.failed[i]);
        }
        printf("\n");
        if(sys.result == SLVS_RESULT_INCONSISTENT) {
            printf("system inconsistent\n");
        } else {
            printf("system nonconvergent\n");
        }
    }
}

int main(void) {
    memset(&sys, 0, sizeof(sys));
    sys.param      = (Slvs_Param*)CheckMalloc(50 * sizeof(sys.param[0]));
    sys.entity     = (Slvs_Entity*)CheckMalloc(50 * sizeof(sys.entity[0]));
    sys.constraint = (Slvs_Constraint*)CheckMalloc(50 * sizeof(sys.constraint[0]));
    sys.failed     = (Slvs_hConstraint*)CheckMalloc(50 * sizeof(sys.failed[0]));

    sys.faileds = 50;

    //    Example3d();
    for(;;) {
        // Example2d();
        // Example3d();
        Example3d_NEW2();
        sys.params = sys.constraints = sys.entities = 0;
        break;
    }
    system("pause");
    return 0;
}
