/*
 * @file PDM_collision_method_mesh_2d.h 
 * @brief class of collision method(two dim) based on mesh for PDM drivers.
 * @author Wei Chen
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_DYNAMICS_PDM_PDM_STEP_METHODS_PDM_COLLISION_METHODS_PDM_COLLISION_METHOD_MESH_2D_H
#define PHYSIKA_DYNAMICS_PDM_PDM_STEP_METHODS_PDM_COLLISION_METHODS_PDM_COLLISION_METHOD_MESH_2D_H

#include "Physika_Dynamics/PDM/PDM_Step_Methods/PDM_Collision_Methods/PDM_collision_method_mesh.h"
#include "Physika_Dynamics/PDM/PDM_Step_Methods/PDM_Collision_Methods/PDM_collision_method_grid_2d.h"

namespace Physika{

template <typename Scalar>
class PDMCollisionMethodMesh<Scalar, 2>:public PDMCollisionMethodGrid<Scalar,2>
{
public:
    PDMCollisionMethodMesh();
    ~PDMCollisionMethodMesh();

protected:
    virtual void locateParticleBin();
    virtual void collisionDectectionAndResponse();

};

}// end of namespace Phyaika
#endif //PHYSIKA_DYNAMICS_PDM_PDM_STEP_METHODS_PDM_COLLISION_METHODS_PDM_COLLISION_METHOD_MESH_2D_H