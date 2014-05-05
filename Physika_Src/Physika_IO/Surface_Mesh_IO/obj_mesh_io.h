﻿/*
 * @file obj_mesh_io.h 
 * @brief load and save mesh to an obj file.
 * @author Fei Zhu
 * 
 * This file is part of Physika, a versatile physics simulation library.
 * Copyright (C) 2013 Physika Group.
 *
 * This Source Code Form is subject to the terms of the GNU General Public License v2.0. 
 * If a copy of the GPL was not distributed with this file, you can obtain one at:
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 */

#ifndef PHYSIKA_IO_SURFACE_MESH_IO_OBJ_MESH_IO_H_
#define PHYSIKA_IO_SURFACE_MESH_IO_OBJ_MESH_IO_H_

#include <string>

namespace Physika{

template <typename Scalar> class SurfaceMesh;

template <typename Scalar>
class ObjMeshIO
{
public:
    ObjMeshIO(){}
    ~ObjMeshIO(){}

    // load a mesh from a obj file.
    static void load(const std::string &filename, SurfaceMesh<Scalar> *mesh);
    // save a mesh to a obj file.
    static void save(const std::string &filename, SurfaceMesh<Scalar> *mesh);

protected:
    static void loadMaterials(const std::string &filename, SurfaceMesh<Scalar> *mesh); //load material of the mesh from material file
    static void saveMaterials(const std::string &filename, SurfaceMesh<Scalar> *mesh);
};

} //end of namespace Physika

#endif //PHYSIKA_IO_SURFACE_MESH_IO_OBJ_MESH_IO_H_
