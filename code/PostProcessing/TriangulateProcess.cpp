/*
---------------------------------------------------------------------------
Open Asset Import Library (assimp)
---------------------------------------------------------------------------

Copyright (c) 2006-2020, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the following
conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------
*/

/** @file  TriangulateProcess.cpp
 *  @brief Implementation of the post processing step to split up
 *    all faces with more than three indices into triangles.
 *
 *
 *  The triangulation algorithm will handle concave or convex polygons.
 *  Self-intersecting or non-planar polygons are not rejected, but
 *  they're probably not triangulated correctly.
 */

#ifndef ASSIMP_BUILD_NO_TRIANGULATE_PROCESS

#include "PostProcessing/TriangulateProcess.h"
#include "Common/PolyTools.h"
#include "PostProcessing/ProcessHelper.h"

#include "contrib/poly2tri/poly2tri/poly2tri.h"

#include <cstdint>
#include <memory>

namespace Assimp {

// ------------------------------------------------------------------------------------------------
// Constructor to be privately used by Importer
TriangulateProcess::TriangulateProcess() {
    // nothing to do here
}

// ------------------------------------------------------------------------------------------------
// Destructor, private as well
TriangulateProcess::~TriangulateProcess() {
    // nothing to do here
}

// ------------------------------------------------------------------------------------------------
// Returns whether the processing step is present in the given flag field.
bool TriangulateProcess::IsActive(unsigned int pFlags) const {
    return (pFlags & aiProcess_Triangulate) != 0;
}

// ------------------------------------------------------------------------------------------------
// Executes the post processing step on the given imported data.
void TriangulateProcess::Execute(aiScene *pScene) {
    ASSIMP_LOG_DEBUG("TriangulateProcess begin");

    bool bHas = false;
    for (unsigned int a = 0; a < pScene->mNumMeshes; a++) {
        if (pScene->mMeshes[a]) {
            if (TriangulateMesh(pScene->mMeshes[a])) {
                bHas = true;
            }
        }
    }
    if (bHas) {
        ASSIMP_LOG_INFO("TriangulateProcess finished. All polygons have been triangulated.");
    } else {
        ASSIMP_LOG_DEBUG("TriangulateProcess finished. There was nothing to be done.");
    }
}

// ------------------------------------------------------------------------------------------------
static bool validateNumIndices(aiMesh *mesh) {
    bool bNeed = false;
    for (unsigned int a = 0; a < mesh->mNumFaces; a++) {
        const aiFace &face = mesh->mFaces[a];
        if (face.mNumIndices != 3) {
            bNeed = true;
            break;
        }
    }

    return bNeed;
}

// ------------------------------------------------------------------------------------------------
static void calulateNumOutputFaces(aiMesh *mesh, size_t &numOut, size_t &maxOut, bool &getNormals) {
    numOut = maxOut = 0;
    getNormals = true;
    for (unsigned int a = 0; a < mesh->mNumFaces; a++) {
        aiFace &face = mesh->mFaces[a];
        if (face.mNumIndices <= 4) {
            getNormals = false;
        }
        if (face.mNumIndices <= 3) {
            numOut++;

        } else {
            numOut += face.mNumIndices - 2;
            maxOut = std::max(maxOut, static_cast<size_t>(face.mNumIndices));
        }
    }
}

// ------------------------------------------------------------------------------------------------
static void quad2Triangles(const aiFace &face, const aiVector3D *verts, aiFace *curOut) {
    // quads can have at maximum one concave vertex. Determine
    // this vertex (if it exists) and start tri-fanning from
    // it.
    unsigned int start_vertex = 0;
    for (unsigned int i = 0; i < 4; ++i) {
        const aiVector3D &v0 = verts[face.mIndices[(i + 3) % 4]];
        const aiVector3D &v1 = verts[face.mIndices[(i + 2) % 4]];
        const aiVector3D &v2 = verts[face.mIndices[(i + 1) % 4]];

        const aiVector3D &v = verts[face.mIndices[i]];

        aiVector3D left = (v0 - v);
        aiVector3D diag = (v1 - v);
        aiVector3D right = (v2 - v);

        left.Normalize();
        diag.Normalize();
        right.Normalize();

        const float angle = std::acos(left * diag) + std::acos(right * diag);
        if (angle > AI_MATH_PI_F) {
            // this is the concave point
            start_vertex = i;
            break;
        }
    }

    const unsigned int temp[] = { face.mIndices[0], face.mIndices[1], face.mIndices[2], face.mIndices[3] };

    aiFace &nface = *curOut++;
    nface.mNumIndices = 3;
    nface.mIndices = face.mIndices;

    nface.mIndices[0] = temp[start_vertex];
    nface.mIndices[1] = temp[(start_vertex + 1) % 4];
    nface.mIndices[2] = temp[(start_vertex + 2) % 4];

    aiFace &sface = *curOut++;
    sface.mNumIndices = 3;
    sface.mIndices = new unsigned int[3];

    sface.mIndices[0] = temp[start_vertex];
    sface.mIndices[1] = temp[(start_vertex + 2) % 4];
    sface.mIndices[2] = temp[(start_vertex + 3) % 4];
}

// ------------------------------------------------------------------------------------------------
bool getContourFromePolyline(aiFace &face, aiMesh *pMesh, std::vector<p2t::Point *> &contour,
        aiMatrix4x4 &m, aiVector3D &vmin, aiVector3D &vmax, ai_real &zcoord) {
    aiVector3D normal;
    bool ok = true;
    m = DerivePlaneCoordinateSpace<ai_real>(pMesh->mVertices, pMesh->mNumVertices, ok, normal);
    if (!ok) {
        false;
    }
    for (unsigned int i = 0; i < face.mNumIndices; ++i) {
        unsigned int index = face.mIndices[i];

        const aiVector3D vv = m * pMesh->mVertices[index];
        // keep Z offset in the plane coordinate system. Ignoring precision issues
        // (which  are present, of course), this should be the same value for
        // all polygon vertices (assuming the polygon is planar).

        // XXX this should be guarded, but we somehow need to pick a suitable
        // epsilon
        // if(coord != -1.0f) {
        //  assert(std::fabs(coord - vv.z) < 1e-3f);
        // }
        zcoord += vv.z;
        vmin = std::min(vv, vmin);
        vmax = std::max(vv, vmax);

        contour.push_back(new p2t::Point(vv.x, vv.y));
    }

    zcoord /= pMesh->mNumVertices;

    // Further improve the projection by mapping the entire working set into
    // [0,1] range. This gives us a consistent data range so all epsilons
    // used below can be constants.
    vmax -= vmin;
    const aiVector2D one_vec(1, 1);

    for (p2t::Point* &vv : contour) {
        vv->x = (vv->x - vmin.x) / vmax.x;
        vv->y = (vv->y - vmin.y) / vmax.y;

        // sanity rounding
        aiVector2D cur_vv((ai_real) vv->x, (ai_real)vv->y);
        cur_vv = std::max(cur_vv, aiVector2D());
        cur_vv = std::min(cur_vv, one_vec);
    }

    aiMatrix4x4 mult;
    mult.a1 = static_cast<ai_real>(1.0) / vmax.x;
    mult.b2 = static_cast<ai_real>(1.0) / vmax.y;

    mult.a4 = -vmin.x * mult.a1;
    mult.b4 = -vmin.y * mult.b2;
    mult.c4 = -zcoord;
    m = mult * m;

    return true;
}

// ------------------------------------------------------------------------------------------------
// Triangulates the given mesh.
bool TriangulateProcess::TriangulateMesh(aiMesh *pMesh) {
    // Now we have aiMesh::mPrimitiveTypes, so this is only here for test cases

    if (!pMesh->mPrimitiveTypes) {
        if (!validateNumIndices(pMesh)) {
            ASSIMP_LOG_DEBUG("Error while validating number of indices.");
            return false;
        }
    } else if (!(pMesh->mPrimitiveTypes & aiPrimitiveType_POLYGON)) {
        ASSIMP_LOG_DEBUG("???!");
        return false;
    }

    // Find out how many output faces we'll get
    size_t numOut = 0, max_out = 0;
    bool getNormals = true;
    calulateNumOutputFaces(pMesh, numOut, max_out, getNormals);
    if (numOut == pMesh->mNumFaces) {
        ASSIMP_LOG_DEBUG("Error while generating contour.");
        return false;
    }

    // the output mesh will contain triangles, but no polys anymore
    pMesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
    pMesh->mPrimitiveTypes &= ~aiPrimitiveType_POLYGON;

    aiFace *out = new aiFace[numOut](), *curOut = out;
    const size_t Capa = max_out + 2;
    std::vector<aiVector3D> temp_verts3d(max_out + 2); /* temporary storage for vertices */
    std::vector<aiVector2D> temp_verts(max_out + 2);

    // Apply vertex colors to represent the face winding?

    const aiVector3D *verts = pMesh->mVertices;

    // use std::unique_ptr to avoid slow std::vector<bool> specialiations
    std::unique_ptr<bool[]> done(new bool[max_out]);
    for (unsigned int a = 0; a < pMesh->mNumFaces; a++) {
        aiFace &face = pMesh->mFaces[a];

        // if it's a simple point,line or triangle: just copy it
        if (face.mNumIndices <= 3) {
            aiFace &nface = *curOut++;
            nface.mNumIndices = face.mNumIndices;
            nface.mIndices = face.mIndices;

            face.mIndices = nullptr;
        } else if (face.mNumIndices == 4) {
            // optimized code for quadrilaterals
            quad2Triangles(face, verts, curOut);
            face.mIndices = nullptr;
        } else {
            std::vector<p2t::Point *> contour;
            aiMatrix4x4 m;
            aiVector3D vmin, vmax;
            ai_real zcoord = -1;
            if (!getContourFromePolyline(face, pMesh, contour, m, vmin, vmax, zcoord)) {
                ASSIMP_LOG_DEBUG("Error while generating contour.");
                continue;
            }
            p2t::CDT cdt(contour);
            cdt.Triangulate();
            const std::vector<p2t::Triangle *> tris = cdt.GetTriangles();
            const aiMatrix4x4 matInv = m.Inverse();
            for (p2t::Triangle *tri : tris) {
                curOut->mNumIndices = 3;
                curOut->mIndices = new unsigned int[curOut->mNumIndices];
                for (int i = 0; i < 3; ++i) {
                    const aiVector2D v = aiVector2D(static_cast<ai_real>(tri->GetPoint(i)->x), static_cast<ai_real>(tri->GetPoint(i)->y));
//                    ai_assert(v.x <= 1.0 && v.x >= 0.0 && v.y <= 1.0 && v.y >= 0.0);
                    const aiVector3D v3 = matInv * aiVector3D(vmin.x + v.x * vmax.x, vmin.y + v.y * vmax.y, zcoord);
                    temp_verts3d.emplace_back(v3);
                    curOut->mIndices[i] = (unsigned int) temp_verts3d.size()-1;
                }
                curOut++;
            }
            face.mIndices = nullptr;
        }
    }

    delete[] pMesh->mFaces;
    pMesh->mFaces = out;
    pMesh->mNumVertices = (unsigned int)temp_verts3d.size();
    delete[] pMesh->mVertices;
    pMesh->mVertices = new aiVector3D[pMesh->mNumVertices];
    for (size_t i = 0; i < temp_verts3d.size(); ++i) {
        pMesh->mVertices[i] = temp_verts3d[i];
    }
    pMesh->mNumFaces = (unsigned int)(curOut - out); /* not necessarily equal to numOut */

    return true;
}

} // namespace Assimp

#endif // !! ASSIMP_BUILD_NO_TRIANGULATE_PROCESS
