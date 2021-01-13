/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2020, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

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

----------------------------------------------------------------------
*/

/** @file PolyTools.h, various utilities for our dealings with arbitrary polygons */

#ifndef AI_POLYTOOLS_H_INCLUDED
#define AI_POLYTOOLS_H_INCLUDED

#include <assimp/ai_assert.h>
#include <assimp/material.h>
#include <assimp/vector3.h>

namespace Assimp {

template<class T>
class TBoundingBox2D {
    T mMin, mMax;

    TBoundingBox2D( const T &min, const T &max ) :
            mMin( min ),
            mMax( max ) {
        // empty
    }
};

using BoundingBox2D = TBoundingBox2D<aiVector2D>;

// -------------------------------------------------------------------------------
/// Compute the normal of an arbitrary polygon in R3.
///
/// The code is based on Newell's formula, that is a polygons normal is the ratio
/// of its area when projected onto the three coordinate axes.
///
/// @param out Receives the output normal
/// @param num Number of input vertices
/// @param x X data source. x[ofs_x*n] is the n'th element.
/// @param y Y data source. y[ofs_y*n] is the y'th element
/// @param z Z data source. z[ofs_z*n] is the z'th element
///
/// @note The data arrays must have storage for at least num+2 elements. Using
/// this method is much faster than the 'other' NewellNormal()
// -------------------------------------------------------------------------------
template <size_t ofs_x, size_t ofs_y, size_t ofs_z, typename TReal>
inline void NewellNormal(aiVector3t<TReal> &out, size_t num, TReal *x, TReal *y, TReal *z, size_t bufferSize) {
    ai_assert(bufferSize > num);

    if (nullptr == x || nullptr == y || nullptr == z || 0 == bufferSize || 0 == num) {
        return;
    }

    // Duplicate the first two vertices at the end
    x[(num + 0) * ofs_x] = x[0];
    x[(num + 1) * ofs_x] = x[ofs_x];

    y[(num + 0) * ofs_y] = y[0];
    y[(num + 1) * ofs_y] = y[ofs_y];

    z[(num + 0) * ofs_z] = z[0];
    z[(num + 1) * ofs_z] = z[ofs_z];

    TReal sum_xy = 0.0, sum_yz = 0.0, sum_zx = 0.0;

    TReal *xptr = x + ofs_x, *xlow = x, *xhigh = x + ofs_x * 2;
    TReal *yptr = y + ofs_y, *ylow = y, *yhigh = y + ofs_y * 2;
    TReal *zptr = z + ofs_z, *zlow = z, *zhigh = z + ofs_z * 2;

    for (size_t tmp = 0; tmp < num; ++tmp ) {
        sum_xy += (*xptr) * ((*yhigh) - (*ylow));
        sum_yz += (*yptr) * ((*zhigh) - (*zlow));
        sum_zx += (*zptr) * ((*xhigh) - (*xlow));

        xptr += ofs_x;
        xlow += ofs_x;
        xhigh += ofs_x;

        yptr += ofs_y;
        ylow += ofs_y;
        yhigh += ofs_y;

        zptr += ofs_z;
        zlow += ofs_z;
        zhigh += ofs_z;
    }
    out = aiVector3t<TReal>(sum_yz, sum_zx, sum_xy);
}

// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------
template <class T>
inline aiMatrix4x4t<T> DerivePlaneCoordinateSpace(const aiVector3t<T> *vertices, size_t numVertices, bool &ok, aiVector3t<T> &norOut) {
    const aiVector3t<T> *out = vertices;
    aiMatrix4x4t<T> m;

    ok = true;

    const size_t s = numVertices;

    const aiVector3t<T> &any_point = out[numVertices - 1u];
    aiVector3t<T> nor;

    // The input polygon is arbitrarily shaped, therefore we might need some tries
    // until we find a suitable normal. Note that Newell's algorithm would give
    // a more robust result, but this variant also gives us a suitable first
    // axis for the 2D coordinate space on the polygon plane, exploiting the
    // fact that the input polygon is nearly always a quad.
    bool done = false;
    size_t idx = 0;
    for (size_t i = 0; !done && i < s - 2; done || ++i) {
        idx = i;
        for (size_t j = i + 1; j < s - 1; ++j) {
            nor = -((out[i] - any_point) ^ (out[j] - any_point));
            if (std::fabs(nor.Length()) > 1e-8f) {
                done = true;
                break;
            }
        }
    }

    if (!done) {
        ok = false;
        return m;
    }

    nor.Normalize();
    norOut = nor;

    aiVector3t<T> r = (out[idx] - any_point);
    r.Normalize();

    // Reconstruct orthonormal basis
    // XXX use Gram Schmidt for increased robustness
    aiVector3t<T> u = r ^ nor;
    u.Normalize();

    m.a1 = r.x;
    m.a2 = r.y;
    m.a3 = r.z;

    m.b1 = u.x;
    m.b2 = u.y;
    m.b3 = u.z;

    m.c1 = -nor.x;
    m.c2 = -nor.y;
    m.c3 = -nor.z;

    return m;
}

} // namespace Assimp

#endif
