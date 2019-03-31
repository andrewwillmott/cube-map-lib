//------------------------------------------------------------------------------
// Utilities for traversing cubemaps, both in floating point face/u/v space
// and, for power-of-two sizes, grid-based face/i/j space.
// Andrew Willmott
//------------------------------------------------------------------------------

#ifndef CUBE_MAP_LIB_H
#define CUBE_MAP_LIB_H

#include "VL234f.h"

#include <stdint.h>

namespace CML
{
    // --- Basic cube map mappings

    enum tCubeMapFace
    /// The uv parameterization is organized so that it is a consistent permutation of x/y/z. That is,
    /// +X -> (y, z), +Y -> (z, x), +Z -> (x, y).
    /// For -X and so on, the u value is negated so that (u, v) is still right handed. Thus -Z -> -x, y.
    {
        kFace_PZ,   ///< (u, v) = ( x, y)
        kFace_NZ,   ///< (u, v) = (-x, y)
        kFace_PX,   ///< (u, v) = ( y, z)
        kFace_NX,   ///< (u, v) = (-y, z)
        kFace_PY,   ///< (u, v) = ( z, x)
        kFace_NY,   ///< (u, v) = (-z, x)
        kMaxCubeMapFaces
    };

    struct cCubeMapCoord
    {
        float mU;
        float mV;
        int mFace;

        cCubeMapCoord(int face, float u, float v) : mU(u), mV(v), mFace(face) {}
        cCubeMapCoord() {}

        bool operator==(const cCubeMapCoord& a) { return mU == a.mU && mV == a.mV && mFace == a.mFace; }
    };

    Vec3f CoordToDirection(const cCubeMapCoord& coord);
    ///< Converts coord to the corresponding point on the unit cube -- normalize to get a unit direction.
    cCubeMapCoord DirectionToCoord(const Vec3f& direction);
    ///< Converts direction to the corresponding coordinate on the unit cube. 'direction' does not need to be normalized.

    Mat3f GetFaceTransform(int face);
    ///< Returns rotation matrix from face-local u/v/w into object x/y/z.
    void  GetFaceMapping  (int face, Vec3f* signVals, int* indices);
    ///< Returns indices and sign values for transforming face-local u/v/w into object x/y/z. (Useful e.g. if you need to do a lot of CoordToDirection transforms to the same face.)
    ///< Example that maps v to fv on face: GetFaceMapping(face, &s, a); fv[a[0]] = s[0] * v[0]; fv[a[1]] = s[1] * v[1]; fv[a[2]] = s[2] * v[2];

    // --- Mapping from height maps/normal maps to the corresponding sphere surface. (Defined as p(c, h) = h * norm(CoordToDirection(c)).)

    void FindHFJacobian(float s, float t, float h, Mat3f& m);
    ///< Finds J, which can be used to rotate the local 'flat' frame at the given point on
    ///< a cube map to the corresponding frame on the sphere.
    void FindHFJacobianInvTranspose(float s, float t, float h, Mat3f& m);
    ///< Returns inverse transpose of FindHFJacobian, suitable for transforming normals
    Vec3f NormalFromHFJacobian(float s, float t, float h, float ds, float dt);
    ///< Directly transforms the normal [ds, dt, 1] into sphere space

    struct cPixel4U8
    {
        // Components reversed for standard BGRA swizzle, but you may prefer
        // to use standard order and a non-colour U8 GPU type.
        uint8_t mZ;
        uint8_t mY;
        uint8_t mX;
        uint8_t mW;
    };

    void CreateNormalMap(int size, float hscale, const uint16_t hmap[], cPixel4U8 nmap[]);
    ///< Creates a standard normal map from a square 16-bit height map. Not intended to be
    ///< used, just for comparison with CreateSphereNormalMap
    void CreateSphereNormalMap(int size, float r, float h, const uint16_t hmap[], cPixel4U8 nmap[]);
    ///< Creates a 'spherical' normal map for the given height map. The normals are generated from
    ///< the r = radius + hscale * h(x)

    void UpdateFaceNormalMap
    ///< Updates a region of the normal map for a particular cube map face. The normals are generated
    ///< in object space, so for all-0 height maps, normalMaps[kPX] = (1, 0, 0), normalMaps[kPY] = (0, 1, 0),
    ///< and so on. Handles sampling onto adjacent faces to avoid seams.
    (
        float         radius,       ///< Radius of sphere
        float         hscale,       ///< Scale of heightmap, so r = radius + hscale * h(x)
        int           size,         ///< Size of faces
        const uint16_t* const heightMaps[6],    ///< Input per-face height maps
        cPixel4U8*      const normalMaps[6],    ///< Normal maps to be modified
        int           face,                     ///< Face to operate on
        Vec2f         minC = vl_0,              ///< [0,1] rectangle to update on the face.
        Vec2f         maxC = vl_1
    );

    // --- Managing cube map face grids

    struct cCubeMapIndex
    {
        int mCol;
        int mRow;
        int mFace;

        cCubeMapIndex(int face, int col, int row) : mCol(col), mRow(row), mFace(face) {}
        cCubeMapIndex() {}
        bool operator==(const cCubeMapIndex& a) { return mCol == a.mCol && mRow == a.mRow && mFace == a.mFace; }
    };
    /// Integer cube map coordinates. Note: all such cubemaps are assumed to be a power-of-two size.

    Vec3f IndexToDirection(int size, const cCubeMapIndex& index);
    ///< Converts index to the corresponding point on the unit cube -- normalize to get a unit direction.
    cCubeMapIndex DirectionToIndex(int size, const Vec3f& direction);
    ///< Converts direction to the corresponding cell index in a grid-based cube map. 'direction' does not need to be normalized.

    void IndexToQuad(int size, const cCubeMapIndex& index, Vec3f corners[4]);
    ///< Get corners of the given cubemap cell on the unit cube.

    cCubeMapCoord IndexToCoord(int size, const cCubeMapIndex& index);
    ///< Convert between float & integer cubemap coords
    cCubeMapIndex CoordToIndex(int size, const cCubeMapCoord& coord);
    ///< Convert between float & integer cubemap coords

    int WrapCubeFace(int size, cCubeMapIndex* index);
    ///< If mRow or mCol have gone out of bounds, wrap to appropriate face. Returns the number of face transitions.
    int WrapCubeFace(int size, int* face, int* x, int* y, int* dx = 0, int* dy = 0);
    ///< Wraps the given face and cell address to a new face if necessary. (I.e., x and/or y have moved off the edge of 'face'.)
    ///< Each face is assumed to consist of size * size cells.
    ///< If dx and dy are specified, they are assumed to be iteration directions, and are also wrapped. 
    ///< Returns the number of face transitions.
    bool NeedsWrap(int size, int x, int y);
    ///< Returns true if the given x/y pair falls outside [0, size), and thus needs to be wrapped. 'size' must be a power of two.
    ///< Convenience for performance-intensive areas.

    int WrapCubeFace(cCubeMapCoord* coord);
    ///< If mU or mV have gone out of bounds, wrap to appropriate face. Returns the number of face transitions.
    int WrapCubeFace(int* face, float* u, float* v, float* du = 0, float* dv = 0);
    ///< Wraps the given face and uv to a new face if necessary. (I.e., uv has gone outside the [0, 1] square.)

    struct cFaceRect
    {
        int mFace;
        int mX0;
        int mY0;
        int mX1;
        int mY1;
    };
    ///< Represents a rectangle within a cube map face grid

    int SplatArea(int size, int face, int x, int y, int r, cFaceRect out[5]);
    int SplatArea(int size, const cCubeMapIndex& faceIndex, int r, cFaceRect out[5]);
    ///< Returns up to 5 face rects covering an area around the given point. If the area is completely
    ///< within a face, it is a rectangle of size 2r+1 x 2r+1, and SplatArea returns 1. Otherwise the area
    ///< overlaps other faces, and SplatArea returns the number of additional rectangles required for coverage.
    int SplatRect(int size, const cFaceRect& in, cFaceRect out[5]);
    ///< As for SplatArea, but you specify the rectangle to be splatted directly.

    int WrapRect(int size, cFaceRect* rect);
    ///< Wraps a rectangle from one face to another. The rectangle is assumed not to straddle faces.
}



/////////////////////////////////////////////////////////////////////////////////////
// Inlines
//

namespace CML
{
    inline int32_t FloorPositiveToInt32(float x)
    {
        return int32_t(x);  // for some platforms it's desirable to avoid int casts
    }

    /// Given a particular axis direction, how do we map (x, y, z) to (u, v, n)?
    const uint8_t kSwizzleTable[3][4] =
    {
        { 0, 1, 2, 0 },        // +Z
        { 1, 2, 0, 0 },        // +X
        { 2, 0, 1, 0 },        // +Y
    };

    const uint8_t kSwizzleTableBack[3][4] = 
    {
        { 0, 1, 2, 0 },        // +Z
        { 2, 0, 1, 0 },        // +X
        { 1, 2, 0, 0 },        // +Y
    };

    inline Vec3f CoordToDirection(const cCubeMapCoord& coord)
    {
        float u = coord.mU * 2.0f - 1.0f;
        float v = coord.mV * 2.0f - 1.0f;
        float invLen = 1.0f / sqrtf(sqr(u) + sqr(v) + 1.0f);

        const uint8_t* swizzle = kSwizzleTable[coord.mFace >> 1];
        float w = (coord.mFace & 1) ? -invLen : invLen;

        Vec3f result;

        result[swizzle[0]] = u * w;
        result[swizzle[1]] = v * invLen;
        result[swizzle[2]] = w;

        return result;
    }

    inline Mat3f GetFaceTransform(int face)
    {
        const uint8_t* swizzle = kSwizzleTable[face >> 1];
        float w = (face & 1) ? -1.0f : 1.0f;

        Mat3f result = vl_0;

        result[0][swizzle[0]] = w;
        result[1][swizzle[1]] = 1.0f;
        result[2][swizzle[2]] = w;

        return result;
    }

    inline void GetFaceMapping(int face, Vec3f* signVals, int* indices)
    {
        const uint8_t* swizzle = kSwizzleTableBack[face >> 1];
        float w = (face & 1) ? -1.0f : 1.0f;
        
        indices[0] = swizzle[0];
        indices[1] = swizzle[1];
        indices[2] = swizzle[2];
        
        Vec3f preSignVals(w, 1.0f, w);
        signVals->x = preSignVals[swizzle[0]];
        signVals->y = preSignVals[swizzle[1]];
        signVals->z = preSignVals[swizzle[2]];
    }

    inline cCubeMapCoord DirectionToCoord(const Vec3f& direction)
    {
        const float x = direction.x;
        const float y = direction.y;
        const float z = direction.z;

        const float ax = fabsf(x);
        const float ay = fabsf(y);
        const float az = fabsf(z);

        cCubeMapCoord result;

        if (az >= ax && az >= ay)
        {
            result.mU = 0.5f * (x / z + 1.0f);    // when face is -ve, we flip the u axis, to maintain right-handedness.
            result.mV = 0.5f * (y / az + 1.0f);
            result.mFace = (z >= 0) ? kFace_PZ : kFace_NZ;
        }
        else if (ay >= ax)
        {
            result.mU = 0.5f * (z / y + 1.0f);
            result.mV = 0.5f * (x / ay + 1.0f);
            result.mFace = (y >= 0) ? kFace_PY : kFace_NY;
        }
        else
        {
            result.mU = 0.5f * (y / x + 1.0f);
            result.mV = 0.5f * (z / ax + 1.0f);
            result.mFace = (x >= 0) ? kFace_PX : kFace_NX;
        }

        return result;
    }

    inline Vec3f IndexToDirection(int size, const cCubeMapIndex& index)
    {
        float invSize = 1.0f / size;
        float u = 2 * (index.mCol + 0.5f) * invSize - 1.0f;
        float v = 2 * (index.mRow + 0.5f) * invSize - 1.0f;
        float invLen = 1.0f / sqrtf(sqr(u) + sqr(v) + 1.0f);

        const uint8_t* swizzle = kSwizzleTable[index.mFace >> 1];
        float w = (index.mFace & 1) ? -invLen : invLen;

        Vec3f result;

        result[swizzle[0]] = u * w;
        result[swizzle[1]] = v * invLen;
        result[swizzle[2]] = w;

        return result;
    }

    inline cCubeMapIndex DirectionToIndex(int size, const Vec3f& direction)
    {
        const float ax = fabsf(direction.x);
        const float ay = fabsf(direction.y);
        const float az = fabsf(direction.z);
        const float halfSize = 0.5f * size;

        cCubeMapIndex result;

        if (az >= ax && az >= ay)
        {
            result.mCol = FloorPositiveToInt32((direction[0] / direction[2] + 1.0f) * halfSize);
            result.mRow = FloorPositiveToInt32((direction[1] / az           + 1.0f) * halfSize);
            result.mFace = (direction[2] >= 0) ? kFace_PZ : kFace_NZ;
        }
        else if (ay >= ax)
        {
            result.mCol = FloorPositiveToInt32((direction[2] / direction[1] + 1.0f) * halfSize);
            result.mRow = FloorPositiveToInt32((direction[0] / ay           + 1.0f) * halfSize);
            result.mFace = (direction[1] >= 0) ? kFace_PY : kFace_NY;
        }
        else
        {
            result.mCol = FloorPositiveToInt32((direction[1] / direction[0] + 1.0f) * halfSize);
            result.mRow = FloorPositiveToInt32((direction[2] / ax           + 1.0f) * halfSize);
            result.mFace = (direction[0] >= 0) ? kFace_PX : kFace_NX;
        }

        // wrap 
        if (result.mCol == size)
            result.mCol--;
        if (result.mRow == size)
            result.mRow--;

        return result;
    }

    inline bool NeedsWrap(int size, int x, int y)
    {
        return ((x | y) & ~(size - 1)) != 0;
    }

    inline int WrapCubeFace(int size, cCubeMapIndex* index)
    {
        if (NeedsWrap(size, index->mCol, index->mRow))
            return WrapCubeFace(size, &index->mFace, &index->mCol, &index->mRow);

        return 0;
    }

    inline int WrapCubeFace(cCubeMapCoord* coord)
    {
        if (coord->mU < 0.0f || coord->mV < 0.0f || coord->mU > 1.0f || coord->mV > 1.0f)
            return WrapCubeFace(&coord->mFace, &coord->mU, &coord->mV);

        return 0;
    }

    inline Vec3f SpherePointCanonical(float s, float t, float h)
    {
        /// Find the direction vector for s, t, and scale by h.
        return h * norm(Vec3f(s, t, 1.0f) / (1.0f + sqr(s) + sqr(t)));
    }


    inline Vec3f SpherePoint(cCubeMapCoord c, float h)
    {
        /// Find the direction vector for s, t, and scale by h.
        return h * norm(Vec3f(c.mU, c.mV, 1.0f) / (1.0f + sqr(c.mU) + sqr(c.mV)));
    }
}

#endif
