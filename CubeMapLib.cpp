//------------------------------------------------------------------------------
// Utilities for traversing cubemaps, both in floating point face/u/v space
// and, for power-of-two sizes, grid-based face/i/j space.
//------------------------------------------------------------------------------

#include "CubeMapLib.h"

#include <string.h>

using namespace CML;

namespace
{
    inline int32_t FloorToI32(float f)
    {
        return int32_t(floorf(f));
    }

    uint8_t FloatToU8(float f)
    {
        int i = int(f * 256.0f);
        return uint8_t(i - (i >> 8));
    }
}

void CML::IndexToQuad(int size, const cCubeMapIndex& index, Vec3f corners[4])
{
    const float invSize = 1.0f / size;
    cCubeMapCoord coord;
    coord.mU = index.mCol * invSize;
    coord.mV = index.mRow * invSize;
    coord.mFace = index.mFace;
    
    corners[0] = CoordToDirection(coord); // UL

    coord.mU += invSize;
    corners[2] = CoordToDirection(coord); // UR

    coord.mV += invSize;
    corners[3] = CoordToDirection(coord); // BR

    coord.mU -= invSize;
    corners[1] = CoordToDirection(coord); // BL
}

cCubeMapIndex CML::CoordToIndex(int size, const cCubeMapCoord& coord)
{
    cCubeMapIndex index;

    index.mFace = coord.mFace;
    index.mCol = FloorPositiveToInt32(coord.mU * size);
    index.mRow = FloorPositiveToInt32(coord.mV * size);

    if (index.mCol == size)
        --index.mCol;
    if (index.mRow == size)
        --index.mRow;

    return index;
}

cCubeMapCoord CML::IndexToCoord(int size, const cCubeMapIndex& index)
{
    cCubeMapCoord coord;

    float invSize = 1.0f / size;
    coord.mFace = index.mFace;
    coord.mU = (index.mCol + 0.5f) * invSize;
    coord.mV = (index.mRow + 0.5f) * invSize;

    return coord;
}

int CML::WrapCubeFace(int size, int* faceIn, int* xIn, int* yIn, int* dx, int* dy)
{    
    int& face = *faceIn;
    int& x = *xIn;
    int& y = *yIn;

    // when wrapping to a new face, we always switch x and y.
    // when wrapping to the left/bottom (coord goes -ve): 
    //   if the source face is positive, 
    //   we then negate x, otherwise we negate y.
    // for wrapping to right/top (coord >= size), it's the other way around.
    
    // We hit one if clause roughly 4/size of the time
    // We hit both an x and y clause roughly 4/sqr(size) of the time.
    
    // if ((x | y) & ~(size - 1) == 0) // if size is a power of 2, this detects whether there is any wrap
    //     return;
    
    int s0  = face & 1;
    int s1  = 1 ^ s0;
    int ss0 = 1 - 2 * s0;
    int ss1 = -ss0;
    
    if (x < 0)
    {
        x += size;

        int x1(y), y1(x);
        
        x = ss1 * x1 + s1 * (size - 1);
        y = ss0 * y1 + s0 * (size - 1);
        
        face = 2 * kSwizzleTable[face >> 1][1] + s1;    // -sign(face) * swizzle(face, 0)

        if (dx)
        {
            int dx1(*dx), dy1(*dy);
            *dx = ss1 * dy1;
            *dy = ss0 * dx1;
        }
        
        // we just switched y to x -- we no longer have to check y, but must check x again.
        if ((x & ~(size - 1)) == 0)
            return 1;
        else
            return 1 + WrapCubeFace(size, &face, &x, &y, dx, dy);   // happens rarely (on a corner diagonal move), so just recurse
    }
    else if (x >= size)
    {
        x -= size;
        
        int x1(y), y1(x);
        
        x = ss0 * x1 + s0 * (size - 1);
        y = ss1 * y1 + s1 * (size - 1);
        
        face = 2 * kSwizzleTable[face >> 1][1] + s0;    // sign(face) * swizzle(face, 0)

        if (dx)
        {
            int dx1(*dx), dy1(*dy);
            *dx = ss0 * dy1;
            *dy = ss1 * dx1;
        }

        // we just switched y to x -- we no longer have to check y, but must check x again.
        if ((x & ~(size - 1)) == 0)
            return 1;
        else
            return 1 + WrapCubeFace(size, &face, &x, &y, dx, dy);   // happens rarely (on a corner diagonal move), so just recurse
    }
    
    if (y < 0)
    {
        y += size;
        
        int x1(y), y1(x);
        
        x = ss1 * x1 + s1 * (size - 1);
        y = ss0 * y1 + s0 * (size - 1);
        
        face = 2 * kSwizzleTable[face >> 1][2] + 1;    // swizzle(face, 0)

        if (dx)
        {
            int dx1(*dx), dy1(*dy);
            *dx = ss1 * dy1;
            *dy = ss0 * dx1;
        }
        
        return 1;
    }
    else if (y >= size)
    {
        y -= size;
        
        int x1(y), y1(x);
        
        x = ss1 * x1 + s1 * (size - 1);
        y = ss0 * y1 + s0 * (size - 1);
        
        face = 2 * kSwizzleTable[face >> 1][2];    // -swizzle(face, 0)

        if (dx)
        {
            int dx1(*dx), dy1(*dy);
            *dx = ss1 * dy1;
            *dy = ss0 * dx1;
        }
        
        return 1;
    }
    
    return 0;
}

int CML::WrapRect(int size, cFaceRect* rect)
{    
    int s0 = rect->mFace & 1;
    int s1 = 1 ^ s0;
    
    if (rect->mX0 < 0)
    {
        rect->mX0 += size;
        rect->mX1 += size;

        rect->mFace = 2 * kSwizzleTable[rect->mFace >> 1][1] + s1;    // -sign(face) * swizzle(face, 0)
        
        cFaceRect prevRect(*rect);

        // for rects, we need to switch the two endpoints if we're negating
        if (s0)
        {
            rect->mX0 = prevRect.mY0;
            rect->mX1 = prevRect.mY1;
            rect->mY0 = size - prevRect.mX1;
            rect->mY1 = size - prevRect.mX0;
        }
        else
        {
            rect->mX0 = size - prevRect.mY1;
            rect->mX1 = size - prevRect.mY0;
            rect->mY0 = prevRect.mX0;
            rect->mY1 = prevRect.mX1;
        }
        
        // we just switched y to x -- we no longer have to check y, but must check x again.
        if ((rect->mX0 & ~(size - 1)) == 0)
            return 1;
        else
            return 1 + WrapRect(size, rect);   // happens rarely (on a corner diagonal move), so just recurse
    }
    else if (rect->mX0 >= size)
    {
        rect->mX0 -= size;
        rect->mX1 -= size;
        
        rect->mFace = 2 * kSwizzleTable[rect->mFace >> 1][1] + s0;    // sign(face) * swizzle(face, 0)

        cFaceRect prevRect(*rect);
        
        if (s0)
        {
            rect->mX0 = size - prevRect.mY1;
            rect->mX1 = size - prevRect.mY0;
            rect->mY0 = prevRect.mX0;
            rect->mY1 = prevRect.mX1;
        }
        else
        {
            rect->mX0 = prevRect.mY0;
            rect->mX1 = prevRect.mY1;
            rect->mY0 = size - prevRect.mX1;
            rect->mY1 = size - prevRect.mX0;
        }

        // we just switched y to x -- we no longer have to check y, but must check x again
        if ((rect->mX0 & ~(size - 1)) == 0)
            return 1;
        else
            return 1 + WrapRect(size, rect);   // happens rarely (on a corner diagonal move), so just recurse
    }
    
    if (rect->mY0 < 0)
    {
        rect->mY0 += size;
        rect->mY1 += size;
        
        rect->mFace = 2 * kSwizzleTable[rect->mFace >> 1][2] + 1;    // swizzle(face, 0)
        
        cFaceRect prevRect(*rect);

        if (s0)
        {
            rect->mX0 = prevRect.mY0;
            rect->mX1 = prevRect.mY1;
            rect->mY0 = size - prevRect.mX1;
            rect->mY1 = size - prevRect.mX0;
        }
        else
        {
            rect->mX0 = size - prevRect.mY1;
            rect->mX1 = size - prevRect.mY0;
            rect->mY0 = prevRect.mX0;
            rect->mY1 = prevRect.mX1;
        }
        
        return 1;
    }
    else if (rect->mY0 >= size)
    {
        rect->mY0 -= size;
        rect->mY1 -= size;
        
        rect->mFace = 2 * kSwizzleTable[rect->mFace >> 1][2];    // -swizzle(face, 0)
        
        cFaceRect prevRect(*rect);
        
        if (s0)
        {
            rect->mX0 = prevRect.mY0;
            rect->mX1 = prevRect.mY1;
            rect->mY0 = size - prevRect.mX1;
            rect->mY1 = size - prevRect.mX0;
        }
        else
        {
            rect->mX0 = size - prevRect.mY1;
            rect->mX1 = size - prevRect.mY0;
            rect->mY0 = prevRect.mX0;
            rect->mY1 = prevRect.mX1;
        }
                
        return 1;
    }
    
    return 0;
}

int CML::SplatArea(int size, int face, int x, int y, int r, cFaceRect out[5])
{
    CL_ASSERT(r <= size);   // larger than this, and we're covering more than the hemisphere, and risk wrapping all the way around to the sixth face.

    cFaceRect in = { face, x - r, y - r, x + r + 1, y + r + 1 };

    return SplatRect(size, in, out);
}

int CML::SplatArea(int size, const cCubeMapIndex& index, int r, cFaceRect out[5])
{
    CL_ASSERT(r <= size);   // larger than this, and we're covering more than the hemisphere, and risk wrapping all the way around to the sixth face.

    cFaceRect in = { index.mFace, index.mCol - r, index.mRow - r, index.mCol + r + 1, index.mRow + r + 1 };

    return SplatRect(size, in, out);
}

int CML::SplatRect(int size, const cFaceRect& in, cFaceRect out[5]) 
{
    int count = 1;   
    out[0] = in;

    if (((in.mX0 | in.mY0 | in.mX1 | in.mY1) & ~(size - 1)) == 0)
        return count;
    
    // clip
    if (in.mX0 < 0)
        out[0].mX0 = 0;
        
    if (in.mY0 < 0)
        out[0].mY0 = 0;
        
    if (in.mX1 > size)
        out[0].mX1 = size;
        
    if (in.mY1 > size)
        out[0].mY1 = size;

    // create extra rects wherever we clipped
    if (out[0].mX0 != in.mX0)
    {
        out[count].mFace = in.mFace;
        out[count].mX0 = in.mX0;
        out[count].mY0 = out[0].mY0;
        out[count].mX1 = 0;
        out[count].mY1 = out[0].mY1;

        WrapRect(size, &out[count]);
        count++;
    }

    if (out[0].mX1 != in.mX1)
    {
        out[count].mFace = in.mFace;
        out[count].mX0 = size;
        out[count].mY0 = out[0].mY0;
        out[count].mX1 = in.mX1;
        out[count].mY1 = out[0].mY1;

        WrapRect(size, &out[count]);
        count++;
    }

    if (out[0].mY0 != in.mY0)
    {
        out[count].mFace = in.mFace;
        out[count].mX0 = out[0].mX0;
        out[count].mY0 = in.mY0;
        out[count].mX1 = out[0].mX1;
        out[count].mY1 = 0;

        WrapRect(size, &out[count]);
        count++;
    }

    if (out[0].mY1 != in.mY1)
    {
        out[count].mFace = in.mFace;
        out[count].mX0 = out[0].mX0;
        out[count].mY0 = size;
        out[count].mX1 = out[0].mX1;
        out[count].mY1 = in.mY1;

        WrapRect(size, &out[count]);
        count++;
    }
    
    return count;
}




int CML::WrapCubeFace(int* faceIn, float* uIn, float* vIn, float* du, float* dv)
// coord version
{
    int& face = *faceIn;
    float& u = *uIn;
    float& v = *vIn;

    // when wrapping to a new face, we always switch u and v.
    // when wrapping to the left/bottom (coord goes -ve): 
    //   if the source face is positive, 
    //   we then negate u, otherwise we negate v.
    // for wrapping to right/top (coord >= 1.0), it's the other way around.
    
    // We hit one if clause roughly 4/size of the time
    // We hit both an u and v clause roughly 4/sqr(size) of the time.
        
    int s0  = face & 1;
    int s1  = 1 ^ s0;

    float sf0 = float(s0);
    float sf1 = float(s1);

    float ss0 = float(1 - 2 * s0);
    float ss1 = -ss0;
    
    if (u < 0.0f)
    {
        u += 1.0f;

        float u1(v), v1(u);
        
        u = ss1 * u1 + sf1;
        v = ss0 * v1 + sf0;
        
        face = 2 * kSwizzleTable[face >> 1][1] + s1;    // -sign(face) * swizzle(face, 0)

        if (du)
        {
            float du1(*du), dy1(*dv);

            *du = ss1 * dy1;
            *dv = ss0 * du1;
        }
        
        // we just switched v to u -- we no longer have to check v, but must check u again.
        if (u >= 0.0f && u <= 1.0f)
            return 1;
        else
            return 1 + WrapCubeFace(&face, &u, &v, du, dv);   // happens rarely (on a corner diagonal move), so just recurse
    }
    else if (u > 1.0f)
    {
        u -= 1.0f;
        
        float u1(v), v1(u);
        
        u = ss0 * u1 + sf0;
        v = ss1 * v1 + sf1;
        
        face = 2 * kSwizzleTable[face >> 1][1] + s0;    // sign(face) * swizzle(face, 0)

        if (du)
        {
            float du1(*du), dy1(*dv);
            *du = ss0 * dy1;
            *dv = ss1 * du1;
        }

        // we just switched v to u -- we no longer have to check v, but must check u again.
        if (u >= 0.0f && u <= 1.0f)
            return 1;
        else
            return 1 + WrapCubeFace(&face, &u, &v, du, dv);   // happens rarely (on a corner diagonal move), so just recurse
    }
    
    if (v < 0.0f)
    {
        v += 1.0f;
        
        float u1(v), v1(u);
        
        u = ss1 * u1 + sf1;
        v = ss0 * v1 + sf0;
        
        face = 2 * kSwizzleTable[face >> 1][2] + 1;    // swizzle(face, 0)

        if (du)
        {
            float du1(*du), dy1(*dv);

            *du = ss1 * dy1;
            *dv = ss0 * du1;
        }
        
        return 1;
    }
    else if (v > 1.0f)
    {
        v -= 1.0f;
        
        float u1(v), v1(u);
        
        u = ss1 * u1 + sf1;
        v = ss0 * v1 + sf0;
        
        face = 2 * kSwizzleTable[face >> 1][2];    // -swizzle(face, 0)

        if (du)
        {
            float du1(*du), dy1(*dv);

            *du = ss1 * dy1;
            *dv = ss0 * du1;
        }
        
        return 1;
    }
    
    return 0;
}




void CML::FindHFJacobian(float s, float t, float h, Mat3f& m)
// 1 divide, 1 sqrt.
{
    float w = sqrtf(1 + sqr(s) + sqr(t));
    float invW = 1 / w;
    float nhinvW3 = -h * invW * invW * invW;

    m(0, 0) = h * invW * (1 - sqr(s * invW));
    m(1, 0) = s * t * nhinvW3;
    m(2, 0) = s * invW;
    m(1, 1) = h * invW * (1 - sqr(t * invW));
    m(2, 1) = t * invW;
    m(0, 2) = s * nhinvW3;
    m(1, 2) = t * nhinvW3;
    m(2, 2) = invW;
}

inline void CML::FindHFJacobianInvTranspose(float s, float t, float h, Mat3f& m)
// 2 divides, 1 sqrt.
{
    float w = sqrtf(1 + sqr(s) + sqr(t));
    float invW = 1 / w;
    float wOverH = w / h;

    m(0, 0) = wOverH;
    m(1, 0) = 0;
    m(2, 0) = s * invW;
    m(0, 1) = 0;
    m(1, 1) = wOverH;
    m(2, 1) = t * invW;
    m(0, 2) = -s * wOverH;
    m(1, 2) = -t * wOverH;
    m(2, 2) = invW;
}

inline Vec3f CML::NormalFromHFJacobian(float s, float t, float h, float horiz, float vert)
// this is just writing out
// FindHFJacobianInvTranspose(s, t, h, J);
// Vec3f v = (Vec3f(horiz, vert, 1) * J);
{
    float w = sqrtf(1 + sqr(s) + sqr(t));
    float invW = 1 / w;
    float wOverH = w / h;

    float horizWOverH = horiz * wOverH;
    float vertWOverH  = vert  * wOverH;

    // All starts to look pretty simple. There must have been an easier way to derive this...
    return Vec3f
    (
        horizWOverH + s * invW,
        vertWOverH  + t * invW,
        -(horizWOverH * s + vertWOverH * t) + invW
    );
}


namespace
{
    inline uint16_t GetCell(const uint16_t* const maps[6], size_t stride, cCubeMapIndex c)
    {
        return maps[c.mFace][c.mCol + stride * c.mRow];
    }

    inline void SetCell(cPixel4U8* const maps[6], size_t stride, cCubeMapIndex c, cPixel4U8 v)
    {
        maps[c.mFace][c.mCol + stride * c.mRow] = v;
    }
}

void CML::CreateNormalMap(int size, float hscale, const uint16_t hmap[], cPixel4U8 nmap[])
{
    float rscale = hscale * float(1.0f / 65535.0f);
    float dscale = rscale * (size / 2.0f);         // inter-two-pixel space is 2/w

    hmap += size;
    nmap += size;

    for (int iv = 1; iv < size - 1; iv++)
    {
        for (int iu = 1; iu < size - 1; iu++)
        {
            float horiz = dscale * (hmap[iu - 1]    - hmap[iu + 1]);
            float vert  = dscale * (hmap[iu - size] - hmap[iu + size]);

            Vec3f v(norm_safe(Vec3f(horiz, vert, 1.0f)));

            nmap[iu].mX = FloatToU8(0.5f * v.z + 0.5f);
            nmap[iu].mY = FloatToU8(0.5f * v.y + 0.5f);
            nmap[iu].mZ = FloatToU8(0.5f * v.x + 0.5f);
            nmap[iu].mW = 255;
        }

        hmap += size;
        nmap += size;
    }
}

void CML::CreateSphereNormalMap(int size, float r, float h, const uint16_t hmap[], cPixel4U8 nmap[])
{
    float rscale = h * float(1.0f / 65535.0f);
    float dscale = rscale * (size / 2.0f);         // inter-two-pixel space is 2/w

    float step = 2.0f / size;
    float t = step * 0.5f - 1.0f;

    hmap += size;
    nmap += size;

    for (int iv = 1; iv < size - 1; iv++, t += step)
    {
        float s = step * 0.5f - 1.0f;

        for (int iu = 1; iu < size - 1; iu++, s += step)
        {
            float horiz = dscale * (hmap[iu - 1]    - hmap[iu + 1]);
            float vert  = dscale * (hmap[iu - size] - hmap[iu + size]);

            float h = r + hmap[iu] * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            nmap[iu].mX = FloatToU8(0.5f * v.x + 0.5f);
            nmap[iu].mY = FloatToU8(0.5f * v.y + 0.5f);
            nmap[iu].mZ = FloatToU8(0.5f * v.z + 0.5f);
            nmap[iu].mW = 255;
        }

        hmap += size;
        nmap += size;
    }
}

void CML::UpdateFaceNormalMap
(
    float         radius,       ///< Radius of sphere
    float         hscale,       ///< Scale of heightmap, so r = radius + hscale * h(x)
    int           size,         ///< Size of faces
    const uint16_t* const heightMaps[6],    ///< Input per-face height maps
    cPixel4U8*      const normalMaps[6],    ///< Normal maps to be modified
    int           face,
    Vec2f         minC,
    Vec2f         maxC
)
{
    // get bbox min and max corner

    // convert to actual indices into raw data
    int startU = FloorToI32(minC.x * size);
    int startV = FloorToI32(minC.y * size);
    int endU   = FloorToI32(maxC.x * size);
    int endV   = FloorToI32(maxC.y * size);

    // the main loop doesn't cross faces
    // mark edges needed
    enum { kLeft, kRight, kBottom, kTop };
    union { uint32_t mFlags; uint8_t mFlag[4]; } edges = { 0 };

    // sanity check indices
    if (startU <= 0)
    {
        edges.mFlag[kLeft] = true;
        startU = 1;
    }
    if (endU >= size - 1)
    {
        edges.mFlag[kRight] = true;
        endU = size - 2;
    }
    if (startV <= 0)
    {
        edges.mFlag[kBottom] = true;
        startV = 1;
    }
    if (endV >= size - 1)
    {
        edges.mFlag[kTop] = true;
        endV = size - 2;
    }

    Vec3f signMap;
    int elemMap[3];

    GetFaceMapping(face, &signMap, elemMap);

    float mx = signMap[0] * 0.5f;
    float my = signMap[1] * 0.5f;
    float mz = signMap[2] * 0.5f;
    int   ix = elemMap[0];
    int   iy = elemMap[1];
    int   iz = elemMap[2];

    float rscale = hscale * float(1.0f / 65535.0f);
    float dscale = rscale * (size / 2.0f);         // inter-two-pixel space is 2/w

    int stride = size;

    // Okay, do face interior: nice and easy, all cells are on the same face.
    const uint16_t* rowPixelsHMap = heightMaps[face] + (stride * startV);
    cPixel4U8*      rowPixelsNMap = normalMaps[face] + (stride * startV);
    cPixel4U8 pixel;

    float step = 2.0f / size;
    float t = step * (startV + 0.5f) - 1;

    for (int iv = startV; iv <= endV; iv++, t += step)
    {
        float s = step * (startU + 0.5f) - 1;

        for (int iu = startU; iu <= endU; iu++, s += step)
        {
            float horiz = dscale * (rowPixelsHMap[iu - 1]      - rowPixelsHMap[iu + 1]);
            float vert  = dscale * (rowPixelsHMap[iu - stride] - rowPixelsHMap[iu + stride]);

            float h = radius + rowPixelsHMap[iu] * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
            pixel.mY = FloatToU8(my * v[iy] + 0.5f);
            pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);

            pixel.mW = 255;

            rowPixelsNMap[iu] = pixel;
        }

        rowPixelsHMap += stride;
        rowPixelsNMap += stride;
    }

    if (!edges.mFlags)
        return;

    // do edges: involves traversing the adjacent edges on two separate faces.
    if (edges.mFlag[kBottom])
    {
        const int kStartV = 0;
        int face2(face), u2(startU), v2(kStartV - 1), du2(1), dv2(0);
        WrapCubeFace(size, &face2, &u2, &v2, &du2, &dv2);   // find start vals and traverse direction for the adjacent face

        CL_ASSERT(du2 == 0); // adjacent faces are always rotated 90

        const uint16_t* pixelsHMap2 = heightMaps[face2] + v2 * stride + u2;
        int stride2 = dv2 * stride;

        const uint16_t* rowPixelsHMap  = heightMaps[face] + (stride * kStartV);
        cPixel4U8*       rowPixelsNMap  = normalMaps[face] + (stride * kStartV);

        float t = step * (kStartV + 0.5f) - 1;
        float s = step * (startU  + 0.5f) - 1;

        for (int iu = startU; iu <= endU; iu++, s += step)
        {
            float horiz = dscale * (rowPixelsHMap[iu - 1] - rowPixelsHMap[iu + 1]);
            float vert  = dscale * (*pixelsHMap2          - rowPixelsHMap[iu + stride]);
            float h = radius + rowPixelsHMap[iu] * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
            pixel.mY = FloatToU8(my * v[iy] + 0.5f);
            pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
            pixel.mW = 255;

            rowPixelsNMap[iu] = pixel;

            pixelsHMap2 += stride2;
        }
    }    

    if (edges.mFlag[kTop])
    {
        const int kStartV = size - 1;
        int face2(face), u2(startU), v2(kStartV + 1), du2(1), dv2(0);
        WrapCubeFace(size, &face2, &u2, &v2, &du2, &dv2);   // find start vals and traverse direction for the adjacent face

        CL_ASSERT(du2 == 0); // adjacent faces are always rotated 90

        const uint16_t* pixelsHMap2 = heightMaps[face2] + v2 * stride + u2;
        int stride2 = dv2 * stride;

        const uint16_t* rowPixelsHMap  = heightMaps[face] + (stride * kStartV);
        cPixel4U8*       rowPixelsNMap  = normalMaps[face] + (stride * kStartV);

        float t = step * (kStartV + 0.5f) - 1;
        float s = step * (startU  + 0.5f) - 1;

        for (int iu = startU; iu <= endU; iu++, s += step)
        {
            float horiz = dscale * (rowPixelsHMap[iu - 1]      - rowPixelsHMap[iu + 1]);
            float vert  = dscale * (rowPixelsHMap[iu - stride] - *pixelsHMap2);
            float h = radius + rowPixelsHMap[iu] * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
            pixel.mY = FloatToU8(my * v[iy] + 0.5f);
            pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
            pixel.mW = 255;

            rowPixelsNMap[iu] = pixel;

            pixelsHMap2 += stride2;
        }
    }

    if (edges.mFlag[kLeft])
    {
        const int kStartU = 0;
        int face2(face), u2(kStartU - 1), v2(startV), du2(0), dv2(1);
        WrapCubeFace(size, &face2, &u2, &v2, &du2, &dv2);

        CL_ASSERT(dv2 == 0);

        const uint16_t* pixelsHMap2    = heightMaps[face2] + stride * v2 + u2;
        const uint16_t* rowPixelsHMap  = heightMaps[face] + stride * startV + kStartU;
        cPixel4U8*       rowPixelsNMap  = normalMaps[face] + stride * startV + kStartU;

        float t = step * (startV  + 0.5f) - 1;
        float s = step * (kStartU + 0.5f) - 1;

        for (int iv = startV; iv <= endV; iv++, t += step)
        {
            float horiz = dscale * (*pixelsHMap2           - rowPixelsHMap[1]);
            float vert  = dscale * (rowPixelsHMap[-stride] - rowPixelsHMap[stride]);
            float h = radius + *rowPixelsHMap * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
            pixel.mY = FloatToU8(my * v[iy] + 0.5f);
            pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
            pixel.mW = 255;

            *rowPixelsNMap = pixel;
            rowPixelsHMap += stride;
            rowPixelsNMap += stride;
            pixelsHMap2 += du2;
        }
    }

    if (edges.mFlag[kRight])
    {
        const int kStartU = size - 1;
        int face2(face), u2(kStartU + 1), v2(startV), du2(0), dv2(1);
        WrapCubeFace(size, &face2, &u2, &v2, &du2, &dv2);

        CL_ASSERT(dv2 == 0);

        const uint16_t* pixelsHMap2    = heightMaps[face2] + stride * v2 + u2;
        const uint16_t* rowPixelsHMap  = heightMaps[face] + stride * startV + kStartU;
        cPixel4U8*       rowPixelsNMap  = normalMaps[face] + stride * startV + kStartU;

        float t = step * (startV  + 0.5f) - 1;
        float s = step * (kStartU + 0.5f) - 1;

        for (int iv = startV; iv <= endV; iv++, t += step)
        {
            float horiz = dscale * (rowPixelsHMap[-1]      - *pixelsHMap2);
            float vert  = dscale * (rowPixelsHMap[-stride] - rowPixelsHMap[stride]);
            float h = radius + *rowPixelsHMap * rscale;

            Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

            pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
            pixel.mY = FloatToU8(my * v[iy] + 0.5f);
            pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
            pixel.mW = 255;

            *rowPixelsNMap = pixel;
            rowPixelsHMap += stride;
            rowPixelsNMap += stride;
            pixelsHMap2 += du2;
        }
    }


    if (edges.mFlag[kBottom] && edges.mFlag[kLeft])
    {
        const int kU = 0;
        const int kV = 0;
        cCubeMapIndex c  (face, kU    , kV    );
        cCubeMapIndex ch0(face, kU - 1, kV    );
        cCubeMapIndex ch1(face, kU + 1, kV    );
        cCubeMapIndex cv0(face, kU    , kV - 1);
        cCubeMapIndex cv1(face, kU    , kV + 1);

        WrapCubeFace(size, &ch0);
        WrapCubeFace(size, &cv0);

        float t = step * (c.mRow + 0.5f) - 1;
        float s = step * (c.mCol + 0.5f) - 1;

        float horiz = dscale * (GetCell(heightMaps, stride, ch0) - GetCell(heightMaps, stride, ch1));
        float vert  = dscale * (GetCell(heightMaps, stride, cv0) - GetCell(heightMaps, stride, cv1));
        float h = radius + GetCell(heightMaps, stride, c) * rscale;

        Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

        pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
        pixel.mY = FloatToU8(my * v[iy] + 0.5f);
        pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
        pixel.mW = 255;

        SetCell(normalMaps, stride, c, pixel);
    }

    if (edges.mFlag[kBottom] && edges.mFlag[kRight])
    {
        const int kU = size - 1;
        const int kV = 0;
        cCubeMapIndex c  (face, kU    , kV    );
        cCubeMapIndex ch0(face, kU - 1, kV    );
        cCubeMapIndex ch1(face, kU + 1, kV    );
        cCubeMapIndex cv0(face, kU    , kV - 1);
        cCubeMapIndex cv1(face, kU    , kV + 1);

        WrapCubeFace(size, &ch1);
        WrapCubeFace(size, &cv0);

        float t = step * (c.mRow + 0.5f) - 1;
        float s = step * (c.mCol + 0.5f) - 1;

        float horiz = dscale * (GetCell(heightMaps, stride, ch0) - GetCell(heightMaps, stride, ch1));
        float vert  = dscale * (GetCell(heightMaps, stride, cv0) - GetCell(heightMaps, stride, cv1));
        float h = radius + GetCell(heightMaps, stride, c) * rscale;

        Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

        pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
        pixel.mY = FloatToU8(my * v[iy] + 0.5f);
        pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
        pixel.mW = 255;

        SetCell(normalMaps, stride, c, pixel);
    }

    if (edges.mFlag[kTop] && edges.mFlag[kLeft])
    {
        const int kU = 0;
        const int kV = size - 1;
        cCubeMapIndex c  (face, kU    , kV    );
        cCubeMapIndex ch0(face, kU - 1, kV    );
        cCubeMapIndex ch1(face, kU + 1, kV    );
        cCubeMapIndex cv0(face, kU    , kV - 1);
        cCubeMapIndex cv1(face, kU    , kV + 1);

        WrapCubeFace(size, &ch0);
        WrapCubeFace(size, &cv1);

        float t = step * (c.mRow + 0.5f) - 1;
        float s = step * (c.mCol + 0.5f) - 1;

        float horiz = dscale * (GetCell(heightMaps, stride, ch0) - GetCell(heightMaps, stride, ch1));
        float vert  = dscale * (GetCell(heightMaps, stride, cv0) - GetCell(heightMaps, stride, cv1));
        float h = radius + GetCell(heightMaps, stride, c) * rscale;

        Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

        pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
        pixel.mY = FloatToU8(my * v[iy] + 0.5f);
        pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
        pixel.mW = 255;

        SetCell(normalMaps, stride, c, pixel);
    }

    if (edges.mFlag[kTop] && edges.mFlag[kRight])
    {
        const int kU = size - 1;
        const int kV = size - 1;
        cCubeMapIndex c  (face, kU    , kV    );
        cCubeMapIndex ch0(face, kU - 1, kV    );
        cCubeMapIndex ch1(face, kU + 1, kV    );
        cCubeMapIndex cv0(face, kU    , kV - 1);
        cCubeMapIndex cv1(face, kU    , kV + 1);

        WrapCubeFace(size, &ch1);
        WrapCubeFace(size, &cv1);

        float t = step * (c.mRow + 0.5f) - 1;
        float s = step * (c.mCol + 0.5f) - 1;

        float horiz = dscale * (GetCell(heightMaps, stride, ch0) - GetCell(heightMaps, stride, ch1));
        float vert  = dscale * (GetCell(heightMaps, stride, cv0) - GetCell(heightMaps, stride, cv1));
        float h = radius + GetCell(heightMaps, stride, c) * rscale;

        Vec3f v(norm_safe(NormalFromHFJacobian(s, t, h, horiz, vert)));

        pixel.mX = FloatToU8(mx * v[ix] + 0.5f);
        pixel.mY = FloatToU8(my * v[iy] + 0.5f);
        pixel.mZ = FloatToU8(mz * v[iz] + 0.5f);
        pixel.mW = 255;

        SetCell(normalMaps, stride, c, pixel);
    }
}
