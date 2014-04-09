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

            nmap[iu].mX = FloatToU8(0.5f * v[2] + 0.5f);
            nmap[iu].mY = FloatToU8(0.5f * v[1] + 0.5f);
            nmap[iu].mZ = FloatToU8(0.5f * v[0] + 0.5f);
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

            nmap[iu].mX = FloatToU8(0.5f * v[0] + 0.5f);
            nmap[iu].mY = FloatToU8(0.5f * v[1] + 0.5f);
            nmap[iu].mZ = FloatToU8(0.5f * v[2] + 0.5f);
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
    int startU = FloorToI32(minC[0] * size);
    int startV = FloorToI32(minC[1] * size);
    int endU   = FloorToI32(maxC[0] * size);
    int endV   = FloorToI32(maxC[1] * size);

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

    Mat3f mf = GetFaceTransform(face);

    float mx = signMap[0] * 0.5f;
    float my = signMap[1] * 0.5f;
    float mz = signMap[2] * 0.5f;
    int ix = elemMap[0];
    int iy = elemMap[1];
    int iz = elemMap[2];

    float rscale = hscale * float(1.0f / 65535.0f);
    float dscale = rscale * (size / 2.0f);         // inter-two-pixel space is 2/w

    int stride = size;

    // Okay, do face interior: nice and easy, all cells are on the same face.
    const uint16_t* rowPixelsHMap = heightMaps[face] + (stride * startV);
    cPixel4U8* rowPixelsNMap = normalMaps[face] + (stride * startV);
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



// -----------------------------------------------------------------------------
// Cubemap Util Test code
// -----------------------------------------------------------------------------

#ifdef TEST_CUBEMAP_UTILS

#define TEST_MAIN
#include <stdio.h>

namespace
{
    void WrapCubeFace(int size, int face, int x, int v, int* faceOut, int* xOut, int* yOut)
    {    
        *faceOut = face;
        *xOut = x;
        *yOut = v;
        
        CML::WrapCubeFace(size, faceOut, xOut, yOut);
    }
}

bool TestWrapFaceDir()
{
    // iterate in a given direction, wrapping right around the cube, and checking
    // that we wound up where we started.
    int size = 4;
    
    for (int startFace = 0; startFace < 6; startFace++)
    {
        int face = startFace;

        printf("\n=== test face %d\n", startFace);
        
        for (int startDir = 0; startDir < 4; startDir++)
        {
            int x = 2;
            int y = 2;
            int dx;
            int dy;
            switch (startDir)
            {
            case 0: dx = 1; dy = 0; break;
            case 1: dx = -1; dy = 0; break;
            case 2: dx = 0; dy = 1; break;
            case 3: dx = 0; dy = -1; break;
            }
    
            printf("f:x:y:dx:dy = %d:%d:%d:%d:%d", face, x, y, dx, dy);
            
            for (int i = 0; i < 16; i++)
            {
                x += dx;
                y += dy;

                //if (true) // do always to check we handle the non-wrap case
                if (NeedsWrap(size, x, y))
                    WrapCubeFace(size, &face, &x, &y, &dx, &dy);

                printf(" > %d:%d:%d:%d:%d", face, x, y, dx, dy);
            }
            
            CL_ASSERT(x == 2 && y == 2 && face == startFace);
            printf("\n\n");
        }
    }


    {
        printf("\ndiag test +1:+1\n");
        int startFace = 0;
        int face = startFace;
        int x = 2;
        int y = 2;
        int dx = 1;
        int dy = 1;

        printf("f:x:y:dx:dy = %d:%d:%d:%d:%d", face, x, y, dx, dy);
        
        do
        {
            x += dx;
            y += dy;

            WrapCubeFace(size, &face, &x, &y, &dx, &dy);
            printf(" > %d:%d:%d:%d:%d", face, x, y, dx, dy);
        }
        while (x != 2 || y != 2 || face != startFace);
        
        printf("\n\n");
    }

    {
        printf("\ndiag test -1:-1\n");
        int startFace = 0;
        int face = startFace;
        int x = 2;
        int y = 2;
        int dx = -1;
        int dy = -1;

        printf("f:x:y:dx:dy = %d:%d:%d:%d:%d", face, x, y, dx, dy);
        do
        {
            x += dx;
            y += dy;

            WrapCubeFace(size, &face, &x, &y, &dx, &dy);
            printf(" > %d:%d:%d:%d:%d", face, x, y, dx, dy);
        }
        while (x != 2 || y != 2 || face != startFace);
        
        printf("\n");
    }
        
    {
        printf("diag test +1:-1\n");
        int startFace = 0;
        int face = startFace;
        int x = 1;
        int y = 2;
        int dx = 1;
        int dy = -1;

        printf("f:x:y:dx:dy = %d:%d:%d:%d:%d", face, x, y, dx, dy);
        
        do
        {
            x += dx;
            y += dy;

            WrapCubeFace(size, &face, &x, &y, &dx, &dy);
            printf(" > %d:%d:%d:%d:%d", face, x, y, dx, dy);
        }
        while (x != 1 || y != 2 || face != startFace);
        
        printf("\n");
    }

    {
        printf("\ndiag test -1:+1\n");
        int startFace = 0;
        int face = startFace;
        int x = 1;
        int y = 2;
        int dx = -1;
        int dy = 1;

        printf("f:x:y:dx:dy = %d:%d:%d:%d:%d", face, x, y, dx, dy);
        
        do
        {
            x += dx;
            y += dy;

            WrapCubeFace(size, &face, &x, &y, &dx, &dy);
            printf(" > %d:%d:%d:%d:%d", face, x, y, dx, dy);
        }
        while (x != 1 || y != 2 || face != startFace);
        
        printf("\n");
    }
        
    return true;
}

void TestCubeMap()
{
    int face;
    int x;
    int y;
    
    // we're testing traversals on a 3x3x3 cube.
    
    WrapCubeFace(3, kFace_NY, -1,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_PZ && x == 1 && y == 0);
    
    face = kFace_PZ; // start in middle of +Z
    x = 1;
    y = 1;

    x++;    
    WrapCubeFace(3, &face, &x, &y); // to z/x edge
    x++;    
    WrapCubeFace(3, &face, &x, &y); // to x/z edge
    x++;    
    WrapCubeFace(3, &face, &x, &y); // to x/y/z corner
    x++;    
    WrapCubeFace(3, &face, &x, &y); // to y/z/x corner
    x++;    
    WrapCubeFace(3, &face, &x, &y); // to z/y/x corner
    CL_ASSERT(face == kFace_PZ && x == 2 && y == 2);

    printf("done corner loop!\n");
    face = kFace_PZ; // start in middle of +Z
    x = 1;
    y = 1;
    
    // Loop the cube: +Z -> +Y -> -Z -> -Y -> +Z
    y++;    
    WrapCubeFace(3, &face, &x, &y);
    y++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PY);
    
    x--;    
    WrapCubeFace(3, &face, &x, &y);
    x--;    
    WrapCubeFace(3, &face, &x, &y);
    x--;
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_NZ);
    
    y--;    
    WrapCubeFace(3, &face, &x, &y);
    y--;    
    WrapCubeFace(3, &face, &x, &y);
    y--;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_NY);

    x--;    
    WrapCubeFace(3, &face, &x, &y);
    x--;    
    WrapCubeFace(3, &face, &x, &y);
    x--;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PZ);
    y++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PZ && x == 1 && y == 1);
    printf("done Z/Y loop!\n");

    // Loop the cube: +Z -> +X -> -Z -> -X -> +Z
    x++;    
    WrapCubeFace(3, &face, &x, &y);
    x++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PX);
    
    y--;
    WrapCubeFace(3, &face, &x, &y);
    y--;    
    WrapCubeFace(3, &face, &x, &y);
    y--;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_NZ);
    
    x++;    
    WrapCubeFace(3, &face, &x, &y);
    x++;    
    WrapCubeFace(3, &face, &x, &y);
    x++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_NX);

    y++;    
    WrapCubeFace(3, &face, &x, &y);
    y++;    
    WrapCubeFace(3, &face, &x, &y);
    y++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PZ);
    x++;    
    WrapCubeFace(3, &face, &x, &y);

    CL_ASSERT(face == kFace_PZ && x == 1 && y == 1);

    printf("done z/x loop!\n");

    // explicitly test all edge transitions
    WrapCubeFace(3, kFace_PZ, -1,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PZ,  3,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PZ,  1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NY && x == 0 && y == 1);
    WrapCubeFace(3, kFace_PZ,  1,  3, &face, &x, &y);
    CL_ASSERT(face == kFace_PY && x == 2 && y == 1);

    WrapCubeFace(3, kFace_NZ, -1,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NZ,  3,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NZ,  1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NY && x == 2 && y == 1);
    WrapCubeFace(3, kFace_NZ,  1,  3, &face, &x, &y);
    CL_ASSERT(face == kFace_PY && x == 0 && y == 1);

    // +Y
    WrapCubeFace(3, kFace_PY, -1,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_NZ && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PY,  3,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_PZ && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PY,  1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 0 && y == 1);
    WrapCubeFace(3, kFace_PY,  1,  3, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 2 && y == 1);

    // -Y
    WrapCubeFace(3, kFace_NY, -1,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_PZ && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NY,  3,  1, &face, &x, &y);
    CL_ASSERT(face == kFace_NZ && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NY,  1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 2 && y == 1);
    WrapCubeFace(3, kFace_NY,  1,  3, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 0 && y == 1);



    ////////////////////////////////////////////////////////////////////////////////
    // Test DirectionToCoord/CoordToDirection
    cCubeMapCoord c;
    Vec3f p;
    
    p = norm(Vec3f(0.5, 0.5, 1));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
    
    p = norm(Vec3f(1, -0.5, 0.5));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
    
    p = norm(Vec3f(0.5, 1, 0.5));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
    
    p = norm(Vec3f(0.5, 0.5, -1));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
    
    p = norm(Vec3f(-1, -0.5, 0.5));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
    
    p = norm(Vec3f(0.5, -1, 0.5));
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    p = CoordToDirection(c);
    printf("(%g %g, %g)\n", p[0], p[1], p[2]);    
    c = DirectionToCoord(p);
    printf("%d: (%g, %g)\n", c.mFace, c.mU, c.mV);
    printf("\n");
}

void TestDirToIndex()
{
    for(int face = 0; face < 6; ++face)
    {
        for (int row = 0; row < 32; ++row)
        {
            for (int col = 0; col < 32; ++col)
            {
                cCubeMapIndex index(face, col, row);
                
                Vec3f dir = IndexToDirection(32, index);
                cCubeMapIndex indexFromDir = DirectionToIndex(32, dir);
                CL_ASSERT(index.mCol == indexFromDir.mCol);
                CL_ASSERT(index.mRow == indexFromDir.mRow);
                CL_ASSERT(index.mFace == indexFromDir.mFace);

                cCubeMapIndex index0(face, col-1, row);
                cCubeMapIndex index1(face, col, row-1);
                cCubeMapIndex index2(face, col+1, row);
                cCubeMapIndex index3(face, col, row+1);

                Vec3f dir0 = IndexToDirection(32, index0);
                Vec3f dir1 = IndexToDirection(32, index1);
                Vec3f dir2 = IndexToDirection(32, index2);
                Vec3f dir3 = IndexToDirection(32, index3);

                cCubeMapIndex indexS0 = index0;
                cCubeMapIndex indexS1 = index1;
                cCubeMapIndex indexS2 = index2;
                cCubeMapIndex indexS3 = index3;

                WrapCubeFace(32, &indexS0);
                WrapCubeFace(32, &indexS1);
                WrapCubeFace(32, &indexS2);
                WrapCubeFace(32, &indexS3);

                Vec3f dirS0 = IndexToDirection(32, indexS0);
                Vec3f dirS1 = IndexToDirection(32, indexS1);
                Vec3f dirS2 = IndexToDirection(32, indexS2);
                Vec3f dirS3 = IndexToDirection(32, indexS3);

                float distS0 = len(dirS0 - dir0);
                float distS1 = len(dirS1 - dir1);
                float distS2 = len(dirS2 - dir2);
                float distS3 = len(dirS3 - dir3);

                if (distS0 > .05f * 1.5f ||
                    distS1 > .05f * 1.5f ||
                    distS2 > .05f * 1.5f ||
                    distS3 > .05f * 1.5f )
                {
                    int i = 0;
                    ++i;
                }
            }
        }
    }
}

#endif

//------------------------------------------------------------------------------
// Spherical cubemap normal map tests
//------------------------------------------------------------------------------


#ifdef TEST_CUBEMAP_NMAP

#define TEST_MAIN
#include "targa.h"

void CreateTestHMap(int size, uint16_t dst[])
{
    int w = size;
    int h = size;

    float s = vl_pi * (2.0f / size);

    for (int y = 0; y < h; y++)
    {
        uint16_t* dstSpan = dst + y * w;
        
        for (int x = 0; x < w; x++)
            dstSpan[x] = FloorToI32(65535.0f * (sqr(sinf(s * x)) * sqr(sinf(s * y))));
    }
}

void CreateTestHMap(int size, uint8_t dst[])
{
    int w = size;
    int h = size;

    float s = vl_pi * (2.0f / size);
    
    for (int y = 0; y < h; y++)
    {
        uint8_t* dstSpan = dst + y * w;
        
        for (int x = 0; x < w; x++)
            dstSpan[x] = FloorToI32(255.0f * (sqr(sinf(s * x)) * sqr(sinf(s * y))));
    }
}

void InitTGA4U8(tga_image* tga, cPixel4U8* data, int size)
{
    init_tga_image(tga, (uint8_t*) data, size, size, 32);
    tga->image_type = TGA_IMAGE_TYPE_BGR;
    tga->image_descriptor &= ~TGA_T_TO_B_BIT;
}

void TestSphereMaps()
{
    const float kR = 10.0f;     ///< Radius of sphere's zero height.
    const float kH = 1.0f;      ///< Scale of heightmap.

    const int kSize = 256;
    uint8_t hmap[kSize * kSize];

    CreateTestHMap(kSize, hmap);

    tga_image tga;
    init_tga_image(&tga, (uint8_t*) hmap, kSize, kSize, 8);
    tga.image_type = TGA_IMAGE_TYPE_MONO;
    tga.image_descriptor &= ~TGA_T_TO_B_BIT;
    tga_write("sphere-hmap.tga", &tga);

    uint16_t hmap16[kSize * kSize];

    CreateTestHMap(kSize, hmap16);

    cPixel4U8 nmap[kSize * kSize];

    CreateNormalMap(kSize, kH / kR, hmap16, nmap);

    InitTGA4U8(&tga, nmap, kSize);
    tga_write("flat-nmap.tga", &tga);

    CreateSphereNormalMap(kSize, kR, kH, hmap16, nmap);

    InitTGA4U8(&tga, nmap, kSize);
    tga_write("sphere-nmap.tga", &tga);

    // Test full-on face/edge/corner handling version
    cPixel4U8 nmapsData[6][kSize * kSize];
    memset(nmapsData, 0x80, sizeof(nmapsData));

    const uint16_t*  hmaps[6] = { hmap16, hmap16, hmap16, hmap16, hmap16, hmap16 };
    cPixel4U8*       nmaps[6] = { nmapsData[0], nmapsData[1], nmapsData[2], nmapsData[3], nmapsData[4], nmapsData[5] };

    // Update entire +X face. Note the generated normals are in world space, so the resulting map is mostly
    // reddish.
    UpdateFaceNormalMap(kR, kH, kSize, hmaps, nmaps, kFace_PX);

    // Update just a patch of the upper (+Z) face.
    UpdateFaceNormalMap(kR, kH, kSize, hmaps, nmaps, kFace_PZ, Vec2f(0.3f), Vec2f(0.7f));

    // Update a splat of the size of face positioned off-centre on the -X face. This wraps around
    // to the adjacent -Z and +Y faces.
    cFaceRect splats[5];
    int splatCount = SplatArea(kSize, cCubeMapIndex(kFace_NX, kSize / 4, kSize / 4), kSize / 2, splats);

    for (int i = 0; i < splatCount; i++)
    {
        Vec2f cmin = Vec2f(splats[i].mX0, splats[i].mY0) / kSize;
        Vec2f cmax = Vec2f(splats[i].mX1, splats[i].mY1) / kSize;
        UpdateFaceNormalMap(kR, kH, kSize, hmaps, nmaps, splats[i].mFace, cmin, cmax);
    }


    InitTGA4U8(&tga, nmapsData[0], kSize);
    tga_write("sphere-nmap-0.tga", &tga);
    InitTGA4U8(&tga, nmapsData[1], kSize);
    tga_write("sphere-nmap-1.tga", &tga);
    InitTGA4U8(&tga, nmapsData[2], kSize);
    tga_write("sphere-nmap-2.tga", &tga);
    InitTGA4U8(&tga, nmapsData[3], kSize);
    tga_write("sphere-nmap-3.tga", &tga);
    InitTGA4U8(&tga, nmapsData[4], kSize);
    tga_write("sphere-nmap-4.tga", &tga);
    InitTGA4U8(&tga, nmapsData[5], kSize);
    tga_write("sphere-nmap-5.tga", &tga);
}

#endif


//------------------------------------------------------------------------------
// Tests main
//------------------------------------------------------------------------------


#ifdef TEST_MAIN
int main(int argc, const char* argv[])
{
    CL_ASSERT(0);
#ifdef TEST_CUBEMAP_NMAP
    TestSphereMaps();
#endif
#ifdef TEST_CUBEMAP_UTILS
    TestWrapFaceDir();
    TestCubeMap();
    TestDirToIndex();
#endif
}
#endif
