// -----------------------------------------------------------------------------
// CubeMapLib Test code
// -----------------------------------------------------------------------------

#define  _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include "CubeMapLib.h"

#include <stdio.h>
#include <string.h>

#include "stb_image_mini.h"

using namespace CML;

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
        } while (x != 2 || y != 2 || face != startFace);

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
        } while (x != 2 || y != 2 || face != startFace);

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
        } while (x != 1 || y != 2 || face != startFace);

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
        } while (x != 1 || y != 2 || face != startFace);

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

    WrapCubeFace(3, kFace_NY, -1, 1, &face, &x, &y);
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
    WrapCubeFace(3, kFace_PZ, -1, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PZ, 3, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PZ, 1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NY && x == 0 && y == 1);
    WrapCubeFace(3, kFace_PZ, 1, 3, &face, &x, &y);
    CL_ASSERT(face == kFace_PY && x == 2 && y == 1);

    WrapCubeFace(3, kFace_NZ, -1, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NZ, 3, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NZ, 1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NY && x == 2 && y == 1);
    WrapCubeFace(3, kFace_NZ, 1, 3, &face, &x, &y);
    CL_ASSERT(face == kFace_PY && x == 0 && y == 1);

    // +Y
    WrapCubeFace(3, kFace_PY, -1, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_NZ && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PY, 3, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_PZ && x == 1 && y == 2);
    WrapCubeFace(3, kFace_PY, 1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 0 && y == 1);
    WrapCubeFace(3, kFace_PY, 1, 3, &face, &x, &y);
    CL_ASSERT(face == kFace_PX && x == 2 && y == 1);

    // -Y
    WrapCubeFace(3, kFace_NY, -1, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_PZ && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NY, 3, 1, &face, &x, &y);
    CL_ASSERT(face == kFace_NZ && x == 1 && y == 0);
    WrapCubeFace(3, kFace_NY, 1, -1, &face, &x, &y);
    CL_ASSERT(face == kFace_NX && x == 2 && y == 1);
    WrapCubeFace(3, kFace_NY, 1, 3, &face, &x, &y);
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
    for (int face = 0; face < 6; ++face)
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

                cCubeMapIndex index0(face, col - 1, row);
                cCubeMapIndex index1(face, col, row - 1);
                cCubeMapIndex index2(face, col + 1, row);
                cCubeMapIndex index3(face, col, row + 1);

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
                    distS3 > .05f * 1.5f)
                {
                    int i = 0;
                    ++i;
                }
            }
        }
    }
}

//------------------------------------------------------------------------------
// Spherical cubemap normal map tests
//------------------------------------------------------------------------------

void CreateTestHMap(int size, uint16_t dst[])
{
    int w = size;
    int h = size;

    float s = vl_pi * (2.0f / size);

    for (int y = 0; y < h; y++)
    {
        uint16_t* dstSpan = dst + y * w;

        for (int x = 0; x < w; x++)
            dstSpan[x] = uint16_t(floorf(65535.0f * (sqr(sinf(s * x)) * sqr(sinf(s * y)))));
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
            dstSpan[x] = uint8_t(floorf(255.0f * (sqr(sinf(s * x)) * sqr(sinf(s * y)))));
    }
}

void TestSphereMaps()
{
    const float kR = 10.0f;     ///< Radius of sphere's zero height.
    const float kH = 1.0f;      ///< Scale of heightmap.

    const int kSize = 256;
    
    {
        uint8_t hmap[kSize * kSize];

        CreateTestHMap(kSize, hmap);

        stbi_write_png("sphere-hmap.png", kSize, kSize, 1, hmap, 0);
    }

    {
        uint16_t hmap16[kSize * kSize];

        CreateTestHMap(kSize, hmap16);

        cPixel4U8 nmap[kSize * kSize];

        CreateNormalMap(kSize, kH / kR, hmap16, nmap);

        stbi_write_png("flat-nmap.png", kSize, kSize, 4, nmap, 0);

        CreateSphereNormalMap(kSize, kR, kH, hmap16, nmap);

        stbi_write_png("sphere-nmap.png", kSize, kSize, 4, nmap, 0);
    }

    // Test full-on face/edge/corner handling version
    {
        uint16_t hmap16[kSize * kSize];

        CreateTestHMap(kSize, hmap16);

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

        stbi_write_png("sphere-nmap-0.png", kSize, kSize, 4, nmapsData[0], 0);
        stbi_write_png("sphere-nmap-1.png", kSize, kSize, 4, nmapsData[1], 0);
        stbi_write_png("sphere-nmap-2.png", kSize, kSize, 4, nmapsData[2], 0);
        stbi_write_png("sphere-nmap-3.png", kSize, kSize, 4, nmapsData[3], 0);
        stbi_write_png("sphere-nmap-4.png", kSize, kSize, 4, nmapsData[4], 0);
        stbi_write_png("sphere-nmap-5.png", kSize, kSize, 4, nmapsData[5], 0);
    }
}


//------------------------------------------------------------------------------
// Tests main
//------------------------------------------------------------------------------


int main(int argc, const char* argv[])
{
    TestWrapFaceDir();
    TestCubeMap();
    TestDirToIndex();
    TestSphereMaps();
}
