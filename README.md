cube-map-lib
============

A library of utilities for dealing with cube maps, and particularly spherically-projected
cube maps, where each face of the cube map stores a height field.

To run a series of tests on the library, use:

    clang++ -DTEST_CUBEMAP_UTILS CubeMapLib.cpp -o cml-test && ./cml-test

To exercise the normal map generation and partial update utilities, run:

    clang++ -DTEST_CUBEMAP_NMAP CubeMapLib.cpp targa.c -o cml-nmap && ./cml-nmap
    
This outputs a test "flat" height map as sphere-[nh]map.tga, the corresponding 
projected-to-sphere-face normal map as sphere-nmap.tga, and a full cube map holding
the results of several splat operations in sphere-nmap-[0-5].tga.
