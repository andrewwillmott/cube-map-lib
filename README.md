cube-map-lib
============

A library of utilities for dealing with cube maps, and particularly spherically-
projected cube maps, where each face of the cube map stores a height field.

To run a series of tests on the library, use:

    c++ CubeMapLib.cpp CubeMapTest.cpp -o cml-test && ./cml-test

Or add those files to your favourite IDE.

In addition to running some standard test, this outputs a "flat" height map as 
flat-[nh]map.tga, the corresponding projected-to-sphere-face normal map as 
sphere-nmap.tga, and a full cube map holding the results of several splat 
operations in sphere-nmap-[0-5].tga.
