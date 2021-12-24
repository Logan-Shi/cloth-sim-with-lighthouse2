# cloth-sim-with-lighthouse2

We built a real-time cloth simulation with Lighthouse 2 framework for real-time ray tracing. 

![Demo Gif](/media/cloth-with-wind3.gif)

Cloth physics are solved using verlet integration which is also accelerated with CUDA. 

## Features

- Natural wind effect with perlin randoms. 

- Simple cloth self-collision. 

- Real-time obstacles collision with paralleled BVH.

## Reference code

- [lighthouse2](https://github.com/jbikker/lighthouse2)

- [cloth sim](https://github.com/Logan-Shi/3D-cloth-simulator-verlet-with-GPU)