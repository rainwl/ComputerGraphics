[TOC]

# CG-GMP-PBCA
`Computer Graphics`
`Geometry Modeling And Processing`
`Physics-based Computer Animation`

## Contents

### Rigid Body Dynamics Solver based on Impulse
`Physics-Based Computer Animation/Assets/Scenes/RigidBodyDynamics.unity`

[https://github.com/rainwl/CG-GMP-PBCA/issues/1#issue-1812139717](https://user-images.githubusercontent.com/51992995/254614281-93e7a466-45c4-4569-a100-1eb618431330.mp4)

![](https://pic4rain.oss-cn-beijing.aliyuncs.com/img/RBD_algorithm.png)


### Implicit Integration Cloth Solver

$v^{[1]} = v^{[0]} +\Delta  t \mathbf{M} ^{-1}f^{[1]}$

### Finite Element Method
`St.Venant-Kirchhoff(StVk)` `explicit time integration` `tetrahedral`

First simulate the object as a simple particle system,each vertex has its own
x and v,and the v is under the influence of gravity.

Then we calculate the edge matrix of a tetrahedron.And based on the FEM,
calculate the `deformation gradient` F and the `Green Strain`G,and the 
`Second Piola-Kirchhoff stress`S,and finally the forces of 4 vertices.

Finally,implement the `Laplacian smoothing` over the vertex velocities.

$x = FX+c$

$F = \partial x/\partial X$ ,known as `deformation gradient`