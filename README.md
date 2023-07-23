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

When $a \ne 0$, there are two solutions to $(ax^2 + bx + c = 0)$ and they are
$$ x = {-b \pm \sqrt{b^2-4ac} \over 2a} $$

The Cauchy-Schwarz Inequality

$$\left( \sum_{k=1}^n a_k b_k \right)^2 \leq \left( \sum_{k=1}^n a_k^2 \right) \left( \sum_{k=1}^n b_k^2 \right)$$
