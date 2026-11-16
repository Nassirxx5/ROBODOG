# Transformation Matrices - robotdog

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | HipFrontLeft_BackRight_f |
| $L_{2}$ | HipFrontLeft_BackRight |
| $L_{3}$ | robotdog_HipFrontRight_BackLeft |
| $L_{4}$ | LowerSpine |
| $L_{5}$ | robotdog_HipFrontRight_BackLeft_2 |
| $L_{6}$ | robotdog_FemurL_f |
| $L_{7}$ | robotdog_FemurR_f |
| $L_{8}$ | robotdog_FemurR_f_2 |
| $L_{9}$ | robotdog_FemurL_f_2 |
| $L_{10}$ | TibiaR_f |
| $L_{11}$ | robotdog_TibiaL_f |
| $L_{12}$ | robotdog_TibiaL_f_2 |
| $L_{13}$ | TibiaR |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | Revolute_2 | revolute (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | Revolute_8 | revolute (rad) | $L_{0}$ | $L_{2}$ |
| $q_{3}$ | Revolute_11 | revolute (rad) | $L_{0}$ | $L_{3}$ |
| $q_{4}$ | Revolute_16 | revolute (rad) | $L_{0}$ | $L_{5}$ |
| $q_{5}$ | Revolute_14 | revolute (rad) | $L_{1}$ | $L_{6}$ |
| $q_{6}$ | Revolute_9 | revolute (rad) | $L_{2}$ | $L_{7}$ |
| $q_{7}$ | Revolute_12 | revolute (rad) | $L_{3}$ | $L_{8}$ |
| $q_{8}$ | Revolute_17 | revolute (rad) | $L_{5}$ | $L_{9}$ |
| $q_{9}$ | Revolute_4 | revolute (rad) | $L_{6}$ | $L_{10}$ |
| $q_{10}$ | Revolute_10 | revolute (rad) | $L_{7}$ | $L_{11}$ |
| $q_{11}$ | Revolute_13 | revolute (rad) | $L_{8}$ | $L_{12}$ |
| $q_{12}$ | Revolute_18 | revolute (rad) | $L_{9}$ | $L_{13}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  |-- [revolute] Revolute_2 (q1)
  |   L1: HipFrontLeft_BackRight_f
  |     +-- [revolute] Revolute_14 (q5)
  |         L6: robotdog_FemurL_f
  |           +-- [revolute] Revolute_4 (q9)
  |               L10: TibiaR_f
  |-- [revolute] Revolute_8 (q2)
  |   L2: HipFrontLeft_BackRight
  |     +-- [revolute] Revolute_9 (q6)
  |         L7: robotdog_FemurR_f
  |           +-- [revolute] Revolute_10 (q10)
  |               L11: robotdog_TibiaL_f
  |-- [revolute] Revolute_11 (q3)
  |   L3: robotdog_HipFrontRight_BackLeft
  |     +-- [revolute] Revolute_12 (q7)
  |         L8: robotdog_FemurR_f_2
  |           +-- [revolute] Revolute_13 (q11)
  |               L12: robotdog_TibiaL_f_2
  |-- [fixed] Rigid_15
  |   L4: LowerSpine
  +-- [revolute] Revolute_16 (q4)
      L5: robotdog_HipFrontRight_BackLeft_2
        +-- [revolute] Revolute_17 (q8)
            L9: robotdog_FemurL_f_2
              +-- [revolute] Revolute_18 (q12)
                  L13: TibiaR
```

## Transforms

## Revolute_2

$L_{0}$ **base_link** -> $L_{1}$ **HipFrontLeft_BackRight_f** (revolute)
  Variable: $q_{1}$

- **origin xyz**: (0.284682, 0.019937, 0.029453) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.436332, 0.261799] rad ([-25deg, 15deg])

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.284682 \\
1 & 0 & 0 & 0.019937 \\
0 & -1 & 0 & 0.029453 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_8

$L_{0}$ **base_link** -> $L_{2}$ **HipFrontLeft_BackRight** (revolute)
  Variable: $q_{2}$

- **origin xyz**: (-0.0396, 0.04928, 0.029568) m
- **origin rpy**: (1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.349066, 0.261799] rad ([-20deg, 15deg])

### Local Transform

$T^{0}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & -0.0396 \\
1 & 0 & 0 & 0.04928 \\
0 & 1 & 0 & 0.029568 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_11

$L_{0}$ **base_link** -> $L_{3}$ **robotdog_HipFrontRight_BackLeft** (revolute)
  Variable: $q_{3}$

- **origin xyz**: (0.284682, 0.049603, 0.029453) m
- **origin rpy**: (-1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.349066, 0.261799] rad ([-20deg, 15deg])

### Local Transform

$T^{0}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.284682 \\
1 & 0 & 0 & 0.049603 \\
0 & -1 & 0 & 0.029453 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Rigid_15

$L_{0}$ **base_link** -> $L_{4}$ **LowerSpine** (fixed)

- **origin xyz**: (0.243584, -0.006273, 0.062907) m
- **origin rpy**: (3.141593, 0, -3.141593) rad

### Local Transform

$$
T^{0}_{4} = \begin{bmatrix}
-1 & 0 & 0 & 0.243584 \\
0 & 1 & 0 & -0.006273 \\
0 & 0 & -1 & 0.062907 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_16

$L_{0}$ **base_link** -> $L_{5}$ **robotdog_HipFrontRight_BackLeft_2** (revolute)
  Variable: $q_{4}$

- **origin xyz**: (-0.0396, 0.019614, 0.029568) m
- **origin rpy**: (1.570796, 0, 1.570796) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.523599, 0.349066] rad ([-30deg, 20deg])

### Local Transform

$T^{0}_{5}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & -0.0396 \\
1 & 0 & 0 & 0.019614 \\
0 & 1 & 0 & 0.029568 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_14

$L_{1}$ **HipFrontLeft_BackRight_f** -> $L_{6}$ **robotdog_FemurL_f** (revolute)
  Variable: $q_{5}$

- **origin xyz**: (-0.103465, -0.035482, -0.020984) m
- **origin rpy**: (3.141593, 1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.308997, 1.047198] rad ([-75deg, 60deg])

### Local Transform

$T^{1}_{6}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.103465 \\
0 & -1 & 0 & -0.035482 \\
-1 & 0 & 0 & -0.020984 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_9

$L_{2}$ **HipFrontLeft_BackRight** -> $L_{7}$ **robotdog_FemurR_f** (revolute)
  Variable: $q_{6}$

- **origin xyz**: (0.103465, 0.035482, -0.026417) m
- **origin rpy**: (-3.141593, -1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0.523599, 2.879793] rad ([30deg, 165deg])

### Local Transform

$T^{2}_{7}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.103465 \\
0 & -1 & 0 & 0.035482 \\
1 & 0 & 0 & -0.026417 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_12

$L_{3}$ **robotdog_HipFrontRight_BackLeft** -> $L_{8}$ **robotdog_FemurR_f_2** (revolute)
  Variable: $q_{7}$

- **origin xyz**: (0.103465, -0.035482, -0.020984) m
- **origin rpy**: (3.141593, 1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-2.70526, -0.436332] rad ([-155deg, -25deg])

### Local Transform

$T^{3}_{8}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & 0.103465 \\
0 & -1 & 0 & -0.035482 \\
-1 & 0 & 0 & -0.020984 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_17

$L_{5}$ **robotdog_HipFrontRight_BackLeft_2** -> $L_{9}$ **robotdog_FemurL_f_2** (revolute)
  Variable: $q_{8}$

- **origin xyz**: (-0.104124, 0.035482, -0.026417) m
- **origin rpy**: (-3.141593, -1.570796, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [0, 4.45059] rad ([0deg, 255deg])

### Local Transform

$T^{5}_{9}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & -0.104124 \\
0 & -1 & 0 & 0.035482 \\
1 & 0 & 0 & -0.026417 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_4

$L_{6}$ **robotdog_FemurL_f** -> $L_{10}$ **TibiaR_f** (revolute)
  Variable: $q_{9}$

- **origin xyz**: (0.071457, 0.071484, -0.00029) m
- **origin rpy**: (-3.141593, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.610865, 1.658063] rad ([-35deg, 95deg])

### Local Transform

$T^{6}_{10}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.071457 \\
0 & -1 & 0 & 0.071484 \\
0 & 0 & -1 & -0.00029 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & -s_{9} & 0 & 0 \\
s_{9} & c_{9} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_10

$L_{7}$ **robotdog_FemurR_f** -> $L_{11}$ **robotdog_TibiaL_f** (revolute)
  Variable: $q_{10}$

- **origin xyz**: (0.087523, -0.050554, -0.00029) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-2.181662, 0] rad ([-125deg, 0deg])

### Local Transform

$$
T^{7}_{11}(q_{10}) = \begin{bmatrix}
c_{10} & -s_{10} & 0 & 0.087523 \\
s_{10} & c_{10} & 0 & -0.050554 \\
0 & 0 & 1 & -0.00029 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_13

$L_{8}$ **robotdog_FemurR_f_2** -> $L_{12}$ **robotdog_TibiaL_f_2** (revolute)
  Variable: $q_{11}$

- **origin xyz**: (0.077415, 0.064984, 0.003065) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-1.658063, 0.610865] rad ([-95deg, 35deg])

### Local Transform

$$
T^{8}_{12}(q_{11}) = \begin{bmatrix}
c_{11} & -s_{11} & 0 & 0.077415 \\
s_{11} & c_{11} & 0 & 0.064984 \\
0 & 0 & 1 & 0.003065 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Revolute_18

$L_{9}$ **robotdog_FemurL_f_2** -> $L_{13}$ **TibiaR** (revolute)
  Variable: $q_{12}$

- **origin xyz**: (0.087523, -0.050554, 0.00329) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.087266, 2.094395] rad ([-5deg, 120deg])

### Local Transform

$$
T^{9}_{13}(q_{12}) = \begin{bmatrix}
c_{12} & -s_{12} & 0 & 0.087523 \\
s_{12} & c_{12} & 0 & -0.050554 \\
0 & 0 & 1 & 0.00329 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{6}(q_{5})\quad (L_0 \to L_{6}: \text{robotdog_FemurL_f})$$

$$T^{0}_{7} = T^{0}_{2}(q_{2}) \cdot T^{2}_{7}(q_{6})\quad (L_0 \to L_{7}: \text{robotdog_FemurR_f})$$

$$T^{0}_{8} = T^{0}_{3}(q_{3}) \cdot T^{3}_{8}(q_{7})\quad (L_0 \to L_{8}: \text{robotdog_FemurR_f_2})$$

$$T^{0}_{9} = T^{0}_{5}(q_{4}) \cdot T^{5}_{9}(q_{8})\quad (L_0 \to L_{9}: \text{robotdog_FemurL_f_2})$$

$$T^{0}_{10} = T^{0}_{1}(q_{1}) \cdot T^{1}_{6}(q_{5}) \cdot T^{6}_{10}(q_{9})\quad (L_0 \to L_{10}: \text{TibiaR_f})$$

$$T^{0}_{11} = T^{0}_{2}(q_{2}) \cdot T^{2}_{7}(q_{6}) \cdot T^{7}_{11}(q_{10})\quad (L_0 \to L_{11}: \text{robotdog_TibiaL_f})$$

$$T^{0}_{12} = T^{0}_{3}(q_{3}) \cdot T^{3}_{8}(q_{7}) \cdot T^{8}_{12}(q_{11})\quad (L_0 \to L_{12}: \text{robotdog_TibiaL_f_2})$$

$$T^{0}_{13} = T^{0}_{5}(q_{4}) \cdot T^{5}_{9}(q_{8}) \cdot T^{9}_{13}(q_{12})\quad (L_0 \to L_{13}: \text{TibiaR})$$

