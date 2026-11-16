# Validation Report: robotdog

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 14 |
| Joints | 13 |
| Assemblies | 1 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [BAKE] [primitive]
  └─ Revolute_2 [revolute]
    HipFrontLeft_BackRight_f [BAKE] [primitive]
      └─ Revolute_14 [revolute]
        robotdog_FemurL_f [BAKE] [primitive]
          └─ Revolute_4 [revolute]
            TibiaR_f [BAKE] [primitive]
  └─ Revolute_8 [revolute]
    HipFrontLeft_BackRight [BAKE] [primitive]
      └─ Revolute_9 [revolute]
        robotdog_FemurR_f [BAKE] [primitive]
          └─ Revolute_10 [revolute]
            robotdog_TibiaL_f [BAKE] [primitive]
  └─ Revolute_11 [revolute]
    robotdog_HipFrontRight_BackLeft [BAKE] [primitive]
      └─ Revolute_12 [revolute]
        robotdog_FemurR_f_2 [BAKE] [primitive]
          └─ Revolute_13 [revolute]
            robotdog_TibiaL_f_2 [BAKE] [primitive]
  └─ Rigid_15 [fixed]
    LowerSpine [primitive]
  └─ Revolute_16 [revolute]
    robotdog_HipFrontRight_BackLeft_2 [BAKE] [primitive]
      └─ Revolute_17 [revolute]
        robotdog_FemurL_f_2 [BAKE] [primitive]
          └─ Revolute_18 [revolute]
            TibiaR [BAKE] [primitive]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `HipFrontLeft_BackRight` | primitive STL | box |
| `HipFrontLeft_BackRight_f` | primitive STL | box |
| `LowerSpine` | primitive STL | box |
| `TibiaR` | primitive STL | box |
| `TibiaR_f` | primitive STL | box |
| `base_link` | primitive STL | box |
| `robotdog_FemurL_f` | primitive STL | box |
| `robotdog_FemurL_f_2` | primitive STL | box |
| `robotdog_FemurR_f` | primitive STL | box |
| `robotdog_FemurR_f_2` | primitive STL | box |
| `robotdog_HipFrontRight_BackLeft` | primitive STL | box |
| `robotdog_HipFrontRight_BackLeft_2` | primitive STL | box |
| `robotdog_TibiaL_f` | primitive STL | box |
| `robotdog_TibiaL_f_2` | primitive STL | box |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `base_link` | (0.0, 0.0, 0.0) |
| `HipFrontLeft_BackRight_f` | (-122.9, 11.3, -136.8) |
| `robotdog_FemurL_f` | (99.7, 120.5, 28.7) |
| `TibiaR_f` | (-152.3, -23.4, 53.2) |
| `HipFrontLeft_BackRight` | (122.9, -11.3, -142.3) |
| `robotdog_FemurR_f` | (127.5, -90.6, 28.7) |
| `robotdog_TibiaL_f` | (-128.1, -85.6, -53.2) |
| `robotdog_HipFrontRight_BackLeft` | (122.9, 11.3, -136.8) |
| `robotdog_FemurR_f_2` | (109.8, 111.4, -24.8) |
| `robotdog_TibiaL_f_2` | (-149.7, 36.6, 49.3) |
| `robotdog_HipFrontRight_BackLeft_2` | (-122.9, -11.3, -142.3) |
| `robotdog_FemurL_f_2` | (127.5, -90.6, -28.7) |
| `TibiaR` | (-1371.6, 865.0, 1293.2) |
