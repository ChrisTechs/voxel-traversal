// MIT License
//
// Copyright (c) 2019 Francis Engelmann
// Ported and Expanded by Christian Steenkamp
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#![cfg_attr(not(feature = "std"), no_std)]

use core::mem;
use num_traits::Float;

pub trait IntoPoint<T> {
    fn into_point(self) -> (T, T, T);
}

impl<T> IntoPoint<T> for (T, T, T) {
    fn into_point(self) -> (T, T, T) {
        self
    }
}

impl<T: Copy> IntoPoint<T> for [T; 3] {
    fn into_point(self) -> (T, T, T) {
        (self[0], self[1], self[2])
    }
}

#[cfg(feature = "glam")]
impl IntoPoint<f32> for glam::Vec3 {
    fn into_point(self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }
}

#[cfg(feature = "glam")]
impl IntoPoint<f64> for glam::DVec3 {
    fn into_point(self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

#[cfg(feature = "nalgebra")]
impl IntoPoint<f32> for nalgebra::Point3<f32> {
    fn into_point(self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }
}

#[cfg(feature = "nalgebra")]
impl IntoPoint<f32> for nalgebra::Vector3<f32> {
    fn into_point(self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }
}

#[cfg(feature = "nalgebra")]
impl IntoPoint<f64> for nalgebra::Point3<f64> {
    fn into_point(self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

#[cfg(feature = "nalgebra")]
impl IntoPoint<f64> for nalgebra::Vector3<f64> {
    fn into_point(self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}

pub struct VoxelRaycast {
    current_voxel: (i32, i32, i32),
    last_voxel: (i32, i32, i32),
    step: (i32, i32, i32),
    t_max: (f32, f32, f32),
    t_delta: (f32, f32, f32),
    just_started: bool,
    is_finished: bool,
}

impl VoxelRaycast {
    pub fn new(start: impl IntoPoint<f32>, end: impl IntoPoint<f32>) -> Self {
        let start = start.into_point();
        let end = end.into_point();

        let current_voxel = (
            Float::floor(start.0) as i32,
            Float::floor(start.1) as i32,
            Float::floor(start.2) as i32,
        );

        let last_voxel = (
            Float::floor(end.0) as i32,
            Float::floor(end.1) as i32,
            Float::floor(end.2) as i32,
        );

        let ray = (end.0 - start.0, end.1 - start.1, end.2 - start.2);

        let step = (
            if ray.0 >= 0.0 { 1 } else { -1 },
            if ray.1 >= 0.0 { 1 } else { -1 },
            if ray.2 >= 0.0 { 1 } else { -1 },
        );

        let next_boundary = (
            if step.0 > 0 {
                (current_voxel.0 + 1) as f32
            } else {
                current_voxel.0 as f32
            },
            if step.1 > 0 {
                (current_voxel.1 + 1) as f32
            } else {
                current_voxel.1 as f32
            },
            if step.2 > 0 {
                (current_voxel.2 + 1) as f32
            } else {
                current_voxel.2 as f32
            },
        );

        let t_max = (
            if ray.0 != 0.0 {
                (next_boundary.0 - start.0) / ray.0
            } else {
                f32::MAX
            },
            if ray.1 != 0.0 {
                (next_boundary.1 - start.1) / ray.1
            } else {
                f32::MAX
            },
            if ray.2 != 0.0 {
                (next_boundary.2 - start.2) / ray.2
            } else {
                f32::MAX
            },
        );

        let t_delta = (
            if ray.0 != 0.0 {
                Float::abs(1.0 / ray.0)
            } else {
                f32::MAX
            },
            if ray.1 != 0.0 {
                Float::abs(1.0 / ray.1)
            } else {
                f32::MAX
            },
            if ray.2 != 0.0 {
                Float::abs(1.0 / ray.2)
            } else {
                f32::MAX
            },
        );

        Self {
            current_voxel,
            last_voxel,
            step,
            t_max,
            t_delta,
            just_started: true,
            is_finished: false,
        }
    }
}

impl Iterator for VoxelRaycast {
    // (Position, Normal)
    type Item = ((i32, i32, i32), (i32, i32, i32));

    fn next(&mut self) -> Option<Self::Item> {
        if self.is_finished {
            return None;
        }

        if self.just_started {
            self.just_started = false;
            if self.current_voxel == self.last_voxel {
                self.is_finished = true;
            }
            return Some((self.current_voxel, (0, 0, 0)));
        }

        let mut normal = (0, 0, 0);

        if self.t_max.0 < self.t_max.1 {
            if self.t_max.0 < self.t_max.2 {
                self.current_voxel.0 += self.step.0;
                self.t_max.0 += self.t_delta.0;
                normal.0 = -self.step.0;
            } else {
                self.current_voxel.2 += self.step.2;
                self.t_max.2 += self.t_delta.2;
                normal.2 = -self.step.2;
            }
        } else if self.t_max.1 < self.t_max.2 {
            self.current_voxel.1 += self.step.1;
            self.t_max.1 += self.t_delta.1;
            normal.1 = -self.step.1;
        } else {
            self.current_voxel.2 += self.step.2;
            self.t_max.2 += self.t_delta.2;
            normal.2 = -self.step.2;
        }

        if self.current_voxel == self.last_voxel {
            self.is_finished = true;
        }

        Some((self.current_voxel, normal))
    }
}

pub struct VoxelRaycastF64 {
    current_voxel: (i32, i32, i32),
    last_voxel: (i32, i32, i32),
    step: (i32, i32, i32),
    t_max: (f64, f64, f64),
    t_delta: (f64, f64, f64),
    just_started: bool,
    is_finished: bool,
}

impl VoxelRaycastF64 {
    pub fn new(start: impl IntoPoint<f64>, end: impl IntoPoint<f64>) -> Self {
        let start = start.into_point();
        let end = end.into_point();

        let current_voxel = (
            Float::floor(start.0) as i32,
            Float::floor(start.1) as i32,
            Float::floor(start.2) as i32,
        );

        let last_voxel = (
            Float::floor(end.0) as i32,
            Float::floor(end.1) as i32,
            Float::floor(end.2) as i32,
        );

        let ray = (end.0 - start.0, end.1 - start.1, end.2 - start.2);

        let step = (
            if ray.0 >= 0.0 { 1 } else { -1 },
            if ray.1 >= 0.0 { 1 } else { -1 },
            if ray.2 >= 0.0 { 1 } else { -1 },
        );

        let next_boundary = (
            if step.0 > 0 {
                (current_voxel.0 + 1) as f64
            } else {
                current_voxel.0 as f64
            },
            if step.1 > 0 {
                (current_voxel.1 + 1) as f64
            } else {
                current_voxel.1 as f64
            },
            if step.2 > 0 {
                (current_voxel.2 + 1) as f64
            } else {
                current_voxel.2 as f64
            },
        );

        let t_max = (
            if ray.0 != 0.0 {
                (next_boundary.0 - start.0) / ray.0
            } else {
                f64::MAX
            },
            if ray.1 != 0.0 {
                (next_boundary.1 - start.1) / ray.1
            } else {
                f64::MAX
            },
            if ray.2 != 0.0 {
                (next_boundary.2 - start.2) / ray.2
            } else {
                f64::MAX
            },
        );

        let t_delta = (
            if ray.0 != 0.0 {
                Float::abs(1.0 / ray.0)
            } else {
                f64::MAX
            },
            if ray.1 != 0.0 {
                Float::abs(1.0 / ray.1)
            } else {
                f64::MAX
            },
            if ray.2 != 0.0 {
                Float::abs(1.0 / ray.2)
            } else {
                f64::MAX
            },
        );

        Self {
            current_voxel,
            last_voxel,
            step,
            t_max,
            t_delta,
            just_started: true,
            is_finished: false,
        }
    }
}

impl Iterator for VoxelRaycastF64 {
    // (Position, Normal)
    type Item = ((i32, i32, i32), (i32, i32, i32));

    fn next(&mut self) -> Option<Self::Item> {
        if self.is_finished {
            return None;
        }

        if self.just_started {
            self.just_started = false;
            if self.current_voxel == self.last_voxel {
                self.is_finished = true;
            }
            return Some((self.current_voxel, (0, 0, 0)));
        }

        let mut normal = (0, 0, 0);

        if self.t_max.0 < self.t_max.1 {
            if self.t_max.0 < self.t_max.2 {
                self.current_voxel.0 += self.step.0;
                self.t_max.0 += self.t_delta.0;
                normal.0 = -self.step.0;
            } else {
                self.current_voxel.2 += self.step.2;
                self.t_max.2 += self.t_delta.2;
                normal.2 = -self.step.2;
            }
        } else if self.t_max.1 < self.t_max.2 {
            self.current_voxel.1 += self.step.1;
            self.t_max.1 += self.t_delta.1;
            normal.1 = -self.step.1;
        } else {
            self.current_voxel.2 += self.step.2;
            self.t_max.2 += self.t_delta.2;
            normal.2 = -self.step.2;
        }

        if self.current_voxel == self.last_voxel {
            self.is_finished = true;
        }

        Some((self.current_voxel, normal))
    }
}

pub fn voxel_traversal<F>(ray_start: (f32, f32, f32), ray_end: (f32, f32, f32), mut visit: F)
where
    F: FnMut((i32, i32, i32), (i32, i32, i32)) -> bool,
{
    for (pos, normal) in VoxelRaycast::new(ray_start, ray_end) {
        if visit(pos, normal) {
            break;
        }
    }
}

pub fn voxel_traversal_f64<F>(ray_start: (f64, f64, f64), ray_end: (f64, f64, f64), mut visit: F)
where
    F: FnMut((i32, i32, i32), (i32, i32, i32)) -> bool,
{
    for (pos, normal) in VoxelRaycastF64::new(ray_start, ray_end) {
        if visit(pos, normal) {
            break;
        }
    }
}

pub fn intersect_aabb(
    ray_start: impl IntoPoint<f32>,
    ray_dir: impl IntoPoint<f32>,
    aabb_min: impl IntoPoint<f32>,
    aabb_max: impl IntoPoint<f32>,
) -> Option<(f32, f32)> {
    let start = ray_start.into_point();
    let dir = ray_dir.into_point();
    let min = aabb_min.into_point();
    let max = aabb_max.into_point();

    let mut t_min = f32::NEG_INFINITY;
    let mut t_max = f32::INFINITY;

    let axes = [
        (start.0, dir.0, min.0, max.0),
        (start.1, dir.1, min.1, max.1),
        (start.2, dir.2, min.2, max.2),
    ];

    for &(o, d, min_bound, max_bound) in axes.iter() {
        if Float::abs(d) < f32::EPSILON {
            if o < min_bound || o > max_bound {
                return None;
            }
        } else {
            let ood = 1.0 / d;
            let mut t1 = (min_bound - o) * ood;
            let mut t2 = (max_bound - o) * ood;

            if t1 > t2 {
                mem::swap(&mut t1, &mut t2);
            }

            t_min = t_min.max(t1);
            t_max = t_max.min(t2);

            if t_min > t_max {
                return None;
            }
        }
    }

    if t_max < 0.0 {
        return None;
    }

    Some((t_min, t_max))
}
