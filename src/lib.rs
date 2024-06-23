// MIT License
//
// Copyright (c) 2019 Francis Engelmann
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

use glam::{DVec3, IVec3, Vec3};

pub fn voxel_traversal<F>(ray_start: Vec3, ray_end: Vec3, mut visit: F)
    where F: FnMut(IVec3, IVec3) -> bool
{
    // Current voxel hit by the ray.
    let mut current_voxel: IVec3 = IVec3::new(
        ray_start.x.floor() as i32,
        ray_start.y.floor() as i32,
        ray_start.z.floor() as i32,
    );

    // The id of the last voxel hit by the ray.
    let last_voxel: IVec3 = IVec3::new(
        ray_end.x.floor() as i32,
        ray_end.y.floor() as i32,
        ray_end.z.floor() as i32,
    );

    // Compute ray direction.
    let ray = ray_end - ray_start;

    // Determine step direction for each axis.
    let step_x = if ray.x >= 0.0 { 1 } else { -1 };
    let step_y = if ray.y >= 0.0 { 1 } else { -1 };
    let step_z = if ray.z >= 0.0 { 1 } else { -1 };

    // Calculate initial distances to the next voxel boundary.
    let next_voxel_boundary_x = if step_x > 0 {
        (current_voxel.x + 1) as f32
    } else {
        current_voxel.x as f32
    };
    let next_voxel_boundary_y = if step_y > 0 {
        (current_voxel.y + 1) as f32
    } else {
        current_voxel.y as f32
    };
    let next_voxel_boundary_z = if step_z > 0 {
        (current_voxel.z + 1) as f32
    } else {
        current_voxel.z as f32
    };

    // tMax -- distance to the next intersection with a voxel boundary.
    let mut t_max_x = if ray.x != 0.0 {
        (next_voxel_boundary_x - ray_start.x) / ray.x
    } else {
        f32::MAX
    };
    let mut t_max_y = if ray.y != 0.0 {
        (next_voxel_boundary_y - ray_start.y) / ray.y
    } else {
        f32::MAX
    };
    let mut t_max_z = if ray.z != 0.0 {
        (next_voxel_boundary_z - ray_start.z) / ray.z
    } else {
        f32::MAX
    };

    // tDelta -- how far along the ray we must move to cross a voxel boundary.
    let t_delta_x = if ray.x != 0.0 {
        (1.0 / ray.x).abs()
    } else {
        f32::MAX
    };
    let t_delta_y = if ray.y != 0.0 {
        (1.0 / ray.y).abs()
    } else {
        f32::MAX
    };
    let t_delta_z = if ray.z != 0.0 {
        (1.0 / ray.z).abs()
    } else {
        f32::MAX
    };

    if visit(current_voxel, IVec3::ZERO) {
        return;
    }

    // Continue traversal until we reach the last voxel
    while current_voxel != last_voxel {

        let mut normal = IVec3::ZERO;

        if t_max_x < t_max_y {
            if t_max_x < t_max_z {
                current_voxel.x += step_x;
                t_max_x += t_delta_x;
                normal.x = -step_x;
            } else {
                current_voxel.z += step_z;
                t_max_z += t_delta_z;
                normal.z = -step_z;
            }
        } else if t_max_y < t_max_z {
            current_voxel.y += step_y;
            t_max_y += t_delta_y;
            normal.y = -step_y;
        } else {
            current_voxel.z += step_z;
            t_max_z += t_delta_z;
            normal.z = -step_z;
        }

        if visit(current_voxel, normal) {
            return;
        }
    }
}

pub fn voxel_traversal_f64<F>(ray_start: DVec3, ray_end: DVec3, mut visit: F)
    where F: FnMut(IVec3, IVec3) -> bool
{
    // Current voxel hit by the ray.
    let mut current_voxel: IVec3 = IVec3::new(
        ray_start.x.floor() as i32,
        ray_start.y.floor() as i32,
        ray_start.z.floor() as i32,
    );

    // The id of the last voxel hit by the ray.
    let last_voxel: IVec3 = IVec3::new(
        ray_end.x.floor() as i32,
        ray_end.y.floor() as i32,
        ray_end.z.floor() as i32,
    );

    // Compute ray direction.
    let ray = ray_end - ray_start;

    // Determine step direction for each axis.
    let step_x = if ray.x >= 0.0 { 1 } else { -1 };
    let step_y = if ray.y >= 0.0 { 1 } else { -1 };
    let step_z = if ray.z >= 0.0 { 1 } else { -1 };

    // Calculate initial distances to the next voxel boundary.
    let next_voxel_boundary_x = if step_x > 0 {
        (current_voxel.x + 1) as f64
    } else {
        current_voxel.x as f64
    };
    let next_voxel_boundary_y = if step_y > 0 {
        (current_voxel.y + 1) as f64
    } else {
        current_voxel.y as f64
    };
    let next_voxel_boundary_z = if step_z > 0 {
        (current_voxel.z + 1) as f64
    } else {
        current_voxel.z as f64
    };

    // tMax -- distance to the next intersection with a voxel boundary.
    let mut t_max_x = if ray.x != 0.0 {
        (next_voxel_boundary_x - ray_start.x) / ray.x
    } else {
        f64::MAX
    };
    let mut t_max_y = if ray.y != 0.0 {
        (next_voxel_boundary_y - ray_start.y) / ray.y
    } else {
        f64::MAX
    };
    let mut t_max_z = if ray.z != 0.0 {
        (next_voxel_boundary_z - ray_start.z) / ray.z
    } else {
        f64::MAX
    };

    // tDelta -- how far along the ray we must move to cross a voxel boundary.
    let t_delta_x = if ray.x != 0.0 {
        (1.0 / ray.x).abs()
    } else {
        f64::MAX
    };
    let t_delta_y = if ray.y != 0.0 {
        (1.0 / ray.y).abs()
    } else {
        f64::MAX
    };
    let t_delta_z = if ray.z != 0.0 {
        (1.0 / ray.z).abs()
    } else {
        f64::MAX
    };

    if visit(current_voxel, IVec3::ZERO) {
        return;
    }

    // Continue traversal until we reach the last voxel
    while current_voxel != last_voxel {

        let mut normal = IVec3::ZERO;

        if t_max_x < t_max_y {
            if t_max_x < t_max_z {
                current_voxel.x += step_x;
                t_max_x += t_delta_x;
                normal.x = -step_x;
            } else {
                current_voxel.z += step_z;
                t_max_z += t_delta_z;
                normal.z = -step_z;
            }
        } else if t_max_y < t_max_z {
            current_voxel.y += step_y;
            t_max_y += t_delta_y;
            normal.y = -step_y;
        } else {
            current_voxel.z += step_z;
            t_max_z += t_delta_z;
            normal.z = -step_z;
        }

        if visit(current_voxel, normal) {
            return;
        }
    }
}
