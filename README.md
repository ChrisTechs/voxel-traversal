# Voxel Traversal

This is a Rust port of [fast_voxel_traversal](https://github.com/francisengelmann/fast_voxel_traversal). It includes the original closure API, an iterator API, and AABB intersection utilities.

## Examples
#### Closure API
```Rust
// f32
voxel_traversal::voxel_traversal(
    start, // (f32,f32,f32)
    end, // (f32,f32,f32)
    |pos, normal| { // (i32,i32,i32), (i32,i32,i32)
        // return true to exit
        if (pos.0 == 0) {
            return true;
        } else {
        // return false to continue traversing
        return false
        }
    }
);

// there is also a f64 implementation called voxel_traversal_f64
```

#### Iterator API

Works with tuples, arrays, or glam/nalgebra types (if features enabled).

```Rust
use voxel_traversal::VoxelRaycast;

for (pos, normal) in VoxelRaycast::new(start, end) {
    if pos.0 == 0 {
        break;
    }
}

// For f64 use VoxelRaycastF64::new(start, end)
```

#### AABB Utility

```Rust
use voxel_traversal::intersect_aabb;

if let Some((t_min, t_max)) = intersect_aabb(ray_start, ray_dir, box_min, box_max) {
    // Ray intersects or starts inside the box
}
```

Features: glam,  nalgebra

# License
Licensed under [MIT LICENSE](https://github.com/ChrisTechs/voxel-traversal/blob/master/LICENSE)
