# Voxel Traversal Algorithm
This is a rust port of https://github.com/francisengelmann/fast_voxel_traversal.

# Example
This crate uses [glam](https://github.com/bitshifter/glam-rs) for linear algebra.
```rust

// f32

voxel_traversal::voxel_traversal(
    start, // Vec3
    end, // Vec3
    |pos| { // IVec3
        // return true to exit
        if (pos.x == 0) {
            return true;
        } else {
        // return false to continue traversing
            return false
        }
    }
);

// there is also a DVec3 implementation called voxel_traversal_f64

```

# License
Licensed under [MIT LICENSE](https://github.com/ChrisTechs/voxel-traversal/blob/master/LICENSE)
