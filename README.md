# Voxel Traversal Algorithm
This is a rust port of https://github.com/francisengelmann/fast_voxel_traversal.

# Example
```rust

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

# License
Licensed under [MIT LICENSE](https://github.com/ChrisTechs/voxel-traversal/blob/master/LICENSE)
