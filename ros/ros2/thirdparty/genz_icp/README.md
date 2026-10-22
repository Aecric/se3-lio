Vendored subset of [GenZ-ICP](https://github.com/cocel-postech/genz-icp) (MIT) at commit 674ccce,
copied from Super-LIO `thirdparty/genz_icp`: `core/VoxelHashMap.{hpp,cpp}` and `core/Registration.{hpp,cpp}`.

- `tsl/` — tsl::robin_map (MIT), upstream Tessil/robin-map as fetched by the GenZ-ICP build.
- Sophus is not vendored here: the refiner is compiled against the Sophus target of the se3-lio core,
  so the binary contains a single Sophus.
