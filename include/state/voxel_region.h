
namespace grrt {

/// @brief A voxel region, representing some 3D volume of space.
class VoxelRegion {
public:
  VoxelRegion();
  Voxel(int x, int y, int z);
  int x;
  int y;
  int z;
  bool operator==(const Voxel &other) const;
  bool operator!=(const Voxel &other) const;
  bool operator<(const Voxel &other) const;
  bool operator>(const Voxel &other) const;
  bool operator<=(const Voxel &other) const;
  bool operator>=(const Voxel &other) const;
};
} // namespace grrt