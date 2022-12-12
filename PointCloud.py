from typing import List

import numpy as np
# import open3d as o3d

class PointCloud:
    def __init__(self, points: np.ndarray=None, colors: np.ndarray=None) -> None:
        self.points = points
        self.colors = colors

    # def voxel_downsample(self, factor):
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(self.points)
    #     pcd.colors = o3d.utility.Vector3dVector(self.colors)
    #     pcd = pcd.voxel_down_sample(factor)
    #     return PointCloud(np.asarray(pcd.points), np.asarray(pcd.colors))
    
    def uniform_downsample(self, skip):
        return PointCloud(self.points[::skip, :], self.colors[::skip, :])
    
    @classmethod
    def load_from_npy_file(cls, path: str):
        points = np.load(f'{path}_points.npy', allow_pickle=True)
        colors = np.load(f'{path}_colors.npy', allow_pickle=True)
        return cls(points, colors)

    def __repr__(self) -> str:
        return f'PointCloud(num_points={self.points.shape[0]})'

class PointCloudSequence:
    def __init__(self, point_clouds:List[PointCloud]) -> None:
        self._sequence = point_clouds
    
    def downsample(self, skip):
        return PointCloudSequence([pc.uniform_downsample(skip) for pc in self._sequence])

    def __len__(self) -> int:
        return len(self._sequence)

    def __getitem__(self, idx: int) -> PointCloud:
        return self._sequence[idx]
    
    def __repr__(self) -> str:
        return f'PointCloudSequence(num_point_clouds={len(self._sequence)}, avg_num_points_per_point_cloud={sum([pc.points.shape[0] for pc in self._sequence])/len(self._sequence)})'