import open3d as o3d
from abc import ABC, abstractmethod

class SurfaceReconstruction(ABC):
    @abstractmethod
    def reconstruct_surface(self, pcd: o3d.geometry.PointCloud):
        pass

class PoissonSurfaceReconstruction(SurfaceReconstruction):
    def reconstruct_surface(self, pcd: o3d.geometry.PointCloud):
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=15)
        return mesh

class BallPivotingSurfaceReconstruction(SurfaceReconstruction):
    def reconstruct_surface(self, pcd: o3d.geometry.PointCloud):
        radii = [0.001, 0.002, 0.004, 0.008, 0.016, 0.032, 0.064, 0.128]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))
        return mesh

