import sqlite3
import numpy as np
import struct
import zlib
from pclpy import pcl
from scipy.spatial import Delaunay
import pyvista as pv

DB_PATH = "/Users/akko/Documents/RTAB-Map/rtabmap.tmp.db"

def load_point_cloud_from_rtabmap():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT data FROM Data WHERE id = (SELECT max(id) FROM Data)")
    row = cursor.fetchone()
    conn.close()

    if not row:
        raise RuntimeError("No data found in RTAB-Map database.")

    blob = zlib.decompress(row[0])
    floats = struct.unpack('f' * (len(blob) // 4), blob)
    points = np.array(floats, dtype=np.float32).reshape(-1, 4)[:, :3]
    return points

def filter_voxel(points, leaf_size=0.1):
    cloud = pcl.PointCloud.PointXYZ()
    cloud.from_array(points.astype(np.float32))

    voxel = pcl.filters.VoxelGrid.PointXYZ()
    voxel.setInputCloud(cloud)
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size)

    filtered = pcl.PointCloud.PointXYZ()
    voxel.filter(filtered)
    return filtered.xyz

def remove_outliers(points, radius=0.2, min_neighbors=5):
    cloud = pcl.PointCloud.PointXYZ()
    cloud.from_array(points.astype(np.float32))

    sor = pcl.filters.RadiusOutlierRemoval.PointXYZ()
    sor.setInputCloud(cloud)
    sor.setRadiusSearch(radius)
    sor.setMinNeighborsInRadius(min_neighbors)

    cleaned = pcl.PointCloud.PointXYZ()
    sor.filter(cleaned)
    return cleaned.xyz

def extract_ground_plane(points, z_threshold=0.2):
    return points[points[:, 2] < z_threshold]

def build_navmesh(points):
    tri = Delaunay(points[:, :2])
    return tri

def filter_bad_triangles(points, tri, max_edge=1.0):
    good = []
    for simplex in tri.simplices:
        pts = points[simplex]
        edges = [np.linalg.norm(pts[i] - pts[j]) for i, j in [(0,1),(1,2),(2,0)]]
        if max(edges) < max_edge:
            good.append(simplex)
    tri.simplices = np.array(good)
    return tri

def create_pyvista_mesh(points, tri):
    faces = np.hstack([np.full((len(tri.simplices), 1), 3), tri.simplices]).astype(np.int32)
    mesh = pv.PolyData(points)
    mesh.faces = faces
    return mesh

def save_navmesh(mesh, filename="navmesh.obj"):
    mesh.save(filename)

def visualize(mesh):
    pv.set_plot_theme("document")
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, show_edges=True, color="lightblue")
    plotter.add_axes()
    plotter.show()

def main():
    raw_points = load_point_cloud_from_rtabmap()
    print(f"Loaded {len(raw_points)} raw points")

    filtered = filter_voxel(raw_points)
    print(f"After voxel filter: {len(filtered)}")

    cleaned = remove_outliers(filtered)
    print(f"After outlier removal: {len(cleaned)}")

    ground = extract_ground_plane(cleaned)
    print(f"Ground points: {len(ground)}")

    if len(ground) < 10:
        raise RuntimeError("Too few ground points.")

    tri = build_navmesh(ground)
    tri = filter_bad_triangles(ground, tri, max_edge=1.5)

    mesh = create_pyvista_mesh(ground, tri)
    save_navmesh(mesh)
    visualize(mesh)

if __name__ == "__main__":
    main()
