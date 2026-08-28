"""
PCD → Voxelized OBJ exporter (exposed-face culling, Y-up output)

Usage:
    /usr/bin/python3 pcd_to_obj.py <input.pcd> <output.obj> [--voxel-size 0.1]

The output OBJ is in Y-up coordinates (Z-up PCD → Y↔Z swap) so it can be
fed directly to bake_tcbin.py without --zup.
"""

import argparse
import sys
import numpy as np


def load_pcd(path: str) -> np.ndarray:
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points, dtype=np.float32)
    print(f"Loaded {len(pts):,} points from {path}")
    return pts


def voxelize(points: np.ndarray, voxel_size: float):
    bb_min = points.min(axis=0)
    indices = np.floor((points - bb_min) / voxel_size).astype(np.int32)
    occupied = set(map(tuple, indices))
    print(f"Voxel grid: {len(occupied):,} occupied voxels at {voxel_size} m resolution")
    return occupied, bb_min, voxel_size


# 6 face directions: (normal_axis, normal_sign, 4 corner offsets in the other two axes)
_FACES = [
    # +X face
    ( 1, 0, 0, [(1,0,0),(1,1,0),(1,1,1),(1,0,1)] ),
    # -X face
    (-1, 0, 0, [(0,0,1),(0,1,1),(0,1,0),(0,0,0)] ),
    # +Y face
    ( 0, 1, 0, [(0,1,1),(1,1,1),(1,1,0),(0,1,0)] ),
    # -Y face
    ( 0,-1, 0, [(0,0,0),(1,0,0),(1,0,1),(0,0,1)] ),
    # +Z face
    ( 0, 0, 1, [(0,0,1),(1,0,1),(1,1,1),(0,1,1)] ),
    # -Z face
    ( 0, 0,-1, [(0,1,0),(1,1,0),(1,0,0),(0,0,0)] ),
]


def exposed_face_mesh(occupied: set, bb_min: np.ndarray, voxel_size: float,
                      swap_yz: bool = True):
    """
    For every occupied voxel, emit the 4 faces that have no occupied neighbor.
    swap_yz=True converts Z-up (ROS) to Y-up (Recast/Detour) at export time.
    """
    verts = []
    faces = []
    vert_index: dict = {}

    def add_vert(xyz):
        key = xyz
        if key not in vert_index:
            vert_index[key] = len(verts)
            verts.append(xyz)
        return vert_index[key]

    nx, ny, nz = (int(1), int(0), int(0))  # unused — kept for clarity

    for vox in occupied:
        ix, iy, iz = vox
        for dx, dy, dz, corners in _FACES:
            neighbor = (ix + dx, iy + dy, iz + dz)
            if neighbor in occupied:
                continue  # internal face — skip
            # emit two triangles for this quad
            quad_verts = []
            for cx, cy, cz in corners:
                wx = (ix + cx) * voxel_size + bb_min[0]
                wy = (iy + cy) * voxel_size + bb_min[1]
                wz = (iz + cz) * voxel_size + bb_min[2]
                if swap_yz:
                    # Z-up → Y-up: swap Y and Z
                    quad_verts.append(add_vert((wx, wz, wy)))
                else:
                    quad_verts.append(add_vert((wx, wy, wz)))
            a, b, c, d = quad_verts
            faces.append((a, b, c))
            faces.append((a, c, d))

    print(f"Mesh: {len(verts):,} verts, {len(faces):,} faces")
    return verts, faces


def export_obj(verts, faces, path: str):
    with open(path, "w") as f:
        f.write("# pcd_to_obj.py — voxelized exposed-face mesh\n")
        for x, y, z in verts:
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            f.write(f"f {a+1} {b+1} {c+1}\n")
    print(f"Saved OBJ to {path}")


def main():
    parser = argparse.ArgumentParser(description="Voxelize PCD → Y-up OBJ (exposed-face culling)")
    parser.add_argument("pcd",  help="Input PCD file")
    parser.add_argument("obj",  help="Output OBJ file")
    parser.add_argument("--voxel-size", type=float, default=0.1,
                        help="Voxel size in metres (default: 0.1)")
    parser.add_argument("--no-swap-yz", action="store_true",
                        help="Keep Z-up instead of converting to Y-up")
    args = parser.parse_args()

    points = load_pcd(args.pcd)
    occupied, bb_min, vs = voxelize(points, args.voxel_size)
    verts, faces = exposed_face_mesh(occupied, bb_min, vs, swap_yz=not args.no_swap_yz)
    export_obj(verts, faces, args.obj)


if __name__ == "__main__":
    main()
