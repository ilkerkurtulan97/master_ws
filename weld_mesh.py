"""
Weld an "exploded" lvr2 mesh HDF5 back into shared-vertex topology.

WHY: mesh_map loads OBJ meshes via Assimp, whose aiProcess_JoinIdenticalVertices
is broken in this build -> it emits ~3 unshared vertices per face (a triangle
soup). With no shared edges the DijkstraMeshPlanner wavefront cannot leave the
goal triangle: "Predecessor of the goal is not set! No path found!" -> Path 0m
-> the rover never moves.

This script reads the exploded geometry from an lvr2 mesh .h5, merges coincident
vertices (recovering shared edges), drops degenerate faces, recomputes vertex
normals, and rewrites the geometry datasets in place. The stale per-element
attribute groups (vertex_/edge_/face_attributes) are removed so mesh_map
recomputes them. Point mesh_map's mesh_file AND mesh_working_file at the result
(equal => mesh_map loads the .h5 directly and skips Assimp).

Usage:  python3 weld_mesh.py <src_exploded.h5> <dst_welded.h5> [mesh_name]
"""
import sys, h5py, numpy as np, open3d as o3d, shutil

src = sys.argv[1] if len(sys.argv) > 1 else "blender_clean_mesh.h5"
dst = sys.argv[2] if len(sys.argv) > 2 else "blender_welded_mesh.h5"
name = sys.argv[3] if len(sys.argv) > 3 else "Low_Bitlab"

shutil.copy(src, dst)
f = h5py.File(dst, "r+")
G = "/" + name + "/"
V = np.asarray(f[G + "geometry/vertices"][:], dtype=np.float64)
F = np.asarray(f[G + "geometry/face_indices"][:], dtype=np.int64)
print("exploded: %d verts %d faces  V/F=%.3f" % (V.shape[0], F.shape[0], V.shape[0] / F.shape[0]))

# weld coincident vertices (round to 0.01 mm) -> shared topology
uniq, inv = np.unique(np.round(V, 5), axis=0, return_inverse=True)
Vw = np.zeros((uniq.shape[0], 3)); Vw[inv] = V
Fw = inv[F].astype(np.uint32)
good = (Fw[:, 0] != Fw[:, 1]) & (Fw[:, 1] != Fw[:, 2]) & (Fw[:, 0] != Fw[:, 2])
Fw = Fw[good]
print("welded:   %d verts %d faces  V/F=%.3f" % (Vw.shape[0], Fw.shape[0], Vw.shape[0] / Fw.shape[0]))

m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(Vw), o3d.utility.Vector3iVector(Fw.astype(np.int32)))

# (2) uniform remesh: the raw scan triangulation is mixed-resolution, so the
# robot's nearest 3D vertex is often NOT a corner of the face it stands on ->
# MeshController bootstrap searchContainingFace() fails ("No containing face
# found!") and the rover never moves. Vertex clustering makes triangles uniform
# (restores nearest-vertex = containing-face-corner). Remove non-manifold edges
# so lvr2/mbf won't drop faces on load, then keep the largest component.
m = m.simplify_vertex_clustering(0.20, o3d.geometry.SimplificationContraction.Average)
m.remove_degenerate_triangles(); m.remove_duplicated_triangles()
m.remove_duplicated_vertices(); m.remove_unreferenced_vertices()
m.remove_non_manifold_edges()
comp = np.asarray(m.cluster_connected_triangles()[0])
big = np.bincount(comp).argmax()
m.remove_triangles_by_mask(comp != big); m.remove_unreferenced_vertices()
print("connected components after remesh:", np.asarray(m.cluster_connected_triangles()[0]).max() + 1)
m.compute_vertex_normals()
Vw = np.asarray(m.vertices)
Fw = np.asarray(m.triangles)
VN = np.asarray(m.vertex_normals, dtype=np.float32)
print("remeshed: %d verts %d faces  V/F=%.3f" % (Vw.shape[0], Fw.shape[0], Vw.shape[0] / Fw.shape[0]))

geo = f[G + "geometry"]
for n in ["vertices", "face_indices", "vertex_normals"]:
    if n in geo:
        del geo[n]
geo.create_dataset("vertices", data=Vw.astype(np.float32))
geo.create_dataset("face_indices", data=Fw.astype(np.uint32))
geo.create_dataset("vertex_normals", data=VN)
for grp in ["vertex_attributes", "edge_attributes", "face_attributes"]:
    if grp in f[G]:
        del f[G][grp]
f.close()
print("WROTE", dst)
