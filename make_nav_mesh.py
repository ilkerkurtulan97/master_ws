#!/usr/bin/env python3
"""
make_nav_mesh.py — build an lvr2/mbf planning mesh (.h5) DIRECTLY from the NDT
navigation point cloud, so the mesh and the NDT localization map share ONE origin
(the real-robot GOLDEN RULE: NDT map PCD == planning mesh == detector static PCD).

WHY build from the PCD (not the old OBJ): the proven blender_welded_mesh.h5 happens
to already sit in this frame, but it came through the Blender/OBJ path and (a) was
trimmed (lost the Z<-2.3 floor + X>3.9 strip the PCD has) and (b) dropped the ramp
via "keep largest component" on a disconnected OBJ. Reconstructing from Bitlab_nav.pcd
guarantees provenance from the exact cloud NDT localizes against, with no frame doubt.

PIPELINE (folds in every hard-won lesson from this stack):
  1. Load Bitlab_nav.pcd (binary x y z intensity, lowercase) -> xyz points.
     Optional Z crop (--zmax) to drop the ceiling so Poisson doesn't lid it.
  2. Estimate + consistently orient normals (Poisson needs oriented normals).
  3. Poisson surface reconstruction (--depth) -> watertight surface; TRIM the
     lowest-density vertices (--density-quantile) to cut Poisson's ballooned
     extrapolation in open/under-sampled areas.
  4. weld_mesh.py cleanup: vertex-cluster remesh @--cluster (uniform triangles so
     MeshController::searchContainingFace works), drop degenerate/dup, remove
     non-manifold edges, KEEP LARGEST connected component, recompute normals.
  5. GREEDY HALF-EDGE MANIFOLD FILTER (manifold_filter.py logic): replicate lvr2
     HalfEdgeMesh::addFace's rejection rule so it discards ZERO faces on load. If
     lvr2 discarded faces it would RESIZE face_indices in place -> glibc heap
     corruption -> mbf SIGSEGV/-6. Zero discards => no resize => no corruption.
  6. Write a real lvr2 .h5 by COPYING an lvr2-written template (preserves the
     root /version=1 + group CLASS=MeshBuffer/IO=MeshIO structure lvr2's reader
     needs; a from-scratch h5py file does NOT load) and replacing the geometry
     datasets; drop stale attribute groups so mbf recomputes all layers.

Usage:
  conda deactivate                                   # system python3 (open3d/h5py)
  python3 make_nav_mesh.py \
      --pcd /home/ilker/Desktop/real_life_maps/Bitlab_nav.pcd \
      --out /home/ilker/meshnavigation_ws/bitlab_real_mesh.h5 \
      --template blender_welded_mesh.h5.bak --part Low_Bitlab
  # tuning: --depth 10  --density-quantile 0.03  --cluster 0.20  --zmax <crop ceiling>
"""
import argparse, shutil, sys
import numpy as np
import open3d as o3d
import h5py


def read_pcd_xyz(path):
    """Parse a binary 'DATA binary' PCD; return Nx3 float64 xyz (any field layout)."""
    with open(path, "rb") as f:
        lines = []
        while True:
            ln = f.readline()
            if not ln:
                sys.exit("EOF before DATA line")
            lines.append(ln)
            if ln.strip().startswith(b"DATA"):
                break
        data_off = f.tell()
        hdr = {}
        for ln in lines:
            t = ln.decode("ascii", "replace").split()
            if t:
                hdr[t[0].upper()] = t[1:]
        if hdr.get("DATA", [""])[0] != "binary":
            sys.exit("only 'DATA binary' PCD supported, got %s" % hdr.get("DATA"))
        names = hdr["FIELDS"]
        sizes = [int(x) for x in hdr["SIZE"]]
        types = [t.upper() for t in hdr["TYPE"]]
        counts = [int(x) for x in hdr["COUNT"]] if "COUNT" in hdr else [1] * len(names)
        _np = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
               ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
        fmt, seen = [], {}
        for nm, sz, ty, ct in zip(names, sizes, types, counts):
            base = _np[(ty, sz)]
            nm = "pad" if nm == "_" else nm
            seen[nm] = seen.get(nm, -1) + 1
            key = nm if seen[nm] == 0 else "%s_%d" % (nm, seen[nm])
            fmt.append((key, base if ct == 1 else (base, (ct,))))
        npts = int(hdr["POINTS"][0])
        f.seek(data_off)
        a = np.fromfile(f, dtype=np.dtype(fmt), count=npts)
    return np.stack([a["x"].astype(np.float64),
                     a["y"].astype(np.float64),
                     a["z"].astype(np.float64)], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pcd", default="/home/ilker/Desktop/real_life_maps/Bitlab_nav.pcd")
    ap.add_argument("--out", default="/home/ilker/meshnavigation_ws/bitlab_real_mesh.h5")
    ap.add_argument("--template", default="blender_welded_mesh.h5.bak",
                    help="an lvr2-written .h5 to copy structure from")
    ap.add_argument("--part", default="Low_Bitlab", help="mesh group name (== config mesh_part)")
    ap.add_argument("--depth", type=int, default=10, help="Poisson octree depth (detail)")
    ap.add_argument("--density-quantile", type=float, default=0.03,
                    help="trim Poisson verts below this density quantile (cut ballooning)")
    ap.add_argument("--cluster", type=float, default=0.20, help="vertex-cluster remesh size (m)")
    ap.add_argument("--normal-radius", type=float, default=0.4)
    ap.add_argument("--normal-k", type=int, default=30)
    ap.add_argument("--zmin", type=float, default=None, help="drop points below this Z")
    ap.add_argument("--zmax", type=float, default=None, help="drop points above this Z (ceiling)")
    args = ap.parse_args()

    P = read_pcd_xyz(args.pcd)
    print("loaded %d points  X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]" % (
        P.shape[0], P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
        P[:, 2].min(), P[:, 2].max()))
    if args.zmin is not None:
        P = P[P[:, 2] >= args.zmin]
    if args.zmax is not None:
        P = P[P[:, 2] <= args.zmax]
    if args.zmin is not None or args.zmax is not None:
        print("after Z crop: %d points" % P.shape[0])

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
    print("estimating normals (radius=%.2f k=%d)..." % (args.normal_radius, args.normal_k))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.normal_radius, max_nn=args.normal_k))
    print("orienting normals consistently (this is the slow step)...")
    pcd.orient_normals_consistent_tangent_plane(args.normal_k)

    print("Poisson reconstruction (depth=%d)..." % args.depth)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=args.depth)
    dens = np.asarray(dens)
    thr = np.quantile(dens, args.density_quantile)
    mesh.remove_vertices_by_mask(dens < thr)
    print("poisson: %d verts %d faces  (trimmed density<%.3f q=%.2f)" % (
        len(mesh.vertices), len(mesh.triangles), thr, args.density_quantile))

    # weld_mesh.py cleanup: uniform remesh + manifold + keep largest component
    mesh = mesh.simplify_vertex_clustering(
        args.cluster, o3d.geometry.SimplificationContraction.Average)
    mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices(); mesh.remove_unreferenced_vertices()
    mesh.remove_non_manifold_edges()
    comp = np.asarray(mesh.cluster_connected_triangles()[0])
    if comp.size:
        big = np.bincount(comp).argmax()
        mesh.remove_triangles_by_mask(comp != big)
        mesh.remove_unreferenced_vertices()
    V = np.asarray(mesh.vertices, dtype=np.float64)
    F = np.asarray(mesh.triangles, dtype=np.int64)
    print("remeshed: %d verts %d faces  V/F=%.3f  (cluster=%.2f)" % (
        V.shape[0], F.shape[0], V.shape[0] / max(F.shape[0], 1), args.cluster))

    # greedy half-edge manifold filter -> guarantee lvr2 discards 0 faces (no resize)
    claimed = set(); keep = np.zeros(F.shape[0], dtype=bool); dropped = 0
    for i in range(F.shape[0]):
        a, b, c = int(F[i, 0]), int(F[i, 1]), int(F[i, 2])
        if a == b or b == c or a == c:
            dropped += 1; continue
        he = ((a, b), (b, c), (c, a))
        if he[0] in claimed or he[1] in claimed or he[2] in claimed:
            dropped += 1; continue
        claimed.update(he); keep[i] = True
    Fk = F[keep]
    print("half-edge filter: kept %d faces, dropped %d (%.2f%%) -> lvr2 discards 0" % (
        Fk.shape[0], dropped, 100.0 * dropped / max(F.shape[0], 1)))

    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V),
                                  o3d.utility.Vector3iVector(Fk.astype(np.int32)))
    m.remove_unreferenced_vertices()
    # keep largest component AGAIN (the filter can split off slivers)
    comp = np.asarray(m.cluster_connected_triangles()[0])
    if comp.size:
        big = np.bincount(comp).argmax()
        m.remove_triangles_by_mask(comp != big); m.remove_unreferenced_vertices()
    m.compute_vertex_normals()
    Vc = np.asarray(m.vertices, dtype=np.float32)
    Fc = np.asarray(m.triangles, dtype=np.uint32)
    VN = np.asarray(m.vertex_normals, dtype=np.float32)
    ncomp = int(np.asarray(m.cluster_connected_triangles()[0]).max()) + 1
    print("FINAL: %d verts %d faces  V/F=%.3f  components=%d" % (
        Vc.shape[0], Fc.shape[0], Vc.shape[0] / max(Fc.shape[0], 1), ncomp))
    print("FINAL bbox X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f]" % (
        Vc[:, 0].min(), Vc[:, 0].max(), Vc[:, 1].min(), Vc[:, 1].max(),
        Vc[:, 2].min(), Vc[:, 2].max()))

    # write lvr2 h5 by copying the template and replacing geometry
    shutil.copy(args.template, args.out)
    G = "/" + args.part + "/"
    g = h5py.File(args.out, "r+")
    if args.part not in g:
        sys.exit("template %s has no group '%s'" % (args.template, args.part))
    geo = g[G + "geometry"]
    for n in ["vertices", "face_indices", "vertex_normals"]:
        if n in geo:
            del geo[n]
    geo.create_dataset("vertices", data=Vc)
    geo.create_dataset("face_indices", data=Fc)
    geo.create_dataset("vertex_normals", data=VN)
    for grp in ["vertex_attributes", "edge_attributes", "face_attributes"]:
        if grp in g[G]:
            del g[G][grp]
    g.close()
    print("WROTE", args.out)


if __name__ == "__main__":
    main()
