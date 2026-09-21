#!/usr/bin/env python3
"""
make_nav_pcd.py — build a small, PCL-clean navigation/localization PCD from the
huge Bitlab master cloud.

WHY: Bitlab_hq.pcd is 6.8 GB / 130.6 M points with 13 fields and the intensity
field named "Intensity" (capital I) buried among Gps_Time/rgb/normals/etc. PCL's
PointXYZI loader (used by NDT localization) looks for a field named exactly
"intensity" (lowercase) in an x/y/z/intensity layout, so that master file triggers
"Failed to find match for field 'intensity'". This script streams the master in
chunks (never loads it whole), voxel-downsamples (centroid per voxel, carrying
intensity), and writes a compact binary PCD with EXACTLY:
    FIELDS x y z intensity   (all float32, lowercase)
which loads cleanly as BOTH:
  * pcl::PointXYZ  (master_ws / mesh_nav dynamic_obstacle_detector) -> x,y,z used
  * pcl::PointXYZI (NDT localization)                               -> intensity used

Keep the result in the SAME coordinate frame as your mesh + NDT map (this script
applies NO rotation/translation, so the frame is preserved).

Usage:
  conda deactivate
  python3 make_nav_pcd.py \
      --src /mnt/storage/3d-scans/bitlab/Bitlab_20260305135305/data/pointcloud/Bitlab_hq.pcd \
      --out /home/ilker/Desktop/real_life_maps/Bitlab_nav.pcd \
      --voxel 0.05
Optional ceiling/floor crop (in the cloud's own Z, metres):
      --zmax 3.0      # drop the ceiling (floor ~ -1.5 m, ceiling ~ 5+ m)
      --zmin -2.5     # drop under-floor noise
"""
import argparse
import sys
import numpy as np

# (TYPE, SIZE) -> numpy base dtype string
_NP = {("F", 4): "f4", ("F", 8): "f8",
       ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
       ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8"}


def read_header(f):
    """Parse a PCD text header; return dict + byte offset where binary data starts."""
    hdr, lines = {}, []
    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("EOF before DATA line")
        lines.append(line)
        s = line.decode("ascii", "replace").strip()
        if s.startswith("DATA"):
            break
    for line in lines:
        t = line.decode("ascii", "replace").strip().split()
        if t:
            hdr[t[0].upper()] = t[1:]
    return hdr, f.tell()


def build_record_dtype(hdr):
    """Packed (unaligned) structured dtype matching one binary PCD point."""
    fields = hdr["FIELDS"]
    sizes = [int(x) for x in hdr["SIZE"]]
    types = [t.upper() for t in hdr["TYPE"]]
    counts = [int(x) for x in hdr["COUNT"]] if "COUNT" in hdr else [1] * len(fields)
    names, formats, seen = [], [], {}
    for i, (nm, sz, ty, ct) in enumerate(zip(fields, sizes, types, counts)):
        base = _NP.get((ty, sz))
        if base is None:
            raise RuntimeError(f"Unsupported field {nm} TYPE {ty} SIZE {sz}")
        nm = "pad" if nm == "_" else nm
        if nm in seen:                      # de-dup duplicate/padding names
            seen[nm] += 1; nm = f"{nm}_{seen[nm]}"
        else:
            seen[nm] = 0
        names.append(nm)
        formats.append(base if ct == 1 else (base, (ct,)))
    dt = np.dtype({"names": names, "formats": formats}, align=False)
    return dt, names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--voxel", type=float, default=0.05, help="voxel size in metres (default 0.05)")
    ap.add_argument("--zmin", type=float, default=None, help="drop points below this Z (optional)")
    ap.add_argument("--zmax", type=float, default=None, help="drop points above this Z, e.g. ceiling (optional)")
    ap.add_argument("--chunk", type=int, default=5_000_000, help="points per streamed chunk")
    args = ap.parse_args()

    with open(args.src, "rb") as f:
        hdr, data_off = read_header(f)
        if hdr.get("DATA", [""])[0] != "binary":
            sys.exit(f"ERROR: only uncompressed 'DATA binary' supported, got {hdr.get('DATA')}")
        rec, names = build_record_dtype(hdr)
        npts = int(hdr["POINTS"][0])
        lower = {n.lower(): n for n in names}
        for need in ("x", "y", "z"):
            if need not in lower:
                sys.exit(f"ERROR: source has no '{need}' field; has {names}")
        iname = lower.get("intensity")          # 'Intensity' or 'intensity' (or None)
        sz = rec.itemsize
        declared = sum(int(s) * (int(c) if "COUNT" in hdr else 1)
                       for s, c in zip(hdr["SIZE"], hdr.get("COUNT", hdr["SIZE"])))
        print(f"src: {args.src}\n  {npts:,} pts, point_step {sz} B (declared {declared}), "
              f"fields={names}\n  intensity field: {iname or 'NONE -> will write zeros'}")
        print(f"  voxel={args.voxel} m  zmin={args.zmin}  zmax={args.zmax}")

        v = args.voxel
        gk = gs = gc = None                      # global voxel idx (K,3 i64), sums (K,4 f8), counts (K,)
        read = 0
        f.seek(data_off)
        while read < npts:
            n = min(args.chunk, npts - read)
            chunk = np.fromfile(f, dtype=rec, count=n)
            if chunk.shape[0] == 0:
                break
            read += chunk.shape[0]
            x = chunk[lower["x"]].astype(np.float64)
            y = chunk[lower["y"]].astype(np.float64)
            z = chunk[lower["z"]].astype(np.float64)
            inten = (chunk[iname].astype(np.float64) if iname else np.zeros_like(x))
            m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            if args.zmin is not None: m &= z >= args.zmin
            if args.zmax is not None: m &= z <= args.zmax
            x, y, z, inten = x[m], y[m], z[m], inten[m]
            if x.size == 0:
                print(f"  ...{read:,}/{npts:,}"); continue
            vox = np.stack([np.floor(x / v), np.floor(y / v), np.floor(z / v)], 1).astype(np.int64)
            uniq, inv = np.unique(vox, axis=0, return_inverse=True)
            inv = np.asarray(inv).ravel()       # numpy 2.0 may return 2D inverse
            K = len(uniq)
            cs = np.stack([np.bincount(inv, x, K), np.bincount(inv, y, K),
                           np.bincount(inv, z, K), np.bincount(inv, inten, K)], 1)
            cc = np.bincount(inv, None, K).astype(np.float64)
            if gk is None:
                gk, gs, gc = uniq, cs, cc
            else:                                # merge into running global accumulator
                allk = np.concatenate([gk, uniq], 0)
                gk, ginv = np.unique(allk, axis=0, return_inverse=True)
                ginv = np.asarray(ginv).ravel()
                ns = np.zeros((len(gk), 4)); nc = np.zeros(len(gk))
                for col in range(4):
                    ns[:, col] = np.bincount(ginv, np.concatenate([gs[:, col], cs[:, col]]), len(gk))
                nc = np.bincount(ginv, np.concatenate([gc, cc]), len(gk))
                gs, gc = ns, nc
            print(f"  ...{read:,}/{npts:,}  voxels so far {len(gk):,}")

    cen = gs / gc[:, None]                        # centroid per voxel (x,y,z,intensity)
    out_xyzi = cen.astype(np.float32)
    N = out_xyzi.shape[0]
    header = (
        "# .PCD v0.7 - nav cloud (x y z intensity, lowercase) from make_nav_pcd.py\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {N}\nDATA binary\n"
    )
    with open(args.out, "wb") as g:
        g.write(header.encode("ascii"))
        g.write(np.ascontiguousarray(out_xyzi).tobytes())

    mn, mx = out_xyzi[:, :3].min(0), out_xyzi[:, :3].max(0)
    import os
    print("\n=== DONE ===")
    print(f"out: {args.out}  ({os.path.getsize(args.out)/1e6:.1f} MB)")
    print(f"points: {npts:,} -> {N:,}  ({npts/max(N,1):.1f}x reduction)")
    print(f"bbox X[{mn[0]:.2f},{mx[0]:.2f}] Y[{mn[1]:.2f},{mx[1]:.2f}] Z[{mn[2]:.2f},{mx[2]:.2f}]")
    print(f"intensity range [{out_xyzi[:,3].min():.1f}, {out_xyzi[:,3].max():.1f}]")
    print("FIELDS line is 'x y z intensity' (lowercase) -> loads as PointXYZ and PointXYZI.")


if __name__ == "__main__":
    main()
