import h5py, shutil
shutil.copy("/home/ilker/meshnavigation_ws/blender_welded_mesh.h5", "/tmp/c.h5")
f = h5py.File("/tmp/c.h5", "r")
v = f["/Low_Bitlab/geometry/vertices"].shape[0]
fa = f["/Low_Bitlab/geometry/face_indices"].shape[0]
print("verts", v, "faces", fa, "V/F=%.3f" % (v / fa),
      "-> GOOD" if v / fa < 1 else "-> STILL EXPLODED")
