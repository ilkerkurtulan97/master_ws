#!/usr/bin/env python3
"""
NavMesh Navigation GUI — Open3D 3D view + tkinter control panel.

3D view : green wireframe navmesh (LineSet), orange rover sphere,
          red goal sphere, path lines, red obstacle spheres.
Panel   : X/Y/Z sliders, Snap-Z button, Drive / Clear buttons.
Picking : Move sliders to X/Y → click "Snap Z" → click "Drive".

ROS topics
  SUB  /navmesh_polys            visualization_msgs/Marker  (latched)
  SUB  /planned_path             nav_msgs/Path
  SUB  /chassis/odom             nav_msgs/Odometry
  SUB  /dynamic_obstacles_cloud  sensor_msgs/PointCloud2
  PUB  /goal_pose                geometry_msgs/PoseStamped
"""

import sys, math, struct, threading, time, traceback
import tkinter as tk
from tkinter import ttk

import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

# ── config ────────────────────────────────────────────────────────────────────
PANEL_W      = 300
PANEL_COL    = '#2a2a3e'
TEXT_COL     = '#e0e0e0'
MAX_OBS_POOL = 20
_HIDDEN      = np.array([0.0, 0.0, -200.0])


# ── navmesh Z snap (barycentric on navmesh triangles) ─────────────────────────
def navmesh_snap_z(tris: np.ndarray, wx: float, wy: float):
    """Return the highest navmesh Z at (wx, wy), or None if outside."""
    best = None
    for tri in tris:
        ax, ay = float(tri[0, 0]), float(tri[0, 1])
        bx, by = float(tri[1, 0]), float(tri[1, 1])
        cx, cy = float(tri[2, 0]), float(tri[2, 1])

        def _s(p1x, p1y, p2x, p2y, p3x, p3y):
            return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y)

        d1 = _s(wx, wy, ax, ay, bx, by)
        d2 = _s(wx, wy, bx, by, cx, cy)
        d3 = _s(wx, wy, cx, cy, ax, ay)
        if (d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0):
            continue
        denom = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(denom) < 1e-9:
            continue
        u = ((by - cy) * (wx - cx) + (cx - bx) * (wy - cy)) / denom
        v = ((cy - ay) * (wx - cx) + (ax - cx) * (wy - cy)) / denom
        z = float(u * tri[0, 2] + v * tri[1, 2] + (1 - u - v) * tri[2, 2])
        if best is None or z > best:
            best = z
    return best


# ── shared state (ROS thread → render thread) ─────────────────────────────────
class SharedState:
    def __init__(self):
        self._lk = threading.Lock()
        self.navmesh_tris = self.path_pts = self.rover_pos = self.obs_pts = None
        self.nm_new = self.path_new = self.rover_new = self.obs_new = False

    def set_navmesh(self, v):
        with self._lk: self.navmesh_tris = v; self.nm_new = True

    def set_path(self, v):
        with self._lk: self.path_pts = v;    self.path_new = True

    def set_rover(self, v):
        with self._lk: self.rover_pos = v;   self.rover_new = True

    def set_obs(self, v):
        with self._lk: self.obs_pts = v;     self.obs_new = True

    def consume(self):
        with self._lk:
            nm    = self.navmesh_tris.copy() if self.nm_new   and self.navmesh_tris is not None else None
            path  = self.path_pts.copy()     if self.path_new and self.path_pts     is not None else None
            rover = self.rover_pos.copy()    if self.rover_new and self.rover_pos   is not None else None
            obs   = self.obs_pts.copy()      if self.obs_new   and self.obs_pts     is not None else None
            self.nm_new = self.path_new = self.rover_new = self.obs_new = False
        return nm, path, rover, obs

    def get_navmesh(self):
        with self._lk:
            return self.navmesh_tris.copy() if self.navmesh_tris is not None else None


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class NavNode(Node):
    def __init__(self, state: SharedState):
        super().__init__('navmesh_gui')
        self._st = state
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Marker,      '/navmesh_polys',           self._cb_nm,   latched)
        self.create_subscription(Path,        '/planned_path',            self._cb_path, 10)
        self.create_subscription(Odometry,    '/chassis/odom',            self._cb_odom, 10)
        self.create_subscription(PointCloud2, '/dynamic_obstacles_cloud', self._cb_obs,  10)
        self._pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.get_logger().info('navmesh_gui ready')

    def _cb_nm(self, msg: Marker):
        if msg.type != Marker.TRIANGLE_LIST: return
        n = len(msg.points)
        if n < 3 or n % 3: return
        v = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        self._st.set_navmesh(v.reshape(-1, 3, 3))
        self.get_logger().info(f'navmesh: {n // 3} tris')

    def _cb_path(self, msg: Path):
        if not msg.poses:
            self._st.set_path(np.empty((0, 3), dtype=np.float32)); return
        self._st.set_path(np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                     for p in msg.poses], dtype=np.float32))

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._st.set_rover(np.array([p.x, p.y, p.z], dtype=np.float32))

    def _cb_obs(self, msg: PointCloud2):
        self._st.set_obs(_parse_pc2(msg))

    def publish_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self.get_logger().info(f'goal → ({x:.2f}, {y:.2f}, {z:.2f})')


def _parse_pc2(msg: PointCloud2) -> np.ndarray:
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        return np.empty((0, 3), dtype=np.float32)
    ox, oy, oz = fields['x'].offset, fields['y'].offset, fields['z'].offset
    step, data = msg.point_step, msg.data
    buf = []
    for i in range(min(msg.width * msg.height, 2000)):
        b = i * step
        x = struct.unpack_from('<f', data, b + ox)[0]
        y = struct.unpack_from('<f', data, b + oy)[0]
        z = struct.unpack_from('<f', data, b + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            buf.append((x, y, z))
    return np.array(buf, dtype=np.float32) if buf else np.empty((0, 3), dtype=np.float32)


# ── Open3D geometry helpers ───────────────────────────────────────────────────
def _make_wireframe(tris: np.ndarray) -> o3d.geometry.LineSet:
    """(N,3,3) float32 → green LineSet wireframe (same as co-supervisor)."""
    verts = tris.reshape(-1, 3).astype(np.float64)
    n = len(tris)
    idx = np.arange(n * 3).reshape(-1, 3)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(idx.astype(np.int32))
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color([0.0, 0.85, 0.35])
    return ls


def _make_path_ls(pts: np.ndarray) -> o3d.geometry.LineSet:
    ls = o3d.geometry.LineSet()
    if pts is None or len(pts) < 2:
        return ls
    ls.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    ls.lines  = o3d.utility.Vector2iVector(
        np.array([[i, i + 1] for i in range(len(pts) - 1)]))
    ls.paint_uniform_color([0.1, 0.5, 1.0])   # blue path
    return ls


def _move_sphere(mesh, base_verts, pos: np.ndarray):
    mesh.vertices = o3d.utility.Vector3dVector(base_verts.copy())
    mesh.translate(pos.astype(np.float64))


# ── Application ───────────────────────────────────────────────────────────────
class App:
    def __init__(self, state: SharedState, node: NavNode):
        self._state   = state
        self._node    = node
        self._running = True

        self._gx = self._gy = self._gz = 0.0
        self._has_goal = False

        self._last_rover_t = self._last_obs_t = 0.0
        self._nm_added = False

        # ── tkinter panel (create first to avoid X11/GLFW conflict) ───────────
        self._root = tk.Tk()
        self._root.title('NavMesh Control Panel')
        self._root.configure(bg=PANEL_COL)
        self._root.geometry(f'{PANEL_W}x780+0+0')
        self._root.resizable(False, True)
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._build_panel()

        # ── Open3D Visualizer ─────────────────────────────────────────────────
        self._vis = o3d.visualization.VisualizerWithKeyCallback()
        self._vis.create_window('NavMesh 3D View', 960, 720, left=PANEL_W + 10)

        ropt = self._vis.get_render_option()
        ropt.background_color = np.array([0.05, 0.05, 0.12])
        ropt.point_size = 6.0
        try:
            ropt.line_width = 1.5
        except Exception:
            pass

        # rover sphere (orange)
        self._rover_sph  = o3d.geometry.TriangleMesh.create_sphere(radius=0.30)
        self._rover_base = np.asarray(self._rover_sph.vertices).copy()
        self._rover_sph.paint_uniform_color([1.0, 0.55, 0.0])
        self._rover_sph.compute_vertex_normals()
        self._rover_sph.translate(_HIDDEN)
        self._vis.add_geometry(self._rover_sph)

        # goal sphere (red)
        self._goal_sph  = o3d.geometry.TriangleMesh.create_sphere(radius=0.35)
        self._goal_base = np.asarray(self._goal_sph.vertices).copy()
        self._goal_sph.paint_uniform_color([0.9, 0.1, 0.1])
        self._goal_sph.compute_vertex_normals()
        self._goal_sph.translate(_HIDDEN)
        self._vis.add_geometry(self._goal_sph)

        # path lineset
        self._path_ls = o3d.geometry.LineSet()
        self._vis.add_geometry(self._path_ls)

        # obstacle pool
        _proto = o3d.geometry.TriangleMesh.create_sphere(radius=0.22)
        self._obs_base_v = np.asarray(_proto.vertices).copy()
        self._obs_base_t = np.asarray(_proto.triangles).copy()
        self._obs_pool = []
        for _ in range(MAX_OBS_POOL):
            m = o3d.geometry.TriangleMesh()
            m.vertices  = o3d.utility.Vector3dVector(self._obs_base_v.copy())
            m.triangles = o3d.utility.Vector3iVector(self._obs_base_t.copy())
            m.paint_uniform_color([1.0, 0.2, 0.2])
            m.compute_vertex_normals()
            m.translate(_HIDDEN)
            self._vis.add_geometry(m)
            self._obs_pool.append(m)

        # navmesh wireframe (added lazily on first navmesh message)
        self._nm_wire = None
        self._nm_center = np.zeros(3, dtype=np.float64)

        # 'R' key resets to the top-down overview camera
        self._vis.register_key_callback(ord('R'), self._cb_reset_view)

    # ── panel build ───────────────────────────────────────────────────────────
    def _L(self, p, text, sz=11, bold=False):
        return tk.Label(p, text=text, bg=PANEL_COL, fg=TEXT_COL,
                        font=('monospace', sz, 'bold' if bold else ''))

    def _sep(self, p):
        tk.Frame(p, bg='#444466', height=1).pack(fill=tk.X, padx=8, pady=4)

    def _build_panel(self):
        p = self._root
        self._L(p, 'NavMesh Planner 3D', 13, bold=True).pack(pady=(14, 4))
        self._sep(p)

        self._lbl_nm    = self._L(p, 'Map:    waiting…')
        self._lbl_rover = self._L(p, 'Rover:  waiting…')
        self._lbl_goal  = self._L(p, 'Goal:   not set')
        for w in (self._lbl_nm, self._lbl_rover, self._lbl_goal):
            w.pack(anchor='w', padx=12, pady=1)

        self._sep(p)
        self._L(p, 'Target Position', 11, bold=True).pack(pady=(4, 0))

        for axis in ('X', 'Y', 'Z'):
            self._L(p, f'Target {axis}', bold=True).pack(anchor='w', padx=12, pady=(6, 0))
            var = tk.DoubleVar(value=0.0)
            setattr(self, f'_var_{axis.lower()}', var)
            lbl = self._L(p, '0.00')
            lbl.pack(anchor='e', padx=14)
            setattr(self, f'_lbl_{axis.lower()}', lbl)
            lo, hi = (-60.0, 60.0) if axis != 'Z' else (-6.0, 9.0)
            ttk.Scale(p, orient=tk.HORIZONTAL, from_=lo, to=hi, variable=var,
                      command=lambda v, a=axis: self._on_slider(a, float(v))
                      ).pack(fill=tk.X, padx=12, pady=(0, 2))

        self._sep(p)
        tk.Button(p, text='Snap Z to Navmesh', command=self._snap_z,
                  bg='#3a3a5a', fg=TEXT_COL, activebackground='#5a5a7a',
                  font=('monospace', 11), relief=tk.FLAT, padx=8, pady=6,
                  cursor='hand2').pack(fill=tk.X, padx=12, pady=4)
        self._sep(p)

        self._btn_drive = tk.Button(p, text='Drive', command=self._drive,
                                    bg='#1a6e2e', fg='white', activebackground='#22993e',
                                    font=('monospace', 13, 'bold'), relief=tk.FLAT,
                                    padx=8, pady=10, cursor='hand2', state=tk.DISABLED)
        self._btn_drive.pack(fill=tk.X, padx=12, pady=4)

        tk.Button(p, text='Clear Goal', command=self._clear,
                  bg='#6e1a1a', fg='white', activebackground='#993a3a',
                  font=('monospace', 12), relief=tk.FLAT, padx=8, pady=8,
                  cursor='hand2').pack(fill=tk.X, padx=12, pady=4)

        self._sep(p)
        for col, lbl in (
            ('#00dd55', 'Driveable area (wireframe)'),
            ('#ff8c00', 'Rover position'),
            ('#ee2222', 'Goal position'),
            ('#ff4444', 'Dynamic obstacles'),
            ('#11ff44', 'Planned path'),
        ):
            row = tk.Frame(p, bg=PANEL_COL)
            row.pack(fill=tk.X, padx=12, pady=1)
            c = tk.Canvas(row, width=14, height=14, bg=PANEL_COL, highlightthickness=0)
            c.create_oval(2, 2, 12, 12, fill=col, outline='')
            c.pack(side=tk.LEFT)
            tk.Label(row, text=f'  {lbl}', bg=PANEL_COL, fg=TEXT_COL,
                     font=('monospace', 9)).pack(side=tk.LEFT)

        self._sep(p)
        for tip in ('Goal:',
                    '  1. Set X/Y sliders to target',
                    '  2. Click "Snap Z to Navmesh"',
                    '  3. Click "Drive"',
                    '3D view controls:',
                    '  Left-drag  → rotate',
                    '  Right-drag → pan',
                    '  Scroll     → zoom',
                    '  R key      → reset view'):
            self._L(p, tip, 9).pack(anchor='w', padx=12)

    # ── Open3D update helpers ─────────────────────────────────────────────────
    def _add_navmesh(self, tris: np.ndarray):
        if self._nm_added:
            return
        self._nm_wire = _make_wireframe(tris)
        self._vis.add_geometry(self._nm_wire)
        self._nm_added = True

        # Compute center for orbit camera
        pts = tris.reshape(-1, 3)
        self._nm_center = pts.mean(axis=0).astype(np.float64)

        # Set a Z-up perspective view (like RViz default) — tilted 40° from above
        vc = self._vis.get_view_control()
        vc.set_lookat(self._nm_center)
        vc.set_up([0.0, 0.0, 1.0])          # Z is world-up (ROS convention)
        vc.set_front([0.0, -0.75, -0.66])    # looking from above-front into the scene
        vc.set_zoom(0.35)

        self._lbl_nm.config(text=f'Map: {len(tris)} tris ✓')
        print(f'[GUI] navmesh {len(tris)} tris loaded')

    def _cb_reset_view(self, vis):
        vc = vis.get_view_control()
        vc.set_lookat(self._nm_center)
        vc.set_up([0.0, 0.0, 1.0])
        vc.set_front([0.0, -0.75, -0.66])
        vc.set_zoom(0.35)
        return False

    def _upd_rover(self, pos: np.ndarray):
        _move_sphere(self._rover_sph, self._rover_base, pos)
        self._vis.update_geometry(self._rover_sph)

    def _upd_goal_marker(self):
        _move_sphere(self._goal_sph, self._goal_base,
                     np.array([self._gx, self._gy, self._gz]))
        self._vis.update_geometry(self._goal_sph)

    def _hide_goal_marker(self):
        _move_sphere(self._goal_sph, self._goal_base, _HIDDEN)
        self._vis.update_geometry(self._goal_sph)

    def _upd_path(self, pts: np.ndarray):
        self._vis.remove_geometry(self._path_ls, reset_bounding_box=False)
        self._path_ls = _make_path_ls(pts)
        self._vis.add_geometry(self._path_ls, reset_bounding_box=False)

    def _upd_obstacles(self, pts: np.ndarray):
        n = min(len(pts), MAX_OBS_POOL) if (pts is not None and len(pts)) else 0
        for i, m in enumerate(self._obs_pool):
            pos = pts[i].astype(np.float64) if i < n else _HIDDEN
            _move_sphere(m, self._obs_base_v, pos)
            self._vis.update_geometry(m)

    # ── goal / slider callbacks ───────────────────────────────────────────────
    def _set_goal(self, x, y, z):
        self._gx, self._gy, self._gz = float(x), float(y), float(z)
        self._has_goal = True
        self._sync_sliders()
        self._upd_goal_marker()
        self._upd_goal_lbl()
        self._btn_drive.config(state=tk.NORMAL)

    def _on_slider(self, axis, v):
        if   axis == 'X': self._gx = v; self._lbl_x.config(text=f'{v:.2f}')
        elif axis == 'Y': self._gy = v; self._lbl_y.config(text=f'{v:.2f}')
        else:             self._gz = v; self._lbl_z.config(text=f'{v:.2f}')
        if self._has_goal or axis == 'Z':
            self._has_goal = True
            self._upd_goal_marker()
            self._upd_goal_lbl()
            self._btn_drive.config(state=tk.NORMAL)

    def _snap_z(self):
        tris = self._state.get_navmesh()
        if tris is None:
            return
        z = navmesh_snap_z(tris, self._gx, self._gy)
        if z is None:
            print(f'[GUI] snap Z: no navmesh polygon at ({self._gx:.2f}, {self._gy:.2f})')
            return
        self._gz = z
        self._var_z.set(max(-6.0, min(9.0, z)))
        self._lbl_z.config(text=f'{z:.2f}')
        self._has_goal = True
        self._upd_goal_marker()
        self._upd_goal_lbl()
        self._btn_drive.config(state=tk.NORMAL)
        print(f'[GUI] snapped Z={z:.3f} at ({self._gx:.2f}, {self._gy:.2f})')

    def _drive(self):
        if self._has_goal:
            self._node.publish_goal(self._gx, self._gy, self._gz)

    def _clear(self):
        self._has_goal = False
        self._gx = self._gy = self._gz = 0.0
        for a in ('x', 'y', 'z'):
            getattr(self, f'_var_{a}').set(0.0)
            getattr(self, f'_lbl_{a}').config(text='0.00')
        self._lbl_goal.config(text='Goal:   not set')
        self._hide_goal_marker()
        self._btn_drive.config(state=tk.DISABLED)

    def _sync_sliders(self):
        self._var_x.set(max(-60.0, min(60.0, self._gx)))
        self._var_y.set(max(-60.0, min(60.0, self._gy)))
        self._var_z.set(max(-6.0,  min(9.0,  self._gz)))
        self._lbl_x.config(text=f'{self._gx:.2f}')
        self._lbl_y.config(text=f'{self._gy:.2f}')
        self._lbl_z.config(text=f'{self._gz:.2f}')

    def _upd_goal_lbl(self):
        self._lbl_goal.config(
            text=f'Goal: ({self._gx:.1f}, {self._gy:.1f}, {self._gz:.2f})')

    # ── process incoming ROS data ─────────────────────────────────────────────
    def _process_state(self):
        try:
            nm, path, rover, obs = self._state.consume()
            now = time.time()

            if nm is not None:
                self._add_navmesh(nm)

            if path is not None:
                self._upd_path(path)

            if rover is not None and now - self._last_rover_t > 0.1:
                self._upd_rover(rover)
                self._last_rover_t = now
                self._lbl_rover.config(
                    text=f'Rover: ({rover[0]:.1f}, {rover[1]:.1f}, {rover[2]:.2f})')

            if obs is not None and now - self._last_obs_t > 0.5:
                self._upd_obstacles(obs)
                self._last_obs_t = now
        except Exception:
            traceback.print_exc()

    # ── main loop ─────────────────────────────────────────────────────────────
    def _on_close(self):
        self._running = False
        try: self._vis.destroy_window()
        except Exception: pass
        try: self._root.destroy()
        except Exception: pass

    def run(self):
        while self._running:
            try:
                if not self._vis.poll_events():
                    self._on_close()
                    break
                self._vis.update_renderer()
                self._process_state()
                self._root.update()
            except tk.TclError:
                self._on_close()
                break
            except Exception:
                traceback.print_exc()


# ── entry point ───────────────────────────────────────────────────────────────
def main():
    rclpy.init(args=sys.argv)
    state = SharedState()
    node  = NavNode(state)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = App(state, node)
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
