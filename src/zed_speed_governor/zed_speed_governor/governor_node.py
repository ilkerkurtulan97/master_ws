#!/usr/bin/env python3
"""ZED 2i human-aware speed governor.

Runs the ZED SDK MULTI_CLASS_BOX object detector (PERSON class only) on the
front-mounted ZED 2i and publishes /speed_limit (std_msgs/Float32):

  - a PERSON inside the forward gate (gate_distance ahead, within
    corridor_half_width of the centerline) -> publish human_slow_speed
  - otherwise -> publish -1.0 (meaning "no limit")

The drive controller caps its commanded speed at fresh, non-negative values
and ignores stale/negative ones — so if this node dies or the camera drops,
the rover reverts to normal operation (fail-open). The LiDAR safety bubble
remains the actual collision guard; this layer only makes the rover careful
around people.

Distances come from the SDK's stereo-depth object positions (camera frame,
RIGHT_HANDED_Y_UP: forward = -z, left/right = x). No separate depth lookup
is needed.

With show_window: true an OpenCV window shows the left camera image with
detection boxes (RED = person in the gate, limit engaged; GREEN = seen but
outside the gate) and a status banner — put it next to RViz for demo videos.
Press q in the window to close it; the governor keeps running.

First run of a given model variant downloads + optimizes a TensorRT engine
(needs internet, takes minutes, one time only). MEDIUM was already optimized
by the July detection tests.
"""
import os
import sys

# The GNOME/AnyDesk desktop on this Jetson is X display :1. Launched from an
# SSH shell without DISPLAY, cv2.imshow would silently draw nowhere.
if not os.environ.get("DISPLAY"):
    os.environ["DISPLAY"] = ":1"
if not os.environ.get("XAUTHORITY") and os.path.exists("/run/user/1000/gdm/Xauthority"):
    os.environ["XAUTHORITY"] = "/run/user/1000/gdm/Xauthority"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import pyzed.sl as sl

MODEL_MAP = {
    "FAST": sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST,
    "MEDIUM": sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_MEDIUM,
    "ACCURATE": sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_ACCURATE,
}

NO_LIMIT = -1.0
WINDOW_NAME = "ZED Speed Governor"

RED = (0, 0, 255)
GREEN = (0, 220, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


class ZedSpeedGovernor(Node):
    def __init__(self):
        super().__init__('zed_speed_governor_node')

        # Trigger geometry: react to persons up to gate_distance AHEAD and
        # within corridor_half_width of the centerline (roughly the driving
        # corridor — a person 2 m to the side is not in the rover's way).
        self.declare_parameter('gate_distance', 2.0)
        self.declare_parameter('corridor_half_width', 1.0)
        # Speed cap while a person is in the gate. 0.2 = min_linear_speed,
        # the same crawl the controller uses on detours and tight turns.
        self.declare_parameter('human_slow_speed', 0.2)
        # Keep the cap this long after the person last qualified, so the
        # limit doesn't flicker when a detection blips at the gate boundary.
        self.declare_parameter('hold_time', 2.5)
        self.declare_parameter('confidence_threshold', 40.0)
        # Camera/model. 15 fps and MEDIUM keep GPU load moderate next to NDT.
        self.declare_parameter('camera_fps', 15)
        self.declare_parameter('detection_model', 'MEDIUM')
        # Live annotated view (for demos, next to RViz). Costs a little CPU.
        self.declare_parameter('show_window', True)

        self.gate_distance = self.get_parameter('gate_distance').value
        self.corridor_half_width = self.get_parameter('corridor_half_width').value
        self.human_slow_speed = self.get_parameter('human_slow_speed').value
        self.hold_time = self.get_parameter('hold_time').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.detection_model = self.get_parameter('detection_model').value
        self.show_window = self.get_parameter('show_window').value

        self.cv2 = None
        if self.show_window:
            try:
                import cv2
                self.cv2 = cv2
                cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WINDOW_NAME, 1280, 720)
            except Exception as e:
                self.get_logger().warn(
                    f'Display unavailable ({e}) — running without window')
                self.show_window = False

        self.pub = self.create_publisher(Float32, '/speed_limit', 10)

        self.last_trigger_time = None   # rclpy Time of last qualifying person
        self.limited = False            # for transition logging only

        self.zed = self.open_camera()

    # ------------------------------------------------------------------
    def open_camera(self):
        zed = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = self.camera_fps
        init.depth_mode = sl.DEPTH_MODE.NEURAL   # object detection needs depth
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

        self.get_logger().info('Opening ZED 2i ...')
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(
                f'Failed to open camera: {status} — is the ZED 2i on a USB 3 port?')
            raise RuntimeError(str(status))
        info = zed.get_camera_information()
        self.get_logger().info(
            f'Opened {info.camera_model} SN {info.serial_number}')

        # Object tracking across frames needs positional tracking.
        if zed.enable_positional_tracking(
                sl.PositionalTrackingParameters()) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn('Positional tracking failed to enable')

        od_params = sl.ObjectDetectionParameters()
        od_params.detection_model = MODEL_MAP[self.detection_model]
        od_params.enable_tracking = True
        od_params.enable_segmentation = False
        self.get_logger().info(
            f'Enabling object detection ({self.detection_model}) — first run of a '
            'model variant optimizes a TensorRT engine, be patient ...')
        status = zed.enable_object_detection(od_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().fatal(f'Failed to enable object detection: {status}')
            zed.close()
            raise RuntimeError(str(status))

        self.od_rt = sl.ObjectDetectionRuntimeParameters()
        self.od_rt.detection_confidence_threshold = self.confidence_threshold
        # PERSON only: cars, bags etc. are ignored entirely.
        self.od_rt.object_class_filter = [sl.OBJECT_CLASS.PERSON]

        self.get_logger().info(
            f'Governor running: person within {self.gate_distance:.1f} m ahead '
            f'(±{self.corridor_half_width:.1f} m) -> speed limit '
            f'{self.human_slow_speed:.2f} m/s, hold {self.hold_time:.1f} s')
        return zed

    # ------------------------------------------------------------------
    def spin_camera(self):
        """Blocking grab loop; publishes one /speed_limit per frame."""
        objects = sl.Objects()
        image = sl.Mat()
        grab_failures = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
                # Don't publish on camera trouble: the controller's staleness
                # timeout then lifts the cap (fail-open by design).
                grab_failures += 1
                if grab_failures % 30 == 1:
                    self.get_logger().warn(
                        f'Camera grab failing ({grab_failures} misses)')
                continue
            grab_failures = 0

            self.zed.retrieve_objects(objects, self.od_rt)
            now = self.get_clock().now()

            persons = self.classify_persons(objects)
            nearest = min((fwd for _, fwd, in_gate in persons if in_gate),
                          default=None)
            if nearest is not None:
                self.last_trigger_time = now
                if not self.limited:
                    self.limited = True
                    self.get_logger().info(
                        f'Person at {nearest:.2f} m ahead — speed limited to '
                        f'{self.human_slow_speed:.2f} m/s')

            holding = (self.last_trigger_time is not None and
                       (now - self.last_trigger_time).nanoseconds * 1e-9
                       < self.hold_time)
            if self.limited and not holding:
                self.limited = False
                self.get_logger().info('Gate clear — speed limit lifted')

            msg = Float32()
            msg.data = self.human_slow_speed if holding else NO_LIMIT
            self.pub.publish(msg)

            if self.show_window:
                self.render(image, persons, holding)

    # ------------------------------------------------------------------
    def classify_persons(self, objects):
        """[(obj, forward_dist, in_gate), ...] for all detected persons.

        Camera frame is RIGHT_HANDED_Y_UP: forward = -z, lateral = x.
        """
        out = []
        for obj in objects.object_list:
            if obj.confidence < self.confidence_threshold:
                continue
            fwd = -obj.position[2]
            lat = obj.position[0]
            if fwd <= 0.0:
                continue
            in_gate = (fwd <= self.gate_distance and
                       abs(lat) <= self.corridor_half_width)
            out.append((obj, fwd, in_gate))
        return out

    # ------------------------------------------------------------------
    def render(self, image, persons, holding):
        cv2 = self.cv2
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)

        for obj, fwd, in_gate in persons:
            bb = obj.bounding_box_2d  # 4x2 corners in pixel coords
            if bb is None or len(bb) < 4:
                continue
            xs = [int(p[0]) for p in bb]
            ys = [int(p[1]) for p in bb]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
            color = RED if in_gate else GREEN
            cap = f'PERSON {fwd:.2f}m' + ('  IN GATE' if in_gate else '')
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.rectangle(frame, (x1, y1 - 26), (x1 + 13 * len(cap), y1), color, -1)
            cv2.putText(frame, cap, (x1 + 3, y1 - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLACK, 2, cv2.LINE_AA)

        # Status banner across the top.
        h, w = frame.shape[:2]
        if holding:
            banner = f'SPEED LIMIT {self.human_slow_speed:.2f} m/s  (person < {self.gate_distance:.1f} m)'
            color = RED
        else:
            banner = 'CLEAR - full speed'
            color = GREEN
        cv2.rectangle(frame, (0, 0), (w, 40), color, -1)
        cv2.putText(frame, banner, (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2, cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME, frame)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            self.get_logger().info('Window closed (q) — governor keeps running')
            cv2.destroyAllWindows()
            self.show_window = False

    # ------------------------------------------------------------------
    def shutdown_camera(self):
        try:
            if self.cv2 is not None:
                self.cv2.destroyAllWindows()
            self.zed.disable_object_detection()
            self.zed.disable_positional_tracking()
            self.zed.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ZedSpeedGovernor()
    except RuntimeError:
        rclpy.shutdown()
        sys.exit(1)
    try:
        node.spin_camera()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_camera()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
