#!/usr/bin/env python3

"""KRAI camera detection node.

ROS2 node that wraps YOLO-based object detection for the KRAI robot.
Detects 'Kotak' (box) and 'Bounding' (rack) objects, computes relative
coordinates, distance estimation, and grid occupancy.

All detection and drawing logic is adopted 1:1 from the standalone script.
"""

import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from krai_interfaces.msg import CameraDetection, DetectedObject

from krai_camera.grid_utils import (
    draw_grid_3x3,
    draw_center_axis,
    get_grid_cell,
    count_objects_in_grid,
)

# Try to import cv_bridge for image publishing.
# If not available, fall back to manual conversion.
try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except ImportError:
    _HAS_CV_BRIDGE = False

# Try to import ultralytics YOLO.
try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False


class CameraDetectionNode(Node):
    """ROS2 node for YOLO-based camera object detection.

    Publishers:
        /camera/detections      (CameraDetection)  - Detection results per frame
        /camera/annotated_image (sensor_msgs/Image) - Annotated frame for debug

    Parameters:
        camera_source          (str)   - Camera device index or video file path
        model_path             (str)   - Path to YOLO model directory
        frame_width            (int)   - Frame resize width
        frame_height           (int)   - Frame resize height
        confidence_threshold   (float) - Minimum YOLO confidence
        imgsz                  (int)   - YOLO inference image size
        tinggi_box             (float) - Real box height in cm
        focal_length           (float) - Camera focal length
        jarak_calibrasi        (float) - Calibration distance in cm
        enable_rak_grid        (bool)  - Enable rack grid mode
        show_gui               (bool)  - Show cv2.imshow window
        publish_image          (bool)  - Publish annotated image topic
        timer_period           (float) - Timer period in seconds (~FPS)
    """

    def __init__(self):
        super().__init__('krai_camera_node')

        # -------------------------
        # Declare parameters
        # -------------------------
        self.declare_parameter('camera_source', 1)
        self.declare_parameter('model_path', '')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 360)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('tinggi_box', 35.0)
        self.declare_parameter('focal_length', 3.7)
        self.declare_parameter('jarak_calibrasi', 50.0)
        self.declare_parameter('enable_rak_grid', False)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('timer_period', 0.033)

        # -------------------------
        # Read parameters
        # -------------------------
        self.camera_source = int(self.get_parameter('camera_source').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.tinggi_box = float(self.get_parameter('tinggi_box').value)
        self.focal_length = float(self.get_parameter('focal_length').value)
        self.jarak_calibrasi = float(self.get_parameter('jarak_calibrasi').value)
        self.enable_rak_grid = bool(self.get_parameter('enable_rak_grid').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.publish_image = bool(self.get_parameter('publish_image').value)
        timer_period = float(self.get_parameter('timer_period').value)

        # -------------------------
        # Publishers
        # -------------------------
        self.detection_pub = self.create_publisher(CameraDetection, '/camera/detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/annotated_image', 10)

        # -------------------------
        # cv_bridge
        # -------------------------
        self.cv_bridge = None
        if _HAS_CV_BRIDGE:
            self.cv_bridge = CvBridge()
        elif self.publish_image:
            self.get_logger().warn(
                'cv_bridge not available. Install with: sudo apt install ros-humble-cv-bridge. '
                'Image publishing will use manual conversion.'
            )

        # -------------------------
        # Load YOLO model
        # -------------------------
        self.model = None
        if not _HAS_YOLO:
            self.get_logger().error(
                'ultralytics not installed. Install with: pip install ultralytics. '
                'Camera node will run without detection.'
            )
        elif not self.model_path:
            self.get_logger().warn(
                'model_path parameter is empty. Set it to YOLO model path. '
                'Camera node will run without detection.'
            )
        else:
            try:
                self.model = YOLO(self.model_path, task='detect')
                self.get_logger().info(f'YOLO model loaded from: {self.model_path}')
            except Exception as exc:
                self.get_logger().error(f'Failed to load YOLO model: {exc}')

        # -------------------------
        # Open camera / video
        # -------------------------
        self.cap = None
        self._open_camera()

        # -------------------------
        # FPS tracking
        # -------------------------
        self.prev_time = 0.0

        # -------------------------
        # Timer for frame processing
        # -------------------------
        self.timer = self.create_timer(timer_period, self._process_frame)

        self.get_logger().info('KRAI camera detection node started')
        self.get_logger().info(f'Camera source: {self.camera_source}')
        self.get_logger().info(f'Frame size: {self.frame_width}x{self.frame_height}')
        self.get_logger().info(f'Rak grid mode: {self.enable_rak_grid}')
        self.get_logger().info(f'Show GUI: {self.show_gui}')
        self.get_logger().info(f'Publish image: {self.publish_image}')

    def _open_camera(self):
        """Open camera by device index."""
        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera index: {self.camera_source}. '
                'Check device connection.'
            )
            self.cap = None

    def _process_frame(self):
        """Main timer callback: capture, detect, annotate, publish."""

        # Re-open camera if closed
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            if self.cap is None:
                return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera', throttle_duration_sec=5.0)
            return

        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        frame_h, frame_w = frame.shape[:2]

        # ===================================
        # Titik tengah kamera (robot)
        # ===================================
        center_frame_x = frame_w // 2
        center_frame_y = frame_h // 2

        # garis silang horizontal
        cv2.line(
            frame,
            (0, center_frame_y),
            (frame_w, center_frame_y),
            (0, 255, 255),
            2
        )

        # garis silang vertikal
        cv2.line(
            frame,
            (center_frame_x, 0),
            (center_frame_x, frame_h),
            (0, 255, 255),
            2
        )

        # titik tengah kamera
        cv2.circle(
            frame,
            (center_frame_x, center_frame_y),
            7,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            frame,
            "(0,0)",
            (center_frame_x + 10, center_frame_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        # ===================================
        # YOLO Detection
        # ===================================
        bounding_box = None
        kotak_list = []
        occupied_cells = []
        detected_objects = []

        if self.model is not None:
            results = self.model(frame, imgsz=self.imgsz, conf=self.confidence_threshold)
            frame = results[0].plot()

            for r in results:
                boxes = r.boxes

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    conf = float(box.conf[0])

                    if class_name == "Bounding":
                        bounding_box = (x1, y1, x2, y2)

                    elif class_name == "Kotak":
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        # titik tengah objek
                        cv2.circle(frame, (cx, cy), 7, (255, 0, 255), -1)

                        # hubungkan pusat kamera ke objek
                        cv2.line(
                            frame,
                            (center_frame_x, center_frame_y),
                            (cx, cy),
                            (0, 0, 255),
                            2
                        )

                        # koordinat relatif terhadap tengah robot
                        dx = cx - center_frame_x
                        dy = center_frame_y - cy

                        # estimasi jarak cm
                        tinggi_pixel = y2 - y1
                        if tinggi_pixel > 0:
                            distance_cm = (self.focal_length * self.tinggi_box) / tinggi_pixel
                            distance_cm = self.jarak_calibrasi - distance_cm
                        else:
                            distance_cm = 0.0

                        cv2.putText(
                            frame,
                            f"X={dx} Y={dy}",
                            (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 255),
                            2
                        )

                        cv2.putText(
                            frame,
                            f"D={distance_cm:.1f} cm",
                            (cx + 10, cy + 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2
                        )

                        self.get_logger().debug(
                            f'Koordinat = ({dx},{dy}) Jarak = {distance_cm:.1f} cm'
                        )

                        kotak_list.append((cx, cy))

                        # Build DetectedObject message
                        obj_msg = DetectedObject()
                        obj_msg.class_name = class_name
                        obj_msg.x1 = x1
                        obj_msg.y1 = y1
                        obj_msg.x2 = x2
                        obj_msg.y2 = y2
                        obj_msg.center_x = cx
                        obj_msg.center_y = cy
                        obj_msg.confidence = conf
                        obj_msg.dx = dx
                        obj_msg.dy = dy
                        obj_msg.distance_cm = distance_cm
                        obj_msg.grid_cell = 0  # updated below if bounding exists
                        detected_objects.append(obj_msg)

        # ===================================
        # FPS
        # ===================================
        current_time = time.time()
        fps = 1.0 / max(current_time - self.prev_time, 0.0001)
        self.prev_time = current_time

        cv2.putText(
            frame,
            f"FPS: {int(fps)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # ===================================
        # Rak Grid Mode
        # ===================================
        grid_count_array = [0] * 9

        if self.enable_rak_grid and bounding_box is not None:

            bx1, by1, bx2, by2 = bounding_box

            # gambar sumbu X dan Y rak
            center_rak_x, center_rak_y = draw_center_axis(
                frame,
                bx1,
                by1,
                bx2,
                by2
            )

            # posisi kotak terhadap grid
            for idx, (cx, cy) in enumerate(kotak_list):

                cell = get_grid_cell(cx, cy, bx1, by1, bx2, by2)

                if cell is not None:
                    occupied_cells.append(cell)

                # Update grid_cell in corresponding DetectedObject
                if idx < len(detected_objects):
                    detected_objects[idx].grid_cell = cell if cell is not None else 0

                # garis dari pusat rak ke objek
                cv2.line(
                    frame,
                    (center_rak_x, center_rak_y),
                    (cx, cy),
                    (0, 0, 255),
                    2
                )

                # offset terhadap pusat rak
                dx = cx - center_rak_x
                dy = cy - center_rak_y

                cv2.putText(
                    frame,
                    f"({dx},{dy})",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2
                )

            grid_data = count_objects_in_grid(
                bounding_box,
                kotak_list
            )

            self.get_logger().debug(f'Grid data: {grid_data}')

            # Convert dict {1:n, 2:n, ...} to array [n, n, ...]
            for i in range(1, 10):
                grid_count_array[i - 1] = grid_data.get(i, 0)

            draw_grid_3x3(
                frame,
                bx1,
                by1,
                bx2,
                by2,
                occupied_cells
            )

        # ===================================
        # Publish CameraDetection message
        # ===================================
        det_msg = CameraDetection()
        det_msg.stamp = self.get_clock().now().to_msg()
        det_msg.objects = detected_objects
        det_msg.occupied_cells = occupied_cells
        det_msg.grid_count = grid_count_array
        det_msg.bounding_detected = bounding_box is not None
        if bounding_box is not None:
            det_msg.bounding_x1 = bounding_box[0]
            det_msg.bounding_y1 = bounding_box[1]
            det_msg.bounding_x2 = bounding_box[2]
            det_msg.bounding_y2 = bounding_box[3]
        det_msg.frame_width = frame_w
        det_msg.frame_height = frame_h
        det_msg.fps = fps

        self.detection_pub.publish(det_msg)

        # ===================================
        # Publish annotated image
        # ===================================
        if self.publish_image:
            if self.cv_bridge is not None:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            else:
                img_msg = self._cv2_to_imgmsg_manual(frame)
            img_msg.header.stamp = det_msg.stamp
            img_msg.header.frame_id = 'camera_link'
            self.image_pub.publish(img_msg)

        # ===================================
        # Show GUI window
        # ===================================
        if self.show_gui:
            cv2.imshow("KRAI YOLO Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('GUI window closed by user (q key)')
                self.show_gui = False
                cv2.destroyAllWindows()

    @staticmethod
    def _cv2_to_imgmsg_manual(cv_image):
        """Manual conversion from OpenCV image to sensor_msgs/Image.

        Fallback when cv_bridge is not installed.
        """
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = cv_image.shape[1] * 3
        msg.data = cv_image.tobytes()
        return msg

    def destroy_node(self):
        """Clean up camera and GUI on shutdown."""
        if self.cap is not None:
            self.cap.release()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
