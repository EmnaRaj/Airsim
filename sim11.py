import airsim
import time
import cv2
import numpy as np
import os
import json
import math
import torch

class DroneExplorer:
    def __init__(self):
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        print("[INFO] Connecting to AirSim...")
        self.client.confirmConnection()
        time.sleep(2)  # Ensure AirSim has time to initialize

        # Enable API Control
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print("[INFO] API Control Enabled.")

        # Video and Image Saving
        self.video_filename = "drone_footage.avi"
        self.fps = 20
        self.video_writer = None
        self.image_data = []
        self.image_folder = "recorded_images"
        os.makedirs(self.image_folder, exist_ok=True)

        # Load YOLOv5 Model
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s", force_reload=True)
        self.model.conf = 0.5  # Confidence Threshold
        self.class_names = self.model.names  # Get class names

    def set_camera_orientation(self):
        """Set the camera to face downward."""
        self.client.simSetCameraPose(
            "0",
            airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(-math.pi / 2, 0, 0))
        )
        print("[INFO] Camera orientation set.")

    def takeoff(self):
        """Take off the drone and start video recording."""
        print("[INFO] Taking off...")
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-10, velocity=2).join()
        print("[INFO] Drone reached -10m altitude.")

        # Start video recording
        self.start_video_recording()

    def start_video_recording(self):
        """Initialize video recording."""
        response = self.client.simGetImage("0", airsim.ImageType.Scene)
        if response is None:
            print("[ERROR] No image received from AirSim!")
            return

        img = self.decode_image(response)
        if img is None:
            print("[ERROR] Failed to decode image!")
            return

        height, width, _ = img.shape
        self.video_writer = cv2.VideoWriter(self.video_filename, cv2.VideoWriter_fourcc(*"XVID"), self.fps, (width, height))
        print(f"[INFO] Recording video: {self.video_filename}")

    def decode_image(self, response):
        """Convert AirSim image response to OpenCV format."""
        if response is None or len(response) == 0:
            return None
        img = np.frombuffer(response, dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        return img

    def detect_objects(self, img):
        """Run YOLO object detection on the image."""
        if img is None:
            return []

        results = self.model(img, size=640, augment=True)  # Improved detection
        detections = results.xyxy[0].cpu().numpy()
        filtered_detections = []

        for detection in detections:
            x1, y1, x2, y2, confidence, class_id = detection
            if confidence > 0.5:  # Filter low-confidence detections
                class_id = int(class_id)
                class_name = self.class_names[class_id]  # Get class name from YOLO
                filtered_detections.append({
                    "class_id": class_id,
                    "class_name": class_name,
                    "bounding_box": [float(x1), float(y1), float(x2), float(y2)],
                    "confidence": float(confidence)
                })

        return filtered_detections

    def capture_frame(self):
        """Capture video frame, perform object detection, and store GPS data."""
        response = self.client.simGetImage("0", airsim.ImageType.Scene)
        gps = self.client.getGpsData().gnss.geo_point  # Get GPS coordinates
        timestamp = time.time()

        if response:
            img = self.decode_image(response)
            if img is None:
                print("[ERROR] Image decoding failed!")
                return

            self.video_writer.write(img)  # Save frame to video

            # Perform YOLO object detection
            detections = self.detect_objects(img)

            # Save frame as image
            img_filename = f"{self.image_folder}/frame_{int(timestamp)}.jpg"
            cv2.imwrite(img_filename, img)

            # Prepare metadata
            frame_metadata = {
                "timestamp": timestamp,
                "gps": {
                    "latitude": gps.latitude,
                    "longitude": gps.longitude,
                    "altitude": gps.altitude
                },
                "detections": detections  # List of detected objects
            }

            # Save metadata
            self.image_data.append(frame_metadata)

            print(f"[INFO] Frame saved: {img_filename}")
            if detections:
                print(f"[INFO] Objects detected: {detections}")

    def explore_area(self):
        """Fly the drone in a circular path around its starting position."""
        print("[INFO] Exploring area in circular path...")

        center_x, center_y = 0, 0  # Start position
        radius = 100  # 100m radius
        num_waypoints = 12  # Divide the circle into 12 points

        for i in range(num_waypoints + 1):
            angle = (i / num_waypoints) * 2 * math.pi
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)

            print(f"[INFO] Moving to ({x:.2f}, {y:.2f})")
            self.client.moveToPositionAsync(x, y, -10, 5).join()
            self.capture_frame()

        print("[INFO] Returning to start position...")
        self.client.moveToPositionAsync(center_x, center_y, -10, 5).join()
        self.capture_frame()

    def land(self):
        """Land the drone and stop recording."""
        print("[INFO] Landing...")
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

        # Stop video recording
        if self.video_writer:
            self.video_writer.release()

        print(f"[INFO] Video saved: {self.video_filename}")

        # Save metadata as JSON
        with open("image_metadata.json", "w") as json_file:
            json.dump(self.image_data, json_file, indent=4)

        print("[INFO] Image metadata saved in JSON.")

    def run(self):
        """Execute full drone mission."""
        self.set_camera_orientation()
        self.takeoff()
        self.explore_area()
        self.land()

if __name__ == "__main__":
    explorer = DroneExplorer()
    explorer.run()

