import airsim
import cv2
import time
import math
import numpy as np
import json

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff
print("[INFO] Taking off...")
client.takeoffAsync().join()
time.sleep(2)  # Allow stabilization

# Grid path parameters
grid_size = 3  # Number of grid rows
step_distance = 5  # Step distance in meters
altitude = -6  # Fixed flying altitude
collision_log = []

# Movement directions for grid path (right → down → left → down → repeat)
directions = [(1, 0), (0, 1), (-1, 0), (0, 1)]  
current_direction = 0
current_position = [0, 0]

print("[INFO] Starting grid-based inspection...")

for i in range(grid_size * 2):  # Loop through the grid pattern
    dx, dy = directions[current_direction]
    target_x = current_position[0] + dx * step_distance
    target_y = current_position[1] + dy * step_distance

    # Move to the next grid point
    print(f"[INFO] Moving to Grid Point: ({target_x}, {target_y})")
    client.moveToPositionAsync(target_x, target_y, altitude, 3).join()

    # Capture front camera image
    response = client.simGetImage("front_center", airsim.ImageType.Scene)
    if response:
        img_array = np.frombuffer(response, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        if frame is not None:
            # Convert to grayscale and detect edges
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            obstacle_detected = any(cv2.contourArea(c) > 500 for c in contours)

            # Draw contours on the image
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.imshow("Drone Vision", frame)
            cv2.waitKey(1)

            # Log collision if an obstacle is detected
            if obstacle_detected:
                print("[ALERT] Obstacle detected! Logging GPS position.")

                gps_data = client.getGpsData()
                collision_point = {
                    "latitude": gps_data.gnss.geo_point.latitude,
                    "longitude": gps_data.gnss.geo_point.longitude,
                    "altitude": gps_data.gnss.geo_point.altitude,
                    "time": time.strftime("%Y-%m-%d %H:%M:%S")
                }
                collision_log.append(collision_point)
                
                # Stop and hover
                client.hoverAsync().join()
                break  

    # Update position and direction for grid movement
    current_position = [target_x, target_y]
    if (i + 1) % grid_size == 0:
        current_direction = (current_direction + 1) % len(directions)  # Change direction

# Land and cleanup
print("[INFO] Landing...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()

# Save collision log
if collision_log:
    with open("collision_report.json", "w") as file:
        json.dump(collision_log, file, indent=4)
    print("[INFO] Collision log saved to collision_report.json")

print("[INFO] Mission complete!")

