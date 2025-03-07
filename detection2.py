import airsim
import numpy as np
import cv2
import time

# Drone parameters
ALTITUDE =10 # meters
GRID_SIZE = 10  # Number of waypoints per row/column
STEP_SIZE = 5  # Meters between waypoints

def takeoff(client):
    """ Command the drone to take off """
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    
    print("Taking off...")
    client.takeoffAsync().join()
    
    print(f"Reaching altitude {ALTITUDE}m...")
    client.moveToZAsync(-ALTITUDE, 2).join()
    time.sleep(2)
    print("Hovering at target altitude.")

def fly_grid_and_capture(client):
    """ Move in a grid pattern and capture images """
    start_position = client.getMultirotorState().kinematics_estimated.position
    start_x, start_y = start_position.x_val, start_position.y_val
    
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            target_x = start_x + (i * STEP_SIZE)
            target_y = start_y + (j * STEP_SIZE if i % 2 == 0 else (GRID_SIZE - 1 - j) * STEP_SIZE)
            
            print(f"Moving to waypoint ({i}, {j}) -> X: {target_x}, Y: {target_y}")
            client.moveToPositionAsync(target_x, target_y, -ALTITUDE, 2).join()
            time.sleep(2)  # Pause before capturing image
            capture_air_sim_image(client)
    
    print("Grid inspection completed.")

def capture_air_sim_image(client):
    """ Capture images using AirSim camera """
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    if img1d.size == 0:
        print("Error: No image data received from AirSim!")
        return
    
    img = img1d.reshape(responses[0].height, responses[0].width, 3)
    cv2.imshow("Drone View", img)
    cv2.waitKey(1)

def main():
    """ Main function to control drone """
    client = airsim.MultirotorClient()
    takeoff(client)
    fly_grid_and_capture(client)
    print("Mission completed.")
    client.enableApiControl(False)

if __name__ == "__main__":
    main()

