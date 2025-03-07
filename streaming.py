import airsim
import numpy as np
import cv2
import time
import argparse

def connect_drone(vehicle_name="PX4"):
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)
    return client

def safe_takeoff(client, altitude, vehicle_name):
    print("Taking off...")
    try:
        client.takeoffAsync(vehicle_name=vehicle_name).join()
        client.moveToZAsync(altitude, 3, vehicle_name=vehicle_name).join()
        time.sleep(2)
        return True
    except Exception as e:
        print(f"Takeoff failed: {str(e)}")
        return False

def stream_and_inspect(client, duration, speed, altitude, vehicle_name):
    start_time = time.time()
    waypoints = [
        (20, 0, altitude),
        (20, 20, altitude),
        (0, 20, altitude),
        (0, 0, altitude)
    ]
    
    try:
        for idx, (x, y, z) in enumerate(waypoints):
            print(f"Moving to waypoint {idx+1}")
            
            # Start async movement
            movement = client.moveToPositionAsync(x, y, z, speed, vehicle_name=vehicle_name)
            
            # Stream video during movement
            while not movement.done():
                # Capture and display video
                response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=vehicle_name)
                if response:
                    img = np.frombuffer(response[0].image_data_uint8, dtype=np.uint8)
                    img = img.reshape(response[0].height, response[0].width, 3)
                    cv2.imshow("Live Inspection", img)
                
                # Check for early exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    raise KeyboardInterrupt
                
                time.sleep(0.1)  # Prevent busy wait

            movement.join()  # Ensure completion

            # Check mission duration
            if time.time() - start_time > duration:
                break

        # Complete remaining duration
        while time.time() - start_time < duration:
            time.sleep(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Mission aborted by user")

def safe_land(client, vehicle_name):
    print("Initiating landing...")
    try:
        client.landAsync(vehicle_name=vehicle_name).join()
    finally:
        client.armDisarm(False, vehicle_name)
        client.enableApiControl(False, vehicle_name)
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--altitude", type=float, default=-20)
    parser.add_argument("--speed", type=float, default=3)
    parser.add_argument("--duration", type=int, default=50)
    parser.add_argument("--vehicle", type=str, default="PX4")
    args = parser.parse_args()

    drone = connect_drone(args.vehicle)
    
    try:
        if safe_takeoff(drone, args.altitude, args.vehicle):
            stream_and_inspect(drone, args.duration, args.speed, args.altitude, args.vehicle)
    finally:
        safe_land(drone, args.vehicle)

if __name__ == "__main__":
    main()
