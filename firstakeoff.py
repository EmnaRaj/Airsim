import airsim
import sys
import time

# Set default altitude
z = 5  
if len(sys.argv) > 1:
    z = float(sys.argv[1])  # Allow altitude input from command line

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Enable API control
client.enableApiControl(True)
client.armDisarm(True)

# Check if the drone is landed
landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("[INFO] Taking off...")
    client.takeoffAsync().join()
else:
    print("[INFO] Already flying, ensuring hover...")
    client.hoverAsync().join()

print(f"[INFO] Hovering at {z} meters...")

if z > 5:
    # Move to target altitude (-z because AirSim uses NED coordinates)
    client.moveToZAsync(-z, velocity=3).join()
    client.hoverAsync().join()
    time.sleep(3)

if z > 10:
    print("[INFO] Descending quickly to 10 meters...")
    z = 10
    client.moveToZAsync(-z, velocity=3).join()
    client.hoverAsync().join()

print("[INFO] Landing...")
client.landAsync().join()

# Disarm and release API control
print("[INFO] Disarming drone...")
client.armDisarm(False)
client.enableApiControl(False)

print("[INFO] Done.")

