import airsim
import sys
import time
import argparse

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def start(self):
        print("[INFO] Arming the drone...")
        self.client.armDisarm(True)

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("[INFO] Taking off...")
            self.client.takeoffAsync().join()

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("[ERROR] Takeoff failed - check Unreal message log for details")
            return
        
        # AirSim uses NED coordinates, so altitude is negative
        x, y, z = 0, 0, -self.altitude

        print(f"[INFO] Climbing to altitude: {self.altitude} meters")
        self.client.moveToPositionAsync(x, y, z, self.velocity).join()

        print("[INFO] Flying to first corner of survey box...")
        self.client.moveToPositionAsync(-self.boxsize, -self.boxsize, z, self.velocity).join()

        # Let it hover before starting the survey
        self.client.hoverAsync().join()
        time.sleep(2)

        # Ensure API control after hover
        self.client.enableApiControl(True)

        # Compute the survey path
        path = []
        x = -self.boxsize
        while x < self.boxsize:
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth
            path.append(airsim.Vector3r(x, self.boxsize, z))
            path.append(airsim.Vector3r(x, -self.boxsize, z))
            x += self.stripewidth
            path.append(airsim.Vector3r(x, -self.boxsize, z))

        estimated_distance = 2 * self.boxsize * (self.boxsize // self.stripewidth)
        estimated_time = estimated_distance / self.velocity
        print(f"[INFO] Starting survey (Estimated Distance: {estimated_distance}m, Time: {estimated_time:.2f}s)")

        try:
            self.client.moveOnPathAsync(path, self.velocity, estimated_time, airsim.DrivetrainType.ForwardOnly, 
                                        airsim.YawMode(False, 0), self.velocity * 1.5, 1).join()
        except Exception as e:
            print(f"[ERROR] moveOnPathAsync exception: {e}")

        print("[INFO] Returning to home position...")
        self.client.moveToPositionAsync(0, 0, z, self.velocity).join()

        if z < -5:
            print("[INFO] Descending to 5m altitude...")
            self.client.moveToPositionAsync(0, 0, -5, 2).join()

        print("[INFO] Landing...")
        self.client.landAsync().join()

        print("[INFO] Disarming the drone...")
        self.client.armDisarm(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Survey a box area using a drone in AirSim.")
    parser.add_argument("--size", type=float, help="Size of the survey box (meters)", default=50)
    parser.add_argument("--stripewidth", type=float, help="Width of each survey stripe (meters)", default=10)
    parser.add_argument("--altitude", type=float, help="Survey altitude (positive meters)", default=30)
    parser.add_argument("--speed", type=float, help="Survey speed (m/s)", default=5)
    args = parser.parse_args()
    
    nav = SurveyNavigator(args)
    nav.start()

