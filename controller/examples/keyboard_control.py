# ready to run example: PythonClient/multirotor/hello_drone.py
from math import cos, degrees, e, radians, sin
from pydoc import cli

import os
import sys
import time

import airsim
from msgpackrpc.future import Future
import setup_path
import keyboard

# Sources: 
# https://microsoft.github.io/AirSim/apis/
# https://microsoft.github.io/AirSim/settings/
# https://microsoft.github.io/AirSim/api_docs/html/
# https://github.com/boppreh/keyboard/blob/master/keyboard/_canonical_names.py
# https://microsoft.github.io/AirSim/api_docs/html/_modules/airsim/client.html
# https://github.com/Microsoft/AirSim/blob/main/docs/settings.md

MINIMUM_DISTANCE = 2.0

class Simulation:
    def __init__(self, client: airsim.MultirotorClient):
        self.client = client
        
    """
    Explaining moveToPositionAsync():
    - x: the North coordinate
    - y: the East coordinate
    - z: the Down coordinate
    - velocity: speed with which the drone moves (affects overshoot)
    - timeout_sec: max time given for a command to take
    - drivetrain: whether it faces in the direction of movement or remains facing where it is currently
    - yaw_mode: 
    - lookahead: the minimum distance needed to be specified in order for the drone to actually move
    - adaptive_lookahead:
    - vehicle_str: 
    """
    def move(self, north: float, east: float, down: float, velocity: int, 
             drivetrain: airsim.DrivetrainType = airsim.DrivetrainType.ForwardOnly) -> Future:
        return self.client.moveToPositionAsync(north, east, down, velocity, 
                                        120, drivetrain, 
                                        airsim.YawMode(False,0))
        
    def rotate(self, angle: float):
        self.client.rotateToYawAsync(yaw=degrees(angle)).join()
        
    def getCurrentPosition(self) -> airsim.Vector3r:
        return self.client.getMultirotorState().kinematics_estimated.position
        
    def moveNorth(self, distance: float, velocity: int = 3) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val + distance, position.y_val, position.z_val, velocity)
        
    def moveSouth(self, distance: float, velocity: int = 3) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val - distance, position.y_val, position.z_val, velocity)
    
    def moveEast(self, distance: float, velocity: int = 3) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val, position.y_val + distance, position.z_val, velocity)
        
    def moveWest(self, distance: float, velocity: int = 3) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val, position.y_val - distance, position.z_val, velocity)
        
    def moveUp(self, distance: float, velocity: int = 3) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val, position.y_val, position.z_val - distance, velocity, airsim.DrivetrainType.MaxDegreeOfFreedom)
        
    def moveDown(self, distance: float = 20, velocity: int = 5) -> Future:
        position = self.getCurrentPosition()
        return self.move(position.x_val, position.y_val, position.z_val + distance, velocity, airsim.DrivetrainType.MaxDegreeOfFreedom)
       
    def turnRight(self, degree: float = 15):
        self.client.rotateToYawAsync(yaw=90).join()
        # orientation = self.client.getMultirotorState().kinematics_estimated.orientation.
        # print(f"[Current orientation: w: {orientation.w_val} x: {orientation.x_val}, y: {orientation.y_val}, z: {orientation.z_val}]")
        # self.rotate(orientation.z_val + radians(degree))
        self.printPose()
        
    def turnLeft(self, degree: float = 15):
        self.client.rotateToYawAsync(yaw=-90).join()
        # orientation = self.client.getMultirotorState().kinematics_estimated.orientation
        # print(f"[Current orientation: w: {orientation.w_val} x: {orientation.x_val}, y: {orientation.y_val}, z: {orientation.z_val}]")
        # self.rotate(orientation.z_val - radians(degree))
        self.printPose()
        
    def printPose(self) -> None:
        pose = self.client.getMultirotorState().kinematics_estimated
        print(f"[Position(n: {pose.position.x_val}, e: {pose.position.y_val}, d: {pose.position.z_val}, r: {pose.orientation.z_val})]")


def keyboard_control(simulation: Simulation):
    
    while True:
        # North/South
        if keyboard.is_pressed('up'):
            print("[Moving north]")
            simulation.moveNorth(2, 3)
            
        elif keyboard.is_pressed('down'):
            print("[Moving south]")
            simulation.moveSouth(2, 3)
            
        # East/West
        if keyboard.is_pressed('right'):
            print("[Moving east]")
            simulation.moveEast(2, 3)
        elif keyboard.is_pressed('left'):
            print("[Moving west]")
            simulation.moveWest(2, 3)
            
        # Up/Down
        if keyboard.is_pressed('w'):
            print("[Moving up]")
            simulation.moveUp(2, 3)
        elif keyboard.is_pressed('s'):
            print("[Moving down]")
            simulation.moveDown(2, 3)
            
        # Rotate left/right
        if keyboard.is_pressed('d'):
            print("[Turning right]")
            simulation.turnRight()
        elif keyboard.is_pressed('a'):
            print("[Turning left]")
            simulation.turnLeft()
               
        
def run_keyboard_control():
    # Connect to the AirSim simulator
    simulation = Simulation(airsim.MultirotorClient(port=41451))
    simulation.client.confirmConnection()
    simulation.client.enableApiControl(True)
    simulation.client.armDisarm(True)
    
    # Controlling weather
    simulation.client.simEnableWeather(enable=True)
    simulation.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.5)
    
    state = simulation.client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("[Taking off]")
        # Async methods returns Future. Call join() to wait for task to complete.
        simulation.client.takeoffAsync().join() # Flies 1.6m up by default
    else:
        simulation.client.hoverAsync().join()

    time.sleep(1)

    state = simulation.client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("[Take off failed]")
        sys.exit(1)
    
    keyboard_control(simulation)
    

run_keyboard_control()