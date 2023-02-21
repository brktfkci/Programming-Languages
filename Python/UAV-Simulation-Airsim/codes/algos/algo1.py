from multiprocessing.pool import ThreadPool as Pool
import airsim
import sys
import time

class SurveyNavigator:
    def __init__(self, args, num):
        self.boxsize = args["size"]
        self.stripewidth = args["stripewidth"]
        self.altitude = args["altitude"]
        self.velocity = args["speed"]
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "Drone{}".format(num))

    def start(self, num):
        if self.boxsize < 50:
            self.boxsize = 50
        if self.stripewidth < 10:
            self.stripewidth = 10
        if self.altitude < 10:
            self.altitude = 10
        if self.velocity < 1:
            self.velocity = 1           
            
        print("arming the drone...".format(num))
        self.client.armDisarm(True, "Drone{}".format(num))

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            self.client.takeoffAsync(vehicle_name="Drone{}".format(num)).join()

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("takeoff failed - check Unreal message log for details")
            return
        
        # starting points
        if num == 1:
            sp_x = 100 +((num-1)*self.boxsize)
        else:
            sp_x = 100 +((num-1)*self.boxsize) + self.stripewidth 
        
        sp_y = -self.boxsize

        # AirSim uses NED coordinates so negative axis is up.
        x = sp_x
        z = -self.altitude

        print("climbing to altitude: " + str(self.altitude))
        self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name="Drone{}".format(num)).join()

        print("flying to first corner of survey box")
        self.client.moveToPositionAsync(sp_x, sp_y, z, self.velocity, vehicle_name="Drone{}".format(num)).join()
        
        # let it settle there a bit.
        self.client.hoverAsync(vehicle_name="Drone{}".format(num)).join()
        time.sleep(2)

        # after hovering we need to re-enabled api control for next leg of the trip
        self.client.enableApiControl(True, "Drone{}".format(num))

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < sp_x + self.boxsize:
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, -self.boxsize, z))
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            result = self.client.moveOnPathAsync(path, self.velocity, trip_time*2, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1, vehicle_name="Drone{}".format(num)).join()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        self.client.moveToPositionAsync(0, 0, z, self.velocity, vehicle_name="Drone{}".format(num)).join()
        
        if z < -5:
            print("descending")
            self.client.moveToPositionAsync(0, 0, -5, 2, vehicle_name="Drone{}".format(num)).join()

        print("landing...")
        self.client.landAsync(vehicle_name="Drone{}".format(num)).join()

        print("disarming.")
        self.client.armDisarm(False, "Drone{}".format(num))

def runAlgo1WithThreads(argsSurvey, UAVNum):

    # define worker function before a Pool is instantiated
    def worker(item):
        try:
            oneThread = SurveyNavigator(argsSurvey, item)
            oneThread.start(item)
        except:
            print('error with item')  

    pool = Pool(UAVNum)
    
    for ii in range(1, UAVNum+1, 1):
        pool.apply_async(worker, (ii,))

    pool.close()
    pool.join()    
