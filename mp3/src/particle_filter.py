import copy

import numpy as np
from maze import Maze, Particle, Robot, Particlenew
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]


class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()
        for i in range(num_particles):
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        self.weight_sum = 0.0
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        tmp1 = np.array(x1)
        tmp2 = np.array(x2)
        # print('robot_sensor =',tmp1, 'particle sensor =', tmp2)
        return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))

    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input: (measurements)
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        (Output: updated weights for every particles/one particle?)
        """
        ## TODO #####
        weights = []
        weights_sum = 0
        for i in range(self.num_particles):
            readings_sensor = self.particles[i].read_sensor()
            # print('particle sensor i =', readings_sensor)
            self.particles[i].weight = self.weight_gaussian_kernel(readings_robot, readings_sensor)
            # print(self.particles[i].weight)
            self.weight_sum += self.particles[i].weight 
            weights.append(self.particles[i].weight)
            weights_sum = weights_sum + self.particles[i].weight

        for j in range(self.num_particles):
            self.particles[j].weight = self.particles[j].weight / weights_sum
            #weights[j] = weights[j]/weights_sum

        ###############
        pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####
        # "Calculate an array of the cumulative sum of the weights"
        range0to1 = np.zeros(self.num_particles)
        range0to1[0] = self.particles[0].weight
        for i in range(1,self.num_particles):
            range0to1[i] = range0to1[i-1] + self.particles[i].weight
        # print('qu jian =', range0to1)
        "Randomly generate a number and determine which range the number belongs."
        "The index of that range would correspond to the particle that should be created. " 
        "Repeat sampling until you have the desired number of samples. "
        for j in range(self.num_particles):
            random_num = np.random.uniform(0,range0to1[-1])
            for k in range(self.num_particles):
                if random_num < range0to1[k]:
                    old_particle = self.particles[k]
                    #particles_new.append(self.particles[k])
                    particles_new.append(Particlenew(x = old_particle.x, y = old_particle.y, heading = old_particle.heading, maze = self.world, sensor_limit = self.sensor_limit))                                      
                    break
           
        # ###############
        #print('length =', len(particles_new))


        # weights = []
        # for particle in self.particles:
        #     weights.append(particle.weight)
        # for w in weights:
        #     weights_sum += w 
        # cdf = np.cumsum(weights)
        # cdf[-1] =1.0
        # random_samples = np.random.random_sample(len(self.particles))
        # particle_idxs = np.searchsorted(cdf, random_samples)
        # particles_new = [copy.deepcopy(self.particles[idx]) for idx in particle_idxs]
        
        # for new_particle in particles_new:
        #     new_particle.x += np.random.normal(0, 300 /self.weight_sum) 
        #     new_particle.y += np.random.normal(0, 300 /self.weight_sum)
        #     new_particle.heading += np.random.normal(0, 50 /self.weights_sum)

        self.particles = particles_new

    def integrator(self, t, vars0, vars1, vars2, vr, delta):
        curr_x = vars0
        curr_y = vars1
        curr_theta = vars2

        dx = vr * np.cos(curr_theta)
        dy = vr * np.sin(curr_theta)
        dtheta = delta
        return [t*dx, t*dy, t*dtheta]

    def particleMotionModel(self):
        """
        Description: (predict step)
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####
        # for i in range(num_particles):
        #     particles.append(Particle(x=x, y=y, maze=world, sensor_limit=sensor_limit))
        #     self.particles = particles
        "using ode to get new states x,y,theta"
        "control input is [velocity, delta], we can get it from the list called self.control"
        # print('control signal =', self.control)

        ## step integrator
        temp_control = []
        if len(self.control) > 0:
            temp_control = copy.deepcopy(self.control)
            self.control = []
            for j in range(len(temp_control)):
                for i in range(self.num_particles):
                    vars0 = self.particles[i].x
                    vars1 = self.particles[i].y
                    vars2 = self.particles[i].heading
                    vr = temp_control[j][0] 
                    delta = temp_control[j][1]
                    
                    [dx,dy,dtheta] = self.integrator(0.01, vars0, vars1, vars2, vr, delta)
                    self.particles[i].x = self.particles[i].x + dx
                    self.particles[i].y = self.particles[i].y + dy
                    self.particles[i].heading = (self.particles[i].heading + dtheta) % (np.pi*2)
        
# ##ode integrator
        # temp_control = []
        # if len(self.control) > 0:
        #     temp_control = copy.deepcopy(self.control)
        #     self.control = []
        #     for j in range(len(temp_control)):
        #         for i in range(self.num_particles):
        #             var = [self.particles[i].x, self.particles[i].y, self.particles[i].heading]
        #             vr = temp_control[j][0]
        #             delta = temp_control[j][1]
        #             dx, dy, dtheta = ode(vehicle_dynamics(0.01, var, vr, delta))
        #             self.particles[i].x = dx
        #             self.particles[i].y = dy
        #             self.particles[i].heading = dtheta        
        #             self.particles[i].heading = self.particles[i].heading % (np.pi*2)
        ###############
        pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        while True:
            ## TODO #####
            # Finish this function to have the particle filter running

            ### step1: predict

            self.particleMotionModel()
            # print('next x state =', self.particles[500].x, 'next y state =', self.particles[500].y, 'next heading state =', self.particles[500].heading)
            # Read sensor msg
            #???reading_sensor = Particle.read_sensor()
            #print('position of particle 501 after prediction =', self.particles[500].x, self.particles[500].y)

            reading_sensor = self.bob.read_sensor()
            # print('actual sensor =', self.bob.read_sensor())

            ### step2: update weightss

            self.updateWeight(reading_sensor)

            "updated weights seem right"
            # for i in range(self.num_particles):
            #     print('NO.', i, 'weight =', self.particles[i].weight)
            # temp_sum = 0
            # for i in range(self.num_particles):
            #     temp_sum += self.particles[i].weight
            # print('sum =', temp_sum)


            ### step3: resample
            self.resampleParticle()
            print(len(self.particles))
            #print('position of particle 501 after resampling =', self.particles[500].x, self.particles[500].y)

            # Display robot and particles on map 
            self.world.show_particles(particles = self.particles, show_frequency = 10)
            self.world.show_robot(robot = self.bob)
            [est_x,est_y] = self.world.show_estimated_location(particles = self.particles)
            self.world.clear_objects()

            ###############
