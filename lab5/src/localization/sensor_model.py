import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        ####################################
        # Adjust these parameters
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale", 1.0)
        self.squash = 1.0/2.2

        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 401

        self.z_max = self.table_width - 1
        self.p_hit_table = None
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_resolution = None #5.0 for TESSE, 1.0 for 2D Racecar Sim referring to params.yaml
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        self.sensor_model_table = np.zeros((self.table_width, self.table_width))

        for d in range(self.table_width):
            p_hit_sum = 0.0
            p_hits = np.zeros(self.table_width)

            for z_k in range(self.table_width):
                # compute specific p_hit values for values of d and z_k
                p_hit = 1.0/(np.sqrt(2 * np.pi * self.sigma_hit**2)) * np.exp(-((z_k - d)**2)/(2*self.sigma_hit**2)) if z_k <= self.z_max else 0

                # assign in array and add to running sum
                p_hits[z_k] = p_hit
                p_hit_sum += p_hit

                # p_short values (0 if distance is 0, since undefined)
                if d == 0:
                    p_short = 0
                else:
                    p_short = 2.0/d * (1 - float(z_k)/d) if z_k <= d else 0.0

                # p_max and p_rand values
                p_max = 1.0 if z_k == self.z_max else 0.0
                p_rand = 1.0/self.z_max if z_k <= self.z_max else 0.0

                # sume of ratio'ed values of different probabilites (minus p_hit, to be normallized then added later)
                p = self.alpha_short * p_short + self.alpha_max * p_max + self.alpha_rand * p_rand

                self.sensor_model_table[z_k,d] = p

            #normalize array of p_hit values individually
            p_hits = self.alpha_hit*p_hits/p_hit_sum

            # add probability of hitting to ensor_model_table
            for i in range(self.table_width):
                self.sensor_model_table[i,d] += p_hits[i]

            # normalize the sensor model data along every axis within the loop.
            self.sensor_model_table[:,d] = self.sensor_model_table[:,d]/np.sum(self.sensor_model_table[:, d], axis = 0) #normalize data to deal with discretization


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """
        #if no map
        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle

        #given particles, converts to analogous "lidar" data
        scans = self.scan_sim.scan(particles) #lidar scans: distances, already downsampled

        # scaled from meters to pixels (both scans)
        scale = self.map_resolution*self.lidar_scale_to_map_scale
        scans = scans/scale #scale from m to pixels
        observation = observation/scale

        # clip both ground truth and observed distances
        scans[scans > self.z_max] = self.z_max
        scans[scans < 0] = 0.0
        observation[observation > self.z_max] = self.z_max
        observation[observation < 0] = 0.0

        # index through each beam in each particle and compute the product of those values
        particle_num, beam_num = scans.shape
        prob_out = np.zeros(particle_num)

        for particle_index in range(particle_num):
            particle = scans[particle_index, :]
            p_part = 1

            # each beam
            for beam_index in range(beam_num):
                # round for indexing
                d = int(round(particle[beam_index]))
                zk = int(round(observation[beam_index]))
                p = self.sensor_model_table[zk, d]
                p_part *= p
            prob_out[particle_index] = p_part

        prob_out = np.power(prob_out, self.squash) #squash
        return prob_out

# zipping the right file structure and default for 1.0
# turn on setting that we can see traceback



        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        self.map_resolution = map_msg.info.resolution

        print("Map initialized")
