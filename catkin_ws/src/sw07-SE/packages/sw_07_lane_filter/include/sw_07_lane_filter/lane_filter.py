
from collections import OrderedDict
from math import floor

from numpy.testing.utils import assert_almost_equal
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal, entropy

from duckietown_msgs.msg import SegmentList
import duckietown_utils as dtu

from duckietown_utils.parameters import Configurable

import numpy as np

from .lane_filter_interface import LaneFilterInterface

from .visualization import plot_phi_d_diagram_bgr

from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, sqrt
import copy

import rospy


class LaneFitlerParticle(Configurable, LaneFilterInterface):

    class Particle():
        def __init__(self, d, phi, config):
            self.d = d
            self.phi = phi
            self.weight = 1

            self.d_max = config['d_max']
            self.d_min = config['d_min']
            self.phi_max = config['phi_max']
            self.phi_min = config['phi_min']
            self.sigma_d = config['sigma_d']
            self.sigma_phi = config['sigma_phi']

        def predict(self, dt, v, w):
            # Update d and phi depending on dt, v and w
            new_d = self.d 
            new_phi = self.phi 
            ########
            # Your code here
            ########
            self.d = new_d
            self.phi = new_phi

        def update(self, ds, phis):
            # Change weight depending on the likelihood of ds and phis from the measurements

            new_weight = 1
            ########
            # Your code here
            ########
            self.weight = new_weight

        def perturb(self, dd, dphi):
            self.d += dd
            self.phi += dphi

    def __init__(self):
        # Parameters
        self.nb_particles = 500

        ## Initialization 
        self.mean_d_0 = 0       # Expected value of d at initialization
        self.mean_phi_0 = 0     # Expected value of phi at initialization
        self.sigma_d_0 = 0.1    # Standard deviation of d at initialization
        self.sigma_phi_0 = 0.1  # Standard deviation of phi at initialization

        ## Prediction step noise
        self.sigma_d = 0.001
        self.sigma_phi = 0.002

        ## Roughening
        self.rough_d =  0.001
        self.rough_phi = 0.002

        ## Environment parameters
        self.linewidth_white = 0.05
        self.linewidth_yellow = 0.025
        self.lanewidth = 0.23

        ## Limits
        self.d_max = 0.3        
        self.d_min = -0.15
        self.phi_min = -1.5
        self.phi_max = 1.5
        self.range_est = 0.33   # Maximum distance for a segment to be considered
        self.delta_d = 0.02     # Maximum error on d for a segment to be considered as an inlier
        self.delta_phi = 0.1    # Maximum error on phi for a segment to be considered as an inlier
        self.min_max = 0.1      # Minimum maximal weight 

        # Attributes
        self.particles = []

        # Initialization
        self.initialize()

    def initialize(self):
        # Initialize the particle filter
        initial_particles = []

        # Parameters to be passed
        config = {}
        config['d_max'] = self.d_max
        config['d_min'] = self.d_min
        config['phi_max'] = self.phi_max
        config['phi_min'] = self.phi_min  
        config['sigma_d'] = self.sigma_d      
        config['sigma_phi'] = self.sigma_phi      

        for i in range(self.nb_particles):
            d = 0
            phi = 0
        ########
        # Your code here
        ########
            initial_particles.append(self.Particle(d, phi, config))
        self.particles = initial_particles

    def predict(self, dt, v, w):
        # Prediction step for the particle filter
        for particle in self.particles:
            particle.predict(dt, v, w)

    def update(self, segment_list):
        # Measurement update state for the particle filter
        segmentArray = self.prepareSegments(segment_list)
        self.updateWeights(segment_list)
        self.resample()
        self.roughen()

    def updateWeights(self, segment_list):
        ds = []
        phis = []
        # Compute the ds and phis from the segments
        for segment in segment_list:
            d, phi, _ = self.process(segment)
            ds.append(d)
            phis.append(phi)
        # Update the particle weights
        for particle in self.particles:
            particle.update(ds, phis) 
        
    def resample(self):
        # Sample a new set of particles
        new_particles = self.particles
        ########
        # Your code here
        ########
        self.particles = new_particles

    def roughen(self):
        # Roughen the particles set to avoid sample impoverishment
        for particle in self.particles:
            dd = np.random.normal(loc = 0.0, scale = self.rough_d)
            dphi = np.random.normal(loc = 0.0, scale = self.rough_phi)
            particle.perturb(dd, dphi)

    def getEstimate(self):
        # Get the estimate of d and phi
        d = 0
        phi = 0
        ########
        # Your code here
        ########

        return [d, phi]

    def isInLane(self):
        # Test to know if the bot is in the lane
        in_lane = True
        ########
        # Your code here
        ########
        return in_lane

### Other functions - no modification needed ###
    def process(self, segment):
        # Returns   d_i the distance from the middle of the lane
        #           phi_i the angle from the lane direction
        #           l_i the distance from the bot in perpendicular projection
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)

        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if (l1 < 0):
            l1 = -l1
        if (l2 < 0):
            l2 = -l2

        l_i = (l1 + l2) / 2
        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE:  # right lane is white
            if(p1[0] > p2[0]):  # right edge of white lane
                d_i = d_i - self.linewidth_white
            else:  # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth / 2

        elif segment.color == segment.YELLOW:  # left lane is yellow
            if (p2[0] > p1[0]):  # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else:  # right edge of white lane
                d_i = -d_i
            d_i = self.lanewidth / 2 - d_i

        return d_i, phi_i, l_i

    def getStatus(self):
        return LaneFilterInterface.GOOD

    def get_inlier_segments(self, segments, d, phi):
        inlier_segments = []
        for segment in segments:
            d_s, phi_s, l = self.process(segment)
            if abs(d_s - d) < self.delta_d and abs(phi_s - phi)<self.delta_phi:
                inlier_segments.append(segment)
        return inlier_segments

    def prepareSegments(self, segments):
    # Filter out segments
        segmentsRangeArray = []
        for segment in segments:
            # Remove RED segments 
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # Remove segments that are behind us
            elif segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            # Remove segments that are too far from the Duckiebot
            elif self.getSegmentDistance(segment) > self.range_est:
                continue
            else:
                segmentsRangeArray.append(segment)
        return segmentsRangeArray

    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return sqrt(x_c**2 + y_c**2)

    def getBeliefArray(self):
        # Returns the representation of the belief as an array (for visualization purposes)
        ds, phis = np.mgrid[self.d_min:self.d_max:self.delta_d, self.phi_min:self.phi_max:self.delta_phi]
        beliefArray = np.zeros(ds.shape)
        # Image of the particle set
        for particle in self.particles:
            d_i = particle.d
            phi_i = particle.phi
            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i > self.phi_max:
                continue

            i = int(floor((d_i - self.d_min) / self.delta_d))
            j = int(floor((phi_i - self.phi_min) / self.delta_phi))
            beliefArray[i, j] = beliefArray[i, j] + 1  

        if np.linalg.norm(beliefArray) == 0:
            return beliefArray
        beliefArray = beliefArray / np.sum(beliefArray)
        return beliefArray
 

class LaneFilterHistogram(Configurable, LaneFilterInterface):
    #"""LaneFilterHistogram"""

    def __init__(self, configuration):
        param_names = [
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'cov_v',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'sigma_d_mask',
            'sigma_phi_mask',
            'curvature_res',
            'range_min',
            'range_est',
            'range_max',
            'curvature_right',
            'curvature_left',
        ]

        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self, param_names, configuration)

        self.d, self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,
                                    self.phi_min:self.phi_max:self.delta_phi]
        self.d_pcolor, self.phi_pcolor = \
            np.mgrid[self.d_min:(self.d_max + self.delta_d):self.delta_d,
                     self.phi_min:(self.phi_max + self.delta_phi):self.delta_phi]

        self.beliefArray = np.empty(self.d.shape)
        self.range_arr = np.zeros(1)
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [[self.sigma_d_0, 0], [0, self.sigma_phi_0]]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        self.d_med_arr = []
        self.phi_med_arr = []
        self.median_filter_size = 5

        self.initialize()
        self.updateRangeArray()

        # Additional variables
        self.red_to_white = False
        self.use_yellow = True
        self.range_est_min = 0
        self.filtered_segments = []

    def getStatus(self):
        return LaneFilterInterface.GOOD

    def predict(self, dt, v, w):
        delta_t = dt
        d_t = self.d + v * delta_t * np.sin(self.phi)
        phi_t = self.phi + w * delta_t
        p_belief = np.zeros(self.beliefArray.shape)

        for i in range(self.beliefArray.shape[0]):
            for j in range(self.beliefArray.shape[1]):
                if self.beliefArray[i, j] > 0:
                    if d_t[i, j] > self.d_max or d_t[i, j] < self.d_min or phi_t[i, j] < self.phi_min or phi_t[i, j] > self.phi_max:
                        continue
                    i_new = int(
                        floor((d_t[i, j] - self.d_min) / self.delta_d))
                    j_new = int(
                        floor((phi_t[i, j] - self.phi_min) / self.delta_phi))
                    p_belief[i_new, j_new] += self.beliefArray[i, j]
        s_belief = np.zeros(self.beliefArray.shape)
        gaussian_filter(p_belief, self.cov_mask,
                        output=s_belief, mode='constant')

        if np.sum(s_belief) == 0:
            return
        self.beliefArray = s_belief / np.sum(s_belief)

    # prepare the segments for the creation of the belief arrays
    def prepareSegments(self, segments):
        segmentsRangeArray = []
        self.filtered_segments = []
        for segment in segments:
            # Optional transform from RED to WHITE
            if self.red_to_white and segment.color == segment.RED:
                segment.color = segment.WHITE

            # Optional filtering out YELLOW
            if not self.use_yellow and segment.color == segment.YELLOW: continue

            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

            self.filtered_segments.append(segment)
            # only consider points in a certain range from the Duckiebot for the position estimation
            point_range = self.getSegmentDistance(segment)
            if point_range < self.range_est and point_range > self.range_est_min:
                segmentsRangeArray.append(segment)

        return segmentsRangeArray

    def updateRangeArray(self):
        self.beliefArray = np.empty(self.d.shape)
        self.initialize()

    # generate the belief arrays
    def update(self, segments):
        # prepare the segments for each belief array
        segmentsRangeArray = self.prepareSegments(segments)
        # generate all belief arrays
        measurement_likelihood = self.generate_measurement_likelihood(segmentsRangeArray)

        if measurement_likelihood is not None:
            self.beliefArray = np.multiply(self.beliefArray, measurement_likelihood)
            if np.sum(self.beliefArray) == 0:
                self.beliefArray = measurement_likelihood
            else:
                self.beliefArray = self.beliefArray / np.sum(self.beliefArray)

    def generate_measurement_likelihood(self, segments):
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape)
        for segment in segments:
            d_i, phi_i, l_i =  self.generateVote(segment)
            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i > self.phi_max:
                continue
            i = int(floor((d_i - self.d_min) / self.delta_d))
            j = int(floor((phi_i - self.phi_min) / self.delta_phi))
            measurement_likelihood[i, j] = measurement_likelihood[i, j] + 1
        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        measurement_likelihood = measurement_likelihood / np.sum(measurement_likelihood)
        return measurement_likelihood

    # get the maximal values d_max and phi_max from the belief array. 
    def getEstimate(self):
        maxids = np.unravel_index(self.beliefArray.argmax(), self.beliefArray.shape)
        d_max = self.d_min + (maxids[0] + 0.5) * self.delta_d
        phi_max = self.phi_min + (maxids[1] + 0.5) * self.delta_phi
        return [d_max, phi_max]

    def get_estimate(self):
        d, phi = self.getEstimate()
        res = OrderedDict()
        res['d'] = d
        res['phi'] = phi
        return res

    # return the maximal value of the beliefArray
    def getMax(self):
        return self.beliefArray.max()

    def isInLane(self):
        return self.getMax() > self.min_max

    def initialize(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:, :, 0] = self.d
        pos[:, :, 1] = self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0, self.cov_0)
        self.beliefArray = RV.pdf(pos)

    # generate a vote for one segment
    def generateVote(self, segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if (l1 < 0):
            l1 = -l1
        if (l2 < 0):
            l2 = -l2
        l_i = (l1 + l2) / 2
        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE:  # right lane is white
            if(p1[0] > p2[0]):  # right edge of white lane
                d_i = d_i - self.linewidth_white
            else:  # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth / 2
        elif segment.color == segment.YELLOW:  # left lane is yellow
            if (p2[0] > p1[0]):  # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else:  # right edge of white lane
                d_i = -d_i
            d_i = self.lanewidth / 2 - d_i
        return d_i, phi_i, l_i

    def get_inlier_segments(self, segments, d_max, phi_max):
        inlier_segments = []
        for segment in segments:
            d_s, phi_s, l = self.generateVote(segment)
            if abs(d_s - d_max) < self.delta_d and abs(phi_s - phi_max)<self.delta_phi:
                inlier_segments.append(segment)
        return inlier_segments

    # get the distance from the center of the Duckiebot to the center point of a segment
    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return sqrt(x_c**2 + y_c**2)
