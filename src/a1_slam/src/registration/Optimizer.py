import gtsam
import rospy
from collections import deque
from threading import Lock

class Optimizer:
    def __init__(self) -> None:
        # Instantiate relevant optimizer data structures.
        self.state_index = 0
        self.factor_queue = deque([])
        self.poses_queue = deque([])

        # Instantiate factor graph data structures.
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.graph = gtsam.NonlinearFactorGraph()
        self.init_estimates = gtsam.Values()
        self.results = gtsam.Values()

        # Instantiate a threading lock.
        self.lock = Lock()

    def optimize(self):
        """Optimized queued factors and estimates using iSAM2."""
        # Add queued factors and initial estimates to factor graph.
        while len(self.factor_queue):
            factor, estimates = self.factor_queue.popleft()
            self.graph.add(factor)
            for key, estimate in estimates:
                self.init_estimates.insert(key, estimate)
        
        # Perform an iSAM2 incremental update.
        self.isam.update(self.graph, self.initial_estimates)
        results = self.isam.calculateEstimate()
        with self.lock:
            self.results = results

        # Clear the graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates.clear()
    
    def send_results(self):
        serialized_str = self.results.serialize()
        return serialized_str

    def send_and_clear_results(self):
        serialized_str = self.results.serialize()
        self.results.clear()
        rospy.loginfo("Results cleared")
        return serialized_str