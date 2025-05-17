import numpy as np
from shapely.geometry import Polygon, Point
from load_geometry import LineSegment
from load_rotate import coord_transformer
from load_geometry import cal_point_to_line_seg_dist
from load_struct import load_car_box, load_car_params_patch_parking

class CollisionDetection:
    def __init__(self, obs_x_vec, obs_y_vec):
        # Convert to numpy array for faster operations
        self.obs_x_vec = np.array(obs_x_vec)
        self.obs_y_vec = np.array(obs_y_vec)
        self.ego_local_x_vec, self.ego_local_y_vec, _ = load_car_params_patch_parking()
        self.ego_vertex_size = len(self.ego_local_x_vec)
        self.coord_tf = coord_transformer()

    def check_pose_collided(self, ego_x, ego_y, ego_heading, safe_dist=0.35):
        self.coord_tf.set_info(ego_x, ego_y, ego_heading)

        # Convert local coordinates to global coordinates
        self.ego_global_x_vec, self.ego_global_y_vec = self.coord_tf.local_to_global(
            self.ego_local_x_vec, self.ego_local_y_vec)

        # Create a polygon representing the vehicle
        coord_vec_global = zip(self.ego_global_x_vec, self.ego_global_y_vec)
        polygon = Polygon(coord_vec_global)

        # Check if any obstacle is inside the vehicle's polygon
        for obs_x, obs_y in zip(self.obs_x_vec, self.obs_y_vec):
            obs_point = Point(obs_x, obs_y)
            if polygon.contains(obs_point):
                return True

            obs_pos = np.array([obs_x, obs_y])

            # Check for collisions with vehicle edges
            for j in range(len(self.ego_global_x_vec)):
                line = LineSegment(self.ego_global_x_vec[j],
                                   self.ego_global_y_vec[j],
                                   self.ego_global_x_vec[(j + 1) % self.ego_vertex_size],
                                   self.ego_global_y_vec[(j + 1) % self.ego_vertex_size])
                if cal_point_to_line_seg_dist(obs_pos, line) < safe_dist:
                    return True
        return False
