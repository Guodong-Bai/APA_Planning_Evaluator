import numpy as np
from shapely.geometry import Polygon, Point
from load_geometry import LineSegment
from load_rotate import coord_transformer
from load_geometry import cal_point_to_line_seg_dist
from load_struct import load_car_box, load_car_params_patch_parking

class CollisionDetection:
    def __init__(self, obs_x_vec, obs_y_vec):
        self.obs_x_vec = obs_x_vec
        self.obs_y_vec = obs_y_vec
        self.ego_local_x_vec, self.ego_local_y_vec, _ = load_car_params_patch_parking()
        self.coord_tf = coord_transformer()

    def check_pose_collided(self, ego_x, ego_y, ego_heading, safe_dist=0.35):
        ego_vertex_size = len(self.ego_local_x_vec)
        self.coord_tf.set_info(ego_x, ego_y, ego_heading)
        self.ego_global_x_vec, self.ego_global_y_vec = self.coord_tf.local_to_global(
            self.ego_local_x_vec, self.ego_local_y_vec)
        for i in range(len(self.obs_x_vec)):
            obs_point = np.array([self.obs_x_vec[i], self.obs_y_vec[i]])
            coord_vec_global = list(zip(self.ego_global_x_vec, self.ego_global_y_vec))
            polygon = Polygon(coord_vec_global)
            obs_pt = Point((obs_point[0], obs_point[1]))
            if polygon.contains(obs_pt):
                return True
            
            for j in range(len(self.ego_global_x_vec)):
                line = LineSegment(self.ego_global_x_vec[j],
                                   self.ego_global_y_vec[j],
                                   self.ego_global_x_vec[(j + 1) % ego_vertex_size],
                                   self.ego_global_y_vec[(j + 1) % ego_vertex_size])
                if cal_point_to_line_seg_dist(obs_point, line) < safe_dist:
                    return True

        return False
