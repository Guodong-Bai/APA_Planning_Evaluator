from lib.load_geometry import LineSegment
from lib.load_rotate import coord_transformer
from load_struct import load_car_box, load_car_params_patch
import numpy as np
from lib.load_geometry import cal_point_to_line_seg_dist

class CollisionDetection:
  def __init__(self, obs_x_vec, obs_y_vec):
    self.obs_x_vec = obs_x_vec
    self.obs_y_vec = obs_y_vec
    self.ego_local_x_vec, self.ego_local_y_vec = load_car_params_patch()
    self.coord_tf = coord_transformer()

  def check_pose_collided(self, ego_x, ego_y, ego_heading, safe_dist = 0.3):
    self.coord_tf.set_info(ego_x, ego_y, ego_heading)
    self.ego_global_x_vec, self.ego_global_y_vec = self.coord_tf.local_to_global(self.ego_local_x_vec, self.ego_local_y_vec)

    for i in range(len(self.obs_x_vec)):
      obs_point = np.array([self.obs_x_vec[i], self.obs_y_vec[i]])
      for j in range(len(self.ego_global_x_vec)):
        ego_vertex_j = np.array([self.ego_global_x_vec[j], self.ego_global_y_vec[j]])
        if (j != len(self.ego_global_x_vec) - 1):
          line = LineSegment(self.ego_global_x_vec[j], self.ego_global_y_vec[j], self.ego_global_x_vec[j + 1], self.ego_global_y_vec[j + 1])
        else:
          line = LineSegment(self.ego_global_x_vec[j], self.ego_global_y_vec[j], self.ego_global_x_vec[0], self.ego_global_y_vec[0])

        if cal_point_to_line_seg_dist(obs_point, line) < safe_dist:
          return True
    return False



