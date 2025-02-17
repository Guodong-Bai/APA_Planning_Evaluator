import sys
import os
import copy
import json
from bokeh.io import output_file
from bokeh.plotting import show
from lib.table_checker import *



class CheckerBase:
  def __init__(self, module_name, scemario_name) -> None:
    self.success = False
    self.module_name = module_name
    self.scemario_name = scemario_name
    self.initial_pose_vec = []
    self.checker_result_vec = []
    self.data = {}


  def GetModuleName(self):
    return self.module_name

  def GetScenarioName(self):
    return self.scemario_name

  def IsSuccess(self):
    return self.success



def find_files_with_extensions(bag_path_list, extensions = ['.record', '.plan', 'no_camera', 'bag']):
  bag_list = []
  for path in bag_path_list:
    for root, dirs, files in os.walk(path):
      for file in files:
        for i in range(len(extensions)):
          if (extensions[i] in file) and (".json" not in file) and (".html" not in file) and ("no_camera.bag" in file):
            file_path = os.path.join(root, file)
            bag_list.append(file_path)
            break

  return bag_list

def get_value_from_dict(dictionary, key):
  if key in dictionary:
    return dictionary[key]
  else:
    return None

def CalMeanOfList(list):
  return sum(list) / len(list)

def progress_bar(total, current):
  percent = current / total * 100
  bar_length = 60
  filled_length = int(percent / 100 * bar_length)
  bar = '#' * filled_length + '-' * (bar_length - filled_length)

  print('\rProgress: [{}] {:.1f}%'.format(bar, percent), end='', flush=True)

def save_list_to_json(lst, filename):
  data = []
  for item in lst:
    try:
      data.append({
          'bag_name': item['bag_name'],
          'checker_result': item['checker_result']
      })
    except:
      pass

  with open(filename, 'w') as file:
    json.dump(data, file, indent=2)

def read_variables_from_json(json_file):
  current_dir = os.path.dirname(os.path.abspath(__file__))
  json_file_path = os.path.join(current_dir, json_file)

  with open(json_file) as file:
    data = json.load(file)

  x_bounds = [data['x_lower_bound'], data['x_upper_bound']]
  y_bounds = [data['y_lower_bound'], data['y_upper_bound']]
  heading_deg_bounds = [data['heading_deg_lower_bound'], data['heading_deg_upper_bound']]

  max_initial_pose_cnt = data['max_initial_pose_cnt']

  max_process = data['max_process']
  output_path = data['output_path']

  return  x_bounds, y_bounds, heading_deg_bounds, max_initial_pose_cnt, max_process, output_path

def save_list_to_table(lst, htmlname):
  table = table_checker(lst)
  output_file(htmlname)
  show(table)