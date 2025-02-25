import json
import numpy
import math
import os
import sys

sys.path.append("..")


def load_json(file_path):
    with open(file_path, "r") as file:
        return json.load(file)


def load_json_keys(file_path):
    json_data = load_json(file_path)
    return list(json_data.keys())


def save_json(data, file_path):
    with open(file_path, "w") as file:
        json.dump(data, file, indent=4)


def LoadScalar(ctrl_data, json_struct, name):
    try:
        ctrl_data[name] = json_struct[name]
    except:
        ctrl_data[name] = -0.01


def LoadVector(ctrl_data, json_struct, name, N=20):
    try:
        data_list = json_struct[name].split(",")
        ctrl_data[name] = list(map(float, data_list))
        return 1
    except:
        tmp = []
        for i in range(N):
            tmp.append(0.0)

        ctrl_data[name] = tmp
        return 0


def LoadScalarList(json_data, json_value_list, json_struct):
    for i in range(len(json_value_list)):
        LoadScalar(json_data, json_struct, json_value_list[i])


def LoadVectorList(json_data, json_vector_list, json_struct):
    for i in range(len(json_vector_list)):
        LoadVector(json_data, json_struct, json_vector_list[i])


def VectorTimeK(x, k):
    new_x = [i * k for i in x]
    return new_x
