import os,sys,time
import numpy as np
import scipy.io
import OpenEXR
import array,Imath
import tqdm
import json
import random

def load_clevr_config(properties_json):
	with open(properties_json, 'r') as f:
		properties = json.load(f)
		color_name_to_rgba = {}
		for name, rgb in properties['colors'].items():
			rgba = [float(c) / 255.0 for c in rgb] + [1.0]
			color_name_to_rgba[name] = rgba
		material_mapping = [(v, k) for k, v in properties['materials'].items()]
		object_mapping = [(v, k) for k, v in properties['shapes'].items()]
		size_mapping = list(properties['sizes'].items())

	return (material_mapping, object_mapping, size_mapping, color_name_to_rgba)

def generate_clevr_object(mapping):
	material_mapping, object_mapping, size_mapping, color_name_to_rgba = mapping

	combos = []
	for size_name, r in size_mapping:
		for obj_name, obj_name_out in object_mapping:
			for color_name, rgba in color_name_to_rgba.items():
				for mat_name, mat_name_out in material_mapping:
					if obj_name == 'Cube':
						r /= math.sqrt(2)
					theta = 360.0 * random.random()
					combos.append((size_name, r, obj_name, obj_name_out, color_name, rgba, mat_name, mat_name_out, theta))

	return combos

CATEGORY = "clevr"
RESOLUTION = 128
FIXED = 8
N = 100
mapping = load_clevr_config("../data/properties.json")
combos = generate_clevr_object(mapping)
def readEXR(fname,RESOLUTION):
	channel_list = ["B","G","R"]
	file = OpenEXR.InputFile(fname)
	dw = file.header()["dataWindow"]
	height,width = RESOLUTION,RESOLUTION
	FLOAT = Imath.PixelType(Imath.PixelType.FLOAT)
	vectors = [np.array(array.array("f",file.channel(c,FLOAT))) for c in channel_list]
	depth = vectors[0].reshape([height,width])
	return depth

for idx in tqdm.trange(len(combos)):
	size_name, r, obj_name, obj_name_out, color_name, rgba, mat_name, mat_name_out, theta = combos[idx]
	model_name = '_'.join([size_name, color_name, mat_name_out, obj_name_out])
	# arbitrary views
	Z = []
	depth_path = "output/{1}_depth/exr_{0}".format(model_name,CATEGORY)
	for i in range(N):
		depth = readEXR("{0}/{1}.exr".format(depth_path,i),RESOLUTION)
		depth[np.isinf(depth)] = 0
		Z.append(depth)
	trans_path = "{0}/trans.mat".format(depth_path)
	trans = scipy.io.loadmat(trans_path)["trans"]
	mat_path = "output/{1}_depth/{0}.mat".format(model_name,CATEGORY)
	scipy.io.savemat(mat_path,{
		"Z": np.stack(Z),
		"trans": trans,
	})
	os.system("rm -rf {0}".format(depth_path))

	# fixed views
	Z = []
	depth_path = "output/{1}_depth_fixed{2}/exr_{0}".format(model_name,CATEGORY,FIXED)
	for i in range(FIXED):
		depth = readEXR("{0}/{1}.exr".format(depth_path,i),RESOLUTION)
		depth[np.isinf(depth)] = 0
		Z.append(depth)
	mat_path = "output/{1}_depth_fixed{2}/{0}.mat".format(model_name,CATEGORY,FIXED)
	scipy.io.savemat(mat_path,{
		"Z": np.stack(Z),
	})
	os.system("rm -rf {0}".format(depth_path))
