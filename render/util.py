import numpy as np
import bpy
import json
import os
import random
from tqdm import tqdm

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


	# size_name, r = random.choice(size_mapping)
	# obj_name, obj_name_out = random.choice(object_mapping)
	# color_name, rgba = random.choice(list(color_name_to_rgba.items()))
	# if obj_name == 'Cube':
	# 	r /= math.sqrt(2)
	# theta = 360.0 * random.random()
	# add_object(shape_dir, obj_name, r, (0, 0), theta=theta)
	# mat_name, mat_name_out = random.choice(material_mapping)
	# add_material(mat_name, Color=rgba)



def add_object(object_dir, name, scale, loc, theta=0):
  """
  Load an object from a file. We assume that in the directory object_dir, there
  is a file named "$name.blend" which contains a single object named "$name"
  that has unit size and is centered at the origin.

  - scale: scalar giving the size that the object should be in the scene
  - loc: tuple (x, y) giving the coordinates on the ground plane where the
    object should be placed.
  """
  # First figure out how many of this object are already in the scene so we can
  # give the new object a unique name
  count = 0
  for obj in bpy.data.objects:
    if obj.name.startswith(name):
      count += 1

  filename = os.path.join(object_dir, '%s.blend' % name, 'Object', name)
  bpy.ops.wm.append(filename=filename)

  # Give it a new name to avoid conflicts
  new_name = '%s_%d' % (name, count)
  bpy.data.objects[name].name = new_name

  # Set the new object as active, then rotate, scale, and translate it
  x, y = loc
  bpy.context.scene.objects.active = bpy.data.objects[new_name]
  bpy.context.object.rotation_euler[2] = theta
  bpy.ops.transform.resize(value=(scale, scale, scale))
  bpy.ops.transform.translate(value=(x, y, scale))
  bpy.ops.object.origin_set(type='GEOMETRY_ORIGIN')
  bpy.ops.object.location_clear()


def load_materials(material_dir):
  """
  Load materials from a directory. We assume that the directory contains .blend
  files with one material each. The file X.blend has a single NodeTree item named
  X; this NodeTree item must have a "Color" input that accepts an RGBA value.
  """
  for fn in os.listdir(material_dir):
    if not fn.endswith('.blend'): continue
    name = os.path.splitext(fn)[0]
    filepath = os.path.join(material_dir, fn, 'NodeTree', name)
    bpy.ops.wm.append(filename=filepath)


def add_material(name, **properties):
  """
  Create a new material and assign it to the active object. "name" should be the
  name of a material that has been previously loaded using load_materials.
  """
  # Figure out how many materials are already in the scene
  mat_count = len(bpy.data.materials)

  # Create a new material; it is not attached to anything and
  # it will be called "Material"
  bpy.ops.material.new()

  # Get a reference to the material we just created and rename it;
  # then the next time we make a new material it will still be called
  # "Material" and we will still be able to look it up by name
  mat = bpy.data.materials['Material']
  mat.name = 'Material_%d' % mat_count
  mat.use_nodes = True

  # Attach the new material to the active object
  # Make sure it doesn't already have materials
  obj = bpy.context.active_object
  assert len(obj.data.materials) == 0
  obj.data.materials.append(mat)

  # Find the output node of the new material
  output_node = None
  for n in mat.node_tree.nodes:
    if n.name == 'Material Output':
      output_node = n
      break

  # Add a new GroupNode to the node tree of the active material,
  # and copy the node tree from the preloaded node group to the
  # new group node. This copying seems to happen by-value, so
  # we can create multiple materials of the same type without them
  # clobbering each other
  group_node = mat.node_tree.nodes.new('ShaderNodeGroup')
  group_node.node_tree = bpy.data.node_groups[name]

  # Find and set the "Color" input of the new group node
  for inp in group_node.inputs:
    if inp.name in properties:
      inp.default_value = properties[inp.name]

  # Wire the output of the new group node to the input of
  # the MaterialOutput node
  mat.node_tree.links.new(
      group_node.outputs['Shader'],
      output_node.inputs['Surface'],
  )

def setupBlender(buffer_path,RESOLUTION):
	scene = bpy.context.scene
	camera = bpy.data.objects["Camera"]
	camera.data.type = "ORTHO"
	camera.data.ortho_scale = 1
	# compositor nodes
	scene.render.use_antialiasing = False
	scene.render.engine = "CYCLES"
	scene.render.alpha_mode = "TRANSPARENT"
	scene.render.image_settings.color_depth = "16"
	scene.render.image_settings.color_mode = "RGBA"
	scene.render.image_settings.use_zbuffer = True
	scene.render.use_compositing = True
	scene.use_nodes = True
	tree = scene.node_tree
	for n in tree.nodes:
		tree.nodes.remove(n)
	rl = tree.nodes.new("CompositorNodeRLayers")
	fo = tree.nodes.new("CompositorNodeOutputFile")
	fo.base_path = buffer_path
	fo.format.file_format = "OPEN_EXR"
	fo.file_slots.new('Depth')
	tree.links.new(rl.outputs['Depth'],fo.inputs['Depth'])
	scene.render.resolution_x = RESOLUTION
	scene.render.resolution_y = RESOLUTION
	scene.render.resolution_percentage = 100
	return scene,camera,fo

def setCameraExtrinsics(camera,camPos,q):
	camera.rotation_mode = "QUATERNION"
	camera.location[0] = camPos[0]
	camera.location[1] = camPos[1]
	camera.location[2] = camPos[2]
	camera.rotation_quaternion[0] = q[0]
	camera.rotation_quaternion[1] = q[1]
	camera.rotation_quaternion[2] = q[2]
	camera.rotation_quaternion[3] = q[3]
	camera.data.sensor_height = camera.data.sensor_width

def projectionMatrix(scene,camera):
	scale = camera.data.ortho_scale
	scale_u,scale_v = scene.render.resolution_x/scale,scene.render.resolution_y/scale
	u_0 = scale_u/2.0
	v_0 = scale_v/2.0
	skew = 0 # only use rectangular pixels
	P = np.array([[scale_u,      0,u_0],
				  [0      ,scale_v,v_0]])
	return P

def cameraExtrinsicMatrix(q,camPos):
	R_world2bcam = quaternionToRotMatrix(q).T
	t_world2bcam = -1*R_world2bcam.dot(np.expand_dims(np.array(camPos),-1))
	R_bcam2cv = np.array([[ 1, 0, 0],
						  [ 0,-1, 0],
						  [ 0, 0,-1]])
	R_world2cv = R_bcam2cv.dot(R_world2bcam)
	t_world2cv = R_bcam2cv.dot(t_world2bcam)
	Rt = np.concatenate([R_world2cv,t_world2cv],axis=1)
	q_world2bcam = rotMatrixToQuaternion(R_world2bcam)
	return q_world2bcam,t_world2bcam

def objectCenteredCamPos(rho,azim,elev):
	phi = np.deg2rad(elev)
	theta = np.deg2rad(azim)
	x = rho*np.cos(theta)*np.cos(phi)
	y = rho*np.sin(theta)*np.cos(phi)
	z = rho*np.sin(phi)
	return [x,y,z]

def camPosToQuaternion(camPos):
	[cx,cy,cz] = camPos
	q1 = [0,0,np.sqrt(2)/2,np.sqrt(2)/2]
	camDist = np.linalg.norm([cx,cy,cz])
	cx,cy,cz = cx/camDist,cy/camDist,cz/camDist
	t = np.linalg.norm([cx,cy])
	tx,ty = cx/t,cy/t
	yaw = np.arccos(ty) 
	yaw = 2*np.pi-np.arccos(ty) if tx>0 else yaw
	pitch = 0
	roll = np.arccos(np.clip(tx*cx+ty*cy,-1,1))
	roll = -roll if cz<0 else roll
	q2 = quaternionFromYawPitchRoll(yaw,pitch,roll)	
	q3 = quaternionProduct(q2,q1)
	return q3

def camRotQuaternion(camPos,theta):
	theta = np.deg2rad(theta)
	[cx,cy,cz] = camPos
	camDist = np.linalg.norm([cx,cy,cz])
	cx,cy,cz = -cx/camDist,-cy/camDist,-cz/camDist
	qa = np.cos(theta/2.0)
	qb = -cx*np.sin(theta/2.0)
	qc = -cy*np.sin(theta/2.0)
	qd = -cz*np.sin(theta/2.0)
	return [qa,qb,qc,qd]

def quaternionProduct(q1,q2): 
	qa = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]
	qb = q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]
	qc = q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]
	qd = q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
	return [qa,qb,qc,qd]

def quaternionFromYawPitchRoll(yaw,pitch,roll):
	c1 = np.cos(yaw/2.0)
	c2 = np.cos(pitch/2.0)
	c3 = np.cos(roll/2.0)
	s1 = np.sin(yaw/2.0)
	s2 = np.sin(pitch/2.0)
	s3 = np.sin(roll/2.0)
	qa = c1*c2*c3+s1*s2*s3
	qb = c1*c2*s3-s1*s2*c3
	qc = c1*s2*c3+s1*c2*s3
	qd = s1*c2*c3-c1*s2*s3
	return [qa,qb,qc,qd]

def quaternionToRotMatrix(q):
	R = np.array([[1-2*(q[2]**2+q[3]**2),2*(q[1]*q[2]-q[0]*q[3]),2*(q[0]*q[2]+q[1]*q[3])],
				  [2*(q[1]*q[2]+q[0]*q[3]),1-2*(q[1]**2+q[3]**2),2*(q[2]*q[3]-q[0]*q[1])],
				  [2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]**2+q[2]**2)]])
	return R

def rotMatrixToQuaternion(R):
	t = R[0,0]+R[1,1]+R[2,2]
	r = np.sqrt(1+t)
	qa = 0.5*r
	qb = np.sign(R[2,1]-R[1,2])*np.abs(0.5*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]))
	qc = np.sign(R[0,2]-R[2,0])*np.abs(0.5*np.sqrt(1-R[0,0]+R[1,1]-R[2,2]))
	qd = np.sign(R[1,0]-R[0,1])*np.abs(0.5*np.sqrt(1-R[0,0]-R[1,1]+R[2,2]))
	return [qa,qb,qc,qd]

def randomRotation():
	pos = np.inf
	while np.linalg.norm(pos)>1:
		pos = np.random.rand(3)*2-1
	pos /= np.linalg.norm(pos)
	phi = np.arcsin(pos[2])
	theta = np.arccos(pos[0]/np.cos(phi))
	if pos[1]<0: theta = 2*np.pi-theta
	elev = np.rad2deg(phi)
	azim = np.rad2deg(theta)
	rho = 1
	theta = np.random.rand()*360
	return rho,azim,elev,theta

def getFixedViews(FIXED):
	if FIXED==4:
		camPosAll = np.array([[1,1,1],[1,-1,-1],[-1,1,-1],[-1,-1,1]],dtype=float)
		camPosAll /= np.sqrt(3)
	elif FIXED==6:
		camPosAll = np.array([[1,0,0],[0,-1/np.sqrt(2),1/np.sqrt(2)],[0,1/np.sqrt(2),-1/np.sqrt(2)],
							  [-1,0,0],[0,1/np.sqrt(2),1/np.sqrt(2)],[0,-1/np.sqrt(2),-1/np.sqrt(2)]],dtype=float)
	elif FIXED==8:
		camPosAll = np.array([[1,1,1],[1,-1,-1],[-1,1,-1],[-1,-1,1],[1,1,-1],[1,-1,1],[-1,1,1],[-1,-1,-1]],dtype=float)
		camPosAll /= np.sqrt(3)
	elif FIXED==12:
		camPosAll = np.array([
			[-0.6000,0,-0.8000],
			[0.4472,0,-0.8944],
			[-0.0472,0.8507,-0.5236],
			[-0.8472,0.5257,0.0764],
			[-0.8472,-0.5257,0.0764],
			[-0.0472,-0.8507,-0.5236],
			[0.8472,0.5257,-0.0764],
			[0.0472,0.8507,0.5236],
			[-0.4472,0.0000,0.8944],
			[0.0472,-0.8507,0.5236],
			[0.8472,-0.5257,-0.0764],
			[0.6000,0,0.8000]],dtype=float)
	elif FIXED==20:
		camPosAll = np.array([
			[-0.9614,0.2750,0],
			[-0.5332,0.8460,0],
			[-0.8083,-0.1155,-0.5774],
			[-0.8083,-0.1155,0.5774],
			[-0.1155,0.8083,-0.5774],
			[-0.1155,0.8083,0.5774],
			[-0.2855,0.2141,-0.9342],
			[-0.2855,0.2141,0.9342],
			[-0.5605,-0.7473,-0.3568],
			[-0.5605,-0.7473,0.3568],
			[0.5605,0.7473,-0.3568],
			[0.5605,0.7473,0.3568],
			[0.2855,-0.2141,-0.9342],
			[0.2855,-0.2141,0.9342],
			[0.1155,-0.8083,-0.5774],
			[0.1155,-0.8083,0.5774],
			[0.8083,0.1155,-0.5774],
			[0.8083,0.1155,0.5774],
			[0.5332,-0.8460,0],
			[0.9614,-0.2750,0]],dtype=float)
	else: camPosAll = None
	return camPosAll
