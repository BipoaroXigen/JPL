import bpy

def cylinder_between(x1, y1, z1, x2, y2, z2, r, mat=None):
    """
    https://blender.stackexchange.com/questions/5898/how-can-i-create-a-cylinder-linking-two-points-with-python
    """
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    dist = math.sqrt(dx**2 + dy**2 + dz**2)

    if dist == 0:
        return

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=8,
        radius=r,
        depth=dist,
        location=(dx / 2 + x1, dy / 2 + y1, dz / 2 + z1)
    )

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)

    bpy.context.object.rotation_euler[1] = theta
    bpy.context.object.rotation_euler[2] = phi

    if mat is not None:
        bpy.context.active_object.data.materials.append(mat)

def create_animation():
   
    for obj in bpy.context.visible_objects:
        bpy.context.collection.objects.unlink(obj)

    # Set light
    lamp_data = bpy.data.lights.new(name="lamp", type='POINT')
    lamp_object = bpy.data.objects.new(name="Lamp", object_data=lamp_data)
    bpy.context.collection.objects.link(lamp_object)
    lamp_object.location = (0, -10, 12)
    lamp = bpy.data.lights[lamp_data.name]
    lamp.energy = 1000
    lamp.use_shadow = False
    lamp.shadow_soft_size = 0.1

    # Set camera
    cam_data = bpy.data.cameras.new(name="camera")
    cam_ob = bpy.data.objects.new(name="Camera", object_data=cam_data)
    bpy.context.collection.objects.link(cam_ob)
    bpy.context.scene.camera = cam_ob
    cam_ob.location = (-5, 120, 120)
    cam_ob.rotation_euler = (math.pi/3.5, math.pi/20, math.pi)  # 58, 0, 117
    # cam_ob.location = (10, 23, 17)
    # cam_ob.rotation_euler = (1.012291, 0, 2.722714)  # 58, 0, 156
    cam = bpy.data.cameras[cam_data.name]
    cam.lens = 50

    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/PSO_RRT/meshes/goal.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_euler = (0, 0, 0)

    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/PSO_RRT/meshes/environment.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_euler = (0, 0, 0)

    
    
    
    
    # Create material
    mat_name = "Trajectory"
    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0, 0.00993459, 0.8, 1)

    # ehlehelejek
    path = []
    with open("/home/krizjona/PSO_RRT/goal.txt", "r") as tree:
        i = 0
        for line in tree:
            i+=1
        i = int(i/2)
        print(i)
        
    with open("/home/krizjona/PSO_RRT/goal.txt", "r") as tree:    
        for pt in range(i):
            a = tree.readline().split()[0:3]
            b = tree.readline().split()[0:3]

            a = [float(x) for x in a]
            b = [float(x) for x in b]
        
            path.append([a,b])
    
    for pair in path:
        #print(pair)
        cylinder_between(*pair[0], *pair[1], 0.05, mat)
    
    with open("/home/krizjona/PSO_RRT/q_init.txt", "r") as tree:    
        for step in tree:
                step = step.split()
                step = [float(x) for x in step]
                imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/PSO_RRT/meshes/robot.obj")
                obj_object = bpy.context.selected_objects[0]
                obj_object.location = (step[0], step[1], step[2])
                obj_object.rotation_mode = 'QUATERNION'
                obj_object.rotation_quaternion = (step[3], -step[6], -step[5], -step[4])
    
    with open("/home/krizjona/PSO_RRT/dist.txt", "r") as tree:    
        for step in tree:
                step = step.split()
                step = [float(x) for x in step]
                imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/PSO_RRT/meshes/best.obj")
                obj_object = bpy.context.selected_objects[0]
                obj_object.location = (step[0], step[1], step[2])
                obj_object.rotation_mode = 'QUATERNION'
                obj_object.dimensions = [step[3], step[5], step[4]]
            
    with open("/home/krizjona/PSO_RRT/distributions.txt", "r") as tree:    
        for step in tree:
                step = step.split()
                step = [float(x) for x in step]
                imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/PSO_RRT/meshes/probe.obj")
                obj_object = bpy.context.selected_objects[0]
                obj_object.location = (step[0], step[1], step[2])
                obj_object.rotation_mode = 'QUATERNION'
                obj_object.dimensions = [step[3], step[5], step[4]]
    
create_animation()
