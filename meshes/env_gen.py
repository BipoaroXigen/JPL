# -*- coding: utf-8 -*-
import bpy
import math

# useful shortcuts
scene = bpy.context.scene
collection = bpy.context.collection

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

    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/goal.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_euler = (0, 0, 0)

    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/environment.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_euler = (0, 0, 0)

    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/robot.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_euler = (0, 0, 0)
    
    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/probe.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_mode = 'QUATERNION'
            
    with open("/home/krizjona/Project/files/points.txt", "r") as tree:    
        i = 0
        for step in tree:
                obj_object = obj_object.copy()
                bpy.context.collection.objects.link(obj_object)


                step = step.split()
                step = [float(x) for x in step]
                obj_object.location = (step[0], step[1], step[2])
                #obj_object.rotation_quaternion = (step[3], -step[6], -step[5], -step[4])
                i += 1
                print(i)
         
    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/best.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_mode = 'QUATERNION'
                
    with open("/home/krizjona/Project/files/path.txt", "r") as tree:    
        i = 0
        for step in tree:
                obj_object = obj_object.copy()
                bpy.context.collection.objects.link(obj_object)


                step = step.split()
                step = [float(x) for x in step]
                obj_object.location = (step[0], step[1], step[2])
                #obj_object.rotation_quaternion = (step[3], -step[6], -step[5], -step[4])
                i += 1
                print(i)
    
create_animation()