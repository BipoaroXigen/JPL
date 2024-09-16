import bpy

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
                
    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/best.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_mode = 'QUATERNION'
                
    with open("/home/krizjona/Project/files/dist.txt", "r") as tree:    
        i = 0
        for step in tree:
                obj_object = obj_object.copy()
                bpy.context.collection.objects.link(obj_object)


                step = step.split()
                step = [float(x) for x in step]
                obj_object.location = (step[0], step[1], step[2])
                obj_object.dimensions = [step[3], step[4], step[5]]
                #obj_object.rotation_quaternion = (step[3], -step[6], -step[5], -step[4])
                i += 1
                print(i)
                
    imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/probe.obj")
    obj_object = bpy.context.selected_objects[0]
    obj_object.rotation_mode = 'QUATERNION'
                
    with open("/home/krizjona/Project/files/distributions.txt", "r") as tree:    
        i = 0
        for step in tree:
                obj_object = obj_object.copy()
                bpy.context.collection.objects.link(obj_object)


                step = step.split()
                step = [float(x) for x in step]
                obj_object.location = (step[0], step[1], step[2])
                obj_object.dimensions = [step[3], step[4], step[5]]
                #obj_object.rotation_quaternion = (step[3], -step[6], -step[5], -step[4])
                i += 1
                print(i)
                
                
