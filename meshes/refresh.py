import bpy

for obj in bpy.context.visible_objects:
    bpy.context.collection.objects.unlink(obj)


imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/environment.obj")
obj_object = bpy.context.selected_objects[0]
obj_object.rotation_euler = (0, 0, 0)

imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/robot.obj")
obj_object = bpy.context.selected_objects[0]
obj_object.rotation_euler = (0, 0, 0)

imported_object = bpy.ops.import_scene.obj(filepath="/home/krizjona/Project/meshes/goal.obj")
obj_object = bpy.context.selected_objects[0]
obj_object.rotation_euler = (0, 0, 0)