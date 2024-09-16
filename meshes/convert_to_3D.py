import bpy

# loads a .tri file and generates 3D to be exported in any format

i = 0
with open("/home/krizjona/Project_2D/meshes/environment.tri", "r") as txt:
    for x in txt:
        i+=1

i = int(i/4)
with open("/home/krizjona/Project_2D/meshes/environment.tri") as txt:
    for _ in range(i):
        x = txt.readline()
        y = txt.readline()
        z = txt.readline()
        txt.readline()
        
        x = x.split()
        y = y.split()
        z = z.split()
        
        x = [float(x)/10 for x in x]
        y = [float(x)/10 for x in y]
        z = [float(x)/10 for x in z]

        xa = x.copy()
        xa[2] = 0.5
        xb = x.copy()
        xb[2] = -0.5
        
        ya = y.copy()
        ya[2] = 0.5
        yb = y.copy()
        yb[2] = -0.5
        
        za = z.copy()
        za[2] = 0.5
        zb = z.copy()
        zb[2] = -0.5
        
        
        vertices = [xa,ya,za,xb,yb,zb]
        
        edges = [(0,1),(1,2),(2,0), (3,4),(4,5),(5,3), (0,3),(1,4),(2,5)]
        faces = [(0,1,2), (3,4,5), (0,1,4,3), (1,2,5,4), (2,0,3,5)]


        # make mesh
        #vertices = [(0, 0, 0),(100,100,100), (0, 100,100)]
        #edges = []
        #faces = []
        new_mesh = bpy.data.meshes.new('new_mesh')
        new_mesh.from_pydata(vertices, edges, faces)
        new_mesh.update()
        # make object from mesh
        new_object = bpy.data.objects.new('new_object', new_mesh)
        # make collection
        new_collection = bpy.data.collections.new('new_collection')
        bpy.context.scene.collection.children.link(new_collection)
        # add object to scene collection
        new_collection.objects.link(new_object)
