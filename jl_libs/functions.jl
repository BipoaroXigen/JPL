using DelimitedFiles

function save_tree(tree)

	open("files/nodes.txt", "w") do txt
		println(txt, "0.0 0.0 0.0")
	end

	root = tree.nodes[1]
	for node in tree.nodes
		for child in node.children
			open("files/nodes.txt","a") do txt
				println(txt, string(node.content.x), " ", string(node.content.y), " ", string(node.content.z), " ", string(node.content.a), " ", string(node.content.b), " ", string(node.content.c))
				println(txt, string(child.content.x), " ", string(child.content.y), " ", string(child.content.z), " ", string(child.content.a), " ", string(child.content.b), " ", string(child.content.c), "\n")
			end
		end
	end

end

function plot_search_tree(tree)			# for gnuplot plotting

	open("files/nodes.txt", "w") do txt
		println(txt, "0.0 0.0 0.0")
	end

	open("files/nodes.txt","a") do txt
		for node in tree.nodes
			for child in node.children
				println(txt, string(node.state[1]), " ", string(node.state[2]), " ", string(node.state[3]))
				println(txt, string(child.state[1]), " ", string(child.state[2]), " ", string(child.state[3]), "\n")
			end
		end
	end

end

function setup_2D_objects(path::String, rob_p::String, env_p::String, gol_p::String, prb_p::String)
	#robot = setup_2D_object(path * "/robot.tri")
	robot = setup_2D_object(path * rob_p)
	#environment = setup_2D_object(path * "/environment.tri")
	environment = setup_2D_object(path * env_p)
	#goal = setup_2D_object(path * "/goal.tri")
	goal = setup_2D_object(path * gol_p)
	#probe = setup_2D_object(path * "/probe.tri")
	probe = setup_2D_object(path * prb_p)

	env = collect(Iterators.flatten(environment.mesh))
	env = mapreduce(permutedims, vcat, env)

	initialize_robot(robot)
	offset = robot.triangles_cnt
	initialize_environment(environment, offset)
	offset = robot.triangles_cnt + environment.triangles_cnt
	initialize_goal(goal, offset)
	initialize_probe(probe)
	initial_point = robot.mesh[1][1]

	normals = [] # makes no sense in 2D

	return initial_point, normals, offset, env
end

function setup_2D_object(path)::Mesh
	triangles = get_triangles(path)
	object = Mesh(triangles, length(triangles))

	return object
end

function setup_objects(path::String, rob_p::String, env_p::String, gol_p::String, prb_p::String)
	
#	robot = setup_object(path * "/robot.obj")
	robot = setup_object(path * rob_p)
	#environment = setup_object(path * "/environment.obj")
	environment = setup_object(path * env_p)
	#goal = setup_object(path * "/goal.obj")
	goal = setup_object(path * gol_p)
	#probe = setup_object(path * "/probe.obj")
	probe = setup_object(path * prb_p)

	env = collect(Iterators.flatten(environment.mesh))
	env = append!(env, collect(Iterators.flatten(robot.mesh)))
	env = append!(env, collect(Iterators.flatten(goal.mesh)))
	env = mapreduce(permutedims, vcat, env)


	initialize_robot(robot)
	offset = robot.triangles_cnt

	initialize_environment(environment, offset)
	offset = robot.triangles_cnt + environment.triangles_cnt
#
	initialize_goal(goal, offset)
	initialize_probe(probe)

	initial_point = robot.mesh[1][1]

	get_normals(robot)
	get_normals(environment)
	get_normals(goal)

	normals = get_normals(robot)
	normals = append!(normals, get_normals(environment))
	normals = append!(normals, get_normals(goal))

	return initial_point, normals, offset, env
end

function get_normals(mesh)
	triangles = mesh.mesh
	normals = Vector{Vector{Float64}}()
	
	for triangle in triangles
		borders = get_triangle_borders(triangle)
		normal = get_plane_normal(borders[1], borders[2])
		push!(normals, vec(normal))
	end

	return normals
end

function get_triangle_borders(triangle)
	a = triangle[1]
	b = triangle[2]
	c = triangle[3]

	C = b - a
	B = c - a

	return [C, B]
end

function get_plane_normal(a, b)::Matrix{Float64}
	A = [a[1] a[2] a[3]
	     b[1] b[2] b[3]]

	return nullspace(A)
end

function get_triangles(path::String)#::Array{Float64, 2}
	triangles = []#Array{Float64, 3}

	file = readdlm(path, ' ', Float64)

	rows = length(file)/3
	for i in 1:3:rows
		i = floor(Int, i)
		triangle = file[i:i+2,:]
		triangle = [triangle[i, :] for i in 1:size(triangle,1)]
		push!(triangles, triangle)
	end
	
	return triangles
end

function split_to_float(str::String)::Vector{Float64}
	res = Vector{Float64}()
	str = split(str)
	for st in str
		append!(res, parse(Float64, st))
	end

	return res
end

function get_triangles_obj(path::String)
	triangles = []#::Vector{Vector{Vector{Float64}}}()
	vertices = []#::Vector{Vector{Float64}}()

	vert_cnt = 0
	if path == "./meshes/probe.obj"
		p = "./meshes/probe.obj"
	else
		p = path
	end
	open(p, "r") do obj
		while !eof(obj)
			line = readline(obj)
			sline = split(line)
			if sline[1] == "v"
				vertex = split_to_float(line[2:end])
				if "./meshes/probe.obj" == path
#					vertex = vertex*0.1
				end
				push!(vertices, vertex)
			end
			if sline[1] == "f"
				triangle = []#::Vector{Vector{Float64}}()

				a = parse(Int, split(sline[2], '/')[1])
			 	b = parse(Int, split(sline[3], '/')[1])
			 	c = parse(Int, split(sline[4], '/')[1])

				push!(triangle, vertices[a])
				push!(triangle, vertices[b])
				push!(triangle, vertices[c])

				push!(triangles, triangle)
			end
		end
	end
	
	return triangles
end

function setup_object(path)::Mesh
	triangles = get_triangles_obj(path)
	object = Mesh(triangles, length(triangles))

	return object
end

function get_bounds(obstacles::Matrix{Float64})
	x_min = minimum(obstacles[:, 1], dims=1)[1]
	x_max = maximum(obstacles[:, 1], dims=1)[1]
	y_min = minimum(obstacles[:, 2], dims=1)[1]
	y_max = maximum(obstacles[:, 2], dims=1)[1]
	z_min = minimum(obstacles[:, 3], dims=1)[1]
	z_max = maximum(obstacles[:, 3], dims=1)[1]

	return x_min, x_max, y_min, y_max, z_min, z_max
end

function get_path_in_tree(node) #returns path from this node to root
	path = [node]
	while node.parent isa Tree_Node
		node = node.parent
		push!(path, node)
	end

	return path
end

function get_shortest_path_in_tree(nodes::Vector{NODE}, offset::Int)::Vector{NODE}
	shortest_len = Inf
	#shortest_path = Vector{Tree_Node}()
	shortest_path = []#Vector{Tree_Node}()
	sub_path = []

	for node in nodes
		path = [node]
		current = node
		parent = node.parent
		prev = false

		while parent != false
			if connect(get_state(parent.state), get_state(current.state), offset)
				prev = parent
				parent = parent.parent
			else
				push!(path, prev)
				current = prev
				prev = parent
				#parent = parent.parent
			end
		end
		push!(path, prev)
		
		len = length(path)
		if len < shortest_len
			shortest_path = path
			shortest_len = len
			sub_path = node
		end
	end
	return shortest_path
end

# obsolete
function get_shortest_path_in_tree(nodes::Vector{Tree_Node}, offset::Int)::Vector{Tree_Node}
	shortest_len = Inf
	#shortest_path = Vector{Tree_Node}()
	shortest_path = []#Vector{Tree_Node}()
	sub_path = []

	for node in nodes
		path = [node]
		current = node
		parent = node.parent
		prev = false

		while parent isa Tree_Node
			if connect(parent.content, current.content, offset)
				prev = parent
				parent = parent.parent
			else
				push!(path, prev)
				current = prev
				prev = parent
				#parent = parent.parent
			end
		end
		push!(path, prev)
		
		len = length(path)
		if len < shortest_len
			shortest_path = path
			shortest_len = len
			sub_path = node
		end
	end
	return shortest_path
end

# obsolete
function gspit(node::Tree_Node, id_offset::Int)::Vector{Tree_Node}
	path = Vector{Tree_Node}()
	push!(path, node)
	parent = node.parent
	prev = false

	while parent isa Tree_Node
		if probe_connect(last(path).content, parent.content, id_offset)
			prev = parent
			parent = parent.parent
		else
			push!(path, prev)
		end
	end
	push!(path, prev)
	
	return path
end

# obsolete
function optimized_probe_path(node::Tree_Node, id_offset::Int)::Vector{Tree_Node}
	path = Vector{Tree_Node}()
	push!(path, node)
	current = node
	parent = node.parent
	prev = false

	while parent isa Tree_Node
		if probe_connect(parent.content, current.content, id_offset)
			prev = parent
			parent = parent.parent
		else
			push!(path, prev)
			current = prev
			prev = parent
		end
	end
	push!(path, prev)

	return path
end

