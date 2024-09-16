using Random#, Distributions

mutable struct State
	x::Float64 # x coordinate
	y::Float64 # y coordinate
	z::Float64 # z coordinate
	a::Float64 # rotation angle around x roll
	b::Float64 # rotation angle around y pitch
	c::Float64 # rotation angle around z yaw

	q::Vector{Float64} # configuation space coordinates
end

mutable struct Dist
	d::Vector{Float64}
	v::Float64
end

mutable struct Tree_Node
	parent#::Tree_Node using boolean for root nodes
	children::Array{Tree_Node, 1}
	content::State
	radius::Float64
	distance::Float64
	dist#::Dist same as parent
end

mutable struct Tree
	dimensions::Int8
	nodes::Array{Tree_Node, 1} # one dimensional array of nodes
	goal::Vector{Tree_Node}# another one
end

mutable struct Env
	offset::Int
	normals::Vector{Vector{Float64}}
	dimensions::Int8
	x_min::Float64
	x_max::Float64
	y_min::Float64
	y_max::Float64
	z_min::Float64
	z_max::Float64
end

mutable struct Params
	dims::Int8	# 2,3 (dimensionality of the represented space)
	rot::Bool	# true: with rotations, false: without
	log::Bool	# write log
	set_goal::Union{Vector{Float64}, Bool} # if goal is one point, specify coords
	save_path::Bool
	save_tree::Bool
	time_limit::Union{Int64, Bool}
	set_radius::Union{Float64, Bool}
	epsilon::Union{Float64, Bool}
	rewiring_factor::Float64
	max_iters::Int64
	name::String
end

mutable struct Mesh
	mesh::Array{Any}
	triangles_cnt::Int64
end

function sample_distribution(dist::Dist)::State
	d = dist.d

	x = rand((d[1]-d[7]):(d[1]+d[7]))
	y = rand((d[2]-d[8]):(d[2]+d[8]))
	z = rand((d[3]-d[9]):(d[3]+d[9]))

	a = rand((d[4]-d[10]):(d[4]+d[10]))
	b = rand((d[5]-d[11]):(d[5]+d[11]))
	c = rand((d[6]-d[12]):(d[6]+d[12]))
	
	return get_state([x,y,z,a,b,c])
end

function get_random_state(env::Env, p::Params)::State
	if p.dims == 2 && p.rot == true
		return get_random_state2D(env)
	end
	if p.dims == 2 && p.rot == false
		q = get_random_state2D(env)
		q.q[4:end] = 0
		q.c = 0
		return q
	end
	if p.dims == 3 && p.rot == true
		return get_random_state(env)
	end
	if p.dims == 3 && p.rot == false
		q = get_random_state(env)
		q.q[4:end] = 0
		q.a = 0
		q.b = 0
		q.c = 0
		return q
	end
end

function get_random_state(env::Env)::State
	x_size = abs(env.x_min-env.x_max)
	x = rand()*x_size+env.x_min
	
	y_size = abs(env.y_min-env.y_max)
	y = rand()*y_size+env.y_min
	
	z_size = abs(env.z_min-env.z_max)
	z = rand()*z_size+env.z_min
	
	a = rand()*2*π
	b = rand()*2*π
	c = rand()*2*π # rotation around z axis

	return get_state([x,y,z,a,b,c])
end

function get_random_state2D(env::Env)::State
	x_size = abs(env.x_min-env.x_max)
	x = rand()*x_size+env.x_min
	
	y_size = abs(env.y_min-env.y_max)
	y = rand()*y_size+env.y_min
	
	r = rand()*2*π

	return get_state([x,y,0,0,0,r])
end


function get_state(coords)::State
	return State(coords[1], coords[2], coords[3], coords[4], coords[5], coords[6], coords)
end

function get_origin_state(env::Env)::State
	z = 0.
	#zz = zeros(env.dimensions)
	zz = zeros(6)
	return State(z,z,z,z,z,z,zz)
end

function tree_append!(nn::Ptr{Any}, tree::Tree, parent, new_content::State)
	id = length(tree.nodes) + 1
	point = new_content.q
	dim = tree.dimensions
	
	ccall((:append_kd, joinpath(@__DIR__, "../c_libs/libnn.so")), Cint, (Ptr{Any}, Cint, Ptr{Float64}, Cint), nn, id, point, dim)
	#ccall((:append_kd, "/home/krizjona/Project/libnn.so"), Cint, (Ptr{Any}, Cint, Ptr{Float64}, Cint), nn, id, point, dim)
	
	new_node = Tree_Node(parent, [], new_content, Inf, 0, false)
	if parent isa Tree_Node
		push!(parent.children, new_node)
		new_node.distance = parent.distance + distance_ND(new_content.q[1:2], parent.content.q[1:2])
	end
	push!(tree.nodes, new_node)
	
	return new_node

end

function build_kd_tree(tree::Tree, root::State)::Ptr{Any}
	dim = tree.dimensions
	nn = ccall((:build_tree, joinpath(@__DIR__, "../c_libs/libnn.so")), Ptr{Any}, (Cint,), dim)
	#nn = ccall((:build_tree, "/home/krizjona/Project/libnn.so"), Ptr{Any}, (Cint,), dim)
	#ccall((:find_nn, "./libnn.so"), Cint, (Cint, Ptr{Float64}, Cint), 1, root.q, dim)
	return nn
end

function init_tree(env::Env, q_init::State)::Tuple{Tree, Ptr{Any}}
	tree = Tree(env.dimensions, [], []) # dimensions, nodes, goal states
	nn = build_kd_tree(tree, q_init) 
	tree_append!(nn, tree, false, q_init) # tree, parent, new content

	return (tree, nn)
end

function free_kd_tree(nn::Ptr{Any})
	ccall((:free_tree, joinpath(@__DIR__, "../c_libs/libnn.so")), Cint, (Ptr{Any},), nn)
	#ccall((:free_tree, "/home/krizjona/Project/libnn.so"), Cint, (Ptr{Any},), nn)
end

function ann(nn::Ptr{Any}, tree::Tree, data::State)
	dimension = tree.dimensions
	id = length(tree.nodes) + 1
	nn_index = ccall((:find_nn, joinpath(@__DIR__, "../c_libs/libnn.so")), Cint, (Ptr{Any}, Ptr{Float64}, Cint), nn, data.q, dimension)
	#nn_index = ccall((:find_nn, "/home/krizjona/Project/libnn.so"), Cint, (Ptr{Any}, Ptr{Float64}, Cint), nn, data.q, dimension)
	nn = tree.nodes[nn_index]
	return nn
end

function distance(a, b)
	A = a[1] - b[1]
	B = a[2] - b[2]
	C = a[3] - b[3]
	D = a[4] - b[4]
	E = a[5] - b[5]
	F = a[6] - b[6]
	
	return sqrt(A^2 + B^2 + C^2 + D^2 + E^2 + F^2)
end

function distance_ND(a::Vector, b::Vector)::Float64
	return norm(a - b)
end
