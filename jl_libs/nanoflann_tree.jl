#compiled as clang++ -O3 -shared -fPIC -o flann_wrap.so flann_wrap.cpp  
#when compiled by g++ julia wont be able to refresh the tree between runs

using Libdl

mutable struct NODE
	state::Vector{Float64}
	parent::Union{Bool, NODE} # false or node
	children::Vector{NODE}
	cost::Float64
#	index::Int
end

mutable struct TREE
	nodes::Vector{NODE}
	length::Int
	goal::Vector{NODE}
	root::NODE
#	flann#::FLANN.FLANNIndex{Float64}
	lib_ptr::Ptr{Nothing}
end

struct PATH
	nodes::Vector{NODE}
	cost::Float64
	length::Int #number of nodes
end

function get_tree(root::Vector{Float64})::TREE
	#lib = Libdl.dlopen("/home/krizjona/nanoflann/flann_wrap.so")
	lib = Libdl.dlopen(joinpath(@__DIR__, "../c_libs/flann_wrap.so"))
	sym = Libdl.dlsym(lib, :add_point_dynamic)

	root_n = NODE(root, false, [], 0)
	tree = TREE([root_n], 1, [], root_n, lib)

	index = @ccall $sym(root::Ptr{Float64})::Cint

	return tree
end

function t_append!(tree::TREE, parent::NODE, state::Vector{Float64})::NODE
	sym = Libdl.dlsym(tree.lib_ptr, :add_point_dynamic)
	index = @ccall $sym(state::Ptr{Float64})::Cint
 	
	node = NODE(state, parent, [], parent.cost + norm(parent.state - state))
	push!(tree.nodes, node)
	push!(parent.children, node)
	tree.length += 1
	
	return node
end

function get_neighbors(tree::TREE, query::Vector{Float64}, r::Float64, maximum::Int=100)::Vector{NODE}

	sym = Libdl.dlsym(tree.lib_ptr, :query)
	length = Vector{Cint}(undef, 1)
	#result = ccall((:query, "/home/krizjona/nanoflann/flann_wrap.so"), Ptr{Cint}, (Ptr{Float64}, Float64, Ref{Cint}), query, r, length)
	
	result = @ccall $sym(query::Ptr{Float64}, r::Float64, length::Ref{Cint})::Ptr{Cint}
	#result = unsafe_wrap(Array, result, min(maximum, length[1]), own=false) # indexes of nodes in tree.nodes
	result = unsafe_wrap(Array, result, min(maximum, length[1]), own=true) # indexes of nodes in tree.nodes

	return tree.nodes[result.+1]
end

function get_neighbor(tree::TREE, query::Vector{Float64})::NODE

	sym = Libdl.dlsym(tree.lib_ptr, :nn_query)
	#result = ccall((:nn_query, "/home/krizjona/nanoflann/flann_wrap.so"), Cint, (Ptr{Float64},), query)
	result = @ccall $sym(query::Ptr{Float64})::Cint

	return tree.nodes[result+1]
end

function clear_tree(tree::TREE)
#	sym = Libdl.dlsym(tree.lib_ptr, :remove_point)
#	for i in 1:length(tree.nodes)
#		a = @ccall $sym(i::Cint)::Cint
#	end

	Libdl.dlclose(tree.lib_ptr)
end

function get_index()
	#lib = Libdl.dlopen("/home/krizjona/nanoflann/flann_wrap.so")
	lib = Libdl.dlopen(joinpath(@__DIR__, "../c_libs/flann_wrap.so"))
	sym = Libdl.dlsym(lib, :construct_points)
	pts = @ccall $sym()::Ptr{Nothing}
	
	sym = Libdl.dlsym(lib, :construct_index)
	a = 3
	b = 3
	index = @ccall $sym(a::Cint, b::Cint, pts::Ptr{Nothing})::Ptr{Nothing}
	return lib, pts, index
end	


function test(lib, pts, index)
	sym = Libdl.dlsym(lib, :add_point)
	root = [0.0,0,0,0,0,0]
	display("pts")
	display(pts)
	display("index")
	display(index)
	index = @ccall $sym(root::Ptr{Float64}, pts::Ptr{Nothing}, index::Ptr{Nothing},)::Cint
	display(index)
end
#=
function open()
	return Libdl.dlopen("/home/krizjona/nanoflann/flann_wrap.so")
end
function close(lib)
	Libdl.dlclose(lib)
end
function add(lib)
	sym = Libdl.dlsym(lib, :add_point_dynamic)
	state = rand(6)
	index = @ccall $sym(state::Ptr{Float64})::Cint
end
function query(lib)
	sym = Libdl.dlsym(lib, :query)
	length = Vector{Cint}(undef, 1)
	query = rand(6)
	r = 30.0^2
	maximum = 100
	result = @ccall $sym(query::Ptr{Float64}, r::Float64, length::Ref{Cint})::Ptr{Cint}
	result = unsafe_wrap(Array, result, min(maximum, length[1]), own=true)

end
=#
