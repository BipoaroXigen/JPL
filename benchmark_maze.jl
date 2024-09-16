include("jl_libs/structures.jl")
include("jl_libs/nanoflann_tree.jl")
include("jl_libs/collisions.jl")
include("jl_libs/writing.jl")
include("jl_libs/backtracking.jl")
include("jl_libs/objects.jl")
include("jl_libs/probe.jl")

include("jl_libs/optimal_planning_tools.jl")
#include("jl_libs/volumes.jl")

# chose the planning algorithm
if "bare" in ARGS
	include("bare_algo.jl")
elseif "slide" in ARGS
	include("algorithms/slide_algo.jl")
elseif "pop" in ARGS
	include("algorithms/pop_algo.jl")
elseif "ppop" in ARGS
	include("algorithms/pure_pop_algo.jl")
elseif "pso" in ARGS
	include("jl_libs/PSO.jl")
	include("algorithms/pso_algo.jl")
elseif "parzen" in ARGS
	include("jl_libs/parzen.jl")	
	include("algorithms/parzen_algo.jl")
elseif "star" in ARGS
	include("algorithms/flann/star_algo.jl")
elseif "convex" in ARGS
	include("jl_libs/convex_sampling.jl")
	include("algorithms/flann/convex_algo.jl")
elseif "informed" in ARGS
	include("algorithms/flann/informed_algo.jl")
elseif "pi" in ARGS
	include("algorithms/flann/pi_algo.jl")
elseif "bit" in ARGS
	include("algorithms/flann/bit_star.jl")
elseif "pcbit" in ARGS
	include("jl_libs/convex_sampling.jl")
	include("algorithms/flann/a_bit_convex.jl")
elseif "cbit" in ARGS
	include("jl_libs/convex_sampling.jl")
	include("algorithms/flann/convex_bit.jl")
elseif "pibit" in ARGS
	include("jl_libs/convex_sampling.jl")
	include("algorithms/flann/pi_bit.jl")
elseif "pc" in ARGS
	include("jl_libs/convex_sampling.jl")
	include("algorithms/flann/pi_convex_algo.jl")
elseif "flann" in ARGS
	include("algorithms/flann/jump_algo.jl")
elseif "prec" in ARGS
	include("algorithms/flann/prec_algo.jl")
else
	include("algo.jl")
end

println("\nloading objects")

# root path to the meshes
meshes_p = joinpath(@__DIR__, "./meshes")
# paths to the meshes of the objects
rob_p = "/2D/maze/robot.tri"
env_p = "/2D/maze/env.tri"
gol_p = "/2D/maze/goal.tri"
prb_p = "/2D/probe.tri"

# set parameters of the run
dir 		= "maze"

dims 		= 2
rot 		= true
make_log 	= true
set_goal 	= [450., 0, 0, 0, 0, 0]
save_pat 	= true
save_tre 	= true
time_limit 	= false
set_radius 	= 30.
eps 		= 20.
fac		= 2.5
iterations	= 30000
name		= dir*"/"*string(ARGS[1])	# NO FIRST SLASH!

runs 		= 100

mkpath("analyze/"*dir)

P = Params(dims,rot,make_log,set_goal,save_pat,save_tre,time_limit,set_radius,eps,fac,iterations,name)

# loads the meshes for the collision detection
#origin, normals, offset, obstacles = setup_objects(meshes_p, rob_p, env_p, gol_p, prb_p)
origin, normals, offset, obstacles = setup_2D_objects(meshes_p, rob_p, env_p, gol_p, prb_p)

# automatically set the bounds of the space
@show x_min, x_max, y_min, y_max, z_min, z_max = get_bounds(obstacles)

# setup the environment
env = Env(offset, normals, dims, x_min, x_max, y_min, y_max, z_min, z_max)
println("objects loaded\n")

# run the planning algorithm
println("building tree")
q_init = get_origin_state(env)
for i in 1:runs
	@time begin
		global tree = RRT(q_init, env, P)
	end
end
println("tree built\n")
