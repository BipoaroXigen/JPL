function initialize_environment(environment::Mesh, id_offset::Int)
	ccall((:init_env, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint, Cint), environment.mesh, environment.triangles_cnt, id_offset)
	#ccall((:init_env, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint, Cint), environment.mesh, environment.triangles_cnt, id_offset)
end

function initialize_robot(robot::Mesh)
	ccall((:init_rob, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint,), robot.mesh, robot.triangles_cnt)
	#ccall((:init_rob, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint,), robot.mesh, robot.triangles_cnt)
end

function initialize_goal(goal::Mesh, id_offset::Int)
	ccall((:init_gol, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint, Cint,), goal.mesh, goal.triangles_cnt, id_offset)
	#ccall((:init_gol, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint, Cint,), goal.mesh, goal.triangles_cnt, id_offset)
end

function initialize_probe(probe::Mesh)
	ccall((:init_prb, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint,), probe.mesh, probe.triangles_cnt)
	#ccall((:init_prb, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Ptr{Float64}}}, Cint,), probe.mesh, probe.triangles_cnt)
end

function steer(start::State, finish::State, epsilon)
	# clip the distance of the sample from nearest neighbor to maximally epsilon

	if distance(start.q, finish.q) < epsilon
		return finish
	end
	dir = finish.q - start.q
	dir = dir/norm(dir)
	pos = dir*epsilon + start.q

	x = pos[1]
	y = pos[2]
	z = pos[3]
	a = pos[4]
	b = pos[5]
	c = pos[6]

	return State(x,y,z,a,b,c,[x,y,z,a,b,c])

end

function get_collision_ids(collision_count::Int32)::Vector{Pair{Cint, Cint}}
	ids = Vector{Pair{Cint, Cint}}()
	if collision_count > 0
		ids = ccall((:tell, joinpath(@__DIR__, "../c_libs/libwrap.so")), Ptr{Pair{Cint, Cint}}, ())
		#ids = ccall((:tell, "/home/krizjona/Project/libwrap.so"), Ptr{Pair{Cint, Cint}}, ())
		ids = unsafe_wrap(Array, ids, collision_count, own = false)
	end
	return ids
end

function connect(a::State, b::State, id_offset::Int)::Bool 
	path = get_path(a, b)

	for step in path
		if detect_collision(step, id_offset)
			return false
		end
	end

	return true
end

function probe_connect(a::State, b::State, id_offset::Int)::Bool 
	path = get_path(a, b)

	for step in path
		if probe_collision(step, id_offset)
			return false
		end
	end

	return true
end

function probe_memory_connect(a::State, b::State, id_offset::Int)::State
	path = get_path(a, b)

	i = 0
	for step in path
		if probe_collision(step, id_offset)
			return path[max(1, i-1)]
		end
		i += 1
	end
	
	return b
end


function memory_connect(a::State, b::State, id_offset::Int)::State
	# return the farthest non colliding point on a line from a neighbor state to the new sample
	
	path = get_path(a, b)

	i = 0
	for step in path
		if detect_collision(step, id_offset)
			return path[max(1, i-1)]
		end
		i += 1
	end
	
	return b
end

function impact_point(a::State, b::State, id_offset::Int)
	path = get_path(a, b)

	i = 0
	for step in path
		ids = decide_collision(step, id_offset)
		if length(ids) > 0
			return path[max(i-1, 1)], ids
		end
		i += 1
	end
	
	return b, []
end

function decide_collision(state::State, id_offset::Int)::Vector{Vector{Int32}}
	# return ids of colliding meshes

	rot = get_rotation_matrix(state.c,state.b,state.a)
	rot = [rot[:,i] for i in 1:size(rot,2)] # change matrix to 2d vector for C++
	tra = get_translation_vector(state.x, state.y, state.z)

	collision_count = ccall((:detect_collision, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	#collision_count = ccall((:detect_collision, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	ids = get_collision_ids(collision_count)
	ids = broadcast(collect, ids)


	if length(ids) > 0
		return ids[first.(ids) .< id_offset]		# only collisions with meshes representing obstacles
		#return ids[first.(ids) .< id_offset-1]
	end
	return ids
end


function probe_goal(state::State, id_offset::Int)::Bool
	rot = get_rotation_matrix(state.c,state.b,state.a)
	rot = [rot[:,i] for i in 1:size(rot,2)] # change matrix to 2d vector for C++
	tra = get_translation_vector(state.x, state.y, state.z)
	collision_count = ccall((:probe_collision, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	#collision_count = ccall((:probe_collision, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	
	if collision_count > 0
		if !goal_id(get_collision_ids(collision_count), id_offset)
			return false
		else
			return true
		end
	end
	return false
end


function is_goal(state::State, id_offset::Int)::Bool
	# if the robot collides with the object representing the goal region return true

	rot = get_rotation_matrix(state.c,state.b,state.a)
	rot = [rot[:,i] for i in 1:size(rot,2)] # change matrix to 2d vector for C++
	tra = get_translation_vector(state.x, state.y, state.z)
	collision_count = ccall((:detect_collision, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	#collision_count = ccall((:detect_collision, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)

	if collision_count > 0
		if !goal_id(get_collision_ids(collision_count), id_offset)
			return false
		else
			return true
		end
	end
	return false
end

function goal_id(ids::Vector{Pair{Cint, Cint}}, id_offset::Int)::Bool
	# is the obstacle the robot collided with the obstacle representing the goal region
	for pair in ids
		if pair[1] < id_offset && pair[2] < id_offset 
			return false
		end
	end
	return true
end

function get_path(start::State, finish::State)::Vector{State} # would returning vector and not state make it faster?
	# connects two states in configuration space with line
	path = Vector{State}()

	p = 100
	for i in 1:p+1 # 1:sampling frequecy + 1
		u = (i-1)/p

		x = start.x+(finish.x-start.x)*u
		y = start.y+(finish.y-start.y)*u
		z = start.z+(finish.z-start.z)*u
		
		a = start.a+(finish.a-start.a)*u
		b = start.b+(finish.b-start.b)*u
		c = start.c+(finish.c-start.c)*u

		push!(path, State(x,y,z,a,b,c,[x,y,z,a,b,c]))
	end

	return path
end

function detect_collision(state::State, id_offset::Int)::Bool
	rot = get_rotation_matrix(state.c,state.b,state.a)
	rot = [rot[:,i] for i in 1:size(rot,2)] # change matrix to 2d vector for C++
	tra = get_translation_vector(state.x, state.y, state.z)

	# detect only collisions with environment
	# if at least one collision with env ==> true
	
	
	collision_count = ccall((:detect_collision, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	#collision_count = ccall((:detect_collision, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	if collision_count > 0
		for pair in get_collision_ids(collision_count)
			if pair[1] < id_offset
#			if pair[1] + pair[2] < id_offset
#			if pair[1] < id_offset && pair[2] < id_offset 
				return true
			end
		end
	end

	return false
end

function probe_collision(state::State, id_offset::Int)::Bool
	rot = get_rotation_matrix(state.c,state.b,state.a)
	rot = [rot[:,i] for i in 1:size(rot,2)] # change matrix to 2d vector for C++
	tra = get_translation_vector(state.x, state.y, state.z)

	collision_count = ccall((:probe_collision, joinpath(@__DIR__, "../c_libs/libwrap.so")), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	#collision_count = ccall((:probe_collision, "/home/krizjona/Project/libwrap.so"), Cint, (Ptr{Ptr{Float64}}, Ptr{Float64},), rot, tra)
	if collision_count > 0
		for pair in get_collision_ids(collision_count)
			#if pair[1] + pair[2] < id_offset
			if pair[1] < id_offset
#		 	if pair[1] < id_offset && pair[2] < id_offset 
				return true
			end
		end
	end

	return false
end

function get_rotation_matrix(α, β, γ)::Matrix{Float64}
	# yaw pitch roll
	m = [cos(α)*cos(β) cos(α)*sin(β)*sin(γ)-sin(α)*cos(γ) cos(α)*sin(β)*cos(γ)+sin(α)*sin(γ)
	     sin(α)*cos(β) sin(α)*sin(β)*sin(γ)+cos(α)*cos(γ) sin(α)*sin(β)*cos(γ)-cos(α)*sin(γ)
	     -sin(β) cos(β)*sin(γ) cos(β)*cos(γ)]
	return m
end

function get_translation_vector(x::Float64, y::Float64, z::Float64)::Vector{Float64}
	return [x, y, z]
end
