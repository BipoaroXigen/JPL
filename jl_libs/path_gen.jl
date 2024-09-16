function generate(q_init::State, offset::Int, K=100000)
	tree, nn = init_tree(env, q_init) # proc zna env?
	#points = Vector{State}()

	for r in 1:K
		#print("\r", r)
		local q_near_node::Tree_Node
		local q_near::State
		local q_new::State
		while true
			#q_new = get_random_state2D(env)
			q_new = get_random_state(env)
			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content
			q_new = steer(q_near, q_new, 10)
			break
		end
		#con = probe_memory_connect(q_near, q_new, env.offset)

		q_new_node = tree_append!(nn, tree, q_near_node, q_new)
		#q_new_node = tree_append!(nn, tree, q_near_node, con)
		#if con == q_new
		
		#if probe_connect(q_near, q_new, offset)
		#	q_new_node = tree_append!(nn, tree, q_near_node, q_new)
			if probe_goal(q_new, offset)
				push!(tree.goal, q_new_node)
				#println()
				#println(r, " goal")
				break
			end
		#else
		#	q_new_node = tree_append!(nn, tree, q_near_node, con)
		#end
	end
	free_kd_tree(nn)
	#println("end")
	return tree
end


function generate_paths(q_init, env::Env, iterations::Int64)::Matrix{Float64}

	path = []
	
	local g
	for i in 1:iterations
		while true
			g = probe(q_init, env.offset)
			if length(g.goal) > 0
				g = g.goal[1]
				break
			end
		end
		#g = reverse(optimized_probe_path(g, env.offset))
		g = reverse(get_path_in_tree(g))
		for q in g
			push!(path, q.content.q)
		end
	end

	path = transpose(mapreduce(permutedims, vcat, path))
	return path[1:3,:]
	#return path[1:6,:]
end

function generate_detailed_paths(q_init, env::Env, iterations::Int64)::Matrix{Float64}
	path = []
	
	local g
	for i in 1:iterations
		while true
			g = probe(q_init, env.offset)
			if length(g.goal) > 0
				g = g.goal[1]
				break
			end
		end
		g = reverse(optimized_probe_path(g, env.offset))
		#g = reverse(get_path_in_tree(g))
		first = g[1]
		for step in g[2:end]
			append!(path, fast_get_path(first.content, step.content))
#			for substep in subpath
#				push!(path, substep.q)
#			end
			first = step
		end
	end

	path = transpose(mapreduce(permutedims, vcat, path))
	return path[1:3,:]
	#return path[1:6,:]
end

function fast_get_path(start::State, finish::State)#::Array{State, 1} # would returning vector and not state make it faster?
    	# connects two states in configuration space with line
#        path = Array{State, 1}()
	path = []

        p = 100

        for i in 1:p+1 # 1:sampling frequecy + 1
                 u = (i-1)/p

                 x = start.x+(finish.x-start.x)*u
                 y = start.y+(finish.y-start.y)*u
                 z = start.z+(finish.z-start.z)*u

                 a = start.a+(finish.a-start.a)*u
                 b = start.b+(finish.b-start.b)*u
                 c = start.c+(finish.c-start.c)*u

                 push!(path, [x,y,z,a,b,c])
         end

         return path
end

function generate_path(q_init, env::Env)::Matrix{Float64}

	path = []
	
	local g
	while true
		g = generate(q_init, env.offset)
		if length(g.goal) > 0
			g = g.goal[1]
			break
		end
	end
	#g = reverse(optimized_probe_path(g, env.offset))
	g = reverse(get_path_in_tree(g))
	for q in g
		push!(path, q.content.q)
	end

	path = transpose(mapreduce(permutedims, vcat, path))
	#return path[[1,2,6],:]
	return path[1:6,:]
end
