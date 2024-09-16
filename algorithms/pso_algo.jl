function approximate_path_pso(q_init::State, offset::Int)::Vector{State}
	g = probe(q_init, offset)
	if length(g.goal) > 0
		g = g.goal[1]
	end
	#p_path = reverse(get_path_in_tree(g))
	p_path = reverse(optimized_probe_path(g, offset))

	path = Vector{State}()
	for step in p_path
		push!(path, step.content)
	end
	return path
end

#function RRT(q_init::State, env::Env, K=100000, s=100)::Tree
function RRT(q_init::State, env::Env, K=100000, s=100)
	#Random.seed!(14)
	tree, nn = init_tree(env, q_init)

	path = approximate_path_pso(q_init, env.offset)
	r_path = reverse(path)

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State
	local dist::Dist

	volume = 25
	helped = volume
	tried = [] # normals of faces, for which PSO was already run
	slider_id = 1
	last = 0

	
	iters = K
	
	for r in 1:K
		#print("\r", r)
		while true
			if helped < volume
#				q_new = sample_distribution(dist)
				sample = [clamp(randn()*3+dist.d[1], dist.d[1]-dist.d[7],  dist.d[1]+dist.d[7]),
				          clamp(randn()*3+dist.d[2], dist.d[2]-dist.d[8],  dist.d[2]+dist.d[8]),
				          clamp(randn()*3+dist.d[3], dist.d[3]-dist.d[9],  dist.d[3]+dist.d[9]),
				          clamp(randn()*1+dist.d[4], dist.d[4]-dist.d[10], dist.d[4]+dist.d[10]),
				          clamp(randn()*1+dist.d[5], dist.d[5]-dist.d[11], dist.d[5]+dist.d[11]),
				          clamp(randn()*1+dist.d[6], dist.d[6]-dist.d[12], dist.d[6]+dist.d[12])]
				#sample = [clamp(randn()*(abs(dist.d[1])+abs(dist.d[7]))+dist.d[1], env.x_min,  env.x_max),
				#          clamp(randn()*(abs(dist.d[2])+abs(dist.d[8]))+dist.d[2], env.y_min,  env.y_max),
				#          clamp(randn()*(abs(dist.d[3])+abs(dist.d[9]))+dist.d[3], env.z_min,  env.z_max),
				#          clamp(randn()*(abs(dist.d[4])+abs(dist.d[10]))+dist.d[4], 0, 2*π),
				#          clamp(randn()*(abs(dist.d[5])+abs(dist.d[11]))+dist.d[5], 0, 2*π),
				#          clamp(randn()*(abs(dist.d[6])+abs(dist.d[12]))+dist.d[6], 0, 2*π)]
				#open("files/path.txt", "a") do txt
				#	line = sample
				#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
				#end
				q_new = get_state(sample)
				helped += 1
			else
				q_new = get_random_state(env)
			end

			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content

			break
		end
		connection, norm_id = impact_point(q_near, q_new, env.offset)
		q_new_node = tree_append!(nn, tree, q_near_node, connection)

		if connection == q_new
			if is_goal(q_new, env.offset)
				#println("found: ", r)
				iters = r
				push!(tree.goal, q_new_node)
				break
			end
		else
			
			if helped >= volume && (r > (2*volume)+last || r < volume)
				normals_id = unique(Iterators.flatten(norm_id)) .+ 1	
				if normals_id[1] ∉ tried
					normals = env.normals[normals_id]
					normal = normals[1]
					append!(tried, normals_id[1])
					for i in 1:length(r_path)
						if probe_connect(r_path[i], connection, env.offset)
							step = r_path[i]
							index = (length(path)+1)-i
							if index >= slider_id
								dist = PSO(step, index, path, env, connection, q_near_node.dist)
								helped = 0
								slider_id += 1
								last = r
							end
							break
						end
					end
				end
			end
		end
	end
	#println("length: ", length(path))
	#println("helped: ", p)
	free_kd_tree(nn)
	#return tree
	return iters, tree
end
