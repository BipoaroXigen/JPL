#function RRT(q_init::State, env::Env, K=100000, s=100)::Tree
function RRT(q_init::State, env::Env, r_path, K=100000, s=100)
	#Random.seed!(13)
	tree, nn = init_tree(env, q_init)

        #u = Uniform(-15, 15)
        #x = Uniform(env.x_min, env.x_max)
        #y = Uniform(env.y_min, env.y_max)
        #z = Uniform(env.z_min, env.z_max)

	#points_in_distribution = generate_paths(q_init, env, 1)
	#path = generate_detailed_paths(q_init, env, 1)

	#while true
		#g = probe(q_init, env.offset)
		#g = solve(q_init, env)
		#if length(g.goal) > 0
			#g = g.goal
		#	g = g.goal[1]
			#break
		#end
	#end
	#r_path = reverse(get_path_in_tree(g))
	#r_path = reverse(get_shortest_path_in_tree(g, env.offset))

#	end
	#for i in 1:size(points_in_distribution, 2)
	#open("files/path.txt", "a") do txt
        #        point = points_in_distribution[:, i]
        #        point = reshape(point, length(point), 1)
	#	line = point 
	#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
	#end
	#end
	#open("files/path.txt", "a") do txt
	#for step in r_path
	#	line = step.content.q
	#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
	#end
	#end

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State

	#volume = 500*2
	volume = 500
	#volume = 100
	center = []
	helped = volume
	slider_id = 1
	l = length(r_path)

	iters = K*5
	
	for r in 1:K*5
		#print("\r", r)
		while true
			#cen = sample_unknown_distribution(point, 50, 1., u,x,y,z)
			#cen = sample_unknown_distribution(points_in_distribution[:, i:min(i+3, size(points_in_distribution, 2))], 50, 1., u,x,y,z)
			#cen = [rand(Normal(point[1])), rand(Normal(point[2])), rand(Normal(point[3]))]	
			if helped < volume
				#center = center[1:3]
				#center = reshape(center, length(center), 1)
				#cen = sample_unknown_distribution(center, 50, 1., u,x,y,z)
				cen = [clamp(randn() * 10 + center[1], env.x_min, env.x_max),
			       	       clamp(randn() * 10 + center[2], env.y_min, env.y_max),
			       	       clamp(randn() * 10 + center[3], env.z_min, env.z_max),]
				rot = [rand()*2*π,rand()*2*π,rand()*2*π]
				tmp = append!(cen, rot)
				q_new = get_state(tmp)
				helped += 1
				#open("files/points.txt", "a") do txt
				#	println(txt, string(cen[1]), " ", string(cen[2]), " ", string(cen[3]))
				#end
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
			#if connect(r_path[slider_id].content, connection, env.offset)# && helped >= volume
			if probe_connect(r_path[slider_id].content, connection, env.offset) && helped >= volume
				#display(slider_id)
				i = 1
				for step in r_path[slider_id:end]
					if !connect(step.content, connection, env.offset)
				#	if !probe_connect(step.content, connection, env.offset)
						center = step.content.q
						center = r_path[slider_id+i-1].content.q
						#center = r_path[slider_id+1].content.q
						#open("files/path.txt", "a") do txt
						#	line = center
						#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
						#end
						helped = 0
						#slider_id += i-1
						slider_id += i
						if slider_id == l
							slider_id = 1
						end
						#slider_id += 1
						#println("slided: ", slider_id, ", ", slider_id+i-1, ", ", slider_id+1)
						break
					end
					i += 1
				end
			end
		end
	end
	free_kd_tree(nn)
	#return tree
	return iters, tree
end
