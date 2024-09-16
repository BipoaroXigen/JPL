function approximate_path_slide(q_init::State, offset::Int)::Vector{Tree_Node}
	g = probe(q_init, env.offset)
	if length(g.goal) > 0
		g = g.goal[1]
	end
	#return get_path_in_tree(g)
	return optimized_probe_path(g, offset)
end

#function RRT(q_init::State, env::Env, K=100000, s=100)::Tree
function RRT(q_init::State, env::Env, K=100000, s=100)
	#Random.seed!(13)
	tree, nn = init_tree(env, q_init)

	r_path = reverse(approximate_path_slide(q_init, env.offset))

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State

	volume = 15
	center = []
	helped = volume
	slider_id = 1
	l = length(r_path)

	iters = K
	
	for r in 1:K
		#print("\r", r)
		while true
			if helped < volume
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
				i = 1
				for step in r_path[slider_id:end]
					if !connect(step.content, connection, env.offset)
				#	if !probe_connect(step.content, connection, env.offset)
						center = step.content.q
						center = r_path[slider_id+i-1].content.q
						helped = 0
						slider_id = min(slider_id+i, l)
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
