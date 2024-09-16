#function RRT(q_init::State, env::Env, K=100000)::Tree
function RRT(q_init::State, env::Env, points_in_distribution, K=100000)
	#Random.seed!(13)
	tree, nn = init_tree(env, q_init)

        u = Uniform(-15, 15)
        x = Uniform(env.x_min, env.x_max)
        y = Uniform(env.y_min, env.y_max)
        z = Uniform(env.z_min, env.z_max)

	#@time points_in_distribution = generate_paths(q_init, env, 1)
	#points_in_distribution = generate_detailed_paths(q_init, env, 1)

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State

	iters = K

	for r in 1:K
		while true
			cen = sample_unknown_distribution(points_in_distribution, 50, 1., u,x,y,z)
			rot = [rand()*2*π,rand()*2*π,rand()*2*π]
			tmp = append!(cen, rot)
			#open("files/points.txt", "a") do txt
			#	println(txt, string(cen[1]), " ", string(cen[2]), " ", string(cen[3]))
			#end
			q_new = get_state(tmp)

			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content
			break
		end
		connection, norm_id = impact_point(q_near, q_new, env.offset)
		q_new_node = tree_append!(nn, tree, q_near_node, connection)

		if connection == q_new
			if is_goal(q_new, env.offset)
				#println("\nfound: ", r)
				iters = r
				push!(tree.goal, q_new_node)
				break
			end
		else
		end
	end
	free_kd_tree(nn)
	#return tree
	return iters, tree
end
