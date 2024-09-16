function probe(q_init::State, offset::Int, K=100000)
	tree, nn = init_tree(env, q_init) # proc zna env?
	#points = Vector{State}()

	for r in 1:K
		#print("\r", r)
		local q_near_node::Tree_Node
		local q_near::State
		local q_new::State
		while true
			q_new = get_random_state(env)
			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content
			break
		end
		con = probe_memory_connect(q_near, q_new, env.offset)

		q_new_node = tree_append!(nn, tree, q_near_node, con)
		if con == q_new
		
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
		end
	end
	free_kd_tree(nn)
	#println("end")
	return tree
end
