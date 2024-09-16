using DataStructures

function RRT(q_init::State, env::Env, P::Params)::TREE
	tree = get_tree(q_init.q) # search tree, FLANN datastructure for finding NN
	
	local q_near_node::NODE
	local q_near::State
	local q_new::State

	kveve = PriorityQueue()

	logf = []

	paths  = Vector{PATH}()
	c_best = Inf
	goal   = q_init.q
	start  = q_init.q

	d = get_dimensions(P)
	V = unit_sphere_volume(d)
	O = environment_free_volume(env, P)
	gamma = 2*(1+1/d)^(1/d)*(O/V)^(1/d)
	gamma = gamma*P.rewiring_factor


	if P.epsilon != false
		epsilon = P.epsilon
	else
		epsilon = 20
	end

	time = 0
	batch_size = 100

	for r in 1:P.max_iters
		time += @elapsed begin
		print("\r", r)
		# sample
		if length(tree.goal) > 0
			start, goal, c_best = get_start_end(paths)
		end
		while true
			q_new = informed_sample(goal, start, c_best, P)
			if q_new.x > env.x_min && q_new.x < env.x_max && q_new.y > env.y_min && q_new.y < env.y_max && q_new.z >= env.z_min && q_new.z <= env.z_max
				break
			end
		end
		enqueue!(kveve, q_new, distance(q_new.q, zeros(6)))

		if r%batch_size == 0

		while !isempty(kveve)
			q_new = dequeue!(kveve)
	
			# get NN	
			if P.set_radius == false
		    		radius = min(gamma*(log(r)/r)^(1/d), epsilon)^2
	    		else
		    		radius = P.set_radius^2
	    		end

			neighbors = get_neighbors(tree, q_new.q, radius)
			cost = Inf
			for neighbor in neighbors
				c = neighbor.cost + norm(q_new.q - neighbor.state)
				if c < cost
					q_near_node = neighbor
					cost = c
				end
			end
			if length(neighbors) == 0
				q_near_node = get_neighbor(tree, q_new.q)
			end
			q_near = get_state(q_near_node.state)
			q_new = steer(q_near, q_new, epsilon)
		
			# connect
			if connect(q_near, q_new, env.offset)
				q_new_node = t_append!(tree, q_near_node, q_new.q)
				if cost < Inf
					for neighbor in neighbors
						c = neighbor.cost
						alt_c = q_new_node.cost + norm(neighbor.state - q_new.q)
						if alt_c < c
							if connect(get_state(neighbor.state), q_new, env.offset)
								deleteat!(neighbor.parent.children, findall(x->x==neighbor,neighbor.parent.children))
								neighbor.parent = q_new_node
								push!(q_new_node.children, neighbor)
								neighbor.cost = alt_c
								update_cost(neighbor)
							end
						end
					end
				end

				if is_goal(q_new, env.offset)
					if set_goal != false
						q_new_node = t_append!(tree, q_new_node, set_goal)
					end

					push!(tree.goal, q_new_node)

					path = [q_new_node]
					parents(q_new_node, path) # add path to the node to the array path
					push!(paths, PATH(reverse(path), q_new_node.cost, length(path)))
				end
			end
		end #while
		end #if
		if r%250 == 0 && length(tree.goal) > 0
			goal_node = sort(tree.goal, by=x->x.cost)[1]
			push!(logf, [r, time, goal_node.cost])
		end
		if P.time_limit != false && time > P.time_limit
			goal_node = sort(tree.goal, by=x->x.cost)[1]
			push!(logf, [r, time, goal_node.cost])
			break
		end
end	# timer
	end

	if P.log == true	
	    open(joinpath(@__DIR__, "../../analyze/"*P.name), "a") do w
		    for data in logf
			    println(w, string(data[1]), " ", string(data[2]), " ", string(data[3]))
		    end
	    end
	end
	
	if P.save_tree == true
		if P.dims == 2
			plot_search_tree(tree)
		end
	end
	if P.save_path == true
		println(sort(tree.goal, by=x->x.cost)[1].cost)
		if P.dims == 2
			plot_path(sort(tree.goal, by=x->x.cost)[1])
		end
		if P.dims == 3
			goals = [sort(tree.goal, by=x->x.cost)[1]]
			save_detailed_path(goals)
			#save_path(goals)
		end
	end

	clear_tree(tree)
	return tree
end

