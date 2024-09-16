function convex_sample(projector::Matrix{Float64}, hull::Linked_list, paths)
	q_new = get_random_state(env)
	point = q_new.q
	i = 1
	while is_inside_rhull(point, projector*point, hull) == false
		q_new = get_random_state(env)
		point = q_new.q
		i += 1
		if i > 10
			q_new = get_random_state(env)
			point = q_new.q
			return q_new
		end
	end
	return q_new
end

function convex_sample2D(projector::Matrix{Float64}, hull::Linked_list, paths)
	q_new = get_random_state2D(env)
	point = q_new.q[[1,2,end]]
	i = 1
	while is_inside_rhull(point, projector*point, hull) == false
		q_new = get_random_state2D(env)
		point = q_new.q[[1,2,end]]
		i += 1
		if i > 10
			q_new = get_random_state2D(env)
			point = q_new.q[[1,2,end]]
		end
	end
	return q_new
end

function get_hull_points(paths::Vector{PATH}, goal_node::NODE, start::Vector{Float64})::Tuple{Linked_list, Matrix{Float64}}
	
	path = [goal_node]
	parents(goal_node, path)
	push!(paths, PATH(reverse(path), goal_node.cost, length(path)))

	p = zeros(length(path), 6)
	i = 1
	for step in path
		p[i, :] = step.state
		i += 1
	end
	
	if length(path) > 6
		ch = chull(p) # input matrix(count, dimension) (vectors in rows)
		hull = p'[:, ch]
	else
		hull = p'[:, :]
	end
				
	line = goal_node.state-q_init.q
	A = reshape(line, 6, 1)
	projector = A*inv(A'A)*A'

	return prepare_rotated_hull_points!(hull, projector*hull, start, goal_node.state), projector
end

function get_hull_points2D(paths::Vector{PATH}, goal_node::NODE, start::Vector{Float64})::Tuple{Linked_list, Matrix{Float64}}
	
	path = [goal_node]
	parents(goal_node, path)
	push!(paths, PATH(reverse(path), goal_node.cost, length(path)))

	p = zeros(length(path), 3)
	i = 1
	for step in path
		p[i, :] = step.state[[1,2,end]]
		i += 1
	end
	
	if length(path) > 3
		ch = chull(p) # input matrix(count, dimension) (vectors in rows)
		hull = p'[:, ch]
	else
		hull = p'[:, :]
	end
				
	line = goal_node.state-q_init.q
	A = reshape(line[[1,2,end]], 3, 1)
	projector = A*inv(A'A)*A'

	return prepare_rotated_hull_points!(hull, projector*hull, start[[1,2,end]], goal_node.state[[1,2,end]]), projector
end


function RRT(q_init::State, env::Env, P::Params)::TREE
	tree = get_tree(q_init.q) # search tree, FLANN datastructure for finding NN
	
	local q_near_node::NODE
	local q_near::State
	local q_new::State
	
	projector = 0
	r_hull = 0

	logf = []
	
	paths  = Vector{PATH}()
	c_best = Inf
	goal   = q_init.q
	start  = q_init.q

	d = get_dimensions(P)
	V = unit_sphere_volume(d)
	O = environment_free_volume(env, P)
	gamma = 2*(1+1/d)^(1/d) * (O/V)^(1/d)
	gamma = gamma*P.rewiring_factor

	if P.epsilon != false
		epsilon = P.epsilon
	else
		epsilon = 20
	end

	time = 0

	for r in 1:P.max_iters
		time += @elapsed begin
		#print("\r", r)
		# sample
		if length(tree.goal) > 0
			if r%1000 == 0
				goal_node = sort(tree.goal, by=x->x.cost)[1]
				push!(logf, [r, time, goal_node.cost])
			    if P.dims == 2      # rotation is expected for now
				    r_hull, projector = get_hull_points2D(paths, goal_node, start)
				else #3
				    r_hull, projector = get_hull_points(paths, goal_node, start)
			    end
			end
			if P.dims == 2      # rotation is expected for now
			    q_new = convex_sample2D(projector, r_hull, paths)
			else #3
			    q_new = convex_sample(projector, r_hull, paths)
			end
			else
				q_new = informed_sample(goal, start, c_best, P)
			end
	
		# get NN
		if P.set_radius == false
			radius = min(gamma*(log(r)/r)^(1/d), epsilon)^2
        else
	        radius = P.set_radius^2
	    end
		#neighbors = get_neighbors(tree, q_new.q, radius, 50)
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
			if cost != Inf
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
				#println("found: ", time, "\t ", sort(tree.goal, by=x->x.cost)[1].cost)
				
				if length(tree.goal) > 1
					continue
				end

				goal_node = q_new_node
			    if P.dims == 2      # rotation is expected for now
				    r_hull, projector = get_hull_points2D(paths, goal_node, start)
				else #3
				    r_hull, projector = get_hull_points(paths, goal_node, start)
			    end
			end
		#end
		end		
		if r%250 == 0 && length(tree.goal) > 0
			goal_node = sort(tree.goal, by=x->x.cost)[1]
			push!(logf, [r, time, goal_node.cost])
			#plot_search_tree(tree)
			#plot_path(sort(tree.goal, by=x->x.cost)[1])
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
