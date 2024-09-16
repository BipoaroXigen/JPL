
function update_cost(node::NODE)
	for child in node.children
		child.cost = node.cost + norm(child.state - node.state)
		update_cost(child)
	end
end

function test_loops(root::NODE)
	for child in root.children
		test_loops(child)
	end
end

function parents(node::NODE, pts::Vector{NODE})
	if node == false
		return nothing
	end
	if node.parent == false
		return nothing
	end
	push!(pts, node.parent)
	parents(node.parent, pts)
end

function get_dimensions(P::Params)::Int64
	if P.dims == 2
		if P.rot == true
			return 3
		else
			return 2
		end

	elseif P.dims == 3
		if P.rot == true
			return 6
		else
			return 3
		end
	end
end

function unit_sphere_volume(d)
	# faster with if than using the Eulers gamma function
	if d == 2
		return pi
	end
	if d == 3
		return (4/3)*pi
	end
	if d == 6
		return pi^3 / 6
	end
	display("computation of volume of unit ball of dimension "*string(d)*" needs to be implemented (jl_libs/optimal_planning_tools.jl row 43)")
end

function environment_free_volume(env::Env, P::Params)	# implement with monte carlo probably
#	return 500*500*pi
	return 9.89657e+7
end

function plot_ellipse(start, goal, path_len, num)

	color = "blue"
	if num == 1
		color = "green"
	end

	center = (start+goal)./2
	w = path_len
	h = sqrt(abs(path_len^2-(norm(start-goal))^2))

	dif = goal[1:2]-start[1:2]
	angle = rad2deg(atan(dif[2], dif[1]))

	text = "set object "*string(num)*" ellipse center "*string(center[1])*","*string(center[2])*" size "*string(w)*","*string(h)*" angle "*string(angle)*" front fc rgb \""*color*"\" fillstyle transparent solid 0.1"
	open("files/ellipses", "a") do txt
		println(txt, text)
	end
end

# get part of path for smaller elypse
function get_start_end(paths::Vector{PATH})
	#selected_path = shuffle!(paths)[1]
	selected_path = sort(paths, by=x->x.cost)[1]

	c 	= 1						# minimal cardinality of chosen path segment
	len	= round(Int, rand()*(selected_path.length-c))	# length of the chosen path segment
	start_m = selected_path.length-len			# highest possible starting index
	start_i = 1 + round(Int, rand()*(start_m-1))		# beware indexing from 1
	goal_i	= start_i + len

	start   = selected_path.nodes[start_i]
	goal    = selected_path.nodes[goal_i]

	c_best = goal.cost - start.cost
	start  = start.state
	goal   = goal.state

	return start, goal, c_best
end

function sample_ball(dims::Int)::Vector{Float64}
	u = randn(dims)
	d = sqrt(sum(u.^2))
	return x = u/d
end

function get_elipse_rotation(goal::Vector{Float64}, start::Vector{Float64}, dims::Int)::Matrix{Float64}
	one = append!([1], zeros(dims-1))
	a = (goal - start)/norm(goal - start)
	M = a*one'
	U, S, V = svd(M)
	u = det(U)
	v = det(V)
	diag = Matrix{Float64}(I, dims, dims)
	diag[dims, dims] = v
	diag[dims-1, dims-1] = u
	C = U*diag*V 
	return C
end

function get_transformation(c_best::Float64, c_min::Float64, dims::Int)::Matrix{Float64}
	ll = c_best/2
	l = insert!(ones(dims-1).*((sqrt(c_best^2-c_min^2))/2), 1, ll)
	L = Diagonal(l)
	return L
end

function informed_sample(goal::Vector, start::Vector, c_max::Float64, p::Params)::State
	if p.dims == 2 && p.rot == true
		return informed_sample2D(goal, start, 3, c_max)
	end
	if p.dims == 2 && p.rot == false
		q = informed_sample2D(goal, start, 3, c_max)
		q.q[4:end] = 0
		q.c = 0
		return q
	end
	if p.dims == 3 && p.rot == true
		return informed_sample(goal, start, 6, c_max)
	end
	if p.dims == 3 && p.rot == false
		q = informed_sample(goal, start, 6, c_max)
		q.q[4:end] = 0
		q.a = 0
		q.b = 0
		q.c = 0
		return q
	end
end

function informed_sample(goal::Vector, start::Vector, dims::Int, c_max::Float64)::State
	c_min = norm(goal - start)
	if c_max < Inf && c_max - c_min > 0 && goal != start && c_max < 400
		centre = (goal + start)/2
		C = get_elipse_rotation(goal, start, dims)
		L = get_transformation(c_max, c_min, dims)
		q_b = sample_ball(dims)
		q_b = reshape(q_b, length(q_b), 1)
		T = C*L
		q_r = T*q_b + centre
	else
		return q_new = get_random_state(env) #"random state", width and height of configuration space
	end
	q_r[4] = rand()*2*pi#q_r[4]%(2*pi)	works significantly better like this
	q_r[5] = rand()*2*pi#q_r[5]%(2*pi)
	q_r[6] = rand()*2*pi#q_r[6]%(2*pi)
	return get_state(vec(q_r)) # ND
end

function informed_sample2D(goal::Vector, start::Vector, dims::Int, c_max::Float64)::State
	c_min = norm(goal - start)

	if c_max < Inf && c_max - c_min > 0 && goal != start
		goal = [goal[1], goal[2], goal[end]]
		start = [start[1], start[2], start[end]]
		
		centre = (goal + start)/2
		C = get_elipse_rotation(goal, start, dims)
		L = get_transformation(c_max, c_min, dims)
		q_b = sample_ball(dims)
		q_b = reshape(q_b, length(q_b), 1)
		T = C*L
		q_r = T*q_b + centre
	else
		return q_new = get_random_state2D(env) #"random state", width and height of configuration space
	end
	q_r[3] = rand()*2*pi
	return get_state([q_r[1], q_r[2], 0, 0, 0, q_r[3]]) # 2D
end


function wrap(goal::NODE, search_tree::TREE)
	# get path
	path = Vector{NODE}()
	node = goal
	while true
		push!(path, node)
		node = node.parent
		if node == false
			break
		end
	end
	i = 2
	for step in path[2:end-1]
		# move the step closer to next step untill possible
		# when stopped, move the previous step untill it connect with the next step
		# we find replacement for the current step
		current  = get_state(step.state)
		previous = get_state(path[i-1].state)
		next     = get_state(path[i+1].state)

		current_to_next = get_path(current, next, 10)

		j = 1
		for point in current_to_next
			if !connect(previous, point, env.offset)
				break
			end
			j += 1
		end

		if j == 1
			i += 1
			continue
		end

		if j > length(current_to_next) # no obstacle to be wrapped around
			#step.state = current_to_next[j-1].q
			#new_node = add_to_tree(search_tree, current_to_next[j-1].q, path[i+1]) # give it parent, no need to change anything else there
			new_node = t_append!(search_tree, path[i+1], current_to_next[j-1].q)
			# set as a parent: change the previous one
			deleteat!(step.children, findall(x->x==path[i-1], step.children)) # get rid of the old one
			path[i-1].parent = new_node # set a new one
			push!(new_node.children, path[i-1])
			i += 1
			continue
		end

		directional = current_to_next[j]
		direction = get_path(previous, directional, 10)

		k = 1
		for point in direction
			k += 1
			if connect(point, next, env.offset)
				break
			end
		end

		if k > length(direction)
			#step.state = current_to_next[floor(Int, k/2)].q
			i += 1
			continue
		end

		wrapped_point = direction[k] # we obtained an alternative to current

		# replace current with wrapped in the search tree
		
		#step.state = wrapped_point.q

		# make new node in the position wrapped_point and add it to the tree
		# parent = previous node = path[i-1]
		# only child = next node = path[i+1]

		#new_node = add_to_tree(search_tree, wrapped_point.q, path[i+1]) # give it parent, no need to change anything else there
		new_node = t_append!(search_tree, path[i+1], wrapped_point.q)
		# set as a parent: change the previous one
		deleteat!(step.children, findall(x->x==path[i-1], step.children)) # get rid of the old one
		path[i-1].parent = new_node # set a new one
		push!(new_node.children, path[i-1])
		#display(step.state)

		i += 1
	end
end

function glue(goal::NODE, obstacles#=::KD_TREE=#, search_tree::TREE)
	path = []
	node = goal
	while true
		push!(path, node)
		node = node.parent
		if node == false
			break
		end
	end


	i = 2
	for step in path[2:end-1]

		# get the glued state
		#s  = [step.state[1], step.state[2], step.state[end]] 			# transform to 2d
		s  = step.state
		nn = copy(get_nn(obstacles, s, false).state)
		nn = append!(nn, zeros(3))
		glued_step = get_state(nn)


		# check if its possidle to be swapped with the unglued version
		if !connect(glued_step, get_state(path[i-1].state), env.offset) || !connect(glued_step, get_state(path[i+1].state), env.offset)

			# glued version does not connect with previous or next step
			# continue to try to glue next step
			i += 1
			continue
		end
		# the glued step is feasible, swap it with the unglued version
		#step.state = nn
		new_node = add_to_tree(search_tree, nn, path[i+1]) # give it parent, no need to change anything else there
		# set as a parent: change the previous one
		deleteat!(step.children, findall(x->x==path[i-1], step.children)) # get rid of the old one
		path[i-1].parent = new_node # set a new one
		push!(new_node.children, path[i-1])
		#display(step.state)


		i += 1
	end
	
   	open("files/goal.txt", "w") do txt
		for step in path
			s = step.state
			println(txt, string(s[1]), " ", string(s[2]), " ", string(s[3]))
		end
	end
end
