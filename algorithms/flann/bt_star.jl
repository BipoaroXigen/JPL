using DataStructures

function RRT(q_init::State, env::Env, K::Int=100000, set_goal::Union{Vector{Float64}, Bool}=false, id::Int=100)::TREE
	tree = get_tree(q_init.q) # search tree, FLANN datastructure for finding NN
	
	local q_near_node::NODE
	local q_near::State
	local q_new::State

	kveve =	PriorityQueue() 
	
	projector = 0
	r_hull = 0

	logf = []
	
	paths  = Vector{PATH}()
	c_best = Inf
	goal   = q_init.q
	start  = q_init.q

	d = 3
	d = 6
	V = (4/3)*pi
	O = 1.7 * 1.6 * 1.6# *100*100*100
	O = 500*500
	O = 200*200*200
	gamma = 2*(1+1/d)^(1/d)*((O)/V)^(1/d)
	#gamma = 250
	epsilon = 20
	#epsilon = 1
	#epsilon = Inf

	time = 0
	#s = 0
	#n = 0
	#c = 0
	
	batch_size = 100

	iters = K
	for r in 1:K
		time += @elapsed begin
		#print("\r", r)
		# sample
		q_new = get_random_state(env)
		enqueue!(kveve, q_new, distance(q_new.q, zeros(6)))

		if r%batch_size == 0

		while !isempty(kveve)
			q_new = dequeue!(kveve)

			# get NN
			radius = (gamma*(log(r)/r)^(1/(d+1)))^2
			radius = 30.0^2
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
				end
			end
		end #while
		end #if
		if r%10 == 0 && length(tree.goal) > 0
			goal_node = sort(tree.goal, by=x->x.cost)[1]
			push!(logf, [r, time, goal_node.cost])
		end
		if time > 10# && 1 == 0
			goal_node = sort(tree.goal, by=x->x.cost)[1]
			push!(logf, [r, time, goal_node.cost])
		#	display(r)
		#	display(s)
		#	display(n)
		#	display(c)
			break
		end
end # timer
	end

	open("/home/krizjona/Project/analyze/random/bit_star", "a") do w
		for data in logf
			println(w, string(data[1]), " ", string(data[2]), " ", string(data[3]))
		end
	end
	
	#plot_search_tree(tree)
	println(sort(tree.goal, by=x->x.cost)[1].cost)
	clear_tree(tree)	# frees the allocated memory in nanoflann

	return tree
end
