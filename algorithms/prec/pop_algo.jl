function RRT(q_init::State, env::Env, points_in_distribution, K=100000, s=100)
#function RRT(q_init::State, env::Env, K=100000, s=100)::Tree
	#Random.seed!(13)
	tree, nn = init_tree(env, q_init)

        u = Uniform(-15, 15)
        x = Uniform(env.x_min, env.x_max)
        y = Uniform(env.y_min, env.y_max)
        z = Uniform(env.z_min, env.z_max)

	#points_in_distribution = generate_paths(q_init, env, 1)
	#points_in_distribution = generate_detailed_paths(q_init, env, 1)

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State

	connections = zeros(size(points_in_distribution, 2))
	
	#volume = 200
	volume = 10
	helped = volume
	normal = [0,0,0]
	impact = [0,0,0]
	tried = []

	n = 0
	p = 0


	i = 1
	c = 0
	b = 0

	iters = K*5

	for r in 1:K*5
		#print("\r", r)
		if c%10 == 0 && c > 1
			i = i + 1
			if i >= length(connections)
				i = 1
			end
		end
		while true
			if helped < volume
				cen = [clamp(randn()*10+impact[1], env.x_min, env.x_max),
				       clamp(randn()*10+impact[2], env.y_min, env.y_max),
				       clamp(randn()*10+impact[3], env.z_min, env.z_max)]
#				       clamp(randn()*1+impact[4], 0, 2*π),
#				       clamp(randn()*1+impact[5], 0, 2*π),
#				       clamp(randn()*1+impact[6], 0, 2*π)] 
				#for _ in 1:10
			#		cen = [clamp(randn()*2+impact[1], env.x_min, env.x_max),
			#		       clamp(randn()*2+impact[2], env.y_min, env.y_max),
			#		       clamp(randn()*1+impact[3], env.z_min, env.z_max), 
			#		if sum(normal.*(cen-impact[1:3])/norm(cen-impact[1:3])) > 0
				#		break
				#	end
				#end
				rot = [clamp(randn()*1+impact[4], 0, 2*π),
				       clamp(randn()*1+impact[5], 0, 2*π),
			       	       clamp(randn()*1+impact[6], 0, 2*π)]
				helped += 1
				#n+=1
				#open("files/points.txt", "a") do txt
#					q = angle_to_quat(rot[1], rot[2], rot[3], :ZYX)
				#	println(txt, string(cen[1]), " ", string(cen[2]), " ", string(cen[3]))#, " ", string(q[1]), " ", string(q[2]), " ", string(q[3]), " ", string(q[4]))
				#end
			
			#rot = [rand()*2*π,rand()*2*π,rand()*2*π]
			tmp = append!(cen, rot)

			q_new = get_state(tmp)
			else
				#p+=1
				q_new = get_random_state(env)
			end
			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content
			break
		end
		connection, norm_id = impact_point(q_near, q_new, env.offset)
		q_new_node = tree_append!(nn, tree, q_near_node, connection)

		if connection == q_new
			c = c+1
			if is_goal(q_new, env.offset)
				#println("found: ", r)
				iters = r
				push!(tree.goal, q_new_node)
				break
			end
		else

			# POP now
			# take normal 
			# for next N samples:
			# if point on the outside of the obstacle, take the sample, otherwise dont
			# take sample from plain under the surface
			#
			# alignment with normal:
			# subtract impact point from new sample
			# multiply with normal
			# when positive, the sample is outside the obstacle
			#
			# sampling from plain:
			# take impact point and subtract normal, with amplitude A<1 -> center of the circle
			# get vector orthogonal to the normal
			# dal si to domysli, protoze jeste nevim
			
			if helped >= volume
				normals_id = unique(Iterators.flatten(norm_id)) .+ 1
				if normals_id[1] ∉ tried
					normals = env.normals[normals_id]
					normal = normals[1]
					append!(tried, normals_id[1])
					#impact = connection.q[1:3]
					impact = connection.q
					helped = 0
				end
			end

			b = b + 1
			if b%5 == 0 && b > 1
				i = i - 1
				if i == 0
					i = length(connections)
				end
			end
		end
	end
#	println()
#	println("helped: ", n)
#	println("path: ", p)
	free_kd_tree(nn)
	#return tree
	return iters, tree
end
