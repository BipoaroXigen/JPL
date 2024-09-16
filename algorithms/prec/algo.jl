#using ReferenceFrameRotations
#using BenchmarkTools
#using StatsBase
#include("jl_libs/parzen.jl")

function RRT(q_init::State, env::Env, points_in_distribution, K=100000, s=100)
#function RRT(q_init::State, env::Env, K=100000, s=100)::Tree
	#Random.seed!(13)
	tree, nn = init_tree(env, q_init)

        u = Uniform(-15, 15)
        x = Uniform(env.x_min, env.x_max)
        y = Uniform(env.y_min, env.y_max)
        z = Uniform(env.z_min, env.z_max)

#	x = Uniform(-30, 40)
#	y = Uniform(-40, 60)
# 	z = Uniform(-44, 60)


	#points_in_distribution = generate_paths(q_init, env, 1)
	#points_in_distribution = generate_detailed_paths(q_init, env, 1)
#	end
	#for i in 1:size(points_in_distribution, 2)
	#open("files/path.txt", "a") do txt
        #        point = points_in_distribution[:, i]
        #        point = reshape(point, length(point), 1)
	#	line = point 
	#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
	#end
	#end

	local q_near_node::Tree_Node
	local q_near::State
	local q_new::State

#	display(points_in_distribution)
	
	connections = zeros(size(points_in_distribution, 2))

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
                        point = points_in_distribution[:, i]
                        point = reshape(point, length(point), 1)

			#cen = sample_unknown_distribution(point, 50, 1., u,x,y,z)
			#cen = sample_unknown_distribution(points_in_distribution[:, i:min(i+3, size(points_in_distribution, 2))], 50, 1., u,x,y,z)
			#cen = [rand(Normal(point[1])), rand(Normal(point[2])), rand(Normal(point[3]))]	
			cen = [clamp(randn() * 3 + point[1], env.x_min, env.x_max),
			       clamp(randn() * 3 + point[2], env.y_min, env.y_max),
			       clamp(randn() * 3 + point[3], env.z_min, env.z_max),
			       ]
			rot = [rand()*2*π,rand()*2*π,rand()*2*π]
			tmp = append!(cen, rot)
			q_new = get_state(tmp)

			q_near_node = ann(nn, tree, q_new)#"closest already connected state"
			q_near = q_near_node.content
			#open("files/points.txt", "a") do txt
			#	line = cen
			#	println(txt, string(line[1]), " ", string(line[2]), " ", string(line[3]))
			#end

			break
		end
		connection, norm_id = impact_point(q_near, q_new, env.offset)
		if connection == q_new
			c = c+1
			q_new_node = tree_append!(nn, tree, q_near_node, q_new)
		
			if is_goal(q_new, env.offset)
				#println("found: ", r)
				iters = r
				push!(tree.goal, q_new_node)
				break
			end
		else
			b = b + 1
			q_new_node = tree_append!(nn, tree, q_near_node, connection)
			if b%5 == 0 && b > 1
				i = i - 1
				if i == 0
					i = length(connections)
				end
			end
		end
	end
	free_kd_tree(nn)
	return iters, tree
	#return tree
end
