mutable struct Point
	position::Vector{Float64} # first half is center, second is deviation
	velocity::Vector{Float64}
	best_position::Vector{Float64}
	best_value::Float64 # maybe Int64
end

function PSO(step, index , path, env::Env, q_init::State, best_dist, I=10, P=20, w=0.5, c1=0.5, c2=0.3) # for maximalization
	interest = [path[2].x,path[2].y,path[2].z,0,0,0,path[2].a,path[2].b,path[2].c,0,0,0]

	#println(index)
	#println(length(path))

	if best_dist isa Dist
		G_best = best_dist.d
		G_best_value = best_dist.v 
	else
		G_best = zeros(12)#[11.,-20,23,5,5,5]
		G_best_value = -Inf
	end

	points = Vector{Point}()
	sizehint!(points, P)
	for i in 1:P
		center = get_random_state(env).q

		deviation = [rand()*(env.x_max-abs(center[1])),
			     rand()*(env.y_max-abs(center[2])),
			     rand()*(env.z_max-abs(center[3])),
			     rand()*π,
			     rand()*π,
			     rand()*π]
		
		position = append!(center, deviation)

		velocity = ones(12)*0.1
		value = evaluate_point(q_init, step, index, position, env.offset)
		
		push!(points, Point(position, velocity, position, value))

		if (value > G_best_value) || (i == 1)
			#G_best_value == evaluate_point(q_init, path, position, env.offset)
			G_best_value == evaluate_point(q_init, step, index, position, env.offset)
			G_best = position
			#show_best(G_best)
		end
	end
	#iters = 0
	#unit = 8+length(path)
	for i in 1:I
#		@time begin
		#println(i)
		v = 0
		top = -Inf
		top_p = zeros(4)
		for point in points
			r1 = rand()
			r2 = rand()
			
			new_velocity = w*point.velocity + c1*r1*(point.best_position-point.position) + c2*r2*(G_best-point.position) + 1.0*r2*(interest-point.position)

			point.position = point.position + new_velocity

			point.velocity = new_velocity
			
			if in_borders(point)
				#value = evaluate_point(q_init, path, point.position, env.offset)
				value = evaluate_point(q_init, step, index, point.position, env.offset)
				#iters += unit
			else
				value = -Inf
				point.velocity = -point.velocity
			end
			v += value

			if (value > top)
				top = value
				top_p = point.position
				point.best_position = point.position
				point.best_value = value
			elseif value > point.best_value
				point.best_value = value
				point.best_position = point.position
			end
		end
		if top > G_best_value 
			G_best_value = top
			G_best = top_p
 			#show_best(G_best)
		end
 		#blender_show(points)
#		end
	end
	#display(iters)
	#evaluate_point(q_init, step, index, G_best, env.offset)
	#prdist(G_best, G_best_value)
	#readline()
#	println(G_best_value)
#	G_best = [11.959, -17.636, 22.059, 5, 5, 5, 0, 0, 0, 0, 0, 0]
	return Dist(G_best, G_best_value)
end

function evaluate_point(q_init::State, step::State, index, point::Vector{Float64}, offset)::Float64
	value = 0
	CPCC = 0
	index = index-1
	
	for sample in get_edges_3D(point)
		if probe_connect(q_init, sample, offset)
			CPCC += 1
		end
	end

	dist = distance(point[1:6], step.q)
	#value = (index*CPCC)/(dist)
	value = (index^2*CPCC)/(dist^2)
	#value = (index^3*CPCC)/(dist^3)
	#value = (index)/(dist^2)
	#value = (1)/(dist^2)
	
	#println()
	#println("index:    ", index)
	#println("CPCC:     ", CPCC)
	#println("distance: ", dist)
	#println("value:    ", value)

	return value
end

function easy_eval(q_init::State, path::Vector{State}, point::Vector{Float64})::Float64
	value = 0
	len = length(path)
	I = 100
	col = 0

	connectible = false

	for sample in get_edges(point)

		value -= distance(sample.q, [0,0,0,0,0,0])	
	end
	return value

end

function kubista_eval(q_init::State, path::Vector{State}, point::Vector{Float64})::Float64

	value = 0
	len = length(path)

	edges = get_edges(point)

	for i in 1:len-1
		for sample in edges
			v = 0
			if connect(sample, path[i])
				v += i^2 - (distance(sample.q, path[i].q)*i)
			#end
			#if connect(sample, path[i+1])
				v += (i+1)^2 - (distance(sample.q, path[i+1].q)*(i+1))
			end
			if v > value
				value = v
			end
		end
	end

	return value

end

function in_borders(point::Point)::Bool
	if (point.position[1] - point.position[7]) < env.x_min
	#	println("out")
		return false
	end
	if (point.position[1] + point.position[7]) > env.x_max
	#	println("out")
		return false
	end
	if (point.position[2] - point.position[8]) < env.y_min
	#	println("out")
		return false
	end
	if (point.position[2] + point.position[8]) > env.y_max
	#	println("out")
		return false
	end
	if (point.position[3] - point.position[9]) < env.z_min
	#	println("out")
		return false
	end
	if (point.position[3] + point.position[9]) > env.z_max
	#	println("out")
		return false
	end
	if point.position[7] < 0
	#	println("negative radius")
		return false
	end
	if point.position[8] < 0
	#	println("megative radius")
		return false
	end
	if point.position[9] < 0
	#	println("negative radius")
		return false
	end

	if (point.position[10] < 0) || (point.position[10] > 2*π) 
	#	println("negative radius")
		return false
	end
	if (point.position[11] < 0) || (point.position[11] > 2*π)
	#	println("megative radius")
		return false
	end
	if (point.position[12] < 0) || (point.position[12] > 2*π)
	#	println("negative radius")
		return false
	end

	return true
end

function prdist(dist, v)
	println()
	println("x center: ", dist[1], ", x width: ", dist[1+6])
	println("y center: ", dist[2], ", y width: ", dist[2+6])
	println("z center: ", dist[3], ", z width: ", dist[3+6])
	println("a center: ", dist[4], ", a width: ", dist[4+6])
	println("b center: ", dist[5], ", b width: ", dist[5+6])
	println("c center: ", dist[6], ", c width: ", dist[6+6])
	println()
	println("value: ", v)
	println()


	open("distributions.txt", "w") do txt
	end
	point = dist
	open("dist.txt", "w") do q
		println(q, string(point[1]), " ", string(point[2]), " ", string(point[3]), " ", string(point[7]), " ", string(point[8]), " ", string(point[9]))
	end
end

function get_edges_3D(point::Vector{Float64})::Vector{State}
	edges = Vector{State}()

	#S = append!(point[1:3], [0,0,0])
	S = point[1:6]
	r1 = [point[7], 0, 0, 0, 0, 0]
	r2 = [0, point[8], 0, 0, 0, 0]
	r3 = [0, 0, point[9], 0, 0, 0]

	push!(edges, get_state(S+r1+r2+r3))
	push!(edges, get_state(S+r1+r2-r3))
	push!(edges, get_state(S+r1-r2+r3))
	push!(edges, get_state(S+r1-r2-r3))

	push!(edges, get_state(S-r1+r2+r3))
	push!(edges, get_state(S-r1+r2-r3))
	push!(edges, get_state(S-r1-r2+r3))
	push!(edges, get_state(S-r1-r2-r3))

	return edges
end

function get_edges(point::Vector{Float64})::Vector{State}
	edges = Vector{State}()

	S = point[1:6]
	r1 = [point[7], 0, 0, 0, 0, 0]
	r2 = [0, point[8], 0, 0, 0, 0]
	r3 = [0, 0, point[9], 0, 0, 0]
	r4 = [0, 0, 0, point[10], 0, 0]
	r5 = [0, 0, 0, 0, point[11], 0]
	r6 = [0, 0, 0, 0, 0, point[12]]

	push!(edges, get_state(S+r1+r2+r3+r4+r5+r6))
      	push!(edges, get_state(S+r1+r2+r3+r4+r5-r6))
      	push!(edges, get_state(S+r1+r2+r3+r4-r5+r6))
	push!(edges, get_state(S+r1+r2+r3+r4-r5-r6))

    	push!(edges, get_state(S+r1+r2+r3-r4+r5+r6))
	push!(edges, get_state(S+r1+r2+r3-r4+r5-r6))
	push!(edges, get_state(S+r1+r2+r3-r4-r5+r6))
	push!(edges, get_state(S+r1+r2+r3-r4-r5-r6))

	push!(edges, get_state(S+r1+r2-r3+r4+r5+r6))
	push!(edges, get_state(S+r1+r2-r3+r4+r5-r6))
	push!(edges, get_state(S+r1+r2-r3+r4-r5+r6))
	push!(edges, get_state(S+r1+r2-r3+r4-r5-r6))

	push!(edges, get_state(S+r1+r2-r3-r4+r5+r6))
	push!(edges, get_state(S+r1+r2-r3-r4+r5-r6))
	push!(edges, get_state(S+r1+r2-r3-r4-r5+r6))
	push!(edges, get_state(S+r1+r2-r3-r4-r5-r6))


	push!(edges, get_state(S+r1-r2+r3+r4+r5+r6))
	push!(edges, get_state(S+r1-r2+r3+r4+r5-r6))
	push!(edges, get_state(S+r1-r2+r3+r4-r5+r6))
	push!(edges, get_state(S+r1-r2+r3+r4-r5-r6))

	push!(edges, get_state(S+r1-r2+r3-r4+r5+r6))
	push!(edges, get_state(S+r1-r2+r3-r4+r5-r6))
	push!(edges, get_state(S+r1-r2+r3-r4-r5+r6))
	push!(edges, get_state(S+r1-r2+r3-r4-r5-r6))

	push!(edges, get_state(S+r1-r2-r3+r4+r5+r6))
	push!(edges, get_state(S+r1-r2-r3+r4+r5-r6))
	push!(edges, get_state(S+r1-r2-r3+r4-r5+r6))
	push!(edges, get_state(S+r1-r2-r3+r4-r5-r6))

	push!(edges, get_state(S+r1-r2-r3-r4+r5+r6))
	push!(edges, get_state(S+r1-r2-r3-r4+r5-r6))
	push!(edges, get_state(S+r1-r2-r3-r4-r5+r6))
	push!(edges, get_state(S+r1-r2-r3-r4-r5-r6))


	push!(edges, get_state(S-r1+r2+r3+r4+r5+r6))
	push!(edges, get_state(S-r1+r2+r3+r4+r5-r6))
	push!(edges, get_state(S-r1+r2+r3+r4-r5+r6))
	push!(edges, get_state(S-r1+r2+r3+r4-r5-r6))

	push!(edges, get_state(S-r1+r2+r3-r4+r5+r6))
	push!(edges, get_state(S-r1+r2+r3-r4+r5-r6))
	push!(edges, get_state(S-r1+r2+r3-r4-r5+r6))
	push!(edges, get_state(S-r1+r2+r3-r4-r5-r6))

	push!(edges, get_state(S-r1+r2-r3+r4+r5+r6))
	push!(edges, get_state(S-r1+r2-r3+r4+r5-r6))
	push!(edges, get_state(S-r1+r2-r3+r4-r5+r6))
	push!(edges, get_state(S-r1+r2-r3+r4-r5-r6))

	push!(edges, get_state(S-r1+r2-r3-r4+r5+r6))
	push!(edges, get_state(S-r1+r2-r3-r4+r5-r6))
	push!(edges, get_state(S-r1+r2-r3-r4-r5+r6))
	push!(edges, get_state(S-r1+r2-r3-r4-r5-r6))


	push!(edges, get_state(S-r1-r2+r3+r4+r5+r6))
	push!(edges, get_state(S-r1-r2+r3+r4+r5-r6))
	push!(edges, get_state(S-r1-r2+r3+r4-r5+r6))
	push!(edges, get_state(S-r1-r2+r3+r4-r5-r6))

	push!(edges, get_state(S-r1-r2+r3-r4+r5+r6))
	push!(edges, get_state(S-r1-r2+r3-r4+r5-r6))
	push!(edges, get_state(S-r1-r2+r3-r4-r5+r6))
	push!(edges, get_state(S-r1-r2+r3-r4-r5-r6))

	push!(edges, get_state(S-r1-r2-r3+r4+r5+r6))
	push!(edges, get_state(S-r1-r2-r3+r4+r5-r6))
	push!(edges, get_state(S-r1-r2-r3+r4-r5+r6))
	push!(edges, get_state(S-r1-r2-r3+r4-r5-r6))

	push!(edges, get_state(S-r1-r2-r3-r4+r5+r6))
	push!(edges, get_state(S-r1-r2-r3-r4+r5-r6))
	push!(edges, get_state(S-r1-r2-r3-r4-r5+r6))
	push!(edges, get_state(S-r1-r2-r3-r4-r5-r6))

	return edges
end

function blender_show(points::Vector{Point})
	open("files/distributions.txt", "w") do txt
	end
	for point in points
		point = point.position
		open("files/distributions.txt", "a") do txt
			#println(txt, string(point[1]), " ", string(point[2]), " ", string(point[3]))
			println(txt, string(point[1]), " ", string(point[2]), " ", string(point[3]), " ", string(point[7]), " ", string(point[8]), " ", string(point[9]))
		end
	end
	println("displaying")
end

function show_best(point)
	#println(point)
	open("files/dist.txt", "w") do txt
	end
	open("files/dist.txt", "a") do txt
		#println(txt, string(point[1]), " ", string(point[2]), " ", string(point[3]))
		println(txt, string(point[1]), " ", string(point[2]), " ", string(point[3]), " ", string(point[7]), " ", string(point[8]), " ", string(point[9]))
	end
end
