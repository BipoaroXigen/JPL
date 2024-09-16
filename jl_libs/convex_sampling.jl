using PyCall
using LinearAlgebra
const spatial = pyimport("scipy.spatial")
py"""
from scipy.spatial import Delaunay
"""
# above is not a comment!!!

mutable struct Link
	prev::Union{Link, Bool}		# point closer to the start of the line
	next::Union{Link, Bool}		# point closer to the end of the line
	point::Vector{Float64}		# point in space
	scalar_p::Float64		# scalar projection of the point onto the line
	projection::Vector{Float64}	# projection on line
	distance::Float64		# distance from line
end

struct Linked_list
	length::Int
	first_link::Link
end

function plot_rhull(list::Linked_list)
	open("files/convex.txt", "w") do txt
	link = list.first_link
	while link != false
		println(txt, link.projection[1], " ", link.projection[2]+link.distance)
		link = link.next
	end
	println(txt)
	link = list.first_link
	while link != false
		println(txt, link.projection[1], " ", link.projection[2]-link.distance)
		link = link.next
	end
	end
end

# get part of path for smaller elypse
#=function get_start_end(paths::Vector{PATH})
	#selected_path = shuffle!(paths)[1]
	selected_path = sort(paths, by=x->x.cost)[1]
	step 	= 5 + round(Int, rand()*(selected_path.length-5))

	c_best  = selected_path.cost
	
	# random whole number < leght(path) -2
	start_i = ceil(Int, rand()*(selected_path.length-2))

	# random whole number > start_i +1 
	goal_i  = start_i + 1 + ceil(Int, rand()*(selected_path.length-start_i-1))
	goal_i  = start_i + min(step, selected_path.length-start_i)
	start   = selected_path.nodes[start_i]
	goal    = selected_path.nodes[goal_i]

	c_best = goal.cost - start.cost
	start  = start.state
	goal   = goal.state

	return start, goal, c_best
end=#

function oriented_norm(x#=::Vector{Float64}=#, a, b)::Float64
	if norm(x-b) > norm(a-b)
		return -norm(x)
	end
	return norm(x)
end
function scalar_projection(x#=::Vector{Float64}=#, start, dir)::Float64
	return -dot((x-start), dir)		# start is for now allways 0 but I will keep it general
end

function is_inside_rhull(point, projection, rhull)::Bool
	# get a coses projection from each side
	links = []
	projections = []
	link = rhull.first_link
	while link != false
		push!(links, link)
		push!(projections, link.projection)
		link = link.next
	end

	first = projections[1]
	for i in 1:length(projections)
		projections[i] = projections[i]-first
	end

	norms = norm.(projections)
	p_distance = norm(point-projection)
	p_norm = norm(projection)
	relative_norms = norms.-p_norm

	#if norm(projection-first) < 0 # LOL DEMENTE
	if norm(projection-first - projections[end]) > norm(projections[end])
		return false
	end
	if norm(projection-first) > norms[end]
		return false
	end

	if length(relative_norms[relative_norms.<0]) == 0
		return false
	end
	if length(relative_norms[relative_norms.>0]) == 0
		return false
	end

	lower_i = length(relative_norms[relative_norms.<0])
	higher_i = lower_i+1

	lower  = links[lower_i]
	higher = links[higher_i]

	point = Link(false, false, point, 0, projection, p_distance)
	return compare_in(lower, point, higher)
end

function max_f(s_p_point, rhull)::Float64

	# get two neighbour points of q (the q is fully defined by scalar projection here, because f(q) is set to 0)
	# a(q) = scalar projection of q - scalar projection of o
	# from their a and f compute max f in a(q)

	# get a closest v from each side
	links = Vector{Link}()
	s_projections = Vector{Float64}()
	link = rhull.first_link
	while link != false
		push!(links, link)
		push!(s_projections, link.scalar_p)
		link = link.next
	end

	relative_s_p 	= s_projections.-s_p_point
	lower_i 	= length(relative_s_p[relative_s_p.<0])
	higher_i 	= lower_i + 1
	lower	 	= links[lower_i]
	higher 		= links[higher_i]

	radius = get_radius(lower, s_p_point, higher)
	return radius
end

function get_radius(v_l::Link, a::Float64, v_r::Link)::Float
	# returns the radius of the hull at given a(q)

	a_c = v_r.scalar_p-v_l.scalar_p # distance between both ends of line segment
	a_b = a-v_l.scalar_p

	if v_l.distance < v_r.distance
		ratio = a_b/a_c 	# what percentage of the segment is the middle point in
	else
		ratio = 1 - a_b/a_c
	end

	max_distance = min(v_l.distance, v_r.distance)+ratio*(max(v_l.distance, v_r.distance)-min(v_l.distance, v_r.distance))

	return max_distance
end


function convex_sample(projector::Matrix{Float64}, hull::Linked_list, paths)
	start, goal, c_best = get_start_end(paths)
	#q_new = get_random_state(env)
	d = 6
	q_new = informed_sample(goal, start, d, c_best)
	point = q_new.q
	i = 1
	while is_inside_rhull(point, projector*point, hull) == false
		#q_new = get_random_state(env)
		q_new = informed_sample(goal, start, d, c_best)
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
	start, goal, c_best = get_start_end(paths)
	#q_new = get_random_state2D(env)
	d = 3
	q_new = informed_sample2D(goal, start, d, c_best)
	point = q_new.q[[1,2,end]]
	i = 1
	while is_inside_rhull(point, projector*point, hull) == false
		#q_new = get_random_state2D(env)
		q_new = informed_sample2D(goal, start, d, c_best)
		point = q_new.q[[1,2,end]]
		i += 1
		if i > 10
			q_new = get_random_state2D(env)
			point = q_new.q[[1,2,end]]
		end
	end
	return q_new
end

function direct_sample(d, hull)
	# d is vector from o to t

	# determine the distance of the sample along the axis
	a = hull_weighted_sample(hull)

	# given a, determine maximal distance from the axis
	f_max = max_f(a+hull.first_link.scalar_p, hull)		# input scalar projection of q instead of a(q) (simpler and more efficient to implement) without ever using explicit a()
	
	# sample the distance from the axis
	f = rand()*f_max

	# get random unit vector orthogonal to d
	r = random_orthogonal_vector(d, dim)

	d = d/norm(d)
	r = r/norm(r)	# not necessary
	return a*d + f*r
end

function random_orthogonal_vector(v, n)
	# gramm shmidt orthogonalization

	w = rand(n)
	projection = (dot(v, w) / dot(v, v)) * v
	u = w - projection
	while norm(u) < 1e-10
		w = rand(n)
		projection = (dot(v, w) / dot(v, v)) * v
		u = w - projection
	end

	return u
end

function hull_weighted_sample(hull)
	# divide the hull into sections between points
	volumes = Vector{Float64}()
	x = Vector{Vector{Float64}}()

	link = hull.first_link
	while link != false
		push!(x, [link.projection, link.distance])
		link = link.next
	end

	for i in 1:length(x)-1
		push!(volumes, get_volume(x[i], x[i+1]))
	end

	full_volume = sum(volumes)
	th = volumes/full_volume
	pushfirst!(th, 0)


	# decide which section of hull will be sampled
	u = rand()
	i = 1
	while u > sum(th[1:i]) && i < length(th)
		i += 1
	end
	i -= 1

	# take the weighted sample from volume i
	
	a1 = x[i][1]
	a2 = x[i+1][1]
	f1 = x[i][2]
	f2 = x[i+1][2]

	wodth = abs(a1-a2)
	slope = abs(f1-f2)/width
	b = min(f1, f2)
	s_ = linearly_weighted_sample(width, slope, b, f1>f2)

	return s_ + x[i][1]

end

function get_volume(a, b)
	widht = abs(a[1]-b[1])
	height_diff = abs(a[2]-b[2])
	height_same = min(a[2], b[2])
	V = 0
	if height_same != 0
		V = height_same*width
	end
	V += height_diff*width*0.5
	return V
end

function linearly_weighted_sample(width, slope, b, flip)
	a = slope*2
	scale = a+b

	c = rand()*scale
	D = b^2+4*a*c
	d = sqrt(D)

	sol_1 = (-b-d)/(2*a)
	sol_2 = (-b+d)/(2*a)

	if flip == true
		sol = (-(max(sol_1,  sol_2)-0.5)+0.5)*width
	else
		sol = max(sol_1, sol_2)*width
	end
	return sol
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

function chull(x::Matrix{T}) where T<:Real
    	py = spatial.ConvexHull(x)
    	return py.vertices .+ 1
end

function check_hull(point, hull)
	py = hull.find_simplex(point)[1]
	if py < 0
		return false
	end
	return true
end

function compare_in(a::Link, b::Link, c::Link)::Bool
	# decides whether the middle point (b) is inside the rotated convex hull or not
	if a.distance > b.distance && c.distance > b.distance # inside
		return true
	end
	if a.distance < b.distance && c.distance < b.distance # outside
		return false
	end
	if a.distance == b.distance && c.distance == b.distance # on border
		return true
	end

	### the point is between
	a_c = norm(c.projection)-norm(a.projection) # distance between both ends of line segment
	a_b = norm(b.projection)-norm(a.projection)

	if a.distance < c.distance
		ratio = a_b/a_c # what percentage of the segment is the middle point in
	else
		ratio = 1 - a_b/a_c
	end

	max_distance = min(a.distance, c.distance)+ratio*(max(a.distance, c.distance)-min(a.distance, c.distance))

	if b.distance > max_distance
		return false
	end

	return true
end


function is_on_border(a::Link, b::Link, c::Link)::Bool
	if a.distance > b.distance && c.distance > b.distance
		return false # is inside
	end
	if a.distance < b.distance && c.distance < b.distance
		return true
	end
	if a.distance == b.distance && c.distance == b.distance
		return true
	end

	a_c = norm(c.projection)-norm(a.projection) # distance between both ends of line segment
	a_b = norm(b.projection)-norm(a.projection)

	if a.distance < c.distance
		ratio = a_b/a_c # what percentage of the segment is the middle point in
	else
		ratio = 1 - a_b/a_c
	end

	max_distance = min(a.distance, c.distance)+ratio*(max(a.distance, c.distance)-min(a.distance, c.distance))
	if b.distance <= max_distance
		return false
	end

	return true
end


function prepare_rotated_hull_points!(projected_points::Matrix{Float64}, projections::Matrix{Float64}, start::Vector{Float64}, goal::Vector{Float64})::Linked_list
	# join points, goal, and start into points
	# sort points
	# build linked list from points
	#
	# take the first point and the two following and check whether the midlle is inside the hull
	# if is inside, emit him from the linked list and begin again
	# if not, move one step forward (new triplet)
	#perm = sortperm(eachcol(projections), by=x->oriented_norm(x, start, goal)) 	# norm(x-start)
	
	dir = (start-goal)/norm(start-goal)
	perm = sortperm(eachcol(projections), by=x->scalar_projection(x, start, dir))	# not neccessary to use start, any point from the projected line will work


	projected_points = projected_points[:, perm]
	points = projections[:, perm]

	norms  = sqrt.(sum((points-projected_points).^2, dims=1))			# distance from axis f(q)
	#points = hcat(start, points, goal)
	#projected_points = hcat(start, projected_points, goal)
	#norms  = hcat(0, norms, 0) # norm(start) is and norm(goal) are zero, because they lie on the projection line
	
	recent = Link(false, false, projected_points[:, 1], scalar_projection(projected_points[:, 1], start, dir), points[:, 1], norms[1])
	list   = Linked_list(length(norms), recent)

	i = 2
	for i in 2:length(norms)
		new = Link(recent, false, projected_points[:, i], scalar_projection(projected_points[:, i], start, dir), points[:, i], norms[i])
		recent.next = new
		recent = new
	end

	first  = list.first_link
	second = first.next
	third  = second.next
	changed = false
	while true # when reached the end of the list, the algorithm is completed
		if is_on_border(first, second, third) == false
			
			# emit second from list
			first.next = third
			third.prev = first

			# start again from the beginning
			first  = list.first_link
			second = first.next
			third  = second.next
			changed = true
		else
			if third.next == false
				if changed == true
					first  = list.first_link
					second = first.next
					third  = second.next
					changed = false
					continue
				else
					break
				end
			end
			first  = first.next
			second = second.next
			third  = third.next
		end
	end
	return list
end

function plotlist(list::Linked_list)
	link = list.first_link
	println("---------------- start ----------------")
	while link != false
		#println(link.point)
		println(link.projection)
		link = link.next
	end
	println("---------------- end ------------------")

end
