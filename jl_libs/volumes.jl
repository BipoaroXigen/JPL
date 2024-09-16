using SpecialFunctions
using LinearAlgebra
function ellipse_volume(dims::Int64, f1::Vector{Float64}, f2::Vector{Float64}, d::Float64)::Float64
	a1 = 0.5*norm(f1-f2)
	as = [a1]
	for i in 2:dims
		ai = sqrt((d/2)^2 - (as[i-1]/2)^2)
		push!(as, ai)
	end
	#display(as)
	ai = 0.5*sqrt(d^2-norm(f1-f2))
	#display(ai)
	volume = (pi^(dims/2)/gamma((dims/2)+1))*prod(as)
	volume = (pi^(dims/2)/gamma((dims/2)+1))*a1*ai^(dims-1)
	return volume
end

function hull_volume(rhull, dims::Int64)::Float64
	volume = 0
	link = rhull.first_link
#	println()
#	while link.next != false
#	while link != false
#		println(link.distance)
#		link = link.next
#	end
	link = rhull.first_link
	while true
		width = norm(link.projection - link.next.projection)
		# I know this could be faster but it is for result plotting pusposes only so it doesnt matter
		d1 = min(link.distance, link.next.distance)	
		d2 = max(link.distance, link.next.distance)

		outer = width * d2^(dims-1) * (pi^((dims-1)/2))/(gamma(((dims-1)/2)+1))
		inner = width * d1^(dims-1) * (pi^((dims-1)/2))/(gamma(((dims-1)/2)+1))
		v1 = inner
		v2 = (outer-inner)/dims

	#=	println("d1: ", d1)
		println("d2: ", d2)
		println("rectangle: ", v1)
		println("triangle: ", v2)
		println("vol: ", v2+v1)
		println()
		readline()=#

		link = link.next
		volume = volume + v1 + v2
		if link.next == false
			break
		end
	end
	return volume
end
