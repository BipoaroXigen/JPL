function parzen_pdf_estimate(x::Matrix{Float64}, x_known::Matrix{Float64}, h::Float64)::Vector{Float64}
	N = size(x_known)[2]
	COV = h^2*I(size(x)[2])
	p = []

	for i in eachrow(x)
		d = MvNormal(i, COV)
		p_k = sum(pdf(d, x_known))/N
		push!(p, p_k)
	end

	return p
end

function sample_unknown_distribution(known_points::Matrix{Float64}, density::Int64, confidence::Float64, u, x, y, z)::Vector{Float64}
	#x = []
	#display(size(known_points))
	#display((6,25))
#	points = rand(u, (density, 6))
#	points = rand(u, (density, 3))
	points = [rand(x, (density,1)) rand(y, (density,1)) rand(z, (density,1))]
	PDF = argmax(parzen_pdf_estimate(points, known_points, confidence))
	#display(points)
	#display(PDF)
	return points[PDF, :]# push!(x, points[:, PDF])
	
	#return x
end
