using ReferenceFrameRotations

function save_detailed_path(goals)
	if length(goals) > 0
		open(joinpath(@__DIR__, "../files/goal.txt"), "w") do txt
			goal_path = reverse(get_shortest_path_in_tree(goals, offset))
			last = goal_path[1]
			for step in goal_path[2:end] 
				for s in get_path(get_state(last.state), get_state(step.state))
					s = s.q	
					q = angle_to_quat(s[4],s[5],s[6], :ZYX)
					println(txt, s[1], " ", s[2], " ", s[3], " ", q[1], " ", q[2], " ", q[3], " ", q[4])
				end
				last = step
			end
		end
	end
end

function save_path(goals)
	if length(goals) > 0
		open(joinpath(@__DIR__, "../files/gol.txt"), "w") do txt
			node = goals[1]
			goal_path = reverse(get_shortest_path_in_tree(goals, offset))
			for step in goal_path 
				#s = step.content.q
				s = step.state
				q = angle_to_quat(s[4],s[5],s[6], :ZYX)
				println(txt, s[1], " ", s[2], " ", s[3], " ", q[1], " ", q[2], " ", q[3], " ", q[4])
			end
		end
	end
end

function save_tree(tree)

	open(joinpath(@__DIR__, "files/nodes.txt"), "w") do txt
		println(txt, "0.0 0.0 0.0")
	end

	root = tree.nodes[1]
	for node in tree.nodes
		for child in node.children
			open(joinpath(@__DIR__, "files/nodes.txt"),"a") do txt
				println(txt, string(node.content.x), " ", string(node.content.y), " ", string(node.content.z), " ", string(node.content.a), " ", string(node.content.b), " ", string(node.content.c))
				println(txt, string(child.content.x), " ", string(child.content.y), " ", string(child.content.z), " ", string(child.content.a), " ", string(child.content.b), " ", string(child.content.c), "\n")
			end
		end
	end

end

function plot_search_tree(tree)			# for gnuplot plotting

	open(joinpath(@__DIR__, "../files/nodes.txt"), "w") do txt
		println(txt, "0.0 0.0 0.0")
	end

	open(joinpath(@__DIR__, "../files/nodes.txt"),"a") do txt
		for node in tree.nodes
			for child in node.children
				println(txt, string(node.state[1]), " ", string(node.state[2]), " ", string(node.state[3]))
				println(txt, string(child.state[1]), " ", string(child.state[2]), " ", string(child.state[3]), "\n")
			end
		end
	end

end

function plot_path(goal)
	path = []
	costs = []
	node = goal
	while true
		push!(path, node.state)
		push!(costs, node.cost)
		node = node.parent
		if node == false
			break
		end
	end

	open(joinpath(@__DIR__, "../files/goal.txt"), "w") do txt
		for step in path
			s = step
			println(txt, string(s[1]), " ", string(s[2]))
		end
	end

end
