
function get_path_in_tree(node) #returns path from this node to root
	path = [node]
	while node.parent isa Tree_Node
		node = node.parent
		push!(path, node)
	end

	return path
end

function get_shortest_path_in_tree(nodes::Vector{NODE}, offset::Int)::Vector{NODE}
	shortest_len = Inf
	#shortest_path = Vector{Tree_Node}()
	shortest_path = []#Vector{Tree_Node}()
	sub_path = []

	for node in nodes
		path = [node]
		current = node
		parent = node.parent
		prev = false

		while parent != false
			if connect(get_state(parent.state), get_state(current.state), offset)
				prev = parent
				parent = parent.parent
			else
				push!(path, prev)
				current = prev
				prev = parent
				#parent = parent.parent
			end
		end
		push!(path, prev)
		
		len = length(path)
		if len < shortest_len
			shortest_path = path
			shortest_len = len
			sub_path = node
		end
	end
	return shortest_path
end

# obsolete
function get_shortest_path_in_tree(nodes::Vector{Tree_Node}, offset::Int)::Vector{Tree_Node}
	shortest_len = Inf
	#shortest_path = Vector{Tree_Node}()
	shortest_path = []#Vector{Tree_Node}()
	sub_path = []

	for node in nodes
		path = [node]
		current = node
		parent = node.parent
		prev = false

		while parent isa Tree_Node
			if connect(parent.content, current.content, offset)
				prev = parent
				parent = parent.parent
			else
				push!(path, prev)
				current = prev
				prev = parent
				#parent = parent.parent
			end
		end
		push!(path, prev)
		
		len = length(path)
		if len < shortest_len
			shortest_path = path
			shortest_len = len
			sub_path = node
		end
	end
	return shortest_path
end

# obsolete
function gspit(node::Tree_Node, id_offset::Int)::Vector{Tree_Node}
	path = Vector{Tree_Node}()
	push!(path, node)
	parent = node.parent
	prev = false

	while parent isa Tree_Node
		if probe_connect(last(path).content, parent.content, id_offset)
			prev = parent
			parent = parent.parent
		else
			push!(path, prev)
		end
	end
	push!(path, prev)
	
	return path
end

# obsolete
function optimized_probe_path(node::Tree_Node, id_offset::Int)::Vector{Tree_Node}
	path = Vector{Tree_Node}()
	push!(path, node)
	current = node
	parent = node.parent
	prev = false

	while parent isa Tree_Node
		if probe_connect(parent.content, current.content, id_offset)
			prev = parent
			parent = parent.parent
		else
			push!(path, prev)
			current = prev
			prev = parent
		end
	end
	push!(path, prev)

	return path
end

