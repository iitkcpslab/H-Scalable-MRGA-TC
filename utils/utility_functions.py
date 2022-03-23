'''
This file contains the definitions of Utility functions which are common to both approaches: the Heuristic and the Naive approach
'''


# import copy
from collections import deque
from utils.class_definitions import Graph, Edge


def vertex_saturated(v, M):
	
	# Determines whether a vertex is saturated by the matched edges

	for e in M:
		if v == e.vertices[0]:
			return e.vertices[1]
		elif v == e.vertices[1]:
			return e.vertices[0]	

	return False


# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------


def find_maxMatch(eq_G, left_vertices):
	
	# Find maximum matching in the given equality subgraph eq_G

	# Getting an initial matching:

	M = set()
	matched = set()    #   matched left side vertices
	unmatched = set()  # unmatched left side vertices


	for x in left_vertices: # for x in eq_G.vertices:  optimized 1 
		# if eq_G.vertices[x].in_left and not vertex_saturated(x, M):  optimized 1
		if not vertex_saturated(x, M):
			min_edge = None
			for y in eq_G.vertices[x].neighbors:
				if not vertex_saturated(y, M):
					edge = eq_G.vertices[x].incident_edges[y]
					if min_edge is None or edge.weight < min_edge.weight:
						min_edge = edge
			if min_edge is not None:
				M.add(min_edge)
				matched.add(x)
	
	unmatched = left_vertices - matched

	# Now maximizing the matching in the given equality-subgraph:

	while ( len(unmatched) != 0 ):


		x = unmatched.pop()
		queue = deque()
		queue.append(x)
		parent = {}
		visited = set()

		y_path_curr = None

		while queue:

			x_iter = queue.pop()
			visited.add( x_iter )
			
			for edge in eq_G.vertices[x_iter].incident_edges.values():
				y = edge.get_pair_vertex( eq_G.vertices[x_iter] )
				if y not in visited:
					visited.add(y)
					parent[y] = x_iter

					matched_x = vertex_saturated( y, M )
					
					if matched_x == False:
						y_path_curr = y
						break
					else:
						queue.append(matched_x)
					
			if y_path_curr != None:
				break
		

		
		if y_path_curr != None:
			while (parent[y_path_curr] != x):
				new_y = vertex_saturated( parent[y_path_curr], M )
				M.add(eq_G.vertices[y_path_curr].incident_edges[parent[y_path_curr]])
				M.remove(eq_G.vertices[parent[y_path_curr]].incident_edges[new_y])
				y_path_curr = new_y

			M.add(eq_G.vertices[y_path_curr].incident_edges[parent[y_path_curr]])



	return M


# ---------------------------------------------------------------------------------------------------------------------


# New function to get an initial matching

def get_initial_match(eq_G, left_vertices):

	M = set()

	for x in left_vertices:
		if not vertex_saturated(x, M):
			for y in eq_G.vertices[x].neighbors:
				if not vertex_saturated(y, M):
					edge = eq_G.vertices[x].incident_edges[y]
					M.add(edge)
					break

	return M



# Revised function to maximize a given matching

def maximize_match(eq_G, M, left_vertices):


	unmatched = set()  # unmatched left side vertices

	for x in left_vertices:
		if not vertex_saturated(x, M):
			unmatched.add(x)	
	

	# Now maximizing the matching in the given equality-subgraph

	while ( len(unmatched) != 0 ):


		x = unmatched.pop()
		queue = deque()
		queue.append(x)
		parent = {}
		visited = set()

		y_path_curr = None

		while queue:

			x_iter = queue.pop()
			visited.add( x_iter )
			
			for edge in eq_G.vertices[x_iter].incident_edges.values():
				y = edge.get_pair_vertex( eq_G.vertices[x_iter] )
				if y not in visited:
					visited.add(y)
					parent[y] = x_iter

					matched_x = vertex_saturated( y, M )
					
					if matched_x == False:
						y_path_curr = y
						break
					else:
						queue.append(matched_x)
					
			if y_path_curr != None:
				break
		

		
		if y_path_curr != None:
			while (parent[y_path_curr] != x):
				new_y = vertex_saturated( parent[y_path_curr], M )
				M.add(eq_G.vertices[y_path_curr].incident_edges[parent[y_path_curr]])
				M.remove(eq_G.vertices[parent[y_path_curr]].incident_edges[new_y])
				y_path_curr = new_y

			M.add(eq_G.vertices[y_path_curr].incident_edges[parent[y_path_curr]])



	return M




# Revised maximization function for max-match ends 

# ---------------------------------------------------------------------------------------------------------------------


def delete_matched_edge(M_dash, v):    # deletes a matched edge which contains vertex v
	for e in M_dash:
		if v == e.vertices[0] or v == e.vertices[1]:
			M_dash.discard(e)
			break


	return M_dash

# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------------------------------------------------------------------------

# New optimized MVC function written on Jan 11, 2022

def find_MinVertexCover(eq_G, M, left_vertices, right_vertices):    # polynomial time algorithm for finding 'Minimum' vertex cover

	mvc = set()
	matched = set()
	unmatched = []



	# M_dash = copy.deepcopy(M)  # may be we can use M itself as it is no longer needed after this function.
	M_dash = set()
	for e in M:
		e_new = Edge(e.vertices[0], e.vertices[1], e.weight)
		e_new.typeOfWeight = e.typeOfWeight
		M_dash.add(e_new)

		matched.add(e.vertices[0])
		matched.add(e.vertices[1])


	all_vertices = left_vertices | right_vertices

	unmatched = list( all_vertices - matched )


	for u in unmatched:
		for v in eq_G.vertices[u].neighbors:
			v_sat = vertex_saturated(v, M)
			if (v not in mvc) and v_sat:
				mvc.add(v)
				unmatched.append(v_sat)
				M_dash = delete_matched_edge(M_dash, v)

	for edge in M_dash:
		mvc.add(edge.vertices[1])



	return mvc

# ---------------------------------------------------------------------------------------------------------------------


# Old unoptimized MVC function - deprecated on Jan 11, 2022

def find_MinVertexCover_v1(eq_G, M, left_vertices, right_vertices):    # polynomial time algorithm for finding 'Minimum' vertex cover

	mvc = set()
	unmatched = []

	

	# collecting all unmatched vertices from X and Y
	for x in left_vertices:
		if not vertex_saturated(x, M):
			unmatched.append(x)

	for y in right_vertices:
		if not vertex_saturated(y, M):
			unmatched.append(y)

	
	# M_dash = copy.deepcopy(M)  # may be we can use M itself as it is no longer needed after this function.
	M_dash = set()
	for e in M:
		e_new = Edge(e.vertices[0], e.vertices[1], e.weight)
		e_new.typeOfWeight = e.typeOfWeight
		M_dash.add(e_new)



	for u in unmatched:
		for v in eq_G.vertices[u].neighbors:
			if v not in mvc and vertex_saturated(v, M):
				mvc.add(v)
				unmatched.append(vertex_saturated(v, M))
				M_dash = delete_matched_edge(M_dash, v)

	for edge in M_dash:
		mvc.add(edge.vertices[1])



	return mvc

# ---------------------------------------------------------------------------------------------------------------------


# New functions for getting initial equality subgraph and then updating the exiting equality subgraph





def get__equality_subgraph( G, left_vertices, right_vertices ):

	
	eq_G = Graph( eq_flag = True )
	
	
	# Creating vertices
	for i in left_vertices:
		eq_G.add_vertex( i, G.vertices[i].label, G.vertices[i].in_left )

	for j in right_vertices:
		eq_G.add_vertex( j, G.vertices[j].label, G.vertices[j].in_left )
	

	# Creating edges

	for i in left_vertices:
		for j in right_vertices:

			edge = G.vertices[i].incident_edges[j]  # edge variable will store the Edge object

			if edge.weight == (G.vertices[i].label + G.vertices[j].label):
				# add edge
				if i[0] == 'r':
					e = Edge(i, j, edge.weight)
				else:
					e = Edge(j, i, edge.weight)
				e.typeOfWeight = edge.typeOfWeight


				eq_G.vertices[i].neighbors.add(j)
				eq_G.vertices[j].neighbors.add(i)
				eq_G.vertices[i].incident_edges[j] = e
				eq_G.vertices[j].incident_edges[i] = e


	return eq_G


