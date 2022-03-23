# import time
import numpy as np
from math import sqrt
from math import inf
from math import isinf

from utils.search_methods import DIJKSTRA_2D, DIJKSTRA_3D, FRASTAR, FRASTAR_3D

from utils.class_definitions import Graph
from utils.utility_functions import find_MinVertexCover, get_initial_match, maximize_match, get__equality_subgraph


left_vertices = set()
right_vertices = set()
# path = {}
# no_of_explorations = 0

# OL_dict = {}
# CL_dict = {}





# ---------------------------------------------------------------------------------------------------------------------

def update_label(G, mvc):
	
	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	brk = 0
	delta_min = inf
	for i in uncovered_robots:
		for j in uncovered_goals: 
			if j in G.vertices[i].neighbors:   # redundant IF 
				delta = G.vertices[i].incident_edges[j].weight - ( G.vertices[i].label + G.vertices[j].label )  # aks - MAIN POINT
				# new_alpha = abs(new_alpha) # aks 98
				# if not isinf(delta):
				# delta_min = delta if delta < delta_min else delta_min        # aks - MAIN POINT
				if delta < delta_min:
					delta_min = delta


	# EXTENDED method of getting the delta_min, according to the paper, is commented below:
	# The extended version may be useful in distributed implementation as we use to verify there whether step 1(a) of paper 
	# is completed or not in the Local_Hungarian function

	'''
	
	E_cand = []
	for i in uncovered_robots:
		delta_min = inf
		for j in uncovered_goals: 
			if j in G.vertices[i].neighbors:   # redundant IF 
				delta = G.vertices[i].get_edge(j).weight - ( G.vertices[i].label + G.vertices[j].label )  # aks - MAIN POINT
				# new_alpha = abs(new_alpha) # aks 98
				if (delta < delta_min) :
					# delta_min = delta if delta_min is None or delta < delta_min else delta_min        # aks - MAIN POINT
					delta_min = delta
					# e_i_cand = G.vertices[i].get_edge(j)   NOT USED 
		E_cand.append(delta_min)

	
	# now getting GLOBAL delta_min
	delta_min = min(E_cand)

	'''
		
	
	if isinf( delta_min ):
		brk = 1
		return G, brk 


	# if delta_min != None:
	for i in left_vertices & mvc:   # for covered robots
		G.vertices[i].label = G.vertices[i].label - delta_min
	for j in uncovered_goals:       # for uncovered goals
		G.vertices[j].label = G.vertices[j].label + delta_min


	return G, brk


# ---------------------------------------------------------------------------------------------------------------------



# ---------------------------------------------------------------------------------------------------------------------
# main function starts
# ---------------------------------------------------------------------------------------------------------------------


def find_assignment_N( _G ):
	

	# global no_of_explorations
	# no_of_explorations = 0
	# path.clear()
	left_vertices.clear()
	right_vertices.clear()
	
	
	# Construct a bipartite graph 
	
	G = Graph( _G )
	
		
	for x in G.vertices:
		if G.vertices[x].in_left:
			left_vertices.add(x)
			# if x[0] == 'r':
			# 	path[x] = {}
		else:
			right_vertices.add(x)
			# if x[0] == 'r':
			# 	path[x] = {}
	
	
	
	# Generate an initial feasible labeling
	G.generate_feasible_labeling( left_vertices, right_vertices )


	
	# Get the equality subgraph
	# eq_G = G.equality_subgraph()
	eq_G = get__equality_subgraph( G, left_vertices, right_vertices )

	
	
	# Get matching in the equality subgraph and then get the corresponding minimum vertex cover:

	
	# Finding an initial matching
	
	M = get_initial_match(eq_G, left_vertices)

	# Maximize the current matching

	# M = find_maxMatch(eq_G, left_vertices)
	M = maximize_match(eq_G, M, left_vertices)

	
	# change 999 - for handling case when no of robots != no of goals
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  # change 999

	
	# while len(M) < int(len(eq_G.vertices)/2):   # change 999
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(eq_G, M, left_vertices, right_vertices)    # polynomial time algorithm for finding 'Minimum' vertex cover
		
		
		G, brk = update_label(G, mvc)
		if brk == 1:
			break
				

		# eq_G = G.equality_subgraph()
		eq_G = get__equality_subgraph( G, left_vertices, right_vertices )
		# M = find_maxMatch(eq_G, left_vertices)
		M = maximize_match(eq_G, M, left_vertices)
		



	
	
	# ------------------ End Result -----------------------

	total = 0
	unassigned_count = 0
	assigned_count = 0
	if (len(M) != 0):
		for e in M:
			if not isinf( e.weight ):
				total = total + e.weight
				assigned_count = assigned_count + 1
	

	unassigned_count = min_robots_goals - assigned_count
		

	return list(map(lambda e: ((e.vertices[0], e.vertices[1]), e.weight), M)), total, unassigned_count
	
# ---------------------------------------------------------------------------------------------------------------------




# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Naive approach
# --------------------------------------------------------------------------------------------------------------

def naive ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity ):

	path = {}
	# costMatrix = np.zeros((no_of_robots, no_of_goals))
	# For adding dummy robots / goals for the one which is less in number
	max_robots_goals = max( no_of_robots, no_of_goals)  # for dummy 
	costMatrix = np.zeros((max_robots_goals, max_robots_goals))  # for dummy



	# if no_of_robots <= no_of_goals:
	
	# NC 1 
	
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		path[robot_name] = {}
		
		h_cost = 0   # for Dijkstra's algorithm


		# Invoking Dijkstra's algorithm
		if ws_type == '3D':
			# one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
			path_dict, cost_dict = DIJKSTRA_3D( workSpace, all_start_loc[robot_name], set(all_goal_loc.values()), h_cost, rtype = 'pathAndCost' ) 
		else:
			path_dict, cost_dict = DIJKSTRA_2D( workSpace, all_start_loc[robot_name], set(all_goal_loc.values()), h_cost, rtype = 'pathAndCost' )

		
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			costMatrix[i][j] = cost_dict[all_goal_loc[goal_name]]
			path[robot_name][goal_name] = path_dict[all_goal_loc[goal_name]]


		if verbosity > 1:
			print( "Baseline case: Paths for robot ", i, " computed" )

		# NC 1 ends
	
	# else:
		
	# 	for j in range(no_of_goals):
	# 		goal_name = 'g' + str(j)
	# 		path[goal_name] = {}
			
	# 		h_cost = 0   # for Dijkstra's algorithm


	# 		# Invoking Dijkstra's algorithm
	# 		if ws_type == '3D':
	# 			# one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
	# 			path_dict, cost_dict = DIJKSTRA_3D( workSpace, all_goal_loc[goal_name], set(all_start_loc.values()), h_cost, rtype = 'pathAndCost' )
	# 		else:
	# 			# path_dict, cost_dict = DIJKSTRA_2D( workSpace, all_start_loc[robot_name], set(all_goal_loc.values()), h_cost, rtype = 'pathAndCost' )
	# 			path_dict, cost_dict = DIJKSTRA_2D( workSpace, all_goal_loc[goal_name], set(all_start_loc.values()), h_cost, rtype = 'pathAndCost' )

			
	# 		for i in range(no_of_robots):
	# 			robot_name = 'r' + str(i)
	# 			costMatrix[i][j] = cost_dict[all_start_loc[robot_name]]
	# 			path[goal_name][robot_name] = path_dict[all_start_loc[robot_name]]


	# 		if verbosity > 1:
	# 			print( "Baseline case: Paths for goal ", j, " computed" )





	'''
	# commented due to NC 1
	for i in range(no_of_robots):
		t_a_begin = time.time()
		robot_name = 'r' + str(i)
		path[robot_name] = {}
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
			else:
				one_path, costMatrix[i][j] = ASTAR( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' )
			path[robot_name][goal_name] = one_path
			if verbosity > 1:
				print( "Baseline case: Path for robot-goal pair (", i, ", ", j, ") computed" )
		if verbosity > 1:
			print("")

		t_a_end = time.time()
		if ( t_a_end - t_a_begin ) > 3000:
			return None, None, None, None, -1   # Sending no_of_exp = -1 to denote BREAK from normal execution owing to too much time taken by ASTAR
	'''
	
	

	result, total_cost, u_count = find_assignment_N( costMatrix )  
	
	no_of_exp = no_of_robots * no_of_goals

	return result, total_cost, path, u_count, no_of_exp




# New naive function with FRASTAR

# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Naive approach
# --------------------------------------------------------------------------------------------------------------

def naive_with_FRASTAR ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type, verbosity ):

	path = {}
	# costMatrix = np.zeros((no_of_robots, no_of_goals))
	# For adding dummy robots / goals for the one which is less in number
	max_robots_goals = max( no_of_robots, no_of_goals)  # for dummy 
	costMatrix = np.zeros((max_robots_goals, max_robots_goals))  # for dummy

	# global OL_dict, CL_dict
	# OL_dict.clear()
	# CL_dict.clear()


	# '''
	# commented due to NC 1
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		path[robot_name] = {}
		# OL_dict[robot_name] = []
		# CL_dict[robot_name] = {}
		OL_dict = []
		CL_dict = {}

		if ws_type == '3D':
			distance_array = np.empty(shape=(workSpace.shape[0], workSpace.shape[1], workSpace.shape[2]))
		else:
			distance_array = np.empty(shape=(workSpace.shape[0], workSpace.shape[1]))
		
		distance_array.fill(inf)



		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				# one_path, costMatrix[i][j] = ASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], rtype = 'pathAndCost' ) 
				one_path, costMatrix[i][j], OL_dict, CL_dict, distance_array = FRASTAR_3D( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict, CL_dict, distance_array, rtype = 'pathAndCost' ) 
			else:
				# one_path, costMatrix[i][j], OL_dict[robot_name], CL_dict[robot_name] = FRASTAR( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict[robot_name], CL_dict[robot_name], rtype = 'pathAndCost' )
				one_path, costMatrix[i][j], OL_dict, CL_dict, distance_array = FRASTAR( workSpace, all_start_loc[robot_name], all_goal_loc[goal_name], OL_dict, CL_dict, distance_array, rtype = 'pathAndCost' )
			path[robot_name][goal_name] = one_path
		
		if verbosity > 1:
			print( "Baseline case with FRASTAR: Paths for robot", i,  " computed" )
		# if verbosity > 1:
		# 	print("")

	# '''
	
	

	result, total_cost, u_count = find_assignment_N( costMatrix )  
	
	no_of_exp = no_of_robots * no_of_goals

	return result, total_cost, path, u_count, no_of_exp