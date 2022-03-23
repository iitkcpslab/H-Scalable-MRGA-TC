# import time
from heapq import *
import numpy as np
from math import sqrt, inf, isinf
# from math import inf, isinf
# from math import isinf
# from operator import itemgetter

from utils.search_methods import FRASTAR, FRASTAR_3D
from utils.class_definitions import Graph
from utils.utility_functions import find_MinVertexCover, get_initial_match, maximize_match, get__equality_subgraph


left_vertices = set()
right_vertices = set()
path = {}
no_of_explorations = 0

OL_dict = {}
CL_dict = {}
distance_lookup = {}


# ---------------------------------------------------------------------------------------------------------------------
# collect_delta : Function to collect delta (slack) for edges between uncovered robots and uncovered goals
# ---------------------------------------------------------------------------------------------------------------------

def collect_delta(G, mvc):
	

	# Collecting delta:

	uncovered_robots = left_vertices - mvc
	uncovered_goals = right_vertices - mvc
	
	
	deltas = []
	for i in uncovered_robots:
		for j in uncovered_goals: 
			edge = G.vertices[i].incident_edges[j]
			slack = edge.weight - ( G.vertices[i].label + G.vertices[j].label )
			# if not math.isinf(delta):
			# deltas.append( ( i, j, slack, edge.typeOfWeight ) )
			deltas.append( ( slack, edge.typeOfWeight, i, j ) )  # c77

	
	heapify(deltas) # c77
			
	return deltas

# **********************************************************************************************************************


# ----------------------------------------------------------------------------------------------------------------------
# get_global_delta_min : This function fetches the minimum slack value for an actual-weight edge (and not heuristic-weight)
# ----------------------------------------------------------------------------------------------------------------------


# Optimized

def get_global_delta_min( deltas, G, workSpace, all_start_loc, all_goal_loc, ws_type ):

	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	while (1):
		
		d_min = deltas[0] # c77

		if d_min[1] == 'a':
			return d_min[0]

		else:
			# Explore

			edge = G.vertices[d_min[2]].incident_edges[d_min[3]]  # edge variable will store the Edge object
			if d_min[2][0] == 'r':
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR_3D( workSpace, all_start_loc[d_min[2]], all_goal_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR( workSpace, all_start_loc[d_min[2]], all_goal_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				path[d_min[2]][d_min[3]] = one_path
			else:
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR_3D( workSpace, all_goal_loc[d_min[2]], all_start_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
					# one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]] = FRASTAR( workSpace, all_goal_loc[d_min[2]], all_start_loc[d_min[3]], OL_dict[d_min[2]], CL_dict[d_min[2]], distance_lookup[d_min[2]], rtype = 'pathAndCost' )
				path[d_min[3]][d_min[2]] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1
			

			new_slack = edge.weight - ( G.vertices[d_min[2]].label + G.vertices[d_min[3]].label )


			new_delta = ( new_slack, 'a', d_min[2], d_min[3], )
			heapreplace( deltas, new_delta )


	
# **********************************************************************************************************************



# Unoptimized


# def get_global_delta_min_v1( deltas, G, workSpace, all_start_loc, all_goal_loc, ws_type ):

# 	global no_of_explorations, OL_dict, CL_dict

# 	while (1):
		
# 		d_min = min(deltas, key = itemgetter(2, 3))   # d_min is complete tuple in form of ( i, j, slack, typeOfWeight )
		

# 		if d_min[3] == 'a':
# 			return d_min[2]

# 		else:
# 			# Explore
# 			edge = G.vertices[d_min[0]].incident_edges[d_min[1]]  # edge variable will store the Edge object
# 			if d_min[0][0] == 'r':
# 				if ws_type == '3D':
# 					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
# 					one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR_3D( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
# 				else:
# 					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], rtype = 'pathAndCost' )
# 					one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR( workSpace, all_start_loc[d_min[0]], all_goal_loc[d_min[1]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
# 				path[d_min[0]][d_min[1]] = one_path
# 			else:
# 				if ws_type == '3D':
# 					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
# 					one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR_3D( workSpace, all_goal_loc[d_min[0]], all_start_loc[d_min[1]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
# 				else:
# 					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], rtype = 'pathAndCost' )
# 					# one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR( workSpace, all_start_loc[d_min[1]], all_goal_loc[d_min[0]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
# 					one_path, edge.weight, OL_dict[d_min[0]], CL_dict[d_min[0]] = FRASTAR( workSpace, all_goal_loc[d_min[0]], all_start_loc[d_min[1]], OL_dict[d_min[0]], CL_dict[d_min[0]], rtype = 'pathAndCost' )
# 				path[d_min[1]][d_min[0]] = one_path[::-1]
			
# 			edge.typeOfWeight = 'a'
# 			no_of_explorations = no_of_explorations + 1
			

# 			new_slack = edge.weight - ( G.vertices[d_min[0]].label + G.vertices[d_min[1]].label )
			
# 			# if new_slack < 0:
# 			# 	check1 = 1
# 			# if math.isnan( new_slack ):
# 			# 	check1 = 2
# 			# 	new_slack = math.inf   # this may not be req as label will not become inf before its weight.


# 			new_delta = ( d_min[0], d_min[1], new_slack, 'a' )
# 			index_d_min = deltas.index(d_min)
# 			del deltas[index_d_min]
# 			deltas.append( new_delta )

	
# **********************************************************************************************************************



# ---------------------------------------------------------------------------------------------------------------------
# 
# ---------------------------------------------------------------------------------------------------------------------


def find_min_Zcost( x, type_of_wt ):  # receives a Vertex object as parameter, Z refers to 'h' or 'a'

	minZcost = inf
	minZindex = None

	for edge in x.incident_edges.values():
		if ( edge.typeOfWeight == type_of_wt ) and ( edge.weight < minZcost ):
			minZcost = edge.weight
			minZindex = edge.get_pair_vertex(x)  # minZindex carries goal g


	return minZcost, minZindex

# **********************************************************************************************************************


# ---------------------------------------------------------------------------------------------------------------------
# Function to explore min actual cost for each robot
# ---------------------------------------------------------------------------------------------------------------------

def explore_min_actual_cost( G, workSpace, all_start_loc, all_goal_loc, ws_type ):

	minAcost = {}
	minAindex = {}
	global no_of_explorations, OL_dict, CL_dict, distance_lookup

	
	
	for x in left_vertices:    # for each robot
		
		OL_dict[x] = []
		CL_dict[x] = {}

		if ws_type == '3D':
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1], workSpace.shape[2]))
		else:
			distance_lookup[x] = np.empty(shape=(workSpace.shape[0], workSpace.shape[1]))
		
		distance_lookup[x].fill(inf)


		minAcost[x] = inf
		minAindex[x] = None

		minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )

		while ( minHindex != None and minHcost <= minAcost[x] ):
			
			edge = G.vertices[x].incident_edges[minHindex]
			if x[0] == 'r':
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[minHindex], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				else:
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_start_loc[x], all_goal_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				path[x][minHindex] = one_path
			else:
				if ws_type == '3D':
					# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_goal_loc[x], all_start_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				else:
					# one_path, edge.weight = ASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], rtype = 'pathAndCost' )
					# Commented below for incorrect R > G costs. 
					# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[minHindex], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
					one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_goal_loc[x], all_start_loc[minHindex], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
				path[minHindex][x] = one_path[::-1]
			
			edge.typeOfWeight = 'a'
			no_of_explorations = no_of_explorations + 1

			minHcost, minHindex = find_min_Zcost( G.vertices[x], 'h' )
			minAcost[x], minAindex[x] = find_min_Zcost( G.vertices[x], 'a' )


	return G, minAcost

# **********************************************************************************************************************


# ---------------------------------------------------------------------------------------------------------------------
# main function starts
# ---------------------------------------------------------------------------------------------------------------------


def find_assignment_H( _G, workSpace, all_start_loc, all_goal_loc, ws_type ):
	

	global no_of_explorations, OL_dict, CL_dict, distance_lookup
	no_of_explorations = 0
	path.clear()
	left_vertices.clear()
	right_vertices.clear()
	OL_dict.clear()
	CL_dict.clear()
	
	
	# Construct a bipartite graph 
	
	G = Graph( _G, heur = True )
	
		
	for x in G.vertices:
		if G.vertices[x].in_left:
			left_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
		else:
			right_vertices.add(x)
			if x[0] == 'r':
				path[x] = {}
	


	G, minAcost = explore_min_actual_cost( G, workSpace, all_start_loc, all_goal_loc, ws_type )
	
	# Generate an initial feasible labeling
	G.generate_feasible_labeling_H( left_vertices, right_vertices, minAcost ) 


	
	# Get the equality subgraph
	# eq_G = G.equality_subgraph()
	eq_G = get__equality_subgraph( G, left_vertices, right_vertices )

	
	
	# Get matching in the equality subgraph and then get the corresponding minimum vertex cover:

	
	# Finding an initial matching
	# M = find_maxMatch(eq_G, left_vertices)
	M = get_initial_match(eq_G, left_vertices)

	# Maximize the current matching
	M = maximize_match(eq_G, M, left_vertices)

	
	# change 999 - for handling case when no of robots != no of goals
	min_robots_goals = min( len(left_vertices), len(right_vertices) )  # change 999

	
	# while len(M) < int(len(eq_G.vertices)/2):   # change 999
	while len(M) < min_robots_goals:

		mvc = find_MinVertexCover(eq_G, M, left_vertices, right_vertices)    # polynomial time algorithm for finding 'Minimum' vertex cover
		
	
		deltas = collect_delta(G, mvc)
		global_d_min = get_global_delta_min( deltas, G, workSpace, all_start_loc, all_goal_loc, ws_type )

		if isinf( global_d_min ):
			break                        # This means no further goal is reachable; So Break and report the available matching. 


		# For heur case, before getting eq-subgraph of 2nd round (and onwards), we need to Explore such "heuristic" cost edges, 
		# whose slack is equal to min_slack (global_d_min), after the just above label-update. 
		# This is to be done ONLY between uncovered robots and uncovered goals. (as a new edge could arise between them only)
		# We didn't need this step before the generation of 1st eq_subgraph because this case arises only after a label-update as
		# new edges would come after a label-update.

		
		uncovered_robots = left_vertices - mvc
		uncovered_goals = right_vertices - mvc

		for x in uncovered_robots:        # for each robot
			for y in uncovered_goals:     # for each goal
			
				edge = G.vertices[x].incident_edges[y]  # edge variable will store the Edge object
				slack = edge.weight - (G.vertices[x].label + G.vertices[y].label)
				if ( edge.typeOfWeight == 'h' ) and ( global_d_min == slack ):
					if x[0] == 'r':
						# Explore A.cost
						if ws_type == '3D':
							# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
							one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
						else:
							# one_path, edge.weight = ASTAR( workSpace, all_start_loc[x], all_goal_loc[y], rtype = 'pathAndCost' )
							one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_start_loc[x], all_goal_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
						path[x][y] = one_path
					else:
						if ws_type == '3D':
							# one_path, edge.weight = ASTAR_3D( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
							one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR_3D( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
						else:
							# one_path, edge.weight = ASTAR( workSpace, all_start_loc[y], all_goal_loc[x], rtype = 'pathAndCost' )
							# one_path, edge.weight, OL_dict[x], CL_dict[x] = FRASTAR( workSpace, all_start_loc[y], all_goal_loc[x], OL_dict[x], CL_dict[x], rtype = 'pathAndCost' )
							one_path, edge.weight, OL_dict[x], CL_dict[x], distance_lookup[x] = FRASTAR( workSpace, all_goal_loc[x], all_start_loc[y], OL_dict[x], CL_dict[x], distance_lookup[x], rtype = 'pathAndCost' )
						path[y][x] = one_path[::-1]
					
					edge.typeOfWeight = 'a'
					no_of_explorations = no_of_explorations + 1

			
			
		# if global_d_min != None:
		for i in left_vertices & mvc:   # for covered robots
			G.vertices[i].label = G.vertices[i].label - global_d_min
		
		for j in uncovered_goals:       # for uncovered goals
			G.vertices[j].label = G.vertices[j].label + global_d_min
			
					


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
		
	
	return list(map(lambda e: ((e.vertices[0], e.vertices[1]), e.weight), M)), total, path, unassigned_count, no_of_explorations
	
# **********************************************************************************************************************



# --------------------------------------------------------------------------------------------------------------
#                          Directly - exposed function for Heuristic-based approach
# --------------------------------------------------------------------------------------------------------------

def heuristic ( no_of_robots, no_of_goals, workSpace, all_start_loc, all_goal_loc, ws_type  ):


	costMatrix = np.zeros((no_of_robots, no_of_goals))
	# Generating heuristic costs for all pairs of robots and goals:
	for i in range(no_of_robots):
		robot_name = 'r' + str(i)
		for j in range(no_of_goals):
			goal_name = 'g' + str(j)
			
			if ws_type == '3D':
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
				(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2) + (
					(all_start_loc[robot_name][2] - all_goal_loc[goal_name][2]) ** 2)
			else:
				sq_of_distance = ((all_start_loc[robot_name][0] - all_goal_loc[goal_name][0]) ** 2) + (
					(all_start_loc[robot_name][1] - all_goal_loc[goal_name][1]) ** 2)

			# costMatrix[i][j] = float(format(sqrt(sq_of_distance), '.2f'))      # <<------ Discuss about decimal points
			costMatrix[i][j] = sqrt(sq_of_distance)
			

		
	result_h, total_cost_h, path_h, u_count_h, no_of_exp_h = find_assignment_H( costMatrix, workSpace, all_start_loc, all_goal_loc, ws_type )



	return result_h, total_cost_h, path_h, u_count_h, no_of_exp_h


# **********************************************************************************************************************
