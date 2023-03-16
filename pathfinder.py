import osmnx
import networkx as nx
import polyline
from heapq import heappop, heappush
from itertools import count
from datetime import datetime

#### FUNCTIONS ####

def bidirectional_dijkstra(G, source, target, weight="weight"):
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if source == target:
        return (0, [source])

    weight = _weight_function(G, weight)
    push = heappush
    pop = heappop
    # Init:  [Forward, Backward]
    dists = [{}, {}]  # dictionary of final distances
    paths = [{source: [source]}, {target: [target]}]  # dictionary of paths
    fringe = [[], []]  # heap of (distance, node) for choosing node to expand
    seen = [{source: 0}, {target: 0}]  # dict of distances to seen nodes
    c = count()
    # initialize fringe heap
    push(fringe[0], (0, next(c), source))
    push(fringe[1], (0, next(c), target))
    # neighs for extracting correct neighbor information
    if G.is_directed():
        neighs = [G._succ, G._pred]
    else:
        neighs = [G._adj, G._adj]
    # variables to hold shortest discovered path
    # finaldist = 1e30000
    finalpath = []
    dir = 1
    while fringe[0] and fringe[1]:
        # choose direction
        # dir == 0 is forward direction and dir == 1 is back
        dir = 1 - dir
        # extract closest to expand
        (dist, _, v) = pop(fringe[dir])
        if v in dists[dir]:
            # Shortest path to v has already been found
            continue
        # update distance
        dists[dir][v] = dist  # equal to seen[dir][v]
        if v in dists[1 - dir]:
            # if we have scanned v in both directions we are done
            # we have now discovered the shortest path
            return (finaldist, finalpath)

        for w, d in neighs[dir][v].items():
            # weight(v, w, d) for forward and weight(w, v, d) for back direction
            cost = weight(v, w, d) if dir == 0 else weight(w, v, d)
            if cost is None:
                continue
            vwLength = dists[dir][v] + cost
            if w in dists[dir]:
                if vwLength < dists[dir][w]:
                    raise ValueError("Contradictory paths found: negative weights?")
            elif w not in seen[dir] or vwLength < seen[dir][w]:
                # relaxing
                seen[dir][w] = vwLength
                push(fringe[dir], (vwLength, next(c), w))
                paths[dir][w] = paths[dir][v] + [w]
                if w in seen[0] and w in seen[1]:
                    # see if this path is better than the already
                    # discovered shortest path
                    totaldist = seen[0][w] + seen[1][w]
                    if finalpath == [] or finaldist > totaldist:
                        finaldist = totaldist
                        revpath = paths[1][w][:]
                        revpath.reverse()
                        finalpath = paths[0][w] + revpath[1:]
    raise nx.NetworkXNoPath(f"No path between {source} and {target}.")

def _weight_function(G, weight):
    if callable(weight):
        return weight
    # If the weight keyword argument is not callable, we assume it is a
    # string representing the edge attribute containing the weight of
    # the edge.
    if G.is_multigraph():
        return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
    return lambda u, v, data: data.get(weight, 1)

def api_profile(weather, profile):
    new_profile = profile
    now = datetime.now().hour

    # If weather = clear:
    # new_profile.pop("flood_hazard")
    #if now not in [19,20,21,22,23,24,1,2,3,4,5]:
    #    new_profile.pop("lighting")
    return new_profile

def adjust_weight(length, row, profile):
    weight = length
    modifier = 1
    for safety_factor,user_preference in profile.items():
        if row[safety_factor] == '0':
            modifier += user_preference
    return weight * modifier

#### PATH FINDING ####

#Returns coordinates of route as an array of tuples
def getCoordinates(route, nodes):
  df = nodes.query('osmid in @route')
  #Drop geometry column
  df = df.drop('geometry',index=None,axis=1)
  coordinates = list(zip(*map(df.get,df)))

  return coordinates

#Returns encoded polyline using the polyline library, requires input as an array of tuples
def getPolyline(route, nodes):
  coordinates = getCoordinates(route, nodes)
  return polyline.encode(coordinates)

#Function to return the total length of the route in meters
def getRouteLength(route, graph):
    route_length = osmnx.utils_graph.get_route_edge_attributes(graph, route, attribute='length')
    return round(sum(route_length))

#Convert bearing value to readable instruction
def getBearingString(degrees, name):
  instruction = None
  if degrees < 45:
    instruction = 'Head North '
  elif degrees < 135:
    instruction = 'Head East '
  elif degrees < 225:
    instruction = 'Head South '
  elif degrees < 315:
    instruction = 'Head West '
  else:
    instruction = 'Head North '
  if name == '':
    return instruction
  return instruction + 'along ' + name

#get maneuever type based on bearing relative_bearing
def getManeuever(heading, true_bearing):
  relative_bearing = abs(true_bearing - heading)
  if relative_bearing <= 15 or relative_bearing >= 345:
    return 'straight'
  return 'turn'

#Translate turn direction in bearings to string
def getTurnDirection(heading, true_bearing, name):
    relative_bearing = true_bearing - heading
    if relative_bearing < 0:
       relative_bearing += 360
    instruction = None
    if relative_bearing <= 15 or relative_bearing >= 345:
        instruction = 'Continue Straight '
    if relative_bearing > 15 and relative_bearing < 45:
        instruction = 'Turn slightly right '
    if relative_bearing >= 45 and relative_bearing < 180:
        instruction = 'Turn Right '
    if relative_bearing > 180 and relative_bearing <= 315:
        instruction = 'Turn Left '
    if relative_bearing > 315 and relative_bearing <= 345:
        instruction = 'Turn slightly left '
    if name == '':
        return instruction
    return instruction + 'onto ' + name

def getRouteDirections(route, graph, safety_factors):
  #generate edge bearings for graph
  bearings_graph = osmnx.bearing.add_edge_bearings(graph, precision=1)
  #Generate a dictionary of relevant keys of the route for directions
  steps = osmnx.utils_graph.get_route_edge_attributes(bearings_graph, route)

  direction = []
  instruction = None
  maneuever = None # depart - first step, turn - for any step in between, arrive - last step
  distance = None
  name = None
  before_maneuever = None
  before_name = None
  footway = None
  bearing_before = 0
  bearing_after = 0
  #Start parsing
  for step in steps:
    present_factors = []
    before_name = name
    before_maneuever = maneuever
    name = step.get("name","")
    footway = step.get("footway")
    distance = round(step.get("length"))

    for factor in safety_factors:
        if step.get(factor) == '1':
            present_factors.append(factor)

    # If the step is the first step
    if steps[0] == step:
      bearing_before = 0
      bearing_after = step.get("bearing")
      maneuever = 'depart'
      instruction = getBearingString(bearing_after, name)
      direction.append({'maneuever':maneuever,
                        'instruction':instruction,
                        'name':name,
                        'bearing_before':bearing_before,
                        'bearing_after':bearing_after,
                        'distance':distance,
                        'factors_present': present_factors})
      continue

    #If the step is any steps in between the first and last step
    if steps[0] != step and steps[-1] != step:
      bearing_before = bearing_after
      bearing_after = step.get("bearing")
      maneuever = getManeuever(bearing_before, bearing_after)
      
      if before_name == name and before_maneuever == 'straight' and maneuever == 'straight':
        direction[-1]["distance"] + distance
        for factor in present_factors:
            if factor not in direction[-1]["factors_present"]:
                direction[-1]["factors_present"].append(factor)    
        continue
      if footway == 'crossing':
        instruction = 'Cross the street'
      if footway == 'footbridge':
        instruction = 'Cross the street via the footbridge'
      else:
        instruction = getTurnDirection(bearing_before, bearing_after, name)
      direction.append({'maneuever':maneuever,
                        'instruction':instruction,
                        'name':name,
                        'bearing_before':bearing_before,
                        'bearing_after':bearing_after,
                        'distance':distance,
                        'factors_present': present_factors})
      continue

    #If the step is the last step
    if steps[-1] == step:
      bearing_before = bearing_after
      bearing_after = step.get("bearing")
      maneuever = 'arrive'
      instruction = getTurnDirection(bearing_before, bearing_after, name) + " and arrive at destination"
      direction.append({'maneuever':maneuever,
                        'instruction':instruction,
                        'name':name,
                        'bearing_before':bearing_before,
                        'bearing_after':bearing_after,
                        'distance':distance,
                        'factors_present': present_factors})

  return direction

def pathfinder(origin, destination):

    #### SETTINGS ####

    safety_factors = ['not_flood_hazard','pwd_friendly','cctv','landmark','lighting','not_major_road']
    osmnx.settings.useful_tags_way = safety_factors + ['name','footway']

    profile = {"not_flood_hazard": 0.5,
                "pwd_friendly": 0.5,
                "cctv": 0.5,
                "landmark": 0.5,
                "lighting": 0.5,
                "not_major_road": 0.5}

    #comes from application request
    origin = {
        "y": origin[0], #14.635749969867808,  
        "x": origin[1]  #121.09445094913893 
    }
    destination = {
        "y": destination[0], #14.63056033942939,  
        "x": destination[1] #121.09807731641334 
    }

    # TODO: insert function to adjust profile weights depending on user preference

    # retrieve map from database
    graph = osmnx.graph_from_xml('marikina_complete.osm', simplify=False)

    # get all edges for weight adjustment
    nodes, edges = osmnx.graph_to_gdfs(graph)

    #adjust weights profile depending on user pref and time & weather conditions
    adjusted_profile = api_profile('clear',profile)

    # create category "weight" for use in path finding
    edges['weight'] = edges.apply(
        lambda row: adjust_weight(row['length'], row, adjusted_profile), axis = 1
        )

    final_graph = osmnx.graph_from_gdfs(
        osmnx.graph_to_gdfs(graph, edges=False),
        edges
    )

    origin_node_id = osmnx.nearest_nodes(final_graph, origin['x'], origin['y'])
    destination_node_id = osmnx.nearest_nodes(final_graph, destination['x'], destination['y'])

    route = bidirectional_dijkstra(
        final_graph, 
        origin_node_id, 
        destination_node_id, 
        weight='weight'
        )
    
    route = route[1]

    response = {
        'origin': [origin['y'],origin['x']], 
        'destination': [destination['y'],destination['x']],
        'length': getRouteLength(route, graph),
        'polyline': getPolyline(route, nodes),
        'steps': getRouteDirections(route, graph, safety_factors)
        }
    
    return response