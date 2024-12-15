import inspect
import math
import numpy as np
from collections import defaultdict, deque

class Taxi:
    """A Taxi roams a service area given by a NetWorld, collecting and dropping off Fares.
    This version implements probabilistic path planning with traffic consideration."""
    
    def __init__(self, world, taxi_num, service_area, start_point=None):
        """Initialize the taxi with its parent world, number, service area and optional starting point."""
        
        self._world = world
        self._taxi_num = taxi_num
        self._service_area = service_area
        self._loc = start_point
        self._onDuty = False
        self._destination = None
        self._passenger = None
        self._account = 256 # taxis start with 256 credits
        self._path = []
        self._traffic_history = defaultdict(lambda: [(0, 0)]) # (time, traffic_level) tuples
        self._last_replan = 0
        self._replan_interval = 5 # minutes between path recalculations
        
        # A* pathfinding parameters
        self._traffic_weight = 2.0  # Weight for traffic cost in path planning
        self._distance_weight = 1.0 # Weight for distance cost in path planning
        self._traffic_decay = 0.95  # Decay factor for historical traffic observations
        
        # Initialize location if not provided
        if self._loc is None:
            self._loc = (0, 0) 
            
    def comeOnDuty(self):
        """Put this taxi on duty."""
        self._onDuty = True
        
    def goOffDuty(self):
        """Take this taxi off duty."""
        self._onDuty = False
        self._destination = None
        self._passenger = None
        
    def pickupFare(self, fare):
        """Attempt to collect the given fare."""
        if self._passenger is None and self._loc == fare.origin:
            fare.pickUp(self)
            self._passenger = fare
            return True
        return False
        
    def dropoffFare(self):
        """Attempt to drop off the current fare."""
        if self._passenger is not None and self._passenger.destination == self._loc:
            self._passenger.dropOff()
            self._passenger = None
            return True
        return False
        
    def bid(self, fare):
        """Generate a bid for the given fare."""
        if not self._onDuty or self._account <= 0:
            return -1
        
        # Calculate estimated costs considering traffic
        est_time = self._estimate_journey_time(self._loc, fare.origin)
        if est_time < 0:  # Invalid path
            return -1
            
        # Add estimated time from pickup to destination
        dest_time = self._estimate_journey_time(fare.origin, fare.destination)
        if dest_time < 0:  # Invalid path
            return -1
            
        total_time = est_time + dest_time
        # Simple bid calculation: 2 * estimated time as a basic fare
        bid = 2 * total_time 
        
        return bid if bid <= self._account else -1
        
    def drive(self):
        """Drive towards the current destination."""
        if not self._onDuty or self._account <= 0:
            return
            
        # Update traffic history
        self._update_traffic_history()
        
        # Check if we need to replan
        if self._should_replan():
            self._replan_path()
            
        if not self._path:
            return
            
        # Get next step from path
        next_loc = self._path[0]
        
        # Attempt to move
        if self._world.travelTime(self._loc, next_loc) <= 0:
            # Path blocked, trigger replan
            self._path = []
            return
            
        # Move to next location
        self._loc = next_loc
        self._path.pop(0)
        self._account -= 1  # Basic movement cost
        
    def planPath(self, destination):
        """Plan a path to the destination using A* with traffic consideration."""
        if destination == self._loc:
            return []
            
        self._destination = destination
        self._path = self._a_star_search(self._loc, destination)
        return self._path
        
    def _a_star_search(self, start, goal):
        """A* search algorithm with traffic-aware heuristics."""
        frontier = []
        frontier.append((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            frontier.sort(reverse=True)  # Sort by priority (lower is better)
            current_cost, current = frontier.pop()
            
            if current == goal:
                break
                
            # Get valid neighbors from service area
            neighbors = self._get_valid_neighbors(current)
            
            for next_node in neighbors:
                # Calculate new cost with traffic
                traffic_cost = self._get_traffic_cost(current, next_node)
                movement_cost = self._world.travelTime(current, next_node)
                if movement_cost < 0:  # Invalid move
                    continue
                    
                new_cost = cost_so_far[current] + movement_cost + traffic_cost
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self._heuristic(next_node, goal)
                    frontier.append((priority, next_node))
                    came_from[next_node] = current
                    
        # Reconstruct path
        path = []
        current = goal
        while current != start:
            if current not in came_from:
                return []  # No path found
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path
        
    def _get_valid_neighbors(self, node):
        """Get valid neighboring nodes from service area."""
        neighbors = []
        x, y = node
        
        # Check all 8 possible directions
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
            new_x, new_y = x + dx, y + dy
            if (new_x, new_y) in self._service_area:
                neighbors.append((new_x, new_y))
                
        return neighbors
        
    def _heuristic(self, node, goal):
        """Calculate heuristic value (Manhattan distance + traffic estimate)."""
        x1, y1 = node
        x2, y2 = goal
        distance = abs(x1 - x2) + abs(y1 - y2)
        traffic_estimate = self._estimate_traffic_ahead(node, goal)
        return self._distance_weight * distance + self._traffic_weight * traffic_estimate
        
    def _get_traffic_cost(self, from_node, to_node):
        """Calculate traffic cost between two adjacent nodes."""
        current_traffic = self._world.traffic.get((from_node, to_node), 0)
        historical_traffic = self._get_historical_traffic(from_node, to_node)
        # Combine current and historical traffic with decay
        return max(current_traffic, historical_traffic * self._traffic_decay)
        
    def _estimate_traffic_ahead(self, current, goal):
        """Estimate traffic conditions between current position and goal."""
        x1, y1 = current
        x2, y2 = goal
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        
        # Sample points along direct path
        points = []
        steps = max(dx, dy)
        if steps == 0:
            return 0
            
        for i in range(steps):
            x = x1 + (x2 - x1) * i / steps
            y = y1 + (y2 - y1) * i / steps
            points.append((round(x), round(y)))
            
        # Calculate average traffic along path
        traffic_sum = sum(self._get_traffic_cost(p1, p2) 
                         for p1, p2 in zip(points[:-1], points[1:]))
        return traffic_sum / (len(points) - 1) if len(points) > 1 else 0
        
    def _update_traffic_history(self):
        """Update traffic history for current location."""
        if not self._loc:
            return
            
        current_time = self._world.simTime
        neighbors = self._get_valid_neighbors(self._loc)
        
        for neighbor in neighbors:
            traffic = self._world.traffic.get((self._loc, neighbor), 0)
            self._traffic_history[(self._loc, neighbor)].append(
                (current_time, traffic))
            # Keep only recent history
            while len(self._traffic_history[(self._loc, neighbor)]) > 100:
                self._traffic_history[(self._loc, neighbor)].pop(0)
                
    def _get_historical_traffic(self, from_node, to_node):
        """Get historical traffic level between nodes."""
        history = self._traffic_history[(from_node, to_node)]
        if not history:
            return 0
        # Weight recent observations more heavily
        weights = [self._traffic_decay ** i for i in range(len(history))]
        traffic_levels = [t[1] for t in history]
        return np.average(traffic_levels, weights=weights)
        
    def _should_replan(self):
        """Determine if path should be recalculated."""
        if not self._path:
            return True
            
        current_time = self._world.simTime
        if current_time - self._last_replan >= self._replan_interval:
            return True
            
        # Check for significant traffic changes along path
        if len(self._path) > 1:
            current = self._loc
            for next_node in self._path:
                if self._get_traffic_cost(current, next_node) >= 4:  # High traffic threshold
                    return True
                current = next_node
                
        return False
        
    def _replan_path(self):
        """Recalculate path to destination."""
        if self._destination:
            self._last_replan = self._world.simTime
            self._path = self._a_star_search(self._loc, self._destination)
            
    def _estimate_journey_time(self, start, end):
        """Estimate journey time between two points considering traffic."""
        path = self._a_star_search(start, end)
        if not path:
            return -1
            
        total_time = 0
        current = start
        for next_node in path:
            movement_time = self._world.travelTime(current, next_node)
            if movement_time < 0:
                return -1
            traffic_time = self._get_traffic_cost(current, next_node)
            total_time += movement_time + traffic_time
            current = next_node
            
        return total_time

    @property
    def number(self):
        return self._taxi_num
        
    @property
    def onDuty(self):
        return self._onDuty
        
    @property
    def location(self):
        return self._loc
        
    @property
    def passenger(self):
        return self._passenger
        
    @property
    def account(self):
        return self._account
