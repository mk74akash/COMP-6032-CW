import inspect
import math
from re import S
import numpy
import heapq
from collections import defaultdict, deque


# a data container object for the taxi's internal list of fares. This
# tells the taxi what fares are available to what destinations at
# what price, and whether they have been bid upon or allocated. The  
# origin is notably missing: that's because the Taxi will keep this
# in a dictionary indexed by fare origin, so we don't need to duplicate that
# here.
class FareInfo:

      def __init__(self, destination, price):

          self.destination = destination
          self.price = price
          # bid is a ternary value: -1 = no, 0 = undecided, 1 = yes indicating whether this
          # taxi has bid for this fare. 
          self.bid = 0
          self.allocated = False


''' A Taxi is an agent that can move about the world it's in and collect fares. All taxis have a
    number that identifies them uniquely to the dispatcher. Taxis have a certain amount of 'idle'
    time they're willing to absorb, but beyond that, they go off duty since it seems to be a waste
    of time to stick around. Eventually, they might come back on duty, but it usually won't be for
    several hours. A Taxi also expects a map of the service area which forms part of its knowledge
    base. Taxis start from some fixed location in the world. Note that they can't just 'appear' there:
    any real Node in the world may have traffic (or other taxis!) there, and if its start node is
    unavailable, the taxi won't enter the world until it is. Taxis collect revenue for fares, and 
    each minute of active time, whether driving, idle, or conducting a fare, likewise costs them £1.
'''           
class Taxi:
      
      # message type constants
      FARE_ADVICE = 1
      FARE_ALLOC = 2
      FARE_PAY = 3
      FARE_CANCEL = 4

      '''constructor. The only required arguments are the world the taxi operates in and the taxi's number.
         optional arguments are:
         idle_loss - how much cost the taxi is prepared to absorb before going off duty. 256 gives about 4
         hours of life given nothing happening. Loss is cumulative, so if a taxi was idle for 120 minutes,
         conducted a fare over 20 minutes for a net gain to it of 40, then went idle for another 120 minutes,
         it would have lost 200, leaving it with only £56 to be able to absorb before going off-duty.
         max_wait - this is a heuristic the taxi can use to decide whether a fare is even worth bidding on.
         It is an estimate of how many minutes, on average, a fare is likely to wait to be collected.
         on_duty_time - this says at what time the taxi comes on duty (default is 0, at the start of the 
         simulation)
         off_duty_time - this gives the number of minutes the taxi will wait before returning to duty if
         it goes off (default is 0, never return)
         service_area - the world can optionally populate the taxi's map at creation time.
         start_point - this gives the location in the network where the taxi will start. It should be an (x,y) tuple.
         default is None which means the world will randomly place the Taxi somewhere on the edge of the service area.
      '''
      def __init__(self, world, taxi_num, service_area=None, start_point=None):

          self._world = world
          self._taxi_num = taxi_num
          self._service_area = service_area
          self._loc = start_point
          self._onDuty = False
          self._destination = None
          self._passenger = None
          self._account = 256
          self._path = []
          self._traffic_history = defaultdict(lambda: [(0, 0)])
          self._last_replan = 0
          self._replan_interval = 5
          
          # A star finding parameters
          self._traffic_weight = 2.0 # weight for traffic cost for planPath
          self._distance_weight = 1.0 # weight for distance cost for planPath
          self._traffic_decay = 0.95 # decay factor for traffic history
          
          if self._loc is None:
              self._loc = (0,0)
         

      def comeOnDuty(self):
          self._onDuty = True
     
      def goOffDuty(self):
          self._onDuty = False
          self._destination = None
          self._passenger = None
      

      def pickupFare(self, fare):
          #Try to pickup the given fare
          if self._passenger is None and self._loc == fare.origin:
              fare.pickUp(self)
              self._passenger = fare
              return True
          return False
      
      def dropoffFare(self):
          #Try to dropoff the passenger
          if self._passenger is not None and self._passenger.destination == self._loc:
              self._passenger.dropOff()
              self._passenger = None
              return True
          return False

      def bid(self, fare):
          #Drop the current fare
          if not self.onDuty or self._account <= 0:
              return -1
          
          # calculate the estimated costs given traffic
          estimated_time = self.estimate_journey_time(self.loc, fare.origin)
          if estimated_time < 0:
              return -1
          
          #add the est time from pickup to destination
          destination_time = self.estimate_journey_time(fare.origin, fare.destination)
          if destination_time < 0: # invalid path
                return -1
          
          total_time = estimated_time + destination_time
          bid = 2 * total_time # we multiply the total time by 2 to make sure we can make a profit
          
          return bid if bid <= self._account else -1
      


          


   

      # get a map if none was provided at the outset
      def importMap(self, newMap):
          # a fresh map can just be inserted
          if self._map is None:
             self._map = newMap
          # but importing a new map where one exists implies adding to the
          # existing one. (Check that this puts in the right values!)
          else:
             for node in newMap.items():
                 neighbours = [(neighbour[1][0],neighbour[0][0],neighbour[0][1]) for neighbour in node[1].items()]
                 self.addMapNode(node[0],neighbours) 
          
      # incrementally add to the map. This can be useful if, e.g. the world itself has a set of
      # nodes incrementally added. It can then call this function on the existing taxis to update
      # their maps.
      def addMapNode(self, coords, neighbours):
          if self._world is None:
             return AttributeError("This Taxi does not exist in any world")
          node = self._world.getNode(coords[0],coords[1])
          if node is None:
             return KeyError("No such node: {0} in this Taxi's service area".format(coords))
          # build up the neighbour dictionary incrementally so we can check for invalid nodes.
          neighbourDict = {}
          for neighbour in neighbours:
              neighbourCoords = (neighbour[1], neighbour[2])
              neighbourNode = self._world.getNode(neighbour[1],neighbour[2])
              if neighbourNode is None:
                 return KeyError("Node {0} expects neighbour {1} which is not in this Taxi's service area".format(coords, neighbour))
              neighbourDict[neighbourCoords] = (neighbour[0],self._world.distance2Node(node, neighbourNode))
          self._map[coords] = neighbourDict

      #---------------------------------------------------------------------------------------------------------------------------
      # automated methods to handle the taxi's interaction with the world. You should not need to change these.

     

      # clockTick should handle all the non-driving behaviour, turn selection, stopping, etc. Drive automatically
      # stops once it reaches its next location so that if continuing on is desired, clockTick has to select
      # that action explicitly. This can be done using the turn and continueThrough methods of the node. Taxis
      # can collect fares using pickupFare, drop them off using dropoffFare, bid for fares issued by the Dispatcher
      # using transmitFareBid, and any other internal activity seen as potentially useful. 
      def clockTick(self, world):
          # automatically go off duty if we have absorbed as much loss as we can in a day
          if self._account <= 0 and self._passenger is None:
             print("Taxi {0} is going off-duty".format(self.number))
             self.onDuty = False
             self._offDutyTime = self._world.simTime
          # have we reached our last known destination? Decide what to do now.
          if len(self._path) == 0:
             # obviously, if we have a fare aboard, we expect to have reached their destination,
             # so drop them off.
             if self._passenger is not None:
                if self._loc.dropoffFare(self._passenger, self._direction):
                   self._passenger = None
                # failure to drop off means probably we're not at the destination. But check
                # anyway, and replan if this is the case.
                elif self._passenger.destination != self._loc.index:
                   self._path = self._planPath(self._loc.index, self._passenger.destination)
                   
          # decide what to do about available fares. This can be done whenever, but should be done
          # after we have dropped off fares so that they don't complicate decisions.
          faresToRemove = []
          for fare in self._availableFares.items():
              # remember that availableFares is a dict indexed by (time, originx, originy). A location,
              # meanwhile, is an (x, y) tuple. So fare[0][0] is the time the fare called, fare[0][1]
              # is the fare's originx, and fare[0][2] is the fare's originy, which we can use to
              # build the location tuple.
              origin = (fare[0][1], fare[0][2])
              # much more intelligent things could be done here. This simply naively takes the first
              # allocated fare we have and plans a basic path to get us from where we are to where
              # they are waiting. 
              if len(self._path) == 0 and fare[1].allocated and self._passenger is None:
                 # at the collection point for our next passenger?
                 if self._loc.index[0] == origin[0] and self._loc.index[1] == origin[1]:
                    self._passenger = self._loc.pickupFare(self._direction)
                    # if a fare was collected, we can start to drive to their destination. If they
                    # were not collected, that probably means the fare abandoned.
                    if self._passenger is not None:
                       self._path = self._planPath(self._loc.index, self._passenger.destination)
                    faresToRemove.append(fare[0])
                 # not at collection point, so determine how to get there
                 else:
                    self._path = self._planPath(self._loc.index, origin)
              # get rid of any unallocated fares that are too stale to be likely customers
              elif self._world.simTime-fare[0][0] > self._maxFareWait:
                   faresToRemove.append(fare[0])
              # may want to bid on available fares. This could be done at any point here, it
              # doesn't need to be a particularly early or late decision amongst the things to do.
              elif fare[1].bid == 0:
                 if self._bidOnFare(fare[0][0],origin,fare[1].destination,fare[1].price):
                    self._world.transmitFareBid(origin, self)
                    fare[1].bid = 1
                 else:
                    fare[1].bid = -1
          for expired in faresToRemove:
              del self._availableFares[expired]
          # may want to do something active whilst enroute - this simple default version does
          # nothing, but that is probably not particularly 'intelligent' behaviour.
          else:
             pass
       
          # the last thing to do is decrement the account - the fixed 'time penalty'. This is always done at
          # the end so that the last possible time tick isn't wasted e.g. if that was just enough time to
          # drop off a fare.
          self._account -= 1
    
      # called automatically by the taxi's world to update its position. If the taxi has indicated a
      # turn or that it is going straight through (i.e., it's not stopping here), drive will
      # move the taxi on to the next Node once it gets the green light.
      def drive(self):
          # drive towards the current destination
          if not self._onDuty or self._account <= 0:
             return
          self.update_traffic_history()
          
          #check if we need to replan the route
          if self._should_replan():
              self._replan_path()
          
          if not self._path:
              return
          
          # get the next node in the path
          next_loc = self._path[0]
          
          #Move
          self._loc = next_loc
          self._path.pop(0)
          self._account -= 1 # A basic movement cost
          

      # recvMsg handles various dispatcher messages. 
      def recvMsg(self, msg, **args):
          timeOfMsg = self._world.simTime
          # A new fare has requested service: add it to the list of availables
          if msg == self.FARE_ADVICE:
             callTime = self._world.simTime
             self._availableFares[callTime,args['origin'][0],args['origin'][1]] = FareInfo(args['destination'],args['price'])
             return
          # the dispatcher has approved our bid: mark the fare as ours
          elif msg == self.FARE_ALLOC:
             for fare in self._availableFares.items():
                 if fare[0][1] == args['origin'][0] and fare[0][2] == args['origin'][1]:
                    if fare[1].destination[0] == args['destination'][0] and fare[1].destination[1] == args['destination'][1]:
                       fare[1].allocated = True
                       return
          # we just dropped off a fare and received payment, add it to the account
          elif msg == self.FARE_PAY:
             self._account += args['amount']
             return
          # a fare cancelled before being collected, remove it from the list
          elif msg == self.FARE_CANCEL:
             for fare in self._availableFares.items():
                 if fare[0][1] == args['origin'][0] and fare[0][2] == args['origin'][1]: # and fare[1].allocated: 
                    del self._availableFares[fare[0]]
                    return

      ''' HERE IS THE PART THAT YOU NEED TO MODIFY
      '''

      # Manhattan distance heuristic function  for A* search
      def _heuristic(self, node, goal):
          return abs(node[0] - goal[0]) + abs(node[1] - goal[1]) # absolutes of the differences in x and y.

      def _planPath(self,  destination):# Our new planPath function that we use A* search
          # we plan a path to the destination using A* search, with traffic into account
          if destination == self._loc:
              return []
          
          self._destination = destination
          self._path = self._a_star_search(self._loc, destination)
          return self._path
      
      def _a_star_search(self, start, goal):
          # A* search algorithm with traffic aware heuristic
          frontier = [] # list of nodes to explore
          frontier.append((0, start)) # add the start node to the frontier))
          came_from = {start: None}
          cost_so_far = {start: 0}
          
          while frontier: # while the frontier is not empty
              frontier.sort(reverse=True) # sort  by the frontier by the cost
              current_cost = current = frontier.pop() # get the node with the lowest cost
              
              if current == goal: # if we reached the goal
                  break
              
              neighbors = self._getvalid_neighbours(current) # get the valid neighbours of the current node
              
                
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
                    
          # reconstruct the path
          path = []
          current = goal
          while currnent != start:
              path.append(current)
              current = came_from[current]
          path.reverse()
          return path
      
      def _getvalid_neighbours(self, node):
          # Get the valid neighbours of the node
            neighbours = []
            x, y = node
            
             # Check all 8 possible directions
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
                new_x, new_y = x + dx, y + dy
                if (new_x, new_y) in self._service_area:
                  neighbours.append((new_x, new_y))
              
            return neighbours

          
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
          #estimate traffic conditions between current position and goal
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
          # update traffic history for current conditions
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
          # get historical traffic between two nodes
            history = self._traffic_history[(from_node, to_node)]
            if not history:
              return 0
             # Weight recent observations more heavily
            weights = [self._traffic_decay ** i for i in range(len(history))]
            traffic_levels = [t[1] for t in history]
            return np.average(traffic_levels, weights=weights)
      
      def _should_replan(self):
          # Check if we should replan the route
            if not self._path:
                return True
            current_time = self.world.simTime
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
            
           
      def _bidOnFare(self, time, origin, destination, price):
          NoCurrentPassengers = self._passenger is None
          NoAllocatedFares = len([fare for fare in self._availableFares.values() if fare.allocated]) == 0
          TimeToOrigin = self._world.travelTime(self._loc, self._world.getNode(origin[0], origin[1]))
          TimeToDestination = self._world.travelTime(self._world.getNode(origin[0], origin[1]),
                                                     self._world.getNode(destination[1], destination[1]))
          FiniteTimeToOrigin = TimeToOrigin > 0
          FiniteTimeToDestination = TimeToDestination > 0
          CanAffordToDrive = self._account > TimeToOrigin
          FairPriceToDestination = price > TimeToDestination
          PriceBetterThanCost = FairPriceToDestination and FiniteTimeToDestination
          FareExpiryInFuture = self._maxFareWait > self._world.simTime-time
          EnoughTimeToReachFare = self._maxFareWait-self._world.simTime+time > TimeToOrigin
          SufficientDrivingTime = FiniteTimeToOrigin and EnoughTimeToReachFare 
          WillArriveOnTime = FareExpiryInFuture and SufficientDrivingTime
          NotCurrentlyBooked = NoCurrentPassengers and NoAllocatedFares
          CloseEnough = CanAffordToDrive and WillArriveOnTime
          Worthwhile = PriceBetterThanCost and NotCurrentlyBooked 
          Bid = CloseEnough and Worthwhile
          return Bid
      
      
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
          return self.passenger
      
      @property
      def account(self):
          return self._account
      




