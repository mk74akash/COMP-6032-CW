import math
from tkinter import SE
import numpy
import heapq

# a data container for all pertinent information related to fares. (Should we
# add an underway flag and require taxis to acknowledge collection to the dispatcher?)
class FareEntry:

      def __init__(self, origin, dest, time, price=0, taxiIndex=-1):

          self.origin = origin
          self.destination = dest
          self.calltime = time
          self.price = price
          # the taxi allocated to service this fare. -1 if none has been allocated
          self.taxi = taxiIndex
          # a list of indices of taxis that have bid on the fare.
          self.bidders = []

'''
A Dispatcher is a static agent whose job is to allocate fares amongst available taxis. Like the taxis, all
the relevant functionality happens in ClockTick. The Dispatcher has a list of taxis, a map of the service area,
and a dictionary of active fares (ones which have called for a ride) that it can use to manage the allocations.
Taxis bid after receiving the price, which should be decided by the Dispatcher, and once a 'satisfactory' number
of bids are in, the dispatcher should run allocateFare in its world (parent) to inform the winning bidder that they
now have the fare.
'''
class Dispatcher:

      # constructor only needs to know the world it lives in, although you can also populate its knowledge base
      # with taxi and map information.
      def __init__(self, parent, taxis=[]):

          self._parent = parent
          # the list of taxis
          self._taxis = taxis # taxis registered with the dispatcher
          #fares waiting to be allocated
          self._fares = []
          #submitted bids - map keyed by fare, value is a list of taxis that have bid
          self._bids = {}
          # A dictionary of taxi scores based on completed fares
          self._taxi_scores = {taxi: 1.0 for taxi in taxis}
          #dict to track fare allocations per taxi
          self._allocations = {taxi: 0 for taxi in taxis}
          
          

      #_________________________________________________________________________________________________________
      # methods to add objects to the Dispatcher's knowledge base
      
      # make a new taxi known.
      def addTaxi(self, taxi):
          if taxi not in self._taxis:
             self._taxis.append(taxi)
             self._taxis.append(taxi)
             self._allocations[taxi] = 0


      def addFare(self , fare):
          self._fares.append(fare)
          self._bids[fare] = []
          
      def removeFare(self, fare):
          self._fares.remove(fare)
          del self._bids_[fare] # remove the associated bids
          
      def addBid(self, taxi, fare, bid):
          if fare in self.fares:
              self._bids[fare].append((taxi, bid))
              

      # incrementally add to the map. This can be useful if, e.g. the world itself has a set of
      # nodes incrementally added. It can then call this function on the dispatcher to add to
      # its map
      def addMapNode(self, coords, neighbours):
          if self._parent is None:
             return AttributeError("This Dispatcher does not exist in any world")
          node = self._parent.getNode(coords[0],coords[1])
          if node is None:
             return KeyError("No such node: {0} in this Dispatcher's service area".format(coords))
          # build up the neighbour dictionary incrementally so we can check for invalid nodes.
          neighbourDict = {}
          for neighbour in neighbours:
              neighbourCoords = (neighbour[1], neighbour[2])
              neighbourNode = self._parent.getNode(neighbour[1],neighbour[2])
              if neighbourNode is None:
                 return KeyError("Node {0} expects neighbour {1} which is not in this Dispatcher's service area".format(coords, neighbour))
              neighbourDict[neighbourCoords] = (neighbour[0],self._parent.distance2Node(node, neighbourNode))
          self._map[coords] = neighbourDict

      # importMap gets the service area map, and can be brought in incrementally as well as
      # in one wodge.
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

      # any legacy fares or taxis from a previous dispatcher can be imported here - future functionality,
      # for the most part
      def handover(self, parent, origin, destination, time, taxi, price):
          if self._parent == parent:
             # handover implies taxis definitely known to a previous dispatcher. The current
             # dispatcher should thus be made aware of them
             if taxi not in self._taxis:
                self._taxis.append(taxi)
             # add any fares found along with their allocations
             self.newFare(parent, origin, destination, time)
             self._fareBoard[origin][destination][time].taxi = self._taxis.index(taxi)
             self._fareBoard[origin][destination][time].price = price

      #--------------------------------------------------------------------------------------------------------------
      # runtime methods used to inform the Dispatcher of real-time events


      # fares will call this when they appear to signal a request for service.
      def newFare(self, parent, origin, destination, time):
          # only add new fares coming from the same world
          if parent == self._parent:
             fare = FareEntry(origin,destination,time)
             if origin in self._fareBoard:               
                if destination not in self._fareBoard[origin]:
                   self._fareBoard[origin][destination] = {}
             else:
                self._fareBoard[origin] = {destination: {}}
             # overwrites any existing fare with the same (origin, destination, calltime) triplet, but
             # this would be equivalent to saying it was the same fare, at least in this world where
             # a given Node only has one fare at a time.
             self._fareBoard[origin][destination][time] = fare
             
      # abandoning fares will call this to cancel their request
      def cancelFare(self, parent, origin, destination, calltime):
          # if the fare exists in our world,
          if parent == self._parent and origin in self._fareBoard:
             if destination in self._fareBoard[origin]:
                if calltime in self._fareBoard[origin][destination]:
                   # get rid of it
                   print("Fare ({0},{1}) cancelled".format(origin[0],origin[1]))
                   # inform taxis that the fare abandoned
                   self._parent.cancelFare(origin, self._taxis[self._fareBoard[origin][destination][calltime].taxi])
                   del self._fareBoard[origin][destination][calltime]
                if len(self._fareBoard[origin][destination]) == 0:
                   del self._fareBoard[origin][destination]
                if len(self._fareBoard[origin]) == 0:
                   del self._fareBoard[origin]

      # taxis register their bids for a fare using this mechanism
      def fareBid(self, origin, taxi):
          # rogue taxis (not known to the dispatcher) can't bid on fares
          if taxi in self._taxis:
             # everyone else bids on fares available
             if origin in self._fareBoard:
                for destination in self._fareBoard[origin].keys():
                    for time in self._fareBoard[origin][destination].keys():
                        # as long as they haven't already been allocated
                        if self._fareBoard[origin][destination][time].taxi == -1:
                           self._fareBoard[origin][destination][time].bidders.append(self._taxis.index(taxi))
                           # only one fare per origin can be actively open for bid, so
                           # immediately return once we[ve found it
                           return
                     
      # fares call this (through the parent world) when they have reached their destination
      def recvPayment(self, parent, amount):
          # don't take payments from dodgy alternative universes
          if self._parent == parent:
             self._revenue += amount

      #________________________________________________________________________________________________________________

      # clockTick is called by the world and drives the simulation for the Dispatcher. It must, at minimum, handle the
      # 2 main functions the dispatcher needs to run in the world: broadcastFare(origin, destination, price) and
      # allocateFare(origin, taxi).
      def clockTick(self, parent):
          if self._parent == parent:
             for origin in self._fareBoard.keys():
                 for destination in self._fareBoard[origin].keys():
                     # TODO - if you can come up with something better. Not essential though.
                     # not super-efficient here: need times in order, dictionary view objects are not
                     # sortable because they are an iterator, so we need to turn the times into a
                     # sorted list. Hopefully fareBoard will never be too big
                     for time in sorted(list(self._fareBoard[origin][destination].keys())):
                         if self._fareBoard[origin][destination][time].price == 0:
                            self._fareBoard[origin][destination][time].price = self._costFare(self._fareBoard[origin][destination][time])
                            # broadcastFare actually returns the number of taxis that got the info, if you
                            # wish to use that information in the decision over when to allocate
                            self._parent.broadcastFare(origin,
                                                       destination,
                                                       self._fareBoard[origin][destination][time].price)
                         elif self._fareBoard[origin][destination][time].taxi < 0 and len(self._fareBoard[origin][destination][time].bidders) > 0:
                              self._allocateFare(origin, destination, time)

      #----------------------------------------------------------------------------------------------------------------

     # function to update the scores of the taxis based on the fares they have completed and the time taken
      def _updateTaxiScore(self, taxi, completionTime, fare_value):
          # a higher score gets given for faster completion times and higher fare values
          score_delta = fare_value / (completionTime + 1)
          alpha = 0.3
          alpha = 0.3  # Weight for new score
          self._taxi_scores[taxi] = (1-alpha) * self._taxi_scores[taxi] + alpha * score_delta
          
      
      # function to calculate the expected fare value based on journey length
      def _calculateFareValue(self, origin, destination):
          dx = abs(destination[0] - origin[0]) # calculating abs difference in x coordinates from dest to origin
          dy = abs(destination[1] - origin[1])# abs difference in y coordinates from dest to origin
          distance = math.sqrt(dx*dx + dy*dy)# using pythagoras theorem to calculate distance from two points
          # Base fare plus distance component
          return 10.0 + (2.0 * distance)
      
        # Calculate a weighted score for a bid considering multiple factors
      def _calculateBidScore(self, taxi, bid, fare):
        if bid <= 0:
            return float('-inf')
        
        # we get the journey endpoints
        pickup = fare.origin
        dropoff = fare.destination

        # Calculate various scoring components
        distance_score = 1.0 / (bid + 1)  # Lower bids (shorter distances) are better
        
        # Calculate expected revenue
        fare_value = self._calculateFareValue(pickup, dropoff)
        revenue_score = fare_value / bid  # Higher revenue per unit distance is better
        
        # Consider taxi's current score from historical performance
        history_score = self._taxi_scores[taxi]
        
        # Factor in allocation fairness
        fairness_score = 1.0 / (self._allocations[taxi] + 1)
        
        # Weights for different components
        w_distance = 0.3
        w_revenue = 0.3  
        w_history = 0.2
        w_fairness = 0.2

        # Calculate final weighted score
        final_score = (w_distance * distance_score +
                      w_revenue * revenue_score +
                      w_history * history_score + 
                      w_fairness * fairness_score)
                      
        return final_score
    
    
      
      def _allocateFare(self):
          # My logic for optimizing the fare allocation service. Called by the world regularly
          # if there are no fares:
          if (len(self._fares) == 0):
             return
          
         # process each fare
          for fare in self._fares.copy():  # Use copy since we'll modify during iteration
            
            # Get all bids for this fare
            bids = self._bids.get(fare, [])
            
            # Need at least 1 bid to allocate
            if not bids:
                continue
                
            # Score each bid
            scored_bids = []
            for taxi, bid in bids:
                score = self._calculateBidScore(taxi, bid, fare)
                scored_bids.append((taxi, bid, score))
                
            # Sort by score, highest first    
            scored_bids.sort(key=lambda x: x[2], reverse=True)
            
            # Select the best bid
            best_taxi, best_bid, best_score = scored_bids[0]
            
            # Only allocate if score meets minimum threshold
            min_score_threshold = 0.1
            if best_score > min_score_threshold:
                # Update allocation tracking
                self._allocations[best_taxi] += 1
                
                # Assign fare to taxi
                if best_taxi.assignFare(fare, best_bid):
                    # Successfully allocated
                    fare.assignTaxi(best_taxi)
                    
                    # Calculate completion metrics for scoring
                    expected_time = best_bid  # Use bid as proxy for completion time
                    fare_value = self._calculateFareValue(fare.origin, fare.destination)
                    
                    # Update taxi's performance score
                    self._updateTaxiScore(best_taxi, expected_time, fare_value)
                    
                    # Remove fare from system
                    self.removeFare(fare)
                    

      # getting the next fare from the shortest path
      def nextFare(self, location):
          # if there are no fees, there is no next fare
          if not self._fares:
              return None
          
          #find the closest fare by stragiht-line distance to the location
          closestFare = self._fares[0]
          dx = location[0] - closestFare.origin[0]
          dy = location[1] - closestFare.origin[1]
          closestDistance = math.sqrt(dx*dx + dy*dy)
          for fare in self._fares[1:]:
            dx = location[0] - fare.origin[0]
            dy = location[1] - fare.origin[1]
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < closestDistance:
                closestDistance = distance
                closestFare = fare
          return closestFare
        
      # get path to given fare
      def fareToTaxi(self, fare, taxi):
        return self._parent.routeBetweenPoints(taxi.currentLocation, fare.origin)

    # get path from origin to destination of given fare
      def farePath(self, fare):
        return self._parent.routeBetweenPoints(fare.origin, fare.destination)              

           
          