#lang dssl2

# Final project: Trip Planner

let eight_principles = ["Know your rights.",
    "Acknowledge your sources.",
    "Protect your work.",
    "Avoid suspicion.",
    "Do your own work.",
    "Never falsify a record or permit another person to do so.",
    "Never fabricate data, citations, or experimental results.",
    "Always tell the truth when discussing your work with your instructor."]

import cons
import sbox_hash
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

### Structs ###
struct place:
    let lat: nat?
    let long: nat?
    let cat: str?
    let name: str?
    
struct road:
    let p1: place?
    let p2: place?
    let distance: nat?
    
struct id_to_dist:
    let id: nat?
    let dist: num?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let pos
    let place
    let pos_to_id
    let id_to_pos
    let name_to_pos
    let graph
    
    def __init__(self, v1, v2):
        self.pos = v1
        self.place = v2
        self.pos_to_id = HashTable(5, make_sbox_hash())
        self.id_to_pos = HashTable(5, make_sbox_hash())
        let count = 0
        for i in self.pos:
            if self.pos_to_id.mem?([i[0], i[1]])==False:
                self.pos_to_id.put([i[0], i[1]], count)
                self.id_to_pos.put(count, [i[0], i[1]])
                count = count + 1
            if self.pos_to_id.mem?([i[2], i[3]])==False:
                self.pos_to_id.put([i[2], i[3]], count)
                self.id_to_pos.put(count, [i[2], i[3]])
                count = count + 1
            else:
                None
        self.name_to_pos = HashTable(5, make_sbox_hash())
        for i in self.place:
            self.name_to_pos.put(i[3], [i[0], i[1]])
        self.graph = WUGraph(self.pos_to_id.len())
        for i in self.pos:
            self.graph.set_edge(self.pos_to_id.get([i[0], i[1]]), self.pos_to_id.get([i[2], i[3]]), self.distance([i[0], i[1]], [i[2], i[3]]))
            
    def mem?(self, element, vector):
        let n = vector.len()
        while n != 0:
            for i in vector:
                if element == i:
                    return True
                else:
                    n = n - 1
        
    def locate_all(self, dst_cat: Cat?) -> ListC[RawPos?]:
        let list = None
        for i in self.place:
            if i[2] == dst_cat:
                if self.mem?([i[0], i[1]], Cons.to_vec(list)) == True:
                    None
                else:
                    list = cons([i[0], i[1]], list)
        return list

    def distance(self, pos1: RawPos?, pos2: RawPos?) -> num?:
        let x = (pos2[0]-pos1[0])**2 + (pos2[1]-pos1[1])**2
        return x.sqrt()
        
    def dijk_alg(self, lat, lon, dist, pred):
        let start_pos = self.pos_to_id.get([lat, lon])
        # To-do Priority Queue
        let h = BinHeap[id_to_dist?](self.graph.len()**2, λ x, y: x.dist < y.dist)
        dist[start_pos] = 0
        h.insert(id_to_dist(start_pos, dist[start_pos]))
        # Done array
        let done = [False; self.graph.len()]
        # Code for Dijkstra's Algorithm
        while h.len() != 0:
            let near = h.find_min().id
            h.remove_min()
            if done[near] == False:
                done[near] = True
                for i in Cons.to_vec(self.graph.get_adjacent(near)):
                    if dist[near] + self.graph.get_edge(near, i) < dist[i]:
                        dist[i] = dist[near] + self.graph.get_edge(near, i)
                        pred[i] = near
                        h.insert(id_to_dist(i, dist[i]))
        
            
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?) -> ListC[RawPos?]:
        #check if dst_name exists
        if self.name_to_pos.mem?(dst_name) == False:
            return None
        # Variable to get the position of dst_name
        let target_pos = self.name_to_pos.get(dst_name)
        #0 step route
        if target_pos == [src_lat, src_lon]:
            return cons([src_lat, src_lon], None)
        # Variable to get the position of the starting node
        let start_pos = self.pos_to_id.get([src_lat, src_lon])
        # Vector mapping distance for Dijkstra's Algorithm(Relaxation)
        let dist = [float(inf); self.graph.len()]
        # Vector mapping predecessor for Dijkstra's Algorithm
        let pred = [None; self.graph.len()]
        # Calling Dijkstra's Algorithm
        self.dijk_alg(src_lat, src_lon, dist, pred)
        # returning list of Raw Positions
        let final = cons(target_pos, None)
        let temp = self.pos_to_id.get(target_pos)
        let last_temp = None
        while pred[temp] != None:
            final = cons(self.id_to_pos.get(pred[temp]), final)
            last_temp = self.id_to_pos.get(pred[temp])
            temp = pred[temp]
        if last_temp != [src_lat, src_lon]:
            return None
        else:
            return final
                
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat:  Cat?, n: nat?) ->  ListC[RawPOI?]:
        # Vector of linked list for POIS stored in each position
        let id_to_pois = [None; self.pos_to_id.len()]
        for i in self.place:
            id_to_pois[self.pos_to_id.get([i[0], i[1]])] = cons(i, id_to_pois[self.pos_to_id.get([i[0], i[1]])])
        # Vector mapping distance for Dijkstra's Algorithm(Relaxation)
        let dist = [float(inf); self.graph.len()]
        # Vector mapping predecessor for Dijkstra's Algorithm
        let pred = [None; self.graph.len()]
        # Calling Dijkstra's Algorithm
        self.dijk_alg(src_lat, src_lon, dist, pred)
        # Filtering the POIs with the right category
        struct dist_to_POI:
            let dist: num?
            let POI: RawPOI?
        let right_cat = None
        for i in range(0, dist.len()):
            for j in Cons.to_vec(id_to_pois[i]):
                if j[2] == dst_cat:
                    right_cat = cons(dist_to_POI(dist[i], j), right_cat) 
        let right_cat_vector = Cons.to_vec(right_cat)
        # Code for heapsorting the distances
        heap_sort(right_cat_vector, λ x, y: x.dist < y.dist)
        # Returning list of POIs
        if right_cat_vector.len() ==0:
            return None
        else:
            let semi_final = None
            if n > right_cat_vector.len():
                for i in range(0, right_cat_vector.len()):
                    semi_final = cons(right_cat_vector[i].POI, semi_final)
            else:
                for i in range(0, n):
                    semi_final = cons(right_cat_vector[i].POI, semi_final)
            let final = None
            for i in Cons.to_vec(semi_final):
                if self.plan_route(src_lat, src_lon, i[3]) == None:
                    final = final
                else:
                    final = cons(i, final)
            return final
        
def example_test():
    return TripPlanner([[0,0, 1,0], [1,0,1,1], [0,0,0,1], [0,1,1,1],[1,1,1,2], [0,1,1,2]], 
                          [[0,0, "school", "NU"],
                           [0,1, "food", "Shangs"],
                           [1,0, "shop", "Wholefoods"],
                           [1,1, "food", "Moge Tea"],
                           [1,2, "shop", "Hmart"]])
    
test 'plan_route test1':
    assert example_test().plan_route(0, 0, "Hmart") == \
       cons([0,0], cons([0,1], cons([1,2],None)))
        
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)

test 'Failed Advanced Route Test':
    let tp = TripPlanner([[0, 214.0, 33.5, 211.0],[33.5, 211.0, 66.5, 203.0],[66.5, 203.0, 98.0, 190.0],[98.0, 190.0, 127.0, 172.0],[127.0, 172.0, 152.5, 149.5]],[[0, 214.0, '"coffee"', 'Starbucks #1'],
       [33.5, 211.0, '"coffee"', 'Starbucks #2'],[66.5, 203.0, '"coffee"', 'Starbucks #3'],[98.0, 190.0, '"coffee"', 'Starbucks #4'],[127.0, 172.0, '"coffee"', 'Starbucks #5']])
    assert Cons.to_vec(tp.find_nearby(0, 214.0, '"coffee"', 1)) == [[0, 214.0, '"coffee"', 'Starbucks #1']]
