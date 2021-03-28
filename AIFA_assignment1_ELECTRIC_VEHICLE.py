import numpy as np
from copy import deepcopy
from collections import defaultdict

cities = int(input("Enter number of cities >> "))

city_map = [[0 for i in range(cities)] for j in range(cities)]

print("Enter distance between cities Eij = distance between ith and jth city")
print('If cities are not connected then Eij = inf or INF')

for i in range(cities):
    for j in range(i+1,cities):
        inp = input("E"+str(i+1)+str(j+1)+" => ")
        if inp=="INF" or inp=="inf":
            city_map[i][j]=float("inf")
        else:
            city_map[i][j] = float(inp)

for i in range(cities):
    for j in range(cities):
        if i>j:
            city_map[i][j]=city_map[j][i]

print(np.array(city_map))

city_map=np.array(city_map)
source={}
destination={}
battery_status={}
charging_rate={}
discharging_rate={}
maximum_battery_capacity={}
average_speed={}
K = int(input("Enter total number of Car => "))
print('\n')
for i in range(K):
    print('CAR'+str(i+1))
    temp1=int(input(f"Source node of car{i+1} => "))-1
    if temp1<0 or temp1>cities:
        temp1=int(input('city does not exists , give in range '+'1 to '+str(cities)+'=> '))-1
    source[i]=temp1
    temp2=int(input(f"Destination node of car{i+1} => "))-1
    if temp2<0 or temp2>cities:
        temp2=int(input('city does not exists , give in range '+'1 to '+str(cities)+'=> '))-1
    destination[i]=temp2
    battery_status[i]=float(input(f"initial battery status of car{i+1} => "))
    charging_rate[i]=float(input(f"Charging rate of car{i+1} => "))
    discharging_rate[i]=float(input(f"Discharging rate of car{i+1} => "))
    maximum_battery_capacity[i]=float(input(f"Max battery capacity of car{i+1} => "))
    average_speed[i]=float(input(f"average speed of car{i+1} => "))
    print('\n')

timings = [[float("inf") for i in range(K)] for i in range(cities)]
class car:
    def __init__(self,ids, source, destination, battery_status, charging_rate, discharging_rate, maximum_battery_capacity, average_speed,city_map):
        self.id = ids
        self.source = source
        self.destination = destination
        self.battery_status = battery_status
        self.charging_rate = charging_rate
        self.discharging_rate=discharging_rate
        self.maximum_battery_capacity=maximum_battery_capacity
        self.average_speed=average_speed
        self.path=[]
        self.graph = deepcopy(city_map)
        for i in range(city_map.shape[0]):
            for j in range(city_map.shape[0]):
                if city_map[i][j]!=float("inf"):
                    if (city_map[i][j]/self.discharging_rate)>self.maximum_battery_capacity:
                        self.graph[i][j]=float("inf")

        self.edges=[]
        for i in range(city_map.shape[0]):
            temp=[]
            for j in range(city_map.shape[0]):
                if self.graph[i][j]!=float('inf'):
                    temp.append(j)
            self.edges.append(temp)

    def short_way(self):
        ini = self.source
        short_ways = {ini: (None, 0)}
        current_node = self.source
        cities_is_visited = set()

        while current_node != self.destination:
            cities_is_visited.add(current_node)
            f_destinatation = self.edges[current_node]
            weights_node = short_ways[current_node][1]

            for next_node in f_destinatation:
                weight = self.graph[current_node][next_node] + weights_node
                if next_node not in short_ways:
                    short_ways[next_node] = (current_node, weight)
                else:
                    temp_weight = short_ways[next_node][1]
                    if temp_weight > weight:
                        short_ways[next_node] = (current_node, weight)

            nf_destin = {node: short_ways[node] for node in short_ways if node not in cities_is_visited}
            current_node = min(nf_destin, key=lambda k: nf_destin[k][1])
        while current_node is not None:
            self.path.append(current_node)
            next_node = short_ways[current_node][0]
            current_node = next_node
        self.path = self.path[::-1]
        self.distance = defaultdict(int)
        for i in range(len(self.path)-2,-1,-1):
            self.distance[self.path[i]] = self.graph[self.path[i]][self.path[i+1]] + self.distance[self.path[i+1]]
        self.min_time = defaultdict(int)
        batteries_states = self.battery_status
        self.charge_time_required = defaultdict(int)
        for i in range(len(self.path)):
            if(i!=0):
                batteries_states -= self.graph[self.path[i-1]][self.path[i]]/self.discharging_rate
            self.min_time[self.path[i]] = self.distance[self.path[i]]/self.average_speed
            min_charge_required = self.distance[self.path[i]]/self.discharging_rate
            self.charge_time_required[self.path[i]] = 0
            if(batteries_states<=min(self.maximum_battery_capacity, min_charge_required)):
                self.charge_time_required[self.path[i]] = (min(self.maximum_battery_capacity, min_charge_required)-batteries_states)/self.charging_rate
                self.min_time[self.path[i]] += self.charge_time_required[self.path[i]]

            timings[self.path[i]][self.id] = 0
            if(i!=0):
                timings[self.path[i]][self.id] = timings[self.path[i-1]][self.id]+self.min_time[self.path[i-1]]-self.min_time[self.path[i]]

    def new_params(self, node, wait_time):
        self.min_time[node] += wait_time
        update = False
        for i in range(len(self.path)):
            if(update):
                self.min_time[self.path[i]] += wait_time
                timings[self.path[i]][self.id] += wait_time
            if(self.path[i]==node):
                update=True
CARS = {}
for i in range(K):
    obj = car(i,source[i],destination[i],battery_status[i],charging_rate[i],discharging_rate[i],maximum_battery_capacity[i],average_speed[i],city_map)
    obj.short_way()
    CARS[i] = obj

CARS = {}
for i in range(K):
    obj = car(i,source[i],destination[i],battery_status[i],charging_rate[i],discharging_rate[i],maximum_battery_capacity[i],average_speed[i],city_map)
    obj.short_way()
    CARS[i] = obj


def schedule(cars_list, precities_charging_car, left_charge_timings_, c_node):
    car_obj_list = [w[0] for w in cars_list]
    if len(car_obj_list)==1:
        return car_obj_list[0].id
    temp = sorted([sorted([(i, w.min_time[c_node] + left_charge_timings_[x.id]) for j,w in enumerate(car_obj_list) if j!=i], key=lambda y:-y[1]) for i,x in enumerate(car_obj_list)], key=lambda y:y[0][1])
    return temp[0][0][0]

time_lists = []
for c_node in range(cities):
    cars_present = [CARS[i] for i in range(K) if timings[c_node][i]!=float("inf")]
    left_charge_timings_ = [0]*K
    for i in cars_present:
        arr_time = timings[c_node][i.id]
        dep_time = arr_time + i.charge_time_required[c_node]
        left_charge_timings_[i.id] = int(i.charge_time_required[c_node])
        time_lists.append([i,"arrival_cities", c_node, arr_time])
        time_lists.append([i,"depart", c_node, dep_time])
time_lists.sort(key=lambda x: x[3])


vehicles = [[]]*cities  
precities_citi_time_list = [0]*cities  
visit_charge_ = [-1]*cities  
cntr=0
while(len(time_lists)!=0):
    cntr+=1
    citi = time_lists.pop(0)
    c_node = citi[2]
    if(citi[1]=="depart"):
        poped=False
        index_citi_id = -1
        for i in range(len(vehicles[c_node])):
            if(vehicles[c_node][i][0].id==citi[0].id):
                index_citi_id = i
        wait_time = citi[3]- precities_citi_time_list[c_node]
        precities_citi_time_list[c_node] = int(citi[3])
        left_charge_timings_[visit_charge_[c_node]] -= wait_time
        if(visit_charge_[c_node]==citi[0].id):
            vehicles[c_node].pop(index_citi_id)
            poped=True
        
        if(len(vehicles[c_node])>0):
            for i in range(len(vehicles[c_node])):
                if(vehicles[c_node][i][0].id!=visit_charge_[c_node]):
                    index_list_in_time_lists=[]
                    for idx,listn in enumerate(time_lists):
                        if listn[0].id==vehicles[c_node][i][0].id and listn[2]==c_node:
                            index_list_in_time_lists.append(idx)

                    vehicles[c_node][i][0].new_params(c_node, wait_time)
                    if index_list_in_time_lists:
                        for index_in_time_lists in index_list_in_time_lists:
                            old_list = time_lists[index_in_time_lists]
                            arr_time = timings[c_node][vehicles[c_node][i][0].id]
                            dep_time = arr_time + vehicles[c_node][i][0].charge_time_required[c_node]
                            new_list = [vehicles[c_node][i][0],"arrival_cities", old_list[2], arr_time] if old_list[1]=="arrival_cities" \
                                        else [vehicles[c_node][i][0],"depart", old_list[2], dep_time]
                            time_lists[index_in_time_lists] = new_list
                    time_lists.sort(key=lambda x: x[3])

        if poped and len(vehicles[c_node])>0:
            visit_charge_[c_node] = schedule(vehicles[c_node], visit_charge_[c_node], left_charge_timings_, c_node)

    else:
        index_citi_id = -1
        for i in range(len(vehicles[c_node])):
            if(vehicles[c_node][i][0].id==citi[0].id):
                index_citi_id = i
        wait_time = citi[3] - precities_citi_time_list[c_node]
        precities_citi_time_list[c_node] = citi[3]
        vehicles[c_node].append(citi)
        left_charge_timings_[visit_charge_[c_node]] -= wait_time
        if(len(vehicles[c_node])>0):   
            for i in range(len(vehicles[c_node])):
                if(vehicles[c_node][i][0].id!=visit_charge_[c_node]):
                    index_list_in_time_lists=[]
                    for idx,listn in enumerate(time_lists):
                        if listn[0].id==vehicles[c_node][i][0].id and listn[2]==c_node:
                            index_list_in_time_lists.append(idx)

                    vehicles[c_node][i][0].new_params(c_node, wait_time)
                    if index_list_in_time_lists:
                        for index_in_time_lists in index_list_in_time_lists:
                            old_list = time_lists[index_in_time_lists]
                            arr_time = timings[c_node][vehicles[c_node][i][0].id]
                            dep_time = arr_time + vehicles[c_node][i][0].charge_time_required[c_node]
                            new_list = [vehicles[c_node][i][0],"arrival_cities", old_list[2], arr_time] if old_list[1]=="arrival_cities" \
                                        else [vehicles[c_node][i][0],"depart", old_list[2], dep_time]
                            time_lists[index_in_time_lists] = new_list
                    time_lists.sort(key=lambda x: x[3])

        if len(vehicles[c_node])>0:
            visit_charge_[c_node] = schedule(vehicles[c_node], visit_charge_[c_node], left_charge_timings_, c_node)

for k in range(K):
    for i in range(len(CARS[k].path)):
        CARS[k].path[i]=CARS[k].path[i]+1
    
for k in range(K):
    print('route by car'+str(k+1)+'=> ')
    print(CARS[k].path)
    
    
    
