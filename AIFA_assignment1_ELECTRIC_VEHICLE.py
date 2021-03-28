from collections import defaultdict
from copy import deepcopy
import numpy as np
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
    source[i]=int(input(f"Source node of car{i+1} => "))-1
    destination[i]=int(input(f"Destination node of car{i+1} => "))-1
    battery_status[i]=float(input(f"Initial battery status of car{i+1} => "))
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

    def shortest_path(self):
        initial = self.source
        shortest_paths = {initial: (None, 0)}
        current_node = self.source
        citiesisited = set()

        while current_node != self.destination:
            citiesisited.add(current_node)
            destinationinations = self.edges[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinationinations:
                weight = self.graph[current_node][next_node] + weight_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, weight)
                else:
                    current_shortest_weight = shortest_paths[next_node][1]
                    if current_shortest_weight > weight:
                        shortest_paths[next_node] = (current_node, weight)

            next_destinationinations = {node: shortest_paths[node] for node in shortest_paths if node not in citiesisited}
            current_node = min(next_destinationinations, key=lambda k: next_destinationinations[k][1])
        while current_node is not None:
            self.path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        self.path = self.path[::-1]
        self.distance = defaultdict(int)
        for i in range(len(self.path)-2,-1,-1):
            self.distance[self.path[i]] = self.graph[self.path[i]][self.path[i+1]] + self.distance[self.path[i+1]]
        self.min_time = defaultdict(int)
        temp_battery_status = self.battery_status
        self.charge_time_required = defaultdict(int)
        for i in range(len(self.path)):
            if(i!=0):
                temp_battery_status -= self.graph[self.path[i-1]][self.path[i]]/self.discharging_rate
            self.min_time[self.path[i]] = self.distance[self.path[i]]/self.average_speed
            min_charge_required = self.distance[self.path[i]]/self.discharging_rate
            self.charge_time_required[self.path[i]] = 0
            if(temp_battery_status<=min(self.maximum_battery_capacity, min_charge_required)):
                self.charge_time_required[self.path[i]] = (min(self.maximum_battery_capacity, min_charge_required)-temp_battery_status)/self.charging_rate
                self.min_time[self.path[i]] += self.charge_time_required[self.path[i]]

            timings[self.path[i]][self.id] = 0
            if(i!=0):
                timings[self.path[i]][self.id] = timings[self.path[i-1]][self.id]+self.min_time[self.path[i-1]]-self.min_time[self.path[i]]

    def update_parameters(self, node, waiting_time):
        self.min_time[node] += waiting_time
        update = False
        for i in range(len(self.path)):
            if(update):
                self.min_time[self.path[i]] += waiting_time
                timings[self.path[i]][self.id] += waiting_time
            if(self.path[i]==node):
                update=True
all_cars = {}
for i in range(K):
    obj = car(i,source[i],destination[i],battery_status[i],charging_rate[i],discharging_rate[i],maximum_battery_capacity[i],average_speed[i],city_map)
    obj.shortest_path()
    all_cars[i] = obj

all_cars = {}
for i in range(K):
    obj = car(i,source[i],destination[i],battery_status[i],charging_rate[i],discharging_rate[i],maximum_battery_capacity[i],average_speed[i],city_map)
    obj.shortest_path()
    all_cars[i] = obj


def schedule(cars_list, precities_charging_car, charge_time_left_list, conflict_node):
    car_obj_list = [w[0] for w in cars_list]
    if len(car_obj_list)==1:
        return car_obj_list[0].id
    temp = sorted([sorted([(i, w.min_time[conflict_node] + charge_time_left_list[x.id]) for j,w in enumerate(car_obj_list) if j!=i], key=lambda y:-y[1]) for i,x in enumerate(car_obj_list)], key=lambda y:y[0][1])
    return temp[0][0][0]

global_time_list = []
for conflict_node in range(cities):
    cars_present = [all_cars[i] for i in range(K) if timings[conflict_node][i]!=float("inf")]
    charge_time_left_list = [0]*K
    for i in cars_present:
        arr_time = timings[conflict_node][i.id]
        dep_time = arr_time + i.charge_time_required[conflict_node]
        charge_time_left_list[i.id] = int(i.charge_time_required[conflict_node])
        global_time_list.append([i,"arricitiesal", conflict_node, arr_time])
        global_time_list.append([i,"departure", conflict_node, dep_time])
global_time_list.sort(key=lambda x: x[3])


cars_list_list = [[]]*cities  
precities_ecitiesent_time_list = [0]*cities  
precities_charging_cars_list = [-1]*cities  
cntr=0
while(len(global_time_list)!=0):
    cntr+=1
    ecitiesent = global_time_list.pop(0)
    conflict_node = ecitiesent[2]
    if(ecitiesent[1]=="departure"):
        poped=False
        index_ecitiesent_id = -1
        for i in range(len(cars_list_list[conflict_node])):
            if(cars_list_list[conflict_node][i][0].id==ecitiesent[0].id):
                index_ecitiesent_id = i
        waiting_time = ecitiesent[3]- precities_ecitiesent_time_list[conflict_node]
        precities_ecitiesent_time_list[conflict_node] = int(ecitiesent[3])
        charge_time_left_list[precities_charging_cars_list[conflict_node]] -= waiting_time
        if(precities_charging_cars_list[conflict_node]==ecitiesent[0].id):
            cars_list_list[conflict_node].pop(index_ecitiesent_id)
            poped=True
        
        if(len(cars_list_list[conflict_node])>0):
            for i in range(len(cars_list_list[conflict_node])):
                if(cars_list_list[conflict_node][i][0].id!=precities_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = timings[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arricitiesal", old_list[2], arr_time] if old_list[1]=="arricitiesal" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if poped and len(cars_list_list[conflict_node])>0:
            precities_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], precities_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)

    else:
        index_ecitiesent_id = -1
        for i in range(len(cars_list_list[conflict_node])):
            if(cars_list_list[conflict_node][i][0].id==ecitiesent[0].id):
                index_ecitiesent_id = i
        waiting_time = ecitiesent[3] - precities_ecitiesent_time_list[conflict_node]
        precities_ecitiesent_time_list[conflict_node] = ecitiesent[3]
        cars_list_list[conflict_node].append(ecitiesent)
        charge_time_left_list[precities_charging_cars_list[conflict_node]] -= waiting_time
        if(len(cars_list_list[conflict_node])>0):   # changing departure time
            for i in range(len(cars_list_list[conflict_node])):
                if(cars_list_list[conflict_node][i][0].id!=precities_charging_cars_list[conflict_node]):
                    index_list_in_global_time_list=[]
                    for idx,listn in enumerate(global_time_list):
                        if listn[0].id==cars_list_list[conflict_node][i][0].id and listn[2]==conflict_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[conflict_node][i][0].update_parameters(conflict_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = timings[conflict_node][cars_list_list[conflict_node][i][0].id]
                            dep_time = arr_time + cars_list_list[conflict_node][i][0].charge_time_required[conflict_node]
                            new_list = [cars_list_list[conflict_node][i][0],"arricitiesal", old_list[2], arr_time] if old_list[1]=="arricitiesal" \
                                        else [cars_list_list[conflict_node][i][0],"departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if len(cars_list_list[conflict_node])>0:
            precities_charging_cars_list[conflict_node] = schedule(cars_list_list[conflict_node], precities_charging_cars_list[conflict_node], charge_time_left_list, conflict_node)
for k in range(K):
    print('route by car'+str(k+1)+'=> ')
    print(all_cars[k].path)
    
    
    
    
    
    
    
    
    