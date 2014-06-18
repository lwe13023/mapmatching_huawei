__author__ = 'waynewen'

from graph import Graph
import trajectory
from traj_point import Traj_Point
import math
import itertools
import scipy.stats as stats
import shapefile
import networkx as nx
import re
from rtree import index
import csv
from datetime import datetime, date, time

class Map_(object):
    def __init__(self, graph_address = None):
        if graph_address == None:
            raise Exception("no graph assigned")

        tmp_split = graph_address.split('/')
        map_name = tmp_split[len(tmp_split)-1]

        print 'initializing...'
        graph = Graph(map_name)
        self.G = graph.read(graph_address)
        self.rtree_index = self.__rtree_generation()
        print 'done'

    def __rtree_generation(self):
        if self.G == None:
            raise Exception("no map to process!")

        p = index.Property()
        rtree_index = index.Index(properties=p)

        edge_id = 0
        for coor in self.G.edges_iter():
            rtree_index.insert(edge_id,self.G.edge[coor[0]][coor[1]]['bbox'], \
                                    obj=coor)
            edge_id += 1

        return rtree_index

    def __check_point(self, point=None):
        g = self.G
        number_of_nodes = g.number_of_nodes()
        for i in range(0,number_of_nodes):
            if abs(point[0]-g.node[i]['x'])<0.0001 \
                    and abs(point[1]-g.node[i]['y'])<0.0001:
                return i

        self.G.add_node(number_of_nodes,x=point[0],y=point[1])
        # print number_of_nodes
        return number_of_nodes

    # return a list of <distance, path>
    def __find_4_path_between_two_points_on_map(self,point1_info,point2_info):
        final_output=[]

        for i,j in itertools.product([0,1],[0,1]):
            try:
                distance,path = nx.bidirectional_dijkstra(self.G,point1_info['edge_id'][i],point2_info['edge_id'][j])

                output = distance + point1_info['dist_to_ends'][i] + point2_info['dist_to_ends'][j] \
                     + point1_info['dist'] + point2_info['dist']

            except:
                output = 100000
                path =[]

            final_output.append([output,path])

        return final_output

    def possible_shortest_path(self, point1, point2, topK=5):
        points_1_info = self.nearest_points_on_map(point1,topK)
        points_2_info = self.nearest_points_on_map(point2,topK)

        # print points_1_info
        # print points_2_info

        final_output=[]

        for point1_info,point2_info in itertools.product(points_1_info,points_2_info):
            final_output.extend(self.__find_4_path_between_two_points_on_map(point1_info,point2_info))

        def getKey(item):
            return item[0]

        return sorted(final_output,key=getKey)[0:topK]

        # find the projection point and the distance between point and line.

    def __nearest_point_and_dist_between(self, p1, p2, p3):
        type = -1
        if abs(p1[0] - p2[0]) < 0.0001:
            if p1[1] < p2[1]:
                tmp_p = p2
                p2 = p1
                p1 = tmp_p

            pp_x = p1[0]
            pp_y = p3[1]

            if pp_y>p1[1]:
                type = 1
            elif pp_y<p2[1]:
                type = 2

        else:
            if p1[0] - p2[0]>0:
                tmp_p = p2
                p2 = p1
                p1 = p2

            if abs(p1[1] - p2[1]) < 0.0001:
                pp_x = p3[0]
                pp_y = p1[1]
            else:
                k = float(p1[1]-p2[1])/(p1[0]-p2[0])
                pp_x = (k*p1[0] + 1/k*p3[0] + p3[1] - p1[1])/(k+1/k)
                pp_y = k*(pp_x - p1[0]) + p1[1]

            if pp_x<p1[0]:
                type = 1
            elif pp_x>p2[0]:
                type = 2

        if type == 1:
            dist = math.sqrt((p3[0]-p1[0])**2 + (p3[1]-p1[1])**2)
            pp = (p1[0],p1[1])
        elif type == 2:
            dist = math.sqrt((p3[0]-p2[0])**2 + (p3[1]-p2[1])**2)
            pp = (p2[0],p2[1])
        else:
            dist = math.sqrt((p3[0]-pp_x)**2 + (p3[1]-pp_y)**2)
            pp = (pp_x,pp_y)

        return (dist, pp)

    # return topk nearest points with the edge_info(edge_id, ith trajectory).
    # return is list of {'pp':,point<longitude,latitude>,'dist_to_ends':dist_to_left, dist_to_right,'edge_id':two points}
    # pp is the exact point on map transferred from query point, dist is the distance between pp and query point
    def nearest_points_on_map(self, point, topK):
        # return hit rectangles. attr:(id, bounds, bbox, object
        rec = (point[0],point[1],point[0],point[1])
        hit_recs = self.rtree_index.nearest(rec,topK,objects=True)

        # define comparable value.
        def getKey(item):
            return item['dist']

        # take two point from each trajectory
        candidate_points_with_info = []
        for i in hit_recs:
            edge_id = i.object
            points = self.G.edge[edge_id[0]][edge_id[1]]['points']
            route_dist = self.G.edge[edge_id[0]][edge_id[1]]['weight']

            candidate_local=[]
            dist_to_left_point = 0.0
            for j in range(1,len(points)):
                dist, pp = self.__nearest_point_and_dist_between(points[j-1], \
                                                                 points[j],point)
                dist_to_left_point += math.sqrt((points[j-1][0]-points[j][0])**2 + (points[j-1][1]-points[j][1])**2)
                dist_to_right_point = route_dist - dist_to_left_point

                # candidate_local.append({'pp':pp,'dist':dist,'trajectory_no':j,\
                #                         'dist_to_ends':[dist_to_left_point,dist_to_right_point],'edge_id':edge_id})
                candidate_local.append({'pp':pp, 'dist':dist, \
                                        'dist_to_ends':[dist_to_left_point,dist_to_right_point], 'edge_id':edge_id})
            candidate_points_with_info.extend(sorted(candidate_local, key=getKey)[0:2:])

        return sorted(candidate_points_with_info, key=getKey)[0:topK:]

    # return a list of generated observation probability
    def __observation_prob_generation(self, points_info, r_point):
        observation_prob = []

        # normal distribution with 20m deviation
        norm = stats.norm(0,20)

        for point_info in points_info:
            gps_dist = math.sqrt((point_info['pp'][0]-r_point[0])**2 + (point_info['pp'][1]-r_point[1])**2)
            dist = gps_dist*60*60*23.

            # print 'dist:'
            # print dist

            observation_prob.append(norm.pdf(dist))

        return observation_prob

    # return <incremental_col, retrieve_col>.
    def __incremental_generation(self, actual_point, points_info_1, points_info_2, base_prob):
        length = len(points_info_1)

        obser_prob_generation = self.__observation_prob_generation(points_info_2, actual_point)

        # print 'obser_prob_generation:'
        # print obser_prob_generation

        incremental_col=[]
        retrieve_col = []
        path_col = []

        def getKey(item):
            return item[0]

        for i in range(length):
            retrieve_index = None
            max_value = 0

            for j in range(length):
                dist_path=self.__find_4_path_between_two_points_on_map(points_info_1[j],points_info_2[i])
                dist_path = sorted(dist_path,key=getKey)

                euclidean_dist = math.sqrt((points_info_1[j]['pp'][0]-points_info_2[i]['pp'][0])**2
                                           + (points_info_1[j]['pp'][1]-points_info_2[i]['pp'][1])**2)

                trans_prob = euclidean_dist/dist_path[0][0]
                # when trans_prob>1, assign it = 1.
                if trans_prob>1:
                    trans_prob = 1

                prob_product = obser_prob_generation[i]* (trans_prob) + base_prob[j]

                # print obser_prob_generation[i]
                # print base_prob[j]
                #
                # print max_value
                # print prob_product

                if max_value < prob_product:
                    max_value = prob_product
                    retrieve_index = j
                    path_max = dist_path[0][1]

            incremental_col.append(max_value)
            retrieve_col.append(retrieve_index)
            path_col.append(path_max)

        return (incremental_col, retrieve_col, path_col)

    # trajectory: raw trajectory data, and take topK candidate points.
    def __map_matching(self, trajectory_info, topK = 1):
        cand_topK = topK*3
        trajectory = trajectory_info.coord_extraction()

        # candidate_local.append({'pp':pp,'dist':dist,'trajectory_no':j,\
        #                         'dist_to_ends':[dist_to_left_point,dist_to_right_point],'edge_id':edge_id})
        candidate_points_info=[self.nearest_points_on_map(trajectory[0],cand_topK)]

        incremental_ = self.__observation_prob_generation(candidate_points_info[0],trajectory[0])
        retrieve_matrix = []
        path_matrix = []

        incremental_matrix=[incremental_]

        # print 'incremental:'
        # print incremental_

        for i in range(1,len(trajectory)):
            candidate_points_info.append(self.nearest_points_on_map(trajectory[i],cand_topK))

            incremental_, retrieve_col, path_col = self.__incremental_generation(
                trajectory[i],candidate_points_info[i-1],candidate_points_info[i],incremental_)

            # if incremental_[0]>78700490:
            #     print i
            #     print trajectory[i]
            #     print candidate_points_info[i-1]
            #     print candidate_points_info[i]
            #     print incremental_
            #     print retrieve_col
            #     break

            retrieve_matrix.append(retrieve_col)
            incremental_matrix.append(incremental_)
            path_matrix.append(path_col)

        length = len(retrieve_matrix)
        indexes = int(incremental_.index(max(incremental_)))
        reverse_map_points = []
        reverse_right_path = []

        for k in range(length-1,-1,-1):
            reverse_map_points.append(candidate_points_info[k+1][indexes])
            reverse_right_path.append(path_matrix[k][indexes])
            indexes = retrieve_matrix[k][indexes]

        reverse_map_points.append(candidate_points_info[0][retrieve_matrix[0][indexes]])

        # return retrieve_matrix, incremental_matrix, reverse_map_points, reverse_right_path
        return reverse_map_points, reverse_right_path

    def test(self):
        # distance1, path1 = nx.bidirectional_dijkstra(self.G,2761,10000)
        # print path1
        # try:
        #     distance2, path2 = nx.bidirectional_dijkstra(self.G,5990,5993)
        # except:
        #     distance3, path3 = nx.bidirectional_dijkstra(self.G,2761,10000)
        #     print path3

        # paths = nx.single_source_dijkstra_path(self.G,5993)
        # print paths

        k = self.nearest_points_on_map([121.4686,31.2226],2)
        print k

    # return <aligned_traj, complete_traj>
    def traj_map(self, file_address, raw_traj=None):
        if file_address == None:
            raise Exception("file not exist")

        if raw_traj == None:
            raw_traj = trajectory.RawTrajectory()
            raw_traj.trajectory_generation_from_csv(file_address)

        reverse_map_points,reverse_path = self.__map_matching(raw_traj)

        length = len(reverse_path)
        raw_points = raw_traj.getPoints()

        complete_points = []
        align_points = []

        for i in range(0,length):
            align_point = reverse_map_points[length-i]

            id = raw_points[i].getId()
            time = raw_points[i].getTime()
            latitude = align_point['pp'][1]
            longitude = align_point['pp'][0]
            type = 'aligned'

            align_points.append(Traj_Point(id,latitude,longitude,type,time))
            complete_points.append(Traj_Point(id,latitude,longitude,type,time))

            # adjacent points on the same route, we connect them directly
            if align_point['edge_id']==reverse_map_points[length-i-1]['edge_id']:
                continue

            for node_id in reversed(reverse_path[length-i-1]):
                complete_points.append(Traj_Point(id,self.G.node[node_id]['y'],
                                                  self.G.node[node_id]['x'],'vertex'))

        # add the end point:
        id = raw_points[length].getId()
        time = raw_points[length].getTime()
        latitude = reverse_map_points[0]['pp'][1]
        longitude = reverse_map_points[0]['pp'][0]

        align_points.append(Traj_Point(id,latitude,longitude,type="aligned",time=time))
        complete_points.append(Traj_Point(id,latitude,longitude,type="aligned",time=time))

        align_traj = trajectory.AlignedTrajectory(align_points)
        complete_traj = trajectory.CompleteTrajectory(complete_points)

        return align_traj, complete_traj

if __name__ == '__main__':
    map = Map_('shanghai_graph')

    # map.test()
    file_address = 'traj_contr_test3.csv'

    aligned_traj, complete_traj = map.traj_map(file_address)
    complete_traj.write_csv_file("complete_traj_output_1.csv")



