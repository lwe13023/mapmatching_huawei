__author__ = 'waynewen'

import shapefile
import networkx as nx

import re

"""
Generate Graph from shapefile

Function:
read(file_address=None):
read generated_graph file

write(file_address):
write file generated from shapefile


"""

class Graph(object):
    def __init__(self, graph_name = 'default'):
        self.G = nx.Graph()
        self.G.graph['map_name'] = graph_name

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

    def __shapefile2graph(self):
        print 'generating map:'
        progress = len(self.shapes)/10

        for index in range(0,len(self.shapes)):
            if index%progress == 0:
                print 'finish: %d%%' % (index/progress*10)

            points = self.shapes[index].points
            bbox = self.shapes[index].bbox

            len_p = len(points)

            start_point = points[0]
            end_point = points[len_p-1]

            edge_weight = 0.0
            for i in range(1,len_p):
                tmp_weight = pow((points[i][0]-points[i-1][0])**2
                                 +(points[i][1]-points[i-1][1])**2,0.5)
                edge_weight += tmp_weight

            start_id = self.__check_point(point=start_point)
            end_id = self.__check_point(point=end_point)
            self.G.add_edge(start_id,end_id,weight=edge_weight,points=points,bbox=bbox)

        print 'finish'

    # str variables: (shp,shx,dbf) or path2file
    def generate_graph(self, *args, **kwargs):
        if len(kwargs) == 1:
            _shp = open(kwargs['shp'])
            r = shapefile.Reader(shp=_shp)

        elif len(kwargs)==3:
            _shp = open(kwargs['shp'])
            _shx = open(kwargs['shx'])
            _dbf = open(kwargs['dbf'])

            r = shapefile.Reader(shp=_shp,dbf=_dbf,shx=_shx)

        else:
            raise Exception('wrong size of input argument')

        self.shapes = r.shapes()
        self.__shapefile2graph()

    def writer(self,file_address='default'):
        nx.write_gml(self.G,file_address)

        # return self.G
    def read(self,file_address=None):
        with open(file_address,'rb') as file:
            file.readline()
            file.readline()
            # map_name = re.split(' |\n',file.readline())

            row = 0
            tmp_row = 0
            for line in file:
                if line != '':
                    judge = re.split(' |\n',line)
                    # print judge

                    if len(judge)<3:
                        continue

                    if judge[2] == 'node':
                        node = []
                        row = 5
                        tmp_row = 0

                    elif judge[2] == 'edge':
                        edge = []
                        row = 6
                        tmp_row = 0

                    elif row == 5:
                        judge = filter(lambda a: a!='',judge)
                        node.extend(judge)
                        tmp_row += 1
                        if tmp_row == 5:
                            # print node
                            self.G.add_node(int(node[1]), x=float(node[7]), y=float(node[5]))

                    elif row == 6:
                        judge = filter(lambda a: a!='',judge)
                        edge.extend(judge)

                        tmp_row += 1
                        if tmp_row == 6:
                            # print edge

                            # points recovery
                            points = []
                            point=[]
                            bbox_posi = 0
                            for i in range(5,len(edge)):
                                value = edge[i]
                                if value=='bbox':
                                    bbox_posi = i
                                    break
                                else:
                                    if i%2 == 1:
                                        posi = value.rfind('[')
                                        point.append(float(value[posi+1:-1:]))
                                    else:
                                        point.append(float(value[:-2:]))
                                        points.append(point)
                                        point=[]

                            # bbox recovery
                            bbox = [float(edge[bbox_posi+1][1:-1:])]
                            for j in range(2,5):
                                bbox.append(float(edge[bbox_posi+j][:-1:]))

                            # weight recovery
                            weight = float(edge[bbox_posi+6])
                            self.G.add_edge(int(edge[1]),int(edge[3]),points=points,bbox=bbox,weight=weight)

        return self.G

    def read_revised(self,file_address=None):
        with open(file_address,'rb') as file:
            file.readline()
            file.readline()
            # map_name = re.split(' |\n',file.readline())

            judge = re.split(' |\n',file.readline())

            while judge[2] == 'node':
                node = []
                for i in range(5):
                    judge = filter(lambda a: a!='', re.split(' |\n',file.readline()))
                    node.extend(judge)

                self.G.add_node(int(node[1]), x=float(node[7]), y=float(node[5]))
                judge = re.split(' |\n',file.readline())

            while judge[0] != ']':
                edge = []
                for j in range(6):
                    judge= filter(lambda a: a!='', re.split(' |\n',file.readline()))
                    edge.extend(judge)

                # points recovery
                points = []
                point=[]
                bbox_posi = 0
                for i in range(5,len(edge)):
                    value = edge[i]
                    if value=='bbox':
                        bbox_posi = i
                        break
                    else:
                        if i%2 == 1:
                            posi = value.rfind('[')
                            point.append(float(value[posi+1:-1:]))
                        else:
                            point.append(float(value[:-2:]))
                            points.append(point)
                            point=[]

                # bbox recovery
                bbox = [float(edge[bbox_posi+1][1:-1:])]
                for j in range(2,5):
                    bbox.append(float(edge[bbox_posi+j][:-1:]))

                # weight recovery
                weight = float(edge[bbox_posi+6])

                # edge_recovery
                self.G.add_edge(int(edge[1]),int(edge[3]),points=points,bbox=bbox,weight=weight)
                judge = re.split(' |\n',file.readline())

        return self.G

if __name__ == '__main__':

    myshp = '../map_info/shanghai.shp'
    mydbf = '../map_info/shanghai.dbf'
    myshx = '../map_info/shanghai.shx'

    graph = Graph('shanghai_graph')
    graph.generate_graph(shp=myshp, dbf=mydbf, shx=myshx)
    graph.generate_graph(shp=myshp)
    graph.writer('shanghai_graph')

