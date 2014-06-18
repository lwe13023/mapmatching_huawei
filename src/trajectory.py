__author__ = 'waynewen'

import csv
from datetime import datetime
from traj_point import Traj_Point

import sklearn.hmm

"""

"""

class _Trajectory(object):
    def __init__(self, type, points=None):
        if points == None:
            self.points = []
        else:
            self.points = points
        self.type = type

    def getPoints(self):
        return self.points

    def setPoints(self, points):
        self.points = points

    def getType(self):
        """return the type of this trajectory"""
        return self.type

    # generate csv file
    def write_csv_file(self, output_file_name):
        with open(output_file_name, 'w') as op_file:
            csv_writer = csv.writer(op_file)

            csv_writer.writerow(['id','latitude','longitude','time','type'])

            for point in self.points:
                row = [point.getId(),point.getLatitude(),point.getLongitude(),point.getTime(),point.getType()]
                csv_writer.writerow(row)

    # extract <longitude, latitude>
    def coord_extraction(self):
        points = []

        for point in self.points:
            points.append([point.getLongitude(), point.getLatitude()])

        return points

    # append points to trajectory form standard csv file
    # erase continuous duplicated points.
    def trajectory_generation_from_csv(self,file_address=None):
        if file_address == None:
            raise Exception('file does not exist')

        with open(file_address,'rU') as csv_file:
            csv_reader = csv.reader(csv_file)

            prev_latitude = 0
            prev_longitude = 0

            for row in csv_reader:
                latitude = float(row[3])
                longitude = float(row[2])

                if abs(prev_latitude-latitude)<0.0001 and abs(prev_longitude-longitude)<0.0001:
                    continue

                dt = datetime.strptime(row[6], "%d/%m/%y %H:%M")
                self.points.append(Traj_Point(id=row[1],latitude=row[3],longitude=row[2],type=self.type,time=dt))
                prev_latitude = latitude
                prev_longitude = longitude

            return self.points

    def traverse_output(self):
        for point in self.points:
            print point.getPoint()

# raw trajectory data
class RawTrajectory(_Trajectory):
    def __init__(self, points=None):
        _Trajectory.__init__(self,type="Raw",points=points)

    def getType(self):
        return "Raw_Trajectory"


# projected points of each raw point
class AlignedTrajectory(_Trajectory):
    def __init__(self, points=None):
        _Trajectory.__init__(self, type="Aligned", points=points)

# projected points and vertexes of each road passed by projected points.
class CompleteTrajectory(_Trajectory):
    def __init__(self, points=None):
        _Trajectory.__init__(self, type="Complete", points=points)