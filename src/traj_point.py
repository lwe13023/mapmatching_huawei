__author__ = 'waynewen'

# point with attrs:
#       <latitude, longitude, time, type>
class Traj_Point(object):
    def __init__(self, id, latitude, longitude, type=None, time=None):
        self.setPoint(id,latitude,longitude,type,time)

    def getId(self):
        return self.id

    def setId(self, id):
        self.id = id

    def getLatitude(self):
        return self.latitude

    def setLatitude(self, latitude):
        self.latitude = float(latitude)

    def getLongitude(self):
        return self.longitude

    def setLongitude(self, longitude):
        self.longitude = float(longitude)

    def getPoint(self):
        return [self.getId(), self.getLatitude(),self.getLongitude(),self.getType(),self.getTime()]

    # <id,latitude,longitude,time,type>
    def setPoint(self, id, latitude, longitude, type = None, time = None):
        self.setId(id)
        self.setLatitude(latitude)
        self.setLongitude(longitude)
        self.setTime(time)
        self.setType(type)

    def getTime(self):
        return self.time

    def setTime(self, time):
        self.time = time

    def getType(self):
        return self.type

    def setType(self, type):
        self.type = type

    def __str__(self):
        return "id:%s, latitude:%f, longitude:%f, type:%s time:%s"%(self.id, self.latitude, self.longitude, self.type, self.time)

