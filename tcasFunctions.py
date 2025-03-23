class vector3:
    x = 0
    y = 0
    z = 0

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def str(self):
        return str(self.x) + ", " + str(self.y) + ", " + str(self.z)


p1 = vector3(x=0, y=-4000, z=15000)
p2 = vector3(x=-4000, y=0, z=14800)
v1 = vector3(x=0, y=400, z=0)
v2 = vector3(x=400, y=0, z=6)

#calculate relative vectors
pr = vector3(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z)
vr = vector3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z)

#CONSTANTS
vmax = 7.62 # max vertical descent velocity in m/s, taken rom tcas book
amax = 0.25 * 9.8 # max vertical acceleration in m/s^2, taken from tcas book
react = 5 # expected time in s between advisory issued and decent starts

tau_ta_th = None
range_ta_th = None
vert_ta_th = None
tau_ra_th = None
range_ra_th = None
vert_ra_th = None


def calculateThresholds(altitude):
    sl = None
    if altitude < 3280:
        sl = 2
    elif altitude < 7710:
        sl = 3
    elif altitude < 16400:
        sl = 4
    elif altitude < 32800:
        sl = 5
    elif altitude < 65600:
        sl = 6
    elif altitude < 137760:
        sl = 7
    else:
        sl = 8
    
    global tau_ta_th, range_ta_th, vert_ta_th, tau_ra_th, range_ra_th, vert_ra_th
    tau_ta_th = ([20, 25, 30, 40, 45, 48, 48])[sl-2]
    range_ta_th = ([560, 615, 890, 1390, 1855, 2410, 2410])[sl-2]
    vert_ta_th = ([260, 260, 260, 260, 260, 260, 365])[sl-2]
    tau_ra_th = ([0, 15, 20, 25, 30, 35])[sl-2]
    range_ra_th = ([0, 375, 650, 1020, 1485, 2040, 2040])[sl-2]
    vert_ra_th = ([0, 185, 185, 185, 185, 215, 250])[sl-2]




def advise_maneuver(p1, p2, v1, v2, pr, vr, vmax, amax, react):
    # calculate time of closest approach
    tca = -(pr.x * vr.x + pr.y * vr.y + pr.z * vr.z) / (vr.x * vr.x + vr.y * vr.y + vr.z * vr.z)


    # calculate the most the aircraft can ascend or descend before time of closest arrival
    reusedValue = (v1.z**2 - vmax**2)/(2*amax)
    downDeviation = -vmax*(tca - react - (v1.z + vmax)/amax) + (reusedValue + react*v1.z)
    upDeviation = vmax*(tca - react - (-v1.z + vmax)/amax) + (-reusedValue + react*v1.z)



    # calculate the final vertical deviation between the two planes 
    #   at closest time of arrival in both climbing and descending scenario
    downDifference = abs(p1.z + downDeviation - p2.z + tca * v2.z)  # if this number is greater, pilot should descend
    upDifference = abs(p1.z + upDeviation - p2.z + tca * v2.z)      # if this number is greater, pilot should climb
  
    return "climb" if upDifference > downDifference else "descend"


def threat_detection(pr, vr, tau_ta_th, range_ta_th, vert_ta_th, tau_ra_th, range_ra_th, vert_ra_th):
    prmagsq = pr.x * pr.x + pr.y * pr.y + pr.z * pr.z
    vrproj = pr.x * vr.x + pr.y * vr.y + pr.z * vr.z
    if vrproj == 0:
        taurange = 500
    else:
        taurange = abs(prmagsq / vrproj)
    
    if vr.z == 0:
        tauvert = 500
    else:
        tauvert = abs(pr.z / vr.z)


    ta = 0
    if (taurange < tau_ta_th or prmagsq < range_ta_th * range_ta_th) and (tauvert < tau_ta_th or pr.z < vert_ta_th):
        ta = 1
    
    ra = 0
    if (taurange < tau_ra_th or prmagsq < range_ra_th * range_ra_th) and (tauvert < tau_ra_th or pr.z < vert_ra_th):
        ra = 1

    return ta, ra
