class vector3:
    x = 0
    y = 0
    z = 0

p1 = vector3(x=0, y=0, z=0)
p2 = vector3(x=0, y=0, z=0)
v1 = vector3(x=0, y=0, z=0)
v2 = vector3(x=0, y=0, z=0)

#calculate relative vectors
pr = vector3(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z)
vr = vector3(v1.x-v2.x, v1.y-v2.y, v1.z-v2.z)

#CONSTANTS
vmax = 7.62 # max vertical descent velocity in m/s, taken rom tcas book
amax = 0.25 * 9.8 # max vertical acceleration in m/s^2, taken from tcas book
react = 5 # expected time in s between advisory issued and decent starts


def advise_maneuver(p1, p2, v1, v2, pr, vr, vmax, amax, react):
    # calculate time of closest approach
    tca = -(pr.x * vr.x + pr.y * vr.y + pr.z * vr.z) / (vr.x * vr.x + vr.y * vr.y + vr.z * vr.z)


    # calculate the most the aircraft can ascend or descend before time of closest arrival
    reusedValue = (v1.z^2 - vmax^2)/(2*amax)
    upDeviation = vmax*(tca - react - (-v1.z + vmax)/amax) + (-reusedValue + react*v1.z)
    downDeviation = -vmax*(tca - react - (v1.z + vmax)/amax) + (reusedValue + react*v1.z)


    # calculate the final vertical deviation between the two planes 
    #   at closest time of arrival in both climbing and descending scenario
    upDifference = abs(p1.z + upDeviation - p2.z + tca * v2.z)      # if this number is greater, pilot should climb
    downDifference = abs(p1.z + downDeviation - p2.z + tca * v2.z)  # if this number is greater, pilot should descend

    return "climb" if upDifference > downDifference else "descend"


def threat_detection(pr, vr, taurange_ta_th, tauvert_ta_th, range_ta_th, vert_ta_th, taurange_ra_th, tauvert_ra_th, range_ra_th, vert_ra_th):
    prmagsq = pr.x * pr.x + pr.y * pr.y + pr.z * pr.z
    vrproj = pr.x * vr.x + pr.y * vr.y + pr.z * vr.z
    taurange = prmagsq / vrproj
    tauvert = pr.z / vr.z

    ta = 0
    if (taurange < min(taurange_ta_th, range_ta_th) and tauvert < min(tauvert_ta_th, vert_ta_th)):
        ta = 1
    
    ra = 0
    if (taurange < min(taurange_ra_th, range_ra_th) and tauvert < min(tauvert_ra_th, vert_ra_th)):
        ra = 1


