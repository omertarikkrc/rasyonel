

from dronekit import connect
from math import cos, pi, sin, tan



# Connect to the Vehicle (in this case a UDP endpoint)
osman = connect("/dev/ttyAMA0", baud=921600, wait_ready=True)



print( osman.location.global_relative_frame.lat)
camera_horizontal_angle = (31.1/180)*pi              # camera horizontal angle
height = 10                                         # height
lon_mav = 41.10162344414639                          # lon plane come from pixhawk
lat_mav =  29.021909590536062                        # lat plane come from pixhawk
yaw_mav = (0/180)*pi
ml_ratio = 0.7                                       # hedef merkeze uzakligi bolu tam ekrann orani + i>


max_observe_view_horizantal_camera_osman = tan(camera_horizontal_angle)*height
distance_from_mav = max_observe_view_horizantal_camera_osman * ml_ratio

lat_distance = distance_from_mav*cos(yaw_mav)
lon_distance = distance_from_mav*sin(yaw_mav)

lon_angle = (360 * lon_distance)/40007.863
lat_angle = (360 * lat_distance)/31323.0231012

lon_red_point = round(lon_angle + lon_mav,8)
lat_red_point = round(lat_angle + lat_mav,8)

print(lon_red_point,",",lat_red_point)
