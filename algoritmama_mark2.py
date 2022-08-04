from dronekit import connect
from math import cos, pi, sin, tan
from deneme8 import cx,cy

# ml data:
x_axis_ratio =cx/640
y_axis_ratio =cy/480



# Connect to the Vehicle (in this case a UDP endpoint)
osman = connect("/dev/ttyAMA0", baud=921600, wait_ready=True)



camera_horizontal_angle = (31.1/180)*pi              # camera horizontal angle
camera_vertical_angle = (24.4/180)*pi
height =   osman.location.global_relative_frame.alt                                       # height
lon_mav =  int(osman.location.global_relative_frame.lon)                         # lon plane come from pixhawk
lat_mav =  int(osman.location.global_relative_frame.lat)                       # lat plane come from pixhawk
yaw_mav = osman.heading                                                   # heading cv 
yaw_mav_rad = (yaw_mav/180)*pi
ml_ratio = 0.7                                                 # hedef merkeze uzakligi bolu tam ekrann orani + i>
field_view_vertical = tan(2*camera_vertical_angle)*height      # tanin icini 2x yapma sebebi on arka olmasi 
field_view_horizantal = tan(2*camera_horizontal_angle)*height  # tanin icini 2x yapma sebebi sağ sol olmasi 
ground_speed = osman.groundspeed 
current_horizantal_field_of_view = height*tan(camera_horizontal_angle)
current_vertical_field_of_view = height*tan(camera_vertical_angle)

additive_degree_for_frame_horizantal =current_horizantal_field_of_view/0.084    #it varies by 0.00001 degrees at every 0.084 m for istanbul
additive_degree_for_frame_vertical = current_vertical_field_of_view/0.11113295   #it varies by 0.00001 degrees at every 0.11113295 m  

if yaw_mav >0 and yaw_mav <90:
    above_right_point=LocationGlobalRelative(lat_mav +additive_degree_for_frame_horizantal,lon_mav+ additive_degree_for_frame_vertical)
    above_left_point =LocationGlobalRelative(lat_mav-additive_degree_for_frame_horizantal,lon_mav + additive_degree_for_frame_vertical)
    bottom_right_point=LocationGlobalRelative(lat_mav+additive_degree_for_frame_horizantal,lon_mav-additive_degree_for_frame_vertical)
    bottom_left_point =LocationGlobalRelative(lat_mav- additive_degree_for_frame_horizantal,lon_mav- additive_degree_for_frame_vertical)

elif yaw_mav > 90 and yaw_mav < 180:
    above_right_point=LocationGlobalRelative(lat_mav -additive_degree_for_frame_horizantal,lon_mav- additive_degree_for_frame_vertical)
    above_left_point =LocationGlobalRelative(lat_mav-additive_degree_for_frame_horizantal,lon_mav + additive_degree_for_frame_vertical)
    bottom_right_point=LocationGlobalRelative(lat_mav+additive_degree_for_frame_horizantal,lon_mav-additive_degree_for_frame_vertical)
    bottom_left_point =LocationGlobalRelative(lat_mav+ additive_degree_for_frame_horizantal,lon_mav+ additive_degree_for_frame_vertical)
elif yaw_mav > 180 and yaw_mav < 270:
    above_right_point=LocationGlobalRelative(lat_mav -additive_degree_for_frame_horizantal,lon_mav- additive_degree_for_frame_vertical)
    above_left_point =LocationGlobalRelative(lat_mav-additive_degree_for_frame_horizantal,lon_mav + additive_degree_for_frame_vertical)
    bottom_right_point=LocationGlobalRelative(lat_mav+additive_degree_for_frame_horizantal,lon_mav-additive_degree_for_frame_vertical)
    bottom_left_point =LocationGlobalRelative(lat_mav+ additive_degree_for_frame_horizantal,lon_mav+ additive_degree_for_frame_vertical)












lat_speed = ground_speed*sin(yaw_mav_rad)
lon_speed = ground_speed*cos(yaw_mav_rad)


max_observe_view_horizantal_camera_osman = tan(camera_horizontal_angle)*height
"""above_right_point =LocationGlobalRelative(arp_lat,arp_lon)
above_left_point =LocationGlobalRelative(alp_lat,alp_lon)
bottom_right_point=LocationGlobalRelative(brp_lat,brp_lon)
bottom_left_point=LocationGlobalRelative(blp_lat,blp_lon)"""




"""distance_from_mav = max_observe_view_horizantal_camera_osman * ml_ratio

lat_distance = distance_from_mav*cos(yaw_mav)
lon_distance = distance_from_mav*sin(yaw_mav)

lon_angle = (360 * lon_distance)/40007.863
lat_angle = (360 * lat_distance)/31323.0231012

lon_red_point = round(lon_angle + lon_mav,8)
lat_red_point = round(lat_angle + lat_mav,8)

print(lon_red_point,",",lat_red_point)"""
