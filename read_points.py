import msgpack
import sys
import dpkt
import json
import time
import numpy as np
import pandas as pd
from velodyne import StatusState, process_frame, calc_coords, GpsPacket, process_position_data


if len(sys.argv) < 4:
  print('usage: python {} <pcap_file> <out_file> <cal_file>'.format(sys.argv[0]))
  exit()

in_file = sys.argv[1]
out_file = sys.argv[2]
cal_file = sys.argv[3]

points = []
points.append(('x','y','z','distance','rotation','laser_id','intensity','gps_time','gyro1', 'temp1', 'accel_1_x', 'accel_1_y', 'gyro_2', 'temp_2', 'accel_2_x', 'accel_2_y', 'gyro_3', 'temp_3', 'accel_3_x', 'accel_3_y', 'gps_timestamp', 'nmea_sentence'))

position_data = []
position_data.append(('gyro1', 'temp1', 'accel_1_x', 'accel_1_y', 'gyro_2', 'temp_2', 'accel_2_x', 'accel_2_y', 'gyro_3', 'temp_3', 'accel_3_x', 'accel_3_y', 'gps_timestamp', 'nmea_sentence'))

status = StatusState()

print('using calibration file '+cal_file)
with open(cal_file, 'r') as f:
  calibration_vals = json.load(f)

def firing_data_callback(laser_idx, rot_pos, dist, intensity):
  vals = calibration_vals[laser_idx]
  coords = calc_coords(dist, rot_pos, vals)
  if coords != (0, 0, 0):
    coord_edited = (coords[0], coords[1], coords[2], dist, rot_pos, laser_idx, intensity, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    points.append(coord_edited)

start_time = 0
print('reading packets from '+in_file)
with open(in_file, 'rb') as f:
  reader = dpkt.pcap.Reader(f)

  frame_index = 0
  last_t = time.time() 

  for ts, buf in reader:
    eth = dpkt.ethernet.Ethernet(buf)
    
    # Only process either position data 554/512 bytes or lidar data 1248/1206 bytes
    if len(eth) == 554 or len(eth) == 1248:
        data = eth.data.data.data
        if (len(data) == 512):
            None
            points.append(process_position_data(data, 0))
        else:
            last_time = process_frame(data, 0, status, firing_data_callback)
            points.append((0,0,0,0,0,0,0, last_time, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        
        frame_index += 1
        if frame_index % 1000 == 0:
            t = time.time()
            print('processing frames: ' + str(frame_index) + ' fps: '+str(int(1000/(t - last_t))))
            last_t = t

#print("Start time:", start_time)
#print("End time:", last_time)
#print("Total time:",((last_time-start_time)*1.66667e-8))

print('writing data to '+out_file)

position_df = pd.DataFrame(position_data)
position_df.to_csv('position.csv')
point_df = pd.DataFrame(points)
point_df.to_csv(out_file)
#point_array = np.array(points)


# Exports to msgpack file
# Commented out at the moment to see if we can make this a pointcloud instead
#with open(out_file, 'wb') as f:
   #msgpack.pack(points, f)

print('done')
