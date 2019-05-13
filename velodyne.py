import math

def read_uint8(data, idx):
  return data[idx]


def read_sint8(data, idx):
  val = read_uint8(data, idx)
  return val-256 if val > 127 else val


def read_uint16(data, idx):
  return data[idx] + data[idx+1]*256

def read_position_uint16(data, idx):
  part_1 = hex(data[idx+1])
  part_1 = '0x0' + part_1[-1]
  part_2 = hex(data[idx])
  
  return int(part_1,16)+int(part_2,16)

def read_sint16(data, idx):
  val = read_uint16(data, idx)
  return val-2**16 if val > 2**15-1 else val


def read_uint32(data, idx):
  return data[idx] + data[idx+1]*256 + data[idx+2]*256*256 + data[idx+3]*256*256*256

def read_nmea(data,idx):
    output = data[idx]
    for i in range(72):
        idx =+ 1
        output = output + data[idx+i]
    return output


_RotSinTab = {}
_RotCosTab = {}

for i in range(360*100):
  _RotSinTab[i/100] = math.sin(math.radians(i/100))
  _RotCosTab[i/100] = math.cos(math.radians(i/100))

# UDP breakdown on page 22 HDL-32E user's manual
class GpsPacket:
    
  def __init__(self):
    self.gyro_1 = None
    self.temp_1 = None
    self.accel_1_x = None
    self.accel_1_y = None
    self.gyro_2 = None
    self.temp_2 = None
    self.accel_2_x = None
    self.accel_2_y = None
    self.gyro_3 = None
    self.temp_3 = None
    self.accel_3_x = None
    self.accel_3_y = None
    self.gps_timestamp = None
    self.nmea_sentence = None

class LaserState:

  def __init__(self):
    self.raw_bytes = [None]*4*7
    self.values = {}

  def convert(self):
    self.values['vertical_correction'] = read_sint16(self.raw_bytes, 1)/100
    self.values['rotational_correction'] = read_sint16(self.raw_bytes, 3)/100
    self.values['distance_far_correction'] = read_sint16(self.raw_bytes, 5)/10
    self.values['distance_correction_x'] = read_sint16(self.raw_bytes, 7)/10
    self.values['distance_correction_y'] = read_sint16(self.raw_bytes, 9)/10
    self.values['vertical_offset_correction'] = read_sint16(self.raw_bytes, 11)/10
    self.values['horizontal_offset_correction'] = read_sint16(self.raw_bytes, 13)/10
    self.values['focal_distance'] = read_sint16(self.raw_bytes, 15)/10
    self.values['focal_slope'] = read_sint16(self.raw_bytes, 17)/10
    self.values['min_intensity'] = read_uint8(self.raw_bytes, 19)
    self.values['max_intensity'] = read_uint8(self.raw_bytes, 20)


class StatusState:

  def __init__(self):
    self.frame_idx = None
    self.block_idx = None
    self.laser_idx = None
    self.block_bytes = [None]*7
    # 65 bytes, index starting at 1
    self.raw_bytes = [None]*66
    self.values = {}
    self.lasers = [LaserState() for i in range(64)]

  def convert(self):
    for l in self.lasers:
      l.convert()
    self.values['checksum'] = read_uint16(self.raw_bytes, 64)

def process_position_data(data, idx):
    '''
    42 Ethernet header (already passed before this function)
    14 Not used
    2 Gyro 1
    2 Temp 1
    2 Accel1 X
    2 Accel1 Y
    2 Gyro 2
    2 Temp 2
    2 Accel2 X
    2 Accel2 Y
    2 Gyro 3
    2 Temp 3
    2 Accel3 X
    2 Accel3 Y
    160 Not used
    4 GPS timestamp from top of hour
    4 Not used
    72 NMEA sequence
    234 Not used
    '''
    gyro_scale_factor = 0.09766 # deg/sec
    temp_scale_factor = 0.1453
    temp_offset = 25 # Celsius
    accel_scale_factor = 0.001221 # G
    
    idx += 14
    gyro_1 = read_position_uint16(data, idx) * gyro_scale_factor
    idx += 2
    temp_1 = (read_position_uint16(data, idx) * temp_scale_factor) + temp_offset
    idx += 2
    accel_1_x = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    accel_1_y = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    gyro_2 = read_position_uint16(data, idx) * gyro_scale_factor
    idx += 2
    temp_2 = (read_position_uint16(data, idx) * temp_scale_factor) + temp_offset
    idx += 2
    accel_2_x = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    accel_2_y = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    gyro_3 = read_position_uint16(data, idx) * gyro_scale_factor
    idx += 2
    temp_3 = (read_position_uint16(data, idx) * temp_scale_factor) + temp_offset
    idx += 2
    accel_3_x = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    accel_3_y = read_position_uint16(data, idx) * accel_scale_factor
    idx += 2
    idx += 160
    gps_timestamp = read_uint32(data, idx)
    idx += 4
    idx += 4
    nmea_sentence = str(data[idx:idx+72])[2:-1]
    if len(nmea_sentence) == 288:
        nmea_sentence = 'no nmea'
    return (0,0,0,0,0,0,0,0,gyro_1, temp_1, accel_1_x, accel_1_y, gyro_2, temp_2, accel_2_x, accel_2_y, gyro_3, temp_3, accel_3_x, accel_3_y, gps_timestamp, nmea_sentence)
    

def read_firing_data(data, idx, fd_callback):
    block_id = read_uint16(data, idx)
    # 0xeeff is upper block, 0xddff is lower block
    
    assert block_id == 0xeeff or block_id == 0xddff 
    idx += 2
    rot_pos = read_uint16(data, idx)/100
    idx += 2

    for l in range(32):
        dist = read_uint16(data, idx)
        idx += 2
        inten = read_uint8(data, idx)
        idx += 1
        # upper laser block ids range from 0 to 31, lower from 32 to 63
        laser_idx = l  + (32 if block_id == 0xddff else 0)
        fd_callback(laser_idx, rot_pos, dist, inten)


def process_block(state):

  if state.block_bytes[0:5] == [ord('U'), ord('N'), ord('I'), ord('T'), ord('#')]:
    state.block_idx = 0
    state.raw_bytes[15] = state.block_bytes[5]
    state.raw_bytes[16] = state.block_bytes[6]

  elif state.block_idx is not None:
    state.block_idx += 1

    if state.block_idx < 64*4+1:
      if (state.block_idx-1)%4 == 0:
        state.laser_idx = state.block_bytes[0]
      idx = ((state.block_idx-1)%4)*7
      state.lasers[state.laser_idx].raw_bytes[idx:idx+7] = state.block_bytes

    else:
      idx = 45 + (state.block_idx-(64*4+1))*7
      state.raw_bytes[idx:idx+7] = state.block_bytes

      # on last block
      if state.block_idx == (1+64*4+3)-1:
        state.convert()


def process_status_byte(type, value, state):

  if type == ord('H'):
    state.frame_idx = 1

  if state.frame_idx is not None:

    if state.frame_idx <= 9:
      state.raw_bytes[state.frame_idx] = value

    else:
      state.block_bytes[state.frame_idx-10] = value

      if state.frame_idx == 16:
        #print(state.block_idx)
        process_block(state)

    state.frame_idx += 1


def process_frame(data, idx, status, fd_callback):
  for b in range(12):
    read_firing_data(data, idx, fd_callback)
    idx += 100

  # Timestamp is recorded as minutes past the hour in microseconds (1 min = 1.66667e-8 usec)
  gps_timestamp = read_uint32(data, idx)
  #print("GPS timestamp:", (gps_timestamp*1.66667e-8))
  
  idx += 4
  status_type = read_uint8(data, idx)
  idx += 1
  status_value = read_uint8(data, idx)
  process_status_byte(status_type, status_value, status)
  return gps_timestamp


def calc_coords(dist, rot, cal):
  # factor cm to m
  VLS_DIM_SCALE = 100
  # factor distance value to cm
  DistLSB = 0.2

  if dist == 0:
    return (0, 0, 0)

  if not '_cos_vert_corr' in cal:
    cal['_cos_vert_corr'] = math.cos(math.radians(cal['vertical_correction']))
    cal['_sin_vert_corr'] = math.sin(math.radians(cal['vertical_correction']))
    cal['_cos_rot_corr'] = math.cos(math.radians(cal['rotational_correction']))
    cal['_sin_rot_corr'] = math.sin(math.radians(cal['rotational_correction']))
    cal['_hor_off_corr'] = cal['horizontal_offset_correction']/VLS_DIM_SCALE
    cal['_vert_off_corr'] = cal['vertical_offset_correction']/VLS_DIM_SCALE
    cal['_dist_corr_x_fact'] = (cal['distance_far_correction']-cal['distance_correction_x'])/(2504-240) # EVAN: field of view?
    cal['_dist_corr_y_fact'] = (cal['distance_far_correction']-cal['distance_correction_y'])/(2504-193)

  distancel = DistLSB * dist
  distance = distancel+ cal['distance_far_correction']

  if rot == 360.0:
      print("Skipped because of 360.0 degrees.. Produces error, come back to this") # EVAN: For debugging
  else:
    cosRotAngle = _RotCosTab[rot]*cal['_cos_rot_corr'] + _RotSinTab[rot]*cal['_sin_rot_corr']
    sinRotAngle = _RotSinTab[rot]*cal['_cos_rot_corr'] - _RotCosTab[rot]*cal['_sin_rot_corr']

    hOffsetCorr = cal['_hor_off_corr']
    vOffsetCorr = cal['_vert_off_corr']


    xyDistance = distance * cal['_cos_vert_corr']
    xx = xyDistance * sinRotAngle
    yy = xyDistance * cosRotAngle

    if xx<0: xx = -xx
    if yy<0: yy = -yy

    distanceCorrX = cal['_dist_corr_x_fact']*(xx-240)+cal['distance_correction_x']
    distanceCorrY = cal['_dist_corr_y_fact']*(yy-193)+cal['distance_correction_y']


    if distancel > 2500:
        distanceCorrX = cal['distance_far_correction']
        distanceCorrY = distanceCorrX

    distancel /= VLS_DIM_SCALE
    distanceCorrX /= VLS_DIM_SCALE
    distanceCorrY /= VLS_DIM_SCALE

    distance = distancel+distanceCorrX
    xyDistance = distance * cal['_cos_vert_corr']

    x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle

    distance = distancel+distanceCorrY
    xyDistance = distance * cal['_cos_vert_corr']

    y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle
    z = distance * cal['_sin_vert_corr'] + vOffsetCorr

    return (x, y, z)

