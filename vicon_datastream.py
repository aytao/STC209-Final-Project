from pythonosc import udp_client
from vicon_dssdk import ViconDataStream
from visualize_rotation import RotationVisualization
from math import pi
from smoother import BufferSmoother

# visual = RotationVisualization()

left_subject_name = 'Wand'
left_root_segment_name = 'Wand'

right_subject_name = 'Other_Wand'
right_root_segment_name = 'Other_Wand'

vicon_host = '192.168.1.36:801'
max_host, max_port = '127.0.0.1', 7000
 
vicon_client = ViconDataStream.Client()
max_udp_client = udp_client.SimpleUDPClient(max_host, max_port)

print('Connecting...', end="")
while not vicon_client.IsConnected():
  vicon_client.Connect( '192.168.1.36:801' )

vicon_client.EnableSegmentData()

print('done')

def print_if_there(res):
  pos, o = res
  if not o:
    print(pos)

rot_bufs = [BufferSmoother() for _ in range(0, 3)]
pos_bufs = [BufferSmoother() for _ in range(0, 3)]
try:
  while vicon_client.IsConnected():
    if vicon_client.GetFrame():
      rot_left, o = vicon_client.GetSegmentGlobalRotationEulerXYZ(left_subject_name, left_root_segment_name)
      rot_right, o = vicon_client.GetSegmentGlobalRotationEulerXYZ(right_subject_name, right_root_segment_name)
            
      pos_left, o = vicon_client.GetSegmentGlobalTranslation(left_subject_name, left_root_segment_name)
      pos_right, o = vicon_client.GetSegmentGlobalTranslation(right_subject_name, right_root_segment_name)
      
      for i in range(0,3):
        pos_bufs[i].add(abs(pos_left[i] - pos_right[i]))
        dr = (rot_left[i] + pi) - (rot_right[i] + pi)
        if dr < 0:
          dr += 2 * pi
        dr = min(dr, 2 * pi - dr)
        rot_bufs[i].add(dr)
        
      pos = [buf.avg() for buf in pos_bufs]
      rot = [buf.avg() for buf in rot_bufs]
      
      if o:
          continue
      # visual.update_rotation(rot)
      max_udp_client.send_message("/rotation", rot)
      max_udp_client.send_message("/position", pos)
 
except ViconDataStream.DataStreamException as e:
  print( 'Error', e )