from math import atan2, sqrt, pi
from vicon_dssdk import ViconDataStream
from visualize_rotation import RotationVisualization

visual = RotationVisualization()

left_subject_name = 'Wand'
left_root_segment_name = 'Wand'

right_subject_name = 'Other_Wand'
right_root_segment_name = 'Other_Wand'


 
client = ViconDataStream.Client()
# frames = []
 
print( 'Connecting' )
while not client.IsConnected():
    print( '.' )
    client.Connect( '192.168.1.36:801' )

client.EnableSegmentData()


def print_if_there(res):
    pos, o = res
    if not o:
        print (pos)

try:
    while client.IsConnected():
        if client.GetFrame():
            rot, o = client.GetSegmentGlobalRotationEulerXYZ(left_subject_name, left_root_segment_name)
            if o:
                continue
            visual.update_rotation(rot)
            # print(rot)

            # frames.append(client.GetFrameNumber())
            
        # if len(frames) > 1_000:
        #     break
 
except ViconDataStream.DataStreamException as e:
    print( 'Error', e )
 

subject_names = client.GetSubjectNames()
assert left_subject_name in subject_names and right_subject_name in subject_names