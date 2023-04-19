from pythonosc import udp_client
from vicon_dssdk import ViconDataStream
from visualize_rotation import RotationVisualization

visual = RotationVisualization()

left_subject_name = 'Wand'
left_root_segment_name = 'Wand'

right_subject_name = 'Other_Wand'
right_root_segment_name = 'Other_Wand'

vicon_host = '192.168.1.36:801'
max_host, max_port = '127.0.0.1', 7000
 
vicon_client = ViconDataStream.Client()
max_udp_client = udp_client.SimpleUDPClient(max_host, max_port)
# frames = []
 
print( 'Connecting' )
while not vicon_client.IsConnected():
    print( '.' )
    vicon_client.Connect( '192.168.1.36:801' )

vicon_client.EnableSegmentData()


def print_if_there(res):
    pos, o = res
    if not o:
        print(pos)

try:
    while vicon_client.IsConnected():
        if vicon_client.GetFrame():
            rot, o = vicon_client.GetSegmentGlobalRotationEulerXYZ(left_subject_name, left_root_segment_name)
            pos, o = vicon_client.GetSegmentGlobalTranslation(left_subject_name, left_root_segment_name)
            if o:
                continue
            visual.update_rotation(rot)
            max_udp_client.send_message("/rotation", rot)
            max_udp_client.send_message("/position", pos)
            # print(rot)

            # frames.append(client.GetFrameNumber())
            
        # if len(frames) > 1_000:
        #     break
 
except ViconDataStream.DataStreamException as e:
    print( 'Error', e )
 

subject_names = vicon_client.GetSubjectNames()
assert left_subject_name in subject_names and right_subject_name in subject_names