from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import numpy as np

def mavheartbeat():
    # Returns unpacked bit array of mavlink heartbeat message
    
    # Mavlink encoder
    mav = mavlink2.MAVLink(None)
    mav.srcSystem = 1
    mav.srcComponent = 1


    # Message creation
    msg = mavlink2.MAVLink_heartbeat_message(
        type=mavlink2.MAV_TYPE_QUADROTOR,
        autopilot=mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode = 0,
        custom_mode = 0,
        system_status=mavlink2.MAV_STATE_ACTIVE,
        mavlink_version = 3
    )

    # serializing to raw bytes

    packet_bytes = msg.pack(mav)

    # create byte array
    byte_array = np.frombuffer(packet_bytes, dtype=np.uint8)

    # Bistream unpack bytes
    bit_array = np.unpackbits(byte_array, bitorder='big')

    return(list(bit_array))

def mavreader(byte_list):
    
    byte_data = bytes(byte_list)

    # mavlink parser
    mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.robust_parsing = True  # Handle errors gracefully

    # Parse packet
    try:
        msgs = mav.parse_buffer(byte_data)
        for msg in msgs:
            print(msg)
            print(msg.to_dict())
    except Exception as e:
        print(f"Parse error: {e}")