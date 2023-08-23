from pymavlink import mavutil

def set_mode(the_connection, mode):
    if mode not in the_connection.mode_mapping():
        return

    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def start_mission():

    the_connection = mavutil.mavlink_connection("172.29.80.1:14550")

    """ ARM THROTTLE """
    the_connection.wait_heartbeat()
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    """ SET THE FLIGHT MODE TO TAKEOFF """
    the_connection.wait_heartbeat()
    set_mode(the_connection, "TAKEOFF")

    """ SET TAKE OFF """
    the_connection.wait_heartbeat()
    latitude  = 0
    longitude = 0

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,latitude,longitude,10)


    """ SET MODE TO AUTO """
    the_connection.wait_heartbeat()
    set_mode(the_connection, 'AUTO')
    the_connection.close()


if __name__ == "__main__":
    start_mission()
