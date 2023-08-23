from pymavlink import mavutil, mavwp


def send_waypoints(coordinates):
    """ We Establish the connection with the software """
    the_connection = mavutil.mavlink_connection("172.29.80.1:14550")
    the_connection.wait_heartbeat(blocking=True)
    wp = mavwp.MAVWPLoader()
    seq = 1
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL
    radius = 10
    for i in range(len(coordinates)):
        wp.add(mavutil.mavlink.MAVLink_mission_item_message(the_connection.target_system,
                the_connection.target_component,
                seq + i,
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, radius, 0, 0,
                coordinates[i][0],coordinates[i][1],coordinates[i][2]))

    the_connection.waypoint_clear_all_send()
    the_connection.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = the_connection.recv_match(type=['MISSION_REQUEST'],blocking=True)
        the_connection.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

    the_connection.close()











