from pymavlink import mavutil

def STIL_set_plane(capacity, voltage):
    """ Set Up the connection """
    the_connection = mavutil.mavlink_connection("172.29.80.1:14550")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))
    """ Set Up the list of parameter values"""
    param_name = ['BATT_CAPACITY', 'SIM_BATT_VOLTAGE']
    new_value = [capacity,voltage]  #Set Up the New Values
    for k in range(len(param_name)):
        the_connection.mav.param_set_send(
            the_connection.target_system,
            the_connection.target_component,
            param_id=param_name[k].encode(),
            param_value=new_value[k],
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    msg = the_connection.mav.param_request_list_send(
        the_connection.target_system,  # ID du système cible (1 pour le simulateur STIL)
        the_connection.target_component,  # ID du composant cible (1 pour le simulateur STIL)
    )
    """ We check if the parameters are set"""
    while len(param_name) != 0:
        message = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
        if message is not None:
            if message.param_id in param_name:
                print(str(message.param_id) + ' is correctly set')
                index = param_name.index(message.param_id)
                param_name.pop(index)
    the_connection.close()


def STIL_change_weather(W_speed, W_direction):
    """ Set Up the connection """
    the_connection = mavutil.mavlink_connection("172.29.80.1:14550")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))
    """ Set Up the list of parameter values"""
    param_name = ['SIM_WIND_SPD', 'SIM_WIND_DIR']
    new_value = [W_speed, W_direction]  #Set Up the New Values

    for k in range(len(param_name)):
        the_connection.mav.param_set_send(
            the_connection.target_system,
            the_connection.target_component,
            param_id=param_name[k].encode(),
            param_value=new_value[k],
            param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    msg = the_connection.mav.param_request_list_send(
        the_connection.target_system,  # ID du système cible (1 pour le simulateur STIL)
        the_connection.target_component,  # ID du composant cible (1 pour le simulateur STIL)
    )
    """ We check if the parameters are set"""
    while len(param_name) != 0:
        message = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
        if message is not None:
            if message.param_id in param_name:
                print(str(message.param_id) + ' is correctly set')
                index = param_name.index(message.param_id)
                param_name.pop(index)
    the_connection.close()

if __name__ == "__main__":
    W_speed = 10
    W_direction = 20
    capacity = 2*8000
    voltage = 14.8
    latitude = 1000
    longitude = 4.593647
    STIL_change_weather(W_speed, W_direction)
    STIL_set_plane(capacity, voltage)
