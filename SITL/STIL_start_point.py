from distance_calculation import convert_coordinates_xy2ll

def sp_input(latitude, longitude, altitude, heading):
    file_input = f"Fuego={latitude},{longitude},{altitude},{heading}"
    print(f"Fuego={latitude},{longitude},{altitude},{heading}")
    return file_input

def replace_last_line(file_path, replacement_line):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    lines[-1] = replacement_line + '\n'
    with open(file_path, 'w') as file:
        file.writelines(lines)

def set_home(latitude, longitude, altitude, heading):
    file_input = f"Fuego={latitude},{longitude},{altitude},{heading}"
    replace_last_line(r"\\wsl.localhost\Ubuntu\home\edouard\ardupilot\Tools\autotest\locations.txt", file_input)


if __name__ == '__main__':

    x_plane = -2000
    y_plane = -0
    latitude, longitude = convert_coordinates_xy2ll(14.4747, -90.8806, x_plane, y_plane)
    altitude  = 1131
    heading   = 138
    last_line = sp_input(14.4322662,-90.9349201,altitude,heading)
    replace_last_line(r"\\wsl.localhost\Ubuntu\home\edouard\ardupilot\Tools\autotest\locations.txt",last_line)

