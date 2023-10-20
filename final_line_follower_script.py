import sim
import numpy as np
import colorsys
import matplotlib.pyplot as plt

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)


# FUNCTIONS TO INTERFACE WITH THE ROBOT
def set_speed(speed_l, speed_r):
    velocities = sim.simxPackFloats([speed_l, speed_r])
    sim.simxSetStringSignal(clientID=clientID, signalName="motors", signalValue=velocities,
                            operationMode=sim.simx_opmode_blocking)


def get_image_sensor():
    """
    Returns the image of the sensor
    :return: the image of the sensor
    """
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="Sensors",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


def image_correction(image, res):
    """
    This function can be applied to images coming directly out of CoppeliaSim.
    It turns the 1-dimensional array into a more useful res*res*3 array, with the first
    two dimensions corresponding to the coordinates of a pixel and the third dimension to the
    RGB values. Aspect ratio of the image is assumed to be square (1x1).

    :param image: the image as a 1D array
    :param res: the resolution of the image, e.g. 64
    :return: an array of shape res*res*3
    """

    image = [int(x * 255) for x in image]
    image = np.array(image).reshape((res, res, 3))
    image = np.flip(m=image, axis=0)
    return image


def show_image(image):
    """
    Prints the image
    :param image: the image to be printed
    :return:
    """
    plt.imshow(image)
    plt.show()


def average(image):
    """
    Calculates the average RGB values from a given image
    :param image: the image from the sensor
    :return: the average RGB values of the image
    """
    total_r = 0
    total_g = 0
    total_b = 0
    for col in range(len(image)):
        for row in range(len(image[col])):
            total_r += image[col][row][0]
            total_g += image[col][row][1]
            total_b += image[col][row][2]
    total_r = total_r / (len(image) * len(image[0]))
    total_g = total_g / (len(image) * len(image[0]))
    total_b = total_b / (len(image) * len(image[0]))
    return [total_r, total_g, total_b]


def determine_color(rgb_list):
    """
    Determines the color that given RGB values represent
    :param rgb_list: the list with the RGB values
    :return: the color represented
    """
    r = rgb_list[0]
    g = rgb_list[1]
    b = rgb_list[2]
    if (r < 35) and (g < 35) and (b < 35):
        return "bk"
    if (r > 180) and (g > 180) and (b > 180):
        return "w"
    if (r > 200) and (g < 40) and (b < 40):
        return "r"
    if (r < 40) and (g > 200) and (b < 40):
        return "g"
    if (r < 40) and (g < 40) and (b > 200):
        return "bu"


def calculate_error(ratio, index):
    """
    Reverses the error (1 - error) for the left side
    :param ratio: the ration of the error (between 0 and 1)
    :param index: 1 for left side, 2 for right side
    :return: the correct ratio for either side
    """
    if index == 1:
        return ratio
    else:
        return 1 - ratio


def black_pixels(image, index):
    """
    Returns a ration between 0 and 1, representing the percentage a given half of the sensor consists of black pixels
    :param image: the image from the sensor
    :param index: 1 for the left side, 2 for the right side
    :return: the ratio of black pixels in the given half (between 0 and 1)
    """
    start_col = 0
    end_col = 8
    counter = 0
    if index == 2:
        start_col = 8
        end_col = 16
    for col in range(start_col, end_col):
        for row in range(16):
            rgb_list = [image[row][col][0], image[row][col][1], image[row][col][2]]
            if determine_color(rgb_list) == "bk":
                counter = counter + 1
    ratio = counter / (8 * 16)
    return ratio


# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    # PID properties
    last_error = 0
    Kp = 0.95
    Kd = 0.25
    Ki = 0.025
    integral = []
    while not (determine_color(average(get_image_sensor())) == "bu"):
        if determine_color(average(get_image_sensor())) == "r":
            set_speed(0.5, 0.5)
        else:
            right_pixels = calculate_error(black_pixels(get_image_sensor(), 2), 2)
            left_pixels = calculate_error(black_pixels(get_image_sensor(), 1), 1)

            error = left_pixels - right_pixels

            if len(integral) > 5:
                integral.pop()

            adjustment = Kp * error + Kd * (error - last_error) + Ki * sum(integral)

            last_error = error
            integral.insert(0, error)
            set_speed(0.6 - adjustment, 0.6 + adjustment)

    set_speed(0, 0)  # for when the robot gets on the goal location
    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
