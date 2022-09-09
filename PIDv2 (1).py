from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())
base_speed = 5

sensor = []
sensor_names = ["sensorLeft", "sensorMid", "sensorRight"]
sensor_value = [0]*len(sensor_names)

for name in sensor_names:
    sensor.append(robot.getDevice(name))
    sensor[-1].enable(timestep)

motors = []
motor_names = ["FR_wheel", "FL_wheel", "BR_wheel", "BL_wheel"]

for name in motor_names:
    motors.append(robot.getDevice(name))
    motors[-1].setPosition(float('inf'))
    motors[-1].setVelocity(0.0)

end_error = prop = integ = dif = err = 0
kp = 0.05
ki = 0
kd = 0.2
l_speed = 0
r_speed = 0

def pid(er):
    global end_error, prop, integ, dif, kp, ki, kd
    prop = er
    integ = er + integ
    dif = er - end_error
    norm = (kp*prop) + (ki*integ) + (kd*dif)
    end_error = er
    return norm

def assign_speed(base, pid_adjust):
    global l_speed, r_speed
    l_speed = base - pid_adjust
    r_speed = base + pid_adjust

while robot.step(timestep) != -1:
    for x in range(len(sensor)):

        sensor_value[x] = sensor[x].getValue()
        print(f"{sensor_names[x]} : {sensor_value[x]}\n" + "*"*40)

        if sensor_value[0] < 450 and sensor_value[1] >= 450 and sensor_value[2] < 450:
            err = 0

        elif sensor_value[0] < 450 and sensor_value[1] >= 450 and sensor_value[2] >= 450:
            err = -1
        
        elif sensor_value[0] >= 450 and sensor_value[1] >= 450 and sensor_value[2] < 450:
            err = 1

        elif sensor_value[0] >= 450 and sensor_value[1] < 450 and sensor_value[2] < 450:
            err = 2

        elif sensor_value[0] < 450 and sensor_value[1] < 450 and sensor_value[2] >= 450:
            err = -2

        limiter = pid(err)
        assign_speed(base_speed, limiter)

        if l_speed > base_speed:
            motors[1].setVelocity(l_speed)
            motors[3].setVelocity(l_speed)
            motors[0].setVelocity(0)
            motors[2].setVelocity(0)
        
        if l_speed < 0:
            motors[1].setVelocity(0)
            motors[3].setVelocity(0)
            motors[0].setVelocity(r_speed)
            motors[2].setVelocity(r_speed)

        if r_speed > base_speed:
            motors[1].setVelocity(0)
            motors[3].setVelocity(0)
            motors[0].setVelocity(r_speed)
            motors[2].setVelocity(r_speed)

        if r_speed < 0:
            motors[1].setVelocity(l_speed)
            motors[3].setVelocity(l_speed)
            motors[0].setVelocity(0)
            motors[2].setVelocity(0)

        if l_speed == r_speed:
            motors[1].setVelocity(l_speed)
            motors[3].setVelocity(l_speed)
            motors[0].setVelocity(r_speed)
            motors[2].setVelocity(r_speed)
        
        pass
