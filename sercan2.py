Your code starts here:
while robot.is_ok():
    sensor_row = robot.get_sensor_data()

    max = 0
    index = 0
    for i in range(0, len(sensor_row)):
        if sensor_row[i] > max:
            max = sensor_row[i]
            index = i
    
    error = (len(sensor_row) / 2) - index
    maxHiz = 2
    birimHiz = maxHiz / (len(sensor_row)/2)
    yavasla = birimHiz * math.fabs(error)
    gain = 0.1
    command = gain * error
    print("Birim Hız: {}".format(birimHiz))
    robot.move(maxHiz-yavasla)
    robot.rotate(command)
