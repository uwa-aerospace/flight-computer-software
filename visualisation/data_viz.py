import csv
import matplotlib.pyplot as plt
import numpy as np

gyrf = open("outputs/gyro.csv", 'w+')
flight = open("outputs/full-flight.csv", "w+")


def convert_alti(p0, pt):
    """
    Manual calculation of the height about ground level using pressure in hPa.
    :param p0: Initial Pressure Reading
    :param pt: Instantaneous Pressure Reading
    :return:   The height above ground level (p0) in metres.
    """
    dif = (pt * 10) / (p0 * 10)
    return 44330 * (1 - pow(dif, 0.1903))

# Splits the gyro rows and the data rows
gyro = []
data = []

with open("./examples/9718.TXT", "r") as file:
    csread = csv.reader(file)


    gy_write = csv.writer(gyrf)
    dat_write = csv.writer(flight)
    for row in csread:
        if len(row) > 0:
            if "GYRO" in row[0]:
                gyro.append(row)
                gy_write.writerow(row)
            else:
                row[2] = round(convert_alti(99, float(row[4])), 2)
                data.append(row)
                dat_write.writerow(row)

gyrf.close()
flight.close()

time = list(map(lambda row: int(row[0]), data))
altitude = list(map(lambda row: float(row[2]), data))

plt.plot(range(len(altitude)), altitude)
plt.title("Competition Launch Profile")
plt.ylabel("Altitude (m)")
plt.savefig("outputs/altitude-time.png", bbox_inches="tight")
plt.show()
