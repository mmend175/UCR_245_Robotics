import sys
import os


def readInputFile(filepath):

    if not os.path.isfile(filepath):
        print("File path {} does not exist. Exiting...".format(filepath))
        sys.exit()

    with open(filepath,  "r") as file:
        lines = file.readlines()
        n_lines = len(lines)
        n_obs = 0

        for i in range(n_lines):
            line = lines[i]
            line = line.lower()             # These lines of code make parsing simpler
            line = line.replace(":",  ";")   # by creating a common delimiter
            line = line.replace(".",  ";")
            line = line.replace("#", ";")
            lines[i] = line
            if line.__contains__("obstacle"):
                n_obs += 1                  # count the number obstacles for pre-allocation

        map_size = [0,  0,  0]
        obstacles = [[0,  0,  0,  0,  0,  0] for i in range(n_obs)]

        i = 0
        for line in lines:
            if line.__contains__("obstacle"):       # find obstacle lines
                vals = line.split(";")
                obstacles[i][0] = int(vals[0])
                obstacles[i][1] = int(vals[1])
                obstacles[i][2] = int(vals[2])
                obstacles[i][3] = int(vals[3])
                obstacles[i][4] = int(vals[4])
                obstacles[i][5] = int(vals[5])
                i += 1
            elif not (line.__contains__("eof") or line.__contains__("blank")):  # Find only other line with info,
                map_val = line.split(";")                                       # which contains the map size
                map_size[0] = int(map_val[0])
                map_size[1] = int(map_val[1])
                map_size[2] = int(map_val[2])

    return map_size,  obstacles


def main():
    map_size,  obs = readInputFile("sample.txt")


if __name__ == '__main__':
    main()