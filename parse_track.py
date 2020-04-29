import scipy.io as sio

X_INDEX = 2
Y_INDEX = 3

if __name__ == "__main__":
    track_file = open("track_data_unsorted.txt","r")
    
    isLeft = True
    isTK = False
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    tk_x = []
    tk_y = []
    for line in track_file:
        if "cones" in line:
            if "cones_left" in line:
                isLeft = True
            if "cones_right" in line:
                isLeft = False
        elif "tk_device" in line:
            isTK = True
        else:
            if "- - " in line:
                split_line = line.split(" ")
                if(isLeft):
                    left_x.append(float(split_line[X_INDEX]))
                elif(isTK):
                    tk_x.append(float(split_line[X_INDEX]))
                else:
                    right_x.append(float(split_line[X_INDEX]))
            else:
                split_line = line.split(" ")
                if(isLeft):
                    left_y.append(float(split_line[Y_INDEX]))
                elif(isTK):
                    tk_y.append(float(split_line[Y_INDEX]))
                else:
                    right_y.append(float(split_line[Y_INDEX]))
    mdict = {"left_x":left_x,"left_y":left_y,"right_x":right_x,"right_y":right_y,"tk_x":tk_x,"tk_y":tk_y}
    sio.savemat("track_data.mat", mdict)
