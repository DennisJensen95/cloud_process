import os
import re

def highest_num_file(files):
    high_num = 0
    for file in files:
        if file != 'frame.jpg':
            num = re.findall('\d+', file)
            if len(num) > 0:
                num = int(num[0])
                if high_num < num:
                    high_num = num
    return high_num

def change_name():
    files = os.listdir('./Images')
    num = highest_num_file(files)
    print(num)
    for file in files:
        if file == 'frame.jpg':
            num = num + 1
            os.rename('./Images/' + file, f'./Images/frame_{num}.jpg')