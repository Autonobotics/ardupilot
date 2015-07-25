import os
import re
import time
import datetime

script_dir = os.path.dirname(__file__)
log_file_dir = r"C:/Program Files (x86)/Mission Planner/MissionPlanner.log"
log_file =  open(log_file_dir)
current_time = datetime.datetime.now().strftime('Y%YM%mD%d-T%H-%M-%S')
LOG = log_file.readlines()
#log_file.close()
new_log_file = open(script_dir + "/AGD_MissionPlanner"+current_time+".log",'w')
for line in LOG:
    log_line = re.match('(.*),(.*?)-(.*?)<AGD>(.*?)<AGD>', line, re.DOTALL)
    if log_line:
        #print(log_line.group())
        new_log_file.write(log_line.group(1) + "    ")
        new_log_file.write(log_line.group(2) + "    ")
        new_log_file.write(log_line.group(4) + "\n")

while 1:
    line = log_file.readline()
    log_line = re.match('(.*),(.*?)-(.*?)<AGD>(.*?)<AGD>', line, re.DOTALL)
    if not log_line:
        #log_file.seek(0,2)
        continue
    else:
        new_log_file.write(log_line.group(1) + "    ")
        new_log_file.write(log_line.group(2) + "    ")
        new_log_file.write(log_line.group(4) + "\n")
        print log_line.group(1) + "    " +log_line.group(2) + "    "+log_line.group(4)