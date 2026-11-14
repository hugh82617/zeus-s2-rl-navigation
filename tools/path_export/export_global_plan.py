import pandas as pd
import re

g = pd.read_csv(
r'navigation_research_20260622_181156\move_base-DWAPlannerROS-global_plan.csv'
)

l = pd.read_csv(
r'navigation_research_20260622_181156\move_base-DWAPlannerROS-local_plan.csv'
)

global_num = len(re.findall("position:", g['poses'][0]))
local_num  = len(re.findall("position:", l['poses'][0]))

print("Global Pose Num =", global_num)
print("Local Pose Num =", local_num)