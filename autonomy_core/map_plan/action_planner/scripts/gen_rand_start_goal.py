#!/usr/bin/env python3
from random import randrange, seed, uniform
import csv
import pandas as pd

for map_id in range(2):
    data = []
    if map_id == 0:
        rand_seed = 123
    else:
        rand_seed = 237
        
    for i in range(rand_seed,rand_seed+50):
        seed(i)
        xi = uniform(0,2)
        yi = uniform(1,10)
        zi = 0.5
        xf = uniform(18,20)
        yf = uniform(1,10)
        zf = 0.5
        vxi = uniform(-1,1)
        vyi = uniform(-1,1)
        vzi = 0.0
        vxf = uniform(-1,1)
        vyf = uniform(-1,1)
        vzf = 0.0
        data.append([xi,yi,zi,xf,yf,zf,vxi,vyi,vzi,vxf,vyf,vzf])
    df = pd.DataFrame(data, columns=['xi','yi','zi','xf','yf','zf','vxi','vyi','vzi','vxf','vyf','vzf'])
    if map_id == 0:
        map_name = "balls"
    else:
        map_name = "forks"
    df.to_csv('map_'+map_name+'_start_goal.csv', index=True, header=True)

