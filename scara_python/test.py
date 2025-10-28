import threading
import pandas as pd
import time

df = pd.DataFrame()

    

# This is a sample Python script.
#import HitbotInterface as hi
from HitbotInterface import HitbotInterface
stop_event = threading.Event()
hi = HitbotInterface(92);  # //92 is robotid? yes
hi.net_port_initial()
ret = hi.initial(1, 210);  # // I add you on wechat
hi.is_connect()
hi.is_collision()
hi.unlock_position()
#ret=hi.movel_xyz(500,0,hi.z,hi.r-160,50)
#hi.wait_stop()
ret=hi.movel_xyz(600,0,0,0,50)
#ret=hi.movel_xyz(hi.x,hi.y,hi.z-10,hi.r,50)
hi.wait_stop()
#ret=hi.movel_xyz(300,-200,0,-20,50)
#hi.wait_stop()
#ret=hi.movej_angle(52,-58,0,-55-20,30,1)  #0.6,-0.001
#ret=hi.movej_angle(5,-44,0,-116+20,30,1)  #0.6,0.001
#ret=hi.movej_angle(-37,50,0,-107+20,30,1)  #0.5,-0.4
#ret=hi.movej_angle(-23,59,0,58-20,30,1) #0.5,-0.3
#ret=hi.movej_angle(-18,50,0,45+20,30,1) #0.5,-0.3 
#ret=hi.movej_angle(-8.9,51,0,52+20,30,1) #0.5,-0.4
#ret=hi.movej_angle(7,-48,0,-54-20,30,1) #0.5,0.4
#ret=hi.movej_angle(20,-55,0,-62-20,30,1) #0.5,0.3
#ret=hi.movej_angle(-14,31,0,-118-20,30,1) #0.5,0.3
#hi.wait_stop()
ret=hi.movej_angle(40,50,0,0,70,1)
hi.wait_stop()
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

 

#import redis

 

#redis_server='192.168.0.100'
#pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
#r = redis.Redis(connection_pool=pool)

 

def move():

    #ret=hi.movel_xyz(300,0,0,0,100)
    #ret=hi.movej_angle(60,0,0,-63,100,1)
    #hi.wait_stop()
    #ret=hi.movej_angle(60,60,0,-63,100,1)
    #hi.wait_stop()
    #ret=hi.movej_angle(60,60,0,0,100,1)
    #hi.wait_stop()
   
    stop_event.set()


def printstatus():
    pre = time.time()
    cur = time.time()
    global df
    print("time-diff,x,y,z,r,angle1,angle2")
    while not stop_event.is_set():
        cur = time.time()
        hi.get_scara_param()
        data = {
            "time":[cur-pre], 
            "x":   [hi.x], 
            "y":  [hi.y],
            "z":   [hi.z], 
            "r":   [hi.r],
            "angle1":  [hi.angle1],  
            "angle2":  [hi.angle2],     
        }
        df_new_rows = pd.DataFrame(data)

        # Call Pandas.concat to create a new DataFrame from several existing ones
        df = pd.concat([df, df_new_rows])
        #hi.wait_stop()
        #print("{},{},{},{},{},{}".format(cur-pre,)
        time.sleep(0.02)
        pre=cur
    df.to_csv("data.csv",sep='\t', encoding='utf-8', index=False, header=True)

 

 

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    watcher_thread = threading.Thread(target=move)
    move_thread = threading.Thread(target=printstatus)
    move_thread.start()
    watcher_thread.start()
    move_thread.join()
    watcher_thread.join()

 
