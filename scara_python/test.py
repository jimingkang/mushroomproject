# This is a sample Python script.
#import HitbotInterface as hi
from HitbotInterface import HitbotInterface


# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import redis

redis_server='192.168.0.100'
pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True,password='jimmy')
r = redis.Redis(connection_pool=pool)

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
    hi=HitbotInterface(92); #//92 is robotid? yes
    hi.net_port_initial()
    ret=hi.initial(1,210); #// I add you on wechat
    print(hi.is_connect())
    print(hi.unlock_position())
    hi.movej_angle(0,0,0,25,20,0)
    print(ret)
    while 1 :
        all=r.hgetall("detections")
        for k,v in all.items():
            hi.get_scara_param()
            r.set("global_camera_xy",str(hi.x)+","+str(hi.y))
            r.set("mode","pickup_ready")
            xyz=v.split(",")
            hi.movel_xyz(float(xyz[0]),float(xyz[1]),0,25,20)
            hi.wait_stop()
            r.delete("detections",k)
            hi.get_scara_param()
            r.set("global_camera_xy",str(hi.x)+","+str(hi.y))

            #hi.new_movej_xyz_lr(hi.x-10,hi.y-10,hi.z+10,0,70,0,1)
            #hi.wait_stop()
            r.set("mode","camera_ready")




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
