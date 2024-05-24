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
    hi.get_scara_param()
    print(hi.x,hi.y)
    ret=hi.movel_xyz(hi.x+30,hi.y,hi.z-50,25,20)
    hi.wait_stop()
    #ret=hi.movej_angle(10,0,0,0,20,0)
    #hi.wait_stop()
    print(ret)
    i=10
    while i<10:
        i=i+1
        hi.get_scara_param()
        print(hi.x,hi.y)
        rett=hi.movel_xyz(hi.x,hi.y-50,hi.z-100,25,20)
        print(rett)
        hi.wait_stop()
        hi.get_scara_param()
        print(hi.x,hi.y)
        hi.movel_xyz(hi.x,hi.y+50,0,25,20)
        hi.wait_stop()




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
