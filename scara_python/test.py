# This is a sample Python script.
#import HitbotInterface as hi
from HitbotInterface import HitbotInterface


# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
    hi=HitbotInterface(92); #//92 is robotid? yes
    hi.net_port_initial()
    ret=hi.initial(1,210); #// I add you on wechat
    print(hi.is_connect())
    print(hi.unlock_position())

    hi.movej_angle(0,0,0,0,70,0)
    print(ret)
    if ret == 1 :
        while True:	
            #hi.movel_xyz_by_offset(10,10,0,0,20)
            #hi.new_movej_xyz_lr(25,25,0,0,20,0,1)
            hi.get_scara_param()
            print(hi.x)
            print(hi.y)
            print(hi.z)
            print(hi.move_flag)
            rett=hi.movel_xyz(hi.x-10,hi.y-10,hi.z,0,20)
            #rett=hi.movej_xyz(hi.x-10,hi.y-10,hi.z,0,70,0)
            #rett=hi.new_movej_xyz_lr(hi.x-100,hi.y-100,hi.z,0,70,0,1)
            hi.wait_stop()
            print(rett)
            hi.get_scara_param()
            rett=hi.movel_xyz(hi.x+10,hi.y+10,hi.z,0,20)
            #rett=hi.new_movej_xyz_lr(hi.x+10,hi.y+10,hi.z,0,70,0,1)
            hi.wait_stop()
            #hi.get_scara_param()
            #hi.new_movej_xyz_lr(hi.x-10,hi.y-10,hi.z+10,0,70,0,1)
            #hi.wait_stop()

            #print(hi.x)
            #print(hi.y)
            #hi.movej_angle(0,0,0,0,20,0)
            #hi.wait_stop()
            #hi.movej_angle(0,0, -200, 0, 20, 0)
            #hi.wait_stop()




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
