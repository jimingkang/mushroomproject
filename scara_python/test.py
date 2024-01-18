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

    print(ret)
    if ret == 1 :
        while True:
            hi.movej_angle(0,0,0,0,20,0)
            hi.wait_stop()
            hi.movej_angle(0,0, -200, 0, 20, 0)
            hi.wait_stop()




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
