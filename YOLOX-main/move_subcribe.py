# python3.6

import random
import subprocess
import time
from os import write

from paho.mqtt import client as mqtt_client
import fcntl
import xyz_publish


broker=''
try:
    for line in open("ip.txt"):
        if line[0:6] == "broker":
            broker = line[9:len(line)]
except:
    pass
print(broker)
#broker = '10.0.0.134'
broker = '192.168.254.42'
port = 1883
topic = "/flask/downmove"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'




def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(" move_sub Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    msg= None
    def on_message(client, userdata, msg):
        print(msg.topic + " " + str(msg.payload) + " mid:" + str(msg.mid))
        val = msg.payload.decode()
        #subprocess.call("C:/Users/jkang7/Downloads/uh/project/MUSHROOM/YOLOX-main/venv/Scripts/python.exe demo_ip.py webcam -f ./exps/example/yolox_voc/yolox_voc_s.py --trt --conf 0.25 --nms 0.45 --tsize 640 --save_result ")
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        file_lock = open("ret.txt", "w")
        try:
            fcntl.flock(file_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            print("File is locked")
            # Do something with the file .
            file_lock.write(val)
        except IOError:
            print("File is already locked by another process")

        finally:
            # Unlock the file (fcntl.F_UNLOCK)
            fcntl.flock(file_lock.fileno(), fcntl.LOCK_UN)
            print("File is unlocked")
            file_lock.close()

        #xyz_publish.run(coordx)
    client.subscribe(topic,qos=1)
    client.on_message = on_message



def run():

    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()





