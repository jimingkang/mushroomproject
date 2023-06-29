import subprocess

from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt


# import config

busnum = 1  # Edit busnum to 0, if you uses Raspberry Pi 1 or 0


app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = '10.0.0.18'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = ''  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 5  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
topic = '/flask/mqtt'

mqtt_client = Mqtt(app)


# app.config.from_object(config)




@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Connected successfully')
        mqtt_client.subscribe(topic)  # subscribe topic
    else:
        print('Bad connection. Code:', rc)


@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    subprocess.call("python ./tools/demo.py image -f ./exps/example/yolox_voc/yolox_voc_s.py -c ./yolox_x_mushroom.pth --path ./muhsroom.jpg --conf 0.25 --nms 0.45 --tsize 640 --save_result --device 0")
    print('Received message on topic: {topic} with payload: {payload}'.format(**data))


@app.route('/publish', methods=['POST'])
def publish_message():
    request_data = request.get_json()
    publish_result = mqtt_client.publish(request_data['topic'], request_data['msg'])
    return jsonify({'code': publish_result[0]})


if __name__ == '__main__':
    app.run()
