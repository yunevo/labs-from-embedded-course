import eventlet
from flask import Flask, render_template
from flask_mqtt import Mqtt
from flask_socketio import SocketIO
from database.chart_database import get_data_in_last_xmin, write_to_dtb, get_graph_param

eventlet.monkey_patch()

app = Flask(__name__)
app.config['MQTT_BROKER_URL'] = 'mqtt.flespi.io'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = 'uaU1Kk7lPi29aQr0zvooVRHsgEW4VMadKU5k20wH8mH82WWph9C2Q9iWC4OlEOoe'
app.config['MQTT_PASSWORD'] = ''
app.config['MQTT_KEEPALIVE'] = 5
app.config['MQTT_TLS_ENABLED'] = False
topic = '/test'

minutes = 3600

mqtt = Mqtt(app)
socketio = SocketIO(app)

def update_data():
    global minutes
    dataO = get_data_in_last_xmin(minutes)
    time_param, tem_param, hum_param = get_graph_param(dataO)
    Output = dict(
        time=time_param,
        temp=tem_param,
        hum=hum_param
    )
    socketio.emit('mqtt_message', data=Output)
    socketio.sleep(1)

@socketio.on('connect')
def connect():
    update_data()

@mqtt.on_connect()
def handle_connect(client, userdata, flags, rc):
    mqtt.subscribe(topic)

@mqtt.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    # emit a mqtt_message event to the socket containing the message data
    write_to_dtb(data['payload'][0:2], data['payload'][3:5])
    update_data()

@app.route('/')
def index():
    return render_template('graph.html')

if __name__ == '__main__':
    socketio.run(app, host='127.0.0.1', port=5000, use_reloader=False, debug=True)