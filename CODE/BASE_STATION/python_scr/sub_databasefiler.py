import paho.mqtt.client as mqtt
import mysql.connector
import os
from dotenv import load_dotenv
import datetime

load_dotenv()

# MySQL database connection
mydb = mysql.connector.connect(
    host=os.getenv("HOST"),
    user=os.getenv("USER"),
    password=os.getenv("PASSWORD"),
    database=os.getenv("DATABASE"),
    auth_plugin=os.getenv("AUTH_PLUGIN")
)

# MQTT Broker settings
broker_address = "diplom-ny7qjv.a01.euc1.aws.hivemq.cloud"
port = 8883
topic = "SKYSAFENET/flightdata/rec"  
username = "martin"
password = "Martin363"

def insert_data_into_database(latitude, longitude, altitude, velocity, heading):
    mysql_query = "INSERT INTO FLIGHT_STATS (timestamp, latitude, longitude, altitude, velocity, heading) VALUES (%s, %s, %s, %s, %s, %s)"
    data = (datetime.datetime.now(), latitude, longitude, altitude, velocity, heading)
    cursor = mydb.cursor()
    cursor.execute(mysql_query, data)
    mydb.commit()
    cursor.close()
    print("Data inserted into database successfully.")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topic)

def on_message(client, userdata, msg):
    print("Data received from hc12")
    print("Received message: " + msg.topic + " " + str(msg.payload))
    try:
        payload_str = msg.payload.decode("utf-8")
        print("Payload string: ", payload_str)  # Debugging: print the raw payload string
        data = payload_str.split(", ")  # Split the payload by comma and space
        if len(data) != 5:
            raise ValueError(f"Unexpected data format: {data}")  # Check if we have exactly 5 elements

        latitude = float(data[0].split(": ")[1])  # Extract latitude from payload
        longitude = float(data[1].split(": ")[1])  # Extract longitude from payload
        altitude = float(data[2].split(": ")[1])  # Extract altitude from payload
        velocity_str = data[3].split(": ")[1]     # Extract velocity string from payload
        velocity = float(velocity_str.split(":")[0])  # Extract velocity value without colon
        heading = float(data[4].split(": ")[1])  # Extract heading from payload

        # Print each element
        print("Latitude: ", latitude)
        print("Longitude: ", longitude)
        print("Altitude: ", altitude)
        print("Velocity: ", velocity)
        print("Heading: ", heading)

        insert_data_into_database(latitude, longitude, altitude, velocity, heading)

    except Exception as e:
        print("Error:", e)

client = mqtt.Client()
client.username_pw_set(username, password)
client.tls_set()
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, port, 60)

client.loop_forever()
