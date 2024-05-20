from flask import Flask, render_template, request, send_file
import mysql.connector
import os
from dotenv import load_dotenv
import matplotlib.pyplot as plt
import io
import datetime

load_dotenv()

app = Flask(__name__)

# Database configuration using environment variables
db_config = {
    'host': os.getenv("HOST"),
    'user': os.getenv("USER"),
    'password': os.getenv("PASSWORD"),
    'database': os.getenv("DATABASE"),
    'auth_plugin': os.getenv("AUTH_PLUGIN")
}

def get_flight_stats(limit):
    connection = mysql.connector.connect(**db_config)
    cursor = connection.cursor()
    query = f"SELECT timestamp, latitude, longitude, altitude, velocity, heading FROM FLIGHT_STATS ORDER BY id DESC LIMIT {limit}"
    cursor.execute(query)
    result = cursor.fetchall()
    cursor.close()
    connection.close()
    return result

def get_graph_data():
    connection = mysql.connector.connect(**db_config)
    cursor = connection.cursor()
    query = "SELECT timestamp, altitude, velocity FROM FLIGHT_STATS ORDER BY id DESC LIMIT 50"
    cursor.execute(query)
    result = cursor.fetchall()
    cursor.close()
    connection.close()
    return result

@app.route('/', methods=['GET', 'POST'])
def index():
    limit = 50  # Default limit
    if request.method == 'POST':
        limit = int(request.form.get('limit'))
    
    flight_stats = get_flight_stats(limit)
    return render_template('index.html', flight_stats=flight_stats, limit=limit)

@app.route('/graph')
def graph():
    return render_template('graph.html')

@app.route('/graph_image')
def graph_image():
    graph_data = get_graph_data()
    timestamps = [row[0] for row in graph_data]
    altitudes = [row[1] for row in graph_data]
    velocities = [row[2] for row in graph_data]
    
    fig, ax1 = plt.subplots()

    color = 'tab:blue'
    ax1.set_xlabel('Timestamp')
    ax1.set_ylabel('Altitude', color=color)
    ax1.plot(timestamps, altitudes, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  
    color = 'tab:red'
    ax2.set_ylabel('Velocity', color=color)  
    ax2.plot(timestamps, velocities, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  

    buf = io.BytesIO()
    plt.xticks(rotation=45)
    plt.savefig(buf, format='png')
    buf.seek(0)
    plt.close(fig)
    return send_file(buf, mimetype='image/png')

if __name__ == '__main__':
    app.run(debug=True)
