from flask import Flask, render_template
import datetime
import mysql.connector


mydb = mysql.connector.connect(
    host = "localhost",
    user = "root",
    passwd = "12345678",
    database = "TREE_MONITOR"
)
# Get cursor
mycursor = mydb.cursor()

# Insert data
def write_to_dtb(temperature, humidity):
    time_point = datetime.datetime.now()
    mycursor.execute("INSERT INTO TREE_STATE VALUES (%s, %s, %s)", (time_point, temperature, humidity))
    mydb.commit()

# Select data
def get_data_to_graph():
    mycursor.execute("SELECT * FROM TREE_STATE ORDER BY TIME DESC LIMIT 10")
    results = mycursor.fetchall()
    return results

# Get data from a point of time
def get_data_from_time_point(str_time_point):
    time_point = datetime.datetime.strptime(str_time_point, '%d-%m-%Y %H:%M:%S')
    mycursor.execute("SELECT * FROM TREE_STATE WHERE TIME >= %s ORDER BY TIME DESC LIMIT 40", (time_point,))
    results = mycursor.fetchall()
    return results


# Get data stored in the last 10 minutes
def get_data_in_last_xmin(minutes):
    now = datetime.datetime.now()
    ten_min = datetime.timedelta(minutes=minutes)
    start_point = now - ten_min
    str_time_point = start_point.strftime('%d-%m-%Y %H:%M:%S')
    data = get_data_from_time_point(str_time_point)
    return data


# Convert data into parameters to draw graph
def get_graph_param(data):
    time_param = []
    tem_param = []
    hum_param = []
    t = 0
    prev_time = 0
    for idx, row in enumerate(data):
        if idx == 0:
            time_param.append(t)
            prev_time = row[0]
        else:
            t = t - (prev_time - row[0]).total_seconds()
            time_param.append(t)
            prev_time = row[0]
        tem_param.append(row[1])
        hum_param.append(row[2])
    time_param = time_param[::-1]
    tem_param = tem_param[::-1]
    hum_param = hum_param[::-1]
    return time_param, tem_param, hum_param


# Delete data before a time point
def delete_data_before_time_point(time_point):
    time_point = datetime.datetime.strptime(time_point, '%d-%m-%Y %H:%M:%S')
    mycursor.execute("DELETE FROM TREE_STATE WHERE TIME < %s", (time_point,))
    mydb.commit()


# Delete all data
def delete_all_data():
    mycursor.execute("DELETE FROM TREE_STATE")
    mydb.commit()


# app = Flask(__name__)
# delete_all_data()
# data = get_data_from_time_point('12-6-2023 0:0:0')
# print(data)
# @app.route('/')
# def graph():
#     data = get_data_in_last_10min()
#     time_param, tem_param, hum_param = get_graph_param(data)
#     return render_template("chart.html",time_param=time_param, tem_param=tem_param, hum_param=hum_param)
    # return render_template("chart.html")

# if __name__ == '__main__':
#     app.run()