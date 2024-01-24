import mysql.connector

mydb = mysql.connector.connect(
    host = "localhost",
    user = "root",
    passwd = "12345678",
    database = "TREE_MONITOR"
)

mycursor = mydb.cursor()

# Create database
# mycursor.execute("CREATE DATABASE TREE_MONITOR")

# Create table
mycursor.execute("CREATE TABLE TREE_STATE (\
                    TIME datetime, \
                    TEMPERATURE float, \
                    HUMIDITY float, \
                    constraint PK_TREESTATE primary key(TIME))")

# STR_TO_DATE('2012-11-29 18:21:11.123', '%Y-%m-%d %T.%f');
# # 2012-11-29 18:21:11.123000
#
# SELECT CONVERT(NOW(), DATETIME);

