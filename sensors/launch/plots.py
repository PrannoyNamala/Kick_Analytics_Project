import pandas as pd
import matplotlib.pyplot as plt

temp = pd.read_csv("temp_save.csv")
speed = pd.read_csv("speed_save.csv")
laser = pd.read_csv("laser_save.csv")

temp.plot(kind='line')
speed.plot(kind='line')
laser.plot(kind='line')
