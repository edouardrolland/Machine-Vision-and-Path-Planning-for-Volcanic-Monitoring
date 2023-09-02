import numpy as np
from sklearn.linear_model import LinearRegression
import pandas as pd

#Program to find the correction function. 

def extract_columns(input_file, sheet_name, columns_to_extract):
    df = pd.read_excel(input_file, sheet_name=sheet_name)
    print(df.columns)
    extracted_data = df[columns_to_extract]
    return extracted_data

columns_to_extract = ['Error', 'Time_Waypoint']
datas = extract_columns(r"Path to your simulation data", 'Sheet1', columns_to_extract)

x = datas['Time_Waypoint']
y = datas['Error']
x = x.values.reshape(-1, 1)
y = y.values


regression = LinearRegression()
regression.fit(x, y)

coefficients = regression.coef_[0]
intercept = regression.intercept_
print("Coefficient (pente) : ", coefficients)
print("Intercept : ", intercept)

coefficient_r2 = regression.score(x, y)
print("Coefficient de corrélation R^2 : ", coefficient_r2)

