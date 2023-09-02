import numpy as np
from sklearn.linear_model import LinearRegression
import pandas as pd

def extract_columns(input_file, sheet_name, columns_to_extract):

    # Charger le fichier XLSX dans un DataFrame pandas
    df = pd.read_excel(input_file, sheet_name=sheet_name)
    print(df.columns)

    # Extraire les colonnes spécifiées
    extracted_data = df[columns_to_extract]

    return extracted_data


columns_to_extract = ['Error', 'Time_Waypoint']

datas = extract_columns(r"C:\Users\edoua\Desktop\Wind Effect\erreurs\10.xlsx", 'Sheet1', columns_to_extract)



# Convertir les listes en tableaux numpy
x = datas['Time_Waypoint']
y = datas['Error']

x = x.values.reshape(-1, 1)
y = y.values

# Créer le modèle de régression linéaire
regression = LinearRegression()

# Entraîner le modèle sur les données
regression.fit(x, y)


# Afficher les coefficients du modèle
coefficients = regression.coef_[0]
intercept = regression.intercept_
print("Coefficient (pente) : ", coefficients)
print("Intercept : ", intercept)

coefficient_r2 = regression.score(x, y)
print("Coefficient de corrélation R^2 : ", coefficient_r2)


