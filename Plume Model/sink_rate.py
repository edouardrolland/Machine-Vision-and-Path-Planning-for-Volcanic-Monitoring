import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn import linear_model
import pandas as pd
import csv
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.preprocessing import PolynomialFeatures
from sklearn import tree
from sklearn.tree import DecisionTreeRegressor
import numpy as np
from sklearn.model_selection import KFold
from IPython.display import display, Math           #To display LaTex Elements in the Jupyter Notebook
from IPython.display import set_matplotlib_formats  #To save figures as svg

# Chemin vers le fichier CSV
csv_file = r"C:\Users\edoua\Documents\Birse\Bristol\MSc Thesis\drone_volcanic_monitoring\Drone_Volcanic_Monitoring\Plume Model\statistics.csv"

# Charger le fichier CSV dans un DataFrame
df = pd.read_csv(csv_file, sep=';')

#Flight_Name = df['Folder Name']
Altitude = df['Altitude']
Distance = df['Distance']
Wind = df['Wind']

# Créer la figure et les axes 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Tracer le graphique en 3D
ax.scatter(Distance, Wind, Altitude, c='r', marker='o', s =30)

# Ajouter des étiquettes d'axe
ax.set_xlabel('Distance to the summit (m)', fontsize = 15)
ax.set_ylabel('Wind ($m.s^{-1}$)', fontsize = 15)
ax.set_zlabel('Altitude of interception (m)', fontsize = 15)

# Afficher le graphique
plt.show()


X = df[['Distance', 'Wind']]
y = df['Altitude']

X = np.array(X)
y = np.array(y)

Xtrain, Xtest, Ytrain, Ytest = train_test_split(X, y, test_size=0.2,shuffle=True, random_state=10)


# Set up a k-fold cross-validation with 10 folds
kf = KFold(n_splits=10, random_state=98, shuffle=True) #the datas are shuffled before the split in order to avoid biais.

max_deg = 4 #define the maximum degree of the polynomial that can be tested

mean_trn_mses = [] #Initialise two variables to store the mean squared errors of the training and validation
mean_val_mses = []

# Loop over degrees of the polynomial
for d in range(max_deg):
    mse_tr=[]  #Initialise two list to store the mean squared errors for one degree of the polynomial
    mse_val=[]
    poly=PolynomialFeatures(degree=d+1)                      #Instantiate a polynomial with the current value of d
    for train_index, val_index in kf.split(Xtrain):          #Loop over cross-validation splits
        Xtrn, Xval = Xtrain[train_index], Xtrain[val_index]
        ytrn, yval = Ytrain[train_index], Ytrain[val_index]
        Xtrain_new = poly.fit_transform(Xtrn)                # fit the model on the current split of data
        Xval_new = poly.fit_transform(Xval)
        regr = LinearRegression()
        regr.fit(Xtrain_new, ytrn)
        pred_tr = regr.predict(Xtrain_new)                   # predict for the training set
        pred_v = regr.predict(Xval_new)                      # predict for the validation set
        mse_tr.append(mean_squared_error(pred_tr, ytrn))     # storage of mses for each fold
        mse_val.append(mean_squared_error(pred_v, yval))
    mean_trn_mses.append(np.mean(mse_tr))                    # storage of mean mses
    mean_val_mses.append(np.mean(mse_val))


fig_poly=plt.figure()
x = list(range(1,max_deg+1)) # X axis list for the plot
plt.plot(x,mean_trn_mses, label = 'Training MSE')
plt.plot(x,mean_val_mses, label = 'Validation MSE')
plt.axvline(0, linestyle='dashed', color='red')
plt.text(6 + 0.1, 18, 'Optimal degree', rotation='vertical')
plt.xlabel('polynom degree')
plt.xticks(x)
plt.ylabel('mean squared error')
plt.yscale('log')
plt.grid()
plt.legend()
plt.savefig("polynomial_regression.svg", format="svg")
plt.show()