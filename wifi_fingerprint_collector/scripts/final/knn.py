import pandas as pd
from sklearn.neighbors import KNeighborsClassifier

# Charger les données depuis le fichier CSV
data = pd.read_csv('wifi_data.csv')

# Séparer les caractéristiques (RSSI) et les labels (positions x, y)
X = data[['s1', 's2', 's3']]
y = data[['x', 'y']]

# Créer le modèle k-NN (k=3, par exemple)
knn = KNeighborsClassifier(n_neighbors=3)

# Entraîner le modèle avec les données
knn.fit(X, y)

# Exemple de nouvelle observation des valeurs RSSI
new_observation = pd.DataFrame([[-40, -42, -45]], columns=['s1', 's2', 's3'])

# Prédire la position (x, y) pour la nouvelle observation
predicted_position = knn.predict(new_observation)

print("Position prédite:", predicted_position)
