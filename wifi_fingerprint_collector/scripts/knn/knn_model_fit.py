import pandas as pd
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split, cross_val_score
import matplotlib.pyplot as plt
import joblib


data = {
    "x": [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3],
    "y": [0, 1, 2, 3, 4, 5, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 5, 4, 3, 2, 1, 0],
    "s1": [-38.5, -46.5, -45.0, -45.5, -46.0, -47.2, -37.0, -36.0, -40.0, -43.0, -42.0, -40.2, -43.0, -41.0, -42.5, -41.0, -53.0, -41.0, -45.0, -40.5, -35.5, -44.5, -36.0, -46.2],
    "s2": [-46.5, -50.0, -41.0, -51.0, -50.0, -56.4, -52.0, -56.0, -50.5, -55.5, -49.0, -48.8, -46.0, -48.5, -51.0, -54.0, -60.0, -55.6, -55.5, -57.0, -51.0, -56.5, -52.0, -46.2],
    "s3": [-44.0, -41.5, -34.5, -43.0, -36.5, -35.2, -31.0, -37.5, -50.5, -40.5, -41.0, -46.4, -47.5, -40.5, -45.5, -33.5, -42.0, -29.6, -35.5, -47.0, -39.0, -39.5, -39.0, -40.2]
}

df = pd.DataFrame(data)
X = df[["s1", "s2", "s3"]]
y = df[["x", "y"]]

k_values = range(1, 11)
scores = []

for k in k_values:
    knn = KNeighborsClassifier(n_neighbors=k)
    cv_scores = cross_val_score(knn, X, y, cv=5)  # 5-fold cross-validation
    scores.append(cv_scores.mean())

plt.plot(k_values, scores)
plt.xlabel("Number of Neighbors (k)")
plt.ylabel("Cross-Validated Accuracy")
plt.title("KNN: Number of Neighbors vs. Accuracy")
plt.show()

best_k = k_values[scores.index(max(scores))]
print(f"The best value for n_neighbors is {best_k}")

knn = KNeighborsClassifier(n_neighbors=best_k)
knn.fit(X, y)

joblib.dump(knn, 'knn_model.pkl')
