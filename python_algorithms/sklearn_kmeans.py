import numpy as np
import matplotlib.pyplot as plt
from sklearn.datasets import make_circles
from sklearn.cluster import KMeans
from sklearn.decomposition import KernelPCA
from sklearn.metrics import accuracy_score

import pandas as pd

# 生成同心圆环数据
# n_samples = 1500
# X, y = make_circles(n_samples=n_samples, factor=0.5, noise=0.05)
data = pd.read_csv('E:/my-github-repos/149_pcd/circle0_600_pca/detected_edge.csv',delimiter=' ',usecols=['y','z'])

# print(data.describe())
print(data.head())
X = data

# 可视化数据

plt.scatter(X['y'], X['z'])
plt.title("Original Data")
plt.show()

# 使用 KernelPCA 进行核变换
kpca = KernelPCA(kernel="rbf")
X_kpca = kpca.fit_transform(X)

# 使用 K-means 进行聚类
kmeans = KMeans(n_clusters=2,max_iter=100000)
y_kmeans = kmeans.fit_predict(X_kpca)

# 可视化聚类结果
plt.scatter(X['y'], X['z'], c=y_kmeans)
plt.title("K-means Clustering on KernelPCA Transformed Data")
plt.show()

# 计算聚类的准确度
# accuracy = accuracy_score(y, y_kmeans)
# print(f"Clustering accuracy: {accuracy:.2f}")
