import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from sklearn.decomposition import KernelPCA
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score

# 生成方形和圆环边界数据，圆环的半径小于方形的边长
def generate_square_and_circle_border(n_samples=1500, noise=0.01, circle_radius=0.5):
    # 生成方形边界数据
    square_samples = int(n_samples / 2)
    square = np.zeros((square_samples, 2))
    for i in range(square_samples):
        if i < square_samples / 4:
            square[i] = [np.random.uniform(-1, 1), -1]  # 下边界
        elif i < square_samples / 2:
            square[i] = [1, np.random.uniform(-1, 1)]  # 右边界
        elif i < 3 * square_samples / 4:
            square[i] = [np.random.uniform(-1, 1), 1]  # 上边界
        else:
            square[i] = [-1, np.random.uniform(-1, 1)]  # 左边界

    # 生成圆环边界数据
    circle_samples = n_samples - square_samples
    theta = np.linspace(0, 2 * np.pi, circle_samples)
    circle = np.c_[circle_radius * np.cos(theta), circle_radius * np.sin(theta)]
    circle += noise * np.random.randn(circle_samples, 2)  # 添加少量噪声

    # 合并数据
    X = np.vstack([square, circle])
    y = np.hstack([np.zeros(square_samples), np.ones(circle_samples)])
    return X, y

# 生成数据
n_samples = 1500
X, y = generate_square_and_circle_border(n_samples=n_samples, circle_radius=0.5)

# 数据标准化
scaler = StandardScaler()
X = scaler.fit_transform(X)

# 可视化数据
plt.scatter(X[:, 0], X[:, 1], c=y)
plt.title("Original Data")
plt.show()

# 使用 KernelPCA 进行核变换
kpca = KernelPCA(kernel="linear")
X_kpca = kpca.fit_transform(X)

# 使用 K-means 进行聚类
kmeans = KMeans(n_clusters=2)
y_kmeans = kmeans.fit_predict(X_kpca)

# 可视化聚类结果
plt.scatter(X[:, 0], X[:, 1], c=y_kmeans)
plt.title("K-means Clustering on KernelPCA Transformed Data")
plt.show()

# 计算聚类的准确度
accuracy = accuracy_score(y, y_kmeans)
print(f"Clustering accuracy: {accuracy:.2f}")
