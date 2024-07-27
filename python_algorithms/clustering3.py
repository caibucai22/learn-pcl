"""
@File:kernel_k-means.py
@Date:2021/5/18 19:54
@Author:博0_oer~
"""
from sklearn import datasets
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from sklearn import metrics
import pandas as pd

# 生成数据集
def make_data():
    return datasets.make_circles(n_samples=200, factor=0.5, noise=0.05)

def make_data_without_label():
    data = pd.read_csv('E:/my-github-repos/149_pcd/circle0_600_pca/detected_edge.csv',delimiter=' ',usecols=['y','z'])
    xy = [[x,y] for x,y in zip(data['y'],data["z"])]
    return (xy,None)

# 数据集处理
def data_processing(data):
    data_ = list(data[0])
    data_list = []
    for d in data_:
        data_list.append(list(d))
    result = None if data[1] is None else list(data[1])

    return data_list, result


# 数据集展示
def show_data(data, result=None):
    for d in data:
        if result is not None:
          if result[data.index(d)] == 0:
              plt.scatter(d[0], d[1], c="r")
          if result[data.index(d)] == 1:
              plt.scatter(d[0], d[1], c="g")
        else:
            plt.scatter(d[0], d[1], c="b")
            
    plt.show()


# 将原始二维数据点映射到三维
# 引入一个新的维度，数据点与远点的距离，转换：（x，y）转换为（x，sqrt(x**2+y**2)，y）
def axes3d(data):
    new_data_list = []
    for d in data:
        new_data = [2*d[0]**2+2*d[1]**2, d[0]+1, d[1]+1]
        # new_data = [np.sqrt(d[0]**2+d[1]**2), d[0]+1, d[1]+1]
        new_data_list.append(new_data)

    return new_data_list


# 三维图像绘制
def show_data_3d(data, result=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for d in data:
        if result is not None:
          if result[data.index(d)] == 0:
              ax.scatter(d[0], d[1], d[2], c="r", marker="8")
          if result[data.index(d)] == 1:
              ax.scatter(d[0], d[1], d[2], c="g", marker="8")
        else:
            ax.scatter(d[0], d[1],d[2], c="b",marker="8")
    plt.show()


# 三维迭代图像绘制
def show_data_iter_3d(data, center):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for cls_d in data:
        for d in cls_d:
            if data.index(cls_d) == 0:
                ax.scatter(d[0], d[1], d[2], c="r", marker="8")
            if data.index(cls_d) == 1:
                ax.scatter(d[0], d[1], d[2], c="g", marker="8")

    for c in center:
        if center.index(c) == 0:
            ax.scatter(c[0], c[1], c[2], c="r", marker="v")
        if center.index(c) == 1:
            ax.scatter(c[0], c[1], c[2], c="g", marker="v")

    plt.show()


# k-means类中心的选择
def select_centre(data, k):
    center_list = []
    for i in range(k):
        ran = np.random.randint(len(data))
        center_list.append(data[ran])
    return center_list


# k-means聚类
def k_means(data, center):
    new_center = []
    cluster_result = []
    class_result = [[] for i in range(len(center))]
    for d in data:
        dis_list = []
        for c in center:
            # 欧氏距离进行距离度量
            dis = np.sqrt(np.sum(np.square(np.array(d)-np.array(c))))
            dis_list.append(dis)
        cluster_result.append(dis_list.index(min(dis_list)))
        class_result[dis_list.index(min(dis_list))].append(d)

    # 更新类中心
    for cls in class_result:
        x = 0
        y = 0
        z = 0
        for c in cls:
            x += c[0]
            y += c[1]
            z += c[2]
        if len(cls) == 0:
            new_center.append(center[class_result.index(cls)])
        else:
            new_center.append([round(x/len(cls), 4), round(y/len(cls), 4), round(z/len(cls), 4)])

    print(new_center)
    # print(len(class_result[0]), len(class_result[1]), len(class_result[2]))

    return cluster_result, class_result, new_center


# 迭代
def iter_(data, center, iter):
    cluster_result, class_result, new_center = k_means(data, center)
    for i in range(iter-1):
        cluster_result, class_result, new_center = k_means(data, new_center)
        # show_data_iter_3d(class_result, new_center)
    return cluster_result, class_result, new_center


if __name__ == '__main__':
    # 生成数据集
    # datas = make_data()
    datas = make_data_without_label()
    # print(datas)
    # 数据处理
    data, result = data_processing(datas)
    print(data, result)
    # 展示数据正确的聚类图
    show_data(data, result)
    # 进行kernel坐标变换
    data_3d = axes3d(data)
    # print(data_3d)
    # 绘制三维图像
    show_data_3d(data_3d, result)

    # 选择k-means的类中心点
    centers = select_centre(data_3d, 2)

    cluster_results, class_results, new_centers = iter_(data_3d, centers, 100)
    show_data_iter_3d(class_results, new_centers)

