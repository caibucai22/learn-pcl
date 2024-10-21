# ref https://blog.csdn.net/LLeetion/article/details/113091785

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse
from ellipse import LsqEllipse
np.random.seed(0)

def read_points(csv_path):
  data = pd.read_csv(csv_path,names=['x','y','z','r','g','b','n_x','n_y','n_z'],delimiter=' ') # names 给全
  # data = pd.read_txt(csv_path,names=['x','y','z','r','g','b','n_x','n_y','n_z'],delimiter=' ') # names 给全
  # points_2d = [(x,y) for x,y in enumerate(zip(data['x'],data['y']))]
  n = 10
  for x,y in zip(data['x'],data['y']):
    print(f"({x},{y})")
    n -= 1
    if n == 0:
      break
    
  return data['x'],data['y']

'''
任意圆心坐标 和 偏移角的椭圆 一般式
AX^2 + BXY + CY^2+DX+EY+F = 0 约束 B^2 - 4AC < 0
非齐次
x^2 + AXY + By^2 + Cx + Dy + E = 0

'''

def random_ellipse_data():
  theta = np.linspace(0, 2 * np.pi, 100)
  x = 5 * np.cos(theta) + np.random.normal(0, 0.1, theta.shape)
  y = 3 * np.sin(theta) + np.random.normal(0, 0.1, theta.shape)
  return x,y

def fit_ellipse(x,y):
  D = np.vstack([x**2,x*y,y**2,x,y,np.ones_like(x)]).T
  S = np.ones_like(x)*(-1)

  coeffs,residuals,rank,s = np.linalg.lstsq(D,S,rcond=None)

  return coeffs

def fit_ellipse_by_lsq_ellipse_pakcage(X1,X2):
  X = np.array(list(zip(X1,X2)))
  reg = LsqEllipse().fit(X)
  center,width,height,phi = reg.as_parameters()
  print(f'center: {center[0]:.3f}, {center[1]:.3f}')
  print(f'width: {width:.3f}')
  print(f'height: {height:.3f}')
  print(f'phi: {phi:.3f}')

  fig = plt.figure(figsize=(6, 6))
  ax = plt.subplot()
  ax.axis('equal')
  ax.plot(X1, X2, 'ro', zorder=1)
  ellipse = Ellipse(
      xy=center, width=2*width, height=2*height, angle=np.rad2deg(phi),
      edgecolor='b', fc='None', lw=2, label='Fit', zorder=2
  )
  ax.add_patch(ellipse)

  plt.xlabel('$X_1$')
  plt.ylabel('$X_2$')

  plt.legend()
  plt.show()

def ellipse_plot(A, B, C, D, E, F, ax=None):
  # 生成拟合的椭圆
  theta = np.linspace(0, 2 * np.pi, 1000)
  R = np.array([[np.cos(t), np.sin(t)] for t in theta])
  a = np.array([[A, B / 2], [B / 2, C]])
  offset = np.array([D / 2, E / 2])

  evals, evecs = np.linalg.eigh(a)
  # 检查特征值是否为正
  # evals = np.maximum(evals, 1e-10)  # 将特征值中的负值处理为一个很小的正数

  scale = 1.0 / np.sqrt(evals)

  ellipse = R @ evecs * scale + offset
  if ax is None:
      ax = plt.gca()

  ax.plot(ellipse[:, 0], ellipse[:, 1], 'r--', label='Fitted ellipse')
  ax.set_aspect('equal', 'datalim')
  ax.legend()

if __name__ == '__main__':
  csv_path = 'E:/01-LabProjects/149/211/incylinder_0003.csv'
  csv_path = 'E:/01-LabProjects/149/211/in_0003.txt'

  # points_2d = read_points(csv_path)
  # print(f"point num :{len(points_2d)}")
  # x = np.array([point[0] for point in points_2d])
  # y = np.array([point[1] for point in points_2d])
  # print(f"len(x)={len(x)} =?= len(y)={len(y)}")
  # assert len(x) == len(y)

  x,y = read_points(csv_path)

  # x,y = random_ellipse_data()

  fit_ellipse_by_lsq_ellipse_pakcage(x,y)


  # ====from gpt=========
  # A, B, C, D, E,F = fit_ellipse(x,y)
  # print(f"Fitted ellipse coefficients: A={A}, B={B}, C={C}, D={D}, E={E}, F={F}")
  
  # # 绘制拟合结果
  # plt.figure(figsize=(6, 6))
  # plt.scatter(x, y, label='Data points')
  # ellipse_plot(A, B, C, D, E, F)
  # plt.xlabel('X')
  # plt.ylabel('Y')
  # plt.legend()
  # plt.show()