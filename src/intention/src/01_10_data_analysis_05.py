import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.animation import FuncAnimation, FFMpegWriter
import os
import glob
from scipy.stats import norm, multivariate_normal

# 데이터 로드 및 필터링
directory_path = 'C:\\jupyter\\241101_first_data\\02.csv'
data = pd.read_csv(directory_path)
# filtered_data = data.drop(columns=['Time']).query('Finish != 1')
filtered_data = data.query('Finish != 1')

# 위치 데이터 추출 및 벡터 거리 계산
pose_data = filtered_data[['Pose.x', 'Pose.y', 'Pose.z']].values
distances = np.linalg.norm(pose_data[1:] - pose_data[:-1], axis=1)

# 시작 및 끝 지점 - 벡터 준비
start_points = pose_data[:-1]
end_points = pose_data[1:]

# 특정 y 평면에서의 교차점 계산
true_point = np.array([-0.2571, 0.7751, 0.2035])
y_plane = 0.7751
intersection_points = []

for start, end in zip(start_points, end_points):
    direction = end - start
    if direction[1] != 0:
        t = (y_plane - start[1]) / direction[1]
        if t > 0:
            intersection_points.append(start + t * direction)
            
intersection_df = pd.DataFrame(intersection_points, columns=['Intersection_X', 'Intersection_Y', 'Intersection_Z'])

# 3D 교차점 시각화
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(intersection_df['Intersection_X'], intersection_df['Intersection_Y'], intersection_df['Intersection_Z'], color='b', alpha=0.3, label="Intersection Points")
# ax.scatter(-0.2571, 0.7751, 0.2035, color='r', label="Point (-0.2571, 0.7751, 0.2035)")
# ax.set_xlabel("X Position")
# ax.set_ylabel("Y Position")
# ax.set_zlabel("Z Position")
# ax.set_title("Intersection Points with y = 0.7751 Plane")
# ax.set_xlim(-0.8, 0.8)
# ax.set_ylim(0.60, 0.90)
# ax.set_zlim(-0.5, 1.0)
# plt.legend()
# plt.show()

# X-Z 평면에서 오차 계산
errors = intersection_df[['Intersection_X', 'Intersection_Z']].values - true_point[[0, 2]]

# 2D 교차점 시각화
# fig2, ax2 = plt.subplots(figsize=(10, 8))
# for error, intersection_point in zip(errors, intersection_df[['Intersection_X', 'Intersection_Z']].values):
#     std_x, std_z = np.std(error[0]), np.std(error[1])
#     ellipse = Ellipse(xy=intersection_point, width=std_x*2, height=std_z*2, edgecolor='b', fc='None', lw=1)
#     ax2.add_patch(ellipse)

# ax2.scatter(intersection_df['Intersection_X'], intersection_df['Intersection_Z'], color='b', alpha=0.3, label="Intersection Points")
# ax2.scatter(true_point[0], true_point[2], color='r', label="True Point (-0.2571, 0.2035)")
# ax2.set_xlabel("X Position")
# ax2.set_ylabel("Z Position")
# ax2.set_title("Error Ellipses in X-Z Plane")
# plt.legend()
# plt.show()

# 시간에 따른 오차 시각화
intersection_points_2d = intersection_df[['Intersection_X', 'Intersection_Z']].values

for i in range(int(len(intersection_points_2d)/10)):
    
    if (i+1)*10 > len(intersection_points_2d):
        intersection_points_parts = intersection_points_2d
    else:    
        intersection_points_parts = intersection_points_2d[:(i+1)*10]
    # 평균과 공분산 계산
    mean_2d = np.mean(intersection_points_parts, axis=0)
    cov_2d = np.cov(intersection_points_parts, rowvar=False)

    # 그리드 생성
    # x = np.linspace(intersection_df['Intersection_X'].min(), intersection_df['Intersection_X'].max(), 100)
    # z = np.linspace(intersection_df['Intersection_Z'].min(), intersection_df['Intersection_Z'].max(), 100)
    x = np.linspace(-10, 10, 100)
    z = np.linspace(-10, 10, 100)
    X, Z = np.meshgrid(x, z)
    pos = np.dstack((X, Z))

    # 2D 가우시안 분포 계산
    rv = multivariate_normal(mean_2d, cov_2d)
    pdf_2d = rv.pdf(pos)

    # 가우시안 분포 시각화
    # fig, ax = plt.subplots(figsize=(10, 8))
    # contour = ax.contourf(X, Z, pdf_2d, levels=50, cmap="viridis")
    # plt.colorbar(contour)
    # ax.scatter(true_point[0], true_point[2], color='r', label="True Point (-0.2571, 0.2035)")
    # ax.set_xlim(-1.5, 1.5)
    # ax.set_ylim(-1.5, 1.5)
    # ax.set_xlabel('X Position')
    # ax.set_ylabel('Z Position')
    # ax.set_title('2D Gaussian Distribution of Intersection Points')
    # plt.show()
    
# animation setting
fig, ax = plt.subplots(figsize=(10, 8))
x = np.linspace(-1.5, 1.5, 100)
z = np.linspace(-1.5, 1.5, 100)
X, Z = np.meshgrid(x, z)
pos = np.dstack((X, Z))

# 초기화 함수
def init():
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Z Position')
    ax.set_title('2D Gaussian Distribution of Intersection Points')
    ax.scatter(true_point[0], true_point[2], color='r', label="True Point (-0.2571, 0.2035)")
    ax.legend()  # 범례는 초기화 단계에서 한 번만 추가
    return ax

# 업데이트 함수
def update(i):
    ax.collections.clear()  # 기존 플롯을 지우지 않고 점과 컨투어만 갱신
    ax.scatter(true_point[0], true_point[2], color='r')  # true_point를 매 프레임 그리기
    
    # 점진적으로 교차점 추가
    intersection_points_parts = intersection_points_2d[:(i+1)*5]
    mean_2d = np.mean(intersection_points_parts, axis=0)
    cov_2d = np.cov(intersection_points_parts, rowvar=False)
    rv = multivariate_normal(mean_2d, cov_2d)
    pdf_2d = rv.pdf(pos)
    
    # 가우시안 분포 컨투어
    contour = ax.contourf(X, Z, pdf_2d, levels=50, cmap="viridis")
    return contour

# 애니메이션 생성
ani = FuncAnimation(fig, update, frames=int(len(intersection_points_2d) / 5), init_func=init, blit=False)

# MP4 파일로 저장
ffmpeg_writer = FFMpegWriter(fps=2, metadata=dict(artist='Me'), bitrate=1800)
# ani.save('C:\\jupyter\\241101_first_data\\02.mp4', writer=ffmpeg_writer)

plt.show()