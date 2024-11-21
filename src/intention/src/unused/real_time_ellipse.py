import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.stats import multivariate_normal
from tf.transformations import translation_matrix
from tf import TransformListener

# ROS 초기화
rospy.init_node('real_time_ellipse', anonymous=True)

# y 평면 설정
y_plane = 0.7751
true_point = np.array([-0.2571, 0.7751, 0.2035])

# TF 리스너 설정
tf_listener = TransformListener()

# 초기화
intersection_points = []

# 시각화 설정
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
    ax.legend()
    return ax

# 실시간 업데이트 함수
def update(i):
    # tool0 위치 가져오기
    try:
        (trans, rot) = tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
        current_position = np.array(trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    # 이전 포지션과 현재 포지션 설정
    global previous_position
    if 'previous_position' not in globals():
        previous_position = current_position
        return

    # 벡터 계산 및 y = 0.7751 평면과 교차점 계산
    direction = current_position - previous_position
    if direction[1] != 0:
        t = (y_plane - previous_position[1]) / direction[1]
        if t > 0:
            intersection_point = previous_position + t * direction
            intersection_points.append(intersection_point[:2])  # X, Z 좌표만 사용

    # 교차점 업데이트
    previous_position = current_position

    # 교차점이 충분히 모이면 가우시안 분포 계산
    if len(intersection_points) > 1:
        intersection_points_np = np.array(intersection_points)
        mean_2d = np.mean(intersection_points_np, axis=0)
        cov_2d = np.cov(intersection_points_np, rowvar=False)

        # 가우시안 분포 및 컨투어 시각화
        ax.cla()
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Z Position')
        ax.set_title('2D Gaussian Distribution of Intersection Points')
        ax.scatter(true_point[0], true_point[2], color='r', label="True Point (-0.2571, 0.2035)")
        
        rv = multivariate_normal(mean_2d, cov_2d)
        pdf_2d = rv.pdf(pos)
        contour = ax.contourf(X, Z, pdf_2d, levels=50, cmap="viridis")
        ax.legend()
    return ax

# 애니메이션 생성
ani = FuncAnimation(fig, update, init_func=init, interval=100, blit=False)

plt.show()
