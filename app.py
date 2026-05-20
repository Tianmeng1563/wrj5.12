import streamlit as st
import folium
from folium.plugins import Draw
import math
import heapq
from datetime import datetime

# 页面配置
st.set_page_config(page_title="无人机智能化应用", layout="wide")
st.title("无人机智能化应用 Demo")

# ===================== 新增：通信拓扑模块（完全匹配截图） =====================
# 顶部在线状态
col1, col2, col3 = st.columns(3)
with col1:
    st.checkbox("GCS在线", value=True, disabled=True)
with col2:
    st.checkbox("OBC在线", value=True, disabled=True)
with col3:
    st.checkbox("FCU在线", value=True, disabled=True)

# 通信链路拓扑与数据流
st.subheader("📡 通信链路拓扑与数据流")
t1, t2, t3 = st.columns(3)
with t1:
    st.markdown("""
    <div style="background:#cce5ff;padding:20px;border-radius:10px;text-align:center">
    <h3>🖥️ GCS</h3>
    <p>地面站</p>
    <p>192.168.1.100</p>
    </div>
    """, unsafe_allow_html=True)
with t2:
    st.markdown("""
    <div style="text-align:center;margin-top:35px">⬆️⬇️<br>UDP:14550<br><span style="background:#90ee90;padding:3px 8px;border-radius:5px">● 已连接</span></div>
    <div style="background:#ffeeba;padding:20px;border-radius:10px;text-align:center">
    <h3>🧠 OBC</h3>
    <p>机载计算机</p>
    <p>Raspberry Pi 4</p>
    </div>
    <div style="text-align:center">⬆️⬇️<br>MAVLink<br><span style="background:#90ee90;padding:3px 8px;border-radius:5px">● 已连接</span></div>
    """, unsafe_allow_html=True)
with t3:
    st.markdown("""
    <div style="background:#e2d9f3;padding:20px;border-radius:10px;text-align:center">
    <h3>⚙️ FCU</h3>
    <p>飞控</p>
    <p>PX4 / ArduPilot</p>
    </div>
    """, unsafe_allow_html=True)
st.info("链路统计：GCS↔OBC:正常  OBC↔FCU:正常  延迟:~25ms  丢包率:0.1%")

# 通信日志
st.subheader("📋 通信日志")
tab1, tab2, tab3 = st.tabs(["业务流程", "GCS→OBC→FCU", "FCU→OBC→GCS"])
with tab1:
    st.code("""航线规划完成 | type:horizontal | 航点数:9 | 路径长度:359.8m
OBC 内部
[14:36:01.607] ✅ 航线规划
航线规划完成 | type:horizontal | 航点数:10 | 路径长度:356.3m
OBC 内部
[14:36:01.607] ✅ 航线规划
航线规划完成 | type:horizontal | 航点数:9 | 路径长度:381.3m
OBC 内部
[14:32:54.650] ⓘ 航线规划
开始航线规划 | 算法:A* | 障碍物数量:6
OBC 内部
[14:32:54.650] ℹ️ 导航目标
起点: (32.234368, 118.744358), 终点: (32.236468, 118.744058), 目标高度: 10m
GCS → OBC""")
with tab2:
    st.code("""FCU → OBC → GCS
[15:03:03.08] FCU→OBC→GCS: ACK | Mode: AUTO
[15:03:10] FCU→OBC→GCS: WP_REACHED #1
[15:03:17] FCU→OBC→GCS: WP_REACHED #2
[15:03:19] FCU→OBC→GCS: WP_REACHED #3
[15:03:20] FCU→OBC→GCS: WP_REACHED #4
[15:03:23] FCU→OBC→GCS: WP_REACHED #5
[15:03:44] FCU→OBC→GCS: WP_REACHED #6
[15:03:46] FCU→OBC→GCS: WP_REACHED #7
[15:03:47] FCU→OBC→GCS: WP_REACHED #8
[15:03:51] FCU→OBC→GCS: WP_REACHED #9
[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE
OBC → GCS""")
with tab3:
    st.code("""[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE
OBC → GCS
[15:03:03.08] FCU→OBC→GCS: ACK | Mode: AUTO
[15:03:10] FCU→OBC→GCS: WP_REACHED #1
[15:03:17] FCU→OBC→GCS: WP_REACHED #2
[15:03:19] FCU→OBC→GCS: WP_REACHED #3
[15:03:20] FCU→OBC→GCS: WP_REACHED #4
[15:03:23] FCU→OBC→GCS: WP_REACHED #5
[15:03:44] FCU→OBC→GCS: WP_REACHED #6
[15:03:46] FCU→OBC→GCS: WP_REACHED #7
[15:03:47] FCU→OBC→GCS: WP_REACHED #8
[15:03:51] FCU→OBC→GCS: WP_REACHED #9
[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE""")

st.divider()
# ===================== 以下是你原来全部完整功能（未做任何删减修改） =====================
st.subheader("🗺️ 航线规划与飞行监控")
# 固定南京科技职业学院坐标
START = (32.234368, 118.744358)
END = (32.236468, 118.744058)

# 初始化地图
m = folium.Map(location=START, zoom_start=16)
Draw(export=True).add_to(m)
# 标记起点终点
folium.Marker(START, tooltip="起点A", icon=folium.Icon(color="green")).add_to(m)
folium.Marker(END, tooltip="终点B", icon=folium.Icon(color="red")).add_to(m)

# A*算法核心（原有完整绕障逻辑）
def distance(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def a_star(start, end, obstacles):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start:0}
    f_score = {start:distance(start, end)}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        for dx, dy in [(-0.0001,0),(0.0001,0),(0,-0.0001),(0,0.0001)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if any(distance(neighbor, obs) < 0.0002 for obs in obstacles):
                continue
            tentative_g = g_score[current] + distance(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + distance(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return [start, end]

# 障碍物存储
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
# 操作按钮
col_a, col_b, col_c = st.columns(3)
with col_a:
    if st.button("添加障碍物"):
        st.session_state.obstacles.append((START[0]+0.001, START[1]+0.001))
with col_b:
    if st.button("生成航线"):
        path = a_star(START, END, st.session_state.obstacles)
        folium.PolyLine(path, color="blue", weight=3).add_to(m)
with col_c:
    if st.button("清空障碍物"):
        st.session_state.obstacles.clear()
# 渲染障碍物
for obs in st.session_state.obstacles:
    folium.CircleMarker(obs, radius=8, color="orange", fill=True).add_to(m)
# 显示地图
st.components.v1.html(m._repr_html_(), height=550)
