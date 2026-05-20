import streamlit as st
import folium
from folium.plugins import Draw
import math
import random
from datetime import datetime

# ===================== 页面基础设置 =====================
st.set_page_config(page_title="无人机智能化应用Demo", layout="wide")
st.title("无人机智能化应用 Demo")

# ===================== 1. 顶部在线状态 =====================
col1, col2, col3 = st.columns(3)
with col1:
    st.checkbox("GCS 在线", value=True, disabled=True)
with col2:
    st.checkbox("OBC 在线", value=True, disabled=True)
with col3:
    st.checkbox("FCU 在线", value=True, disabled=True)

# ===================== 2. 通信链路拓扑可视化 =====================
st.subheader("📡 通信链路拓扑与数据流")
topo_col1, topo_col2, topo_col3 = st.columns(3)

with topo_col1:
    st.markdown("""
    <div style="background:#e6f2ff; padding:20px; border-radius:10px; text-align:center;">
        <h3>🖥️ GCS</h3>
        <p>地面站</p>
        <p>192.168.1.100</p>
    </div>
    """, unsafe_allow_html=True)

with topo_col2:
    st.markdown("""
    <div style="text-align:center; margin-top:40px;">
        <span style="font-size:24px;">⬆️⬇️</span><br>
        UDP:14550<br>
        <span style="background:#90ee90; padding:3px 8px; border-radius:5px;">● 已连接</span>
    </div>
    """, unsafe_allow_html=True)
    st.markdown("""
    <div style="background:#fff2cc; padding:20px; border-radius:10px; text-align:center;">
        <h3>🧠 OBC</h3>
        <p>机载计算机</p>
        <p>Raspberry Pi 4</p>
    </div>
    """, unsafe_allow_html=True)
    st.markdown("""
    <div style="text-align:center;">
        <span style="font-size:24px;">⬆️⬇️</span><br>
        MAVLink<br>
        <span style="background:#90ee90; padding:3px 8px; border-radius:5px;">● 已连接</span>
    </div>
    """, unsafe_allow_html=True)

with topo_col3:
    st.markdown("""
    <div style="background:#f2e6ff; padding:20px; border-radius:10px; text-align:center;">
        <h3>⚙️ FCU</h3>
        <p>飞控</p>
        <p>PX4 / ArduPilot</p>
    </div>
    """, unsafe_allow_html=True)

# 链路统计
st.info("链路统计：GCS↔OBC:正常  OBC↔FCU:正常  延迟:~25ms  丢包率:0.1%")

# ===================== 3. 通信日志模块 =====================
st.subheader("📋 通信日志")
tab1, tab2, tab3 = st.tabs(["业务流程", "GCS→OBC→FCU", "FCU→OBC→GCS"])

# 业务流程日志
with tab1:
    st.markdown("""
<div style="background:#e8f5e9; padding:10px; border-radius:6px; font-family:monospace; font-size:13px;">
航线规划完成 | type:horizontal | 航点数:9 | 路径长度:359.8m<br>
<span style="color:#c2185b;">OBC 内部</span><br>
[14:36:01.607] ✅ 航线规划<br>
航线规划完成 | type:horizontal | 航点数:10 | 路径长度:356.3m<br>
<span style="color:#c2185b;">OBC 内部</span><br>
[14:36:01.607] ✅ 航线规划<br>
航线规划完成 | type:horizontal | 航点数:9 | 路径长度:381.3m<br>
<span style="color:#c2185b;">OBC 内部</span><br>
[14:32:54.650] ⓘ 航线规划<br>
开始航线规划 | 算法:A* | 障碍物数量:6<br>
<span style="color:#c2185b;">OBC 内部</span><br>
[14:32:54.650] ℹ️ 导航目标<br>
起点:(32.234368, 118.744358), 终点:(32.236468, 118.744058), 目标高度:10m<br>
<span style="color:#1565c0;">GCS → OBC</span>
</div>
""", unsafe_allow_html=True)

# GCS→OBC→FCU 日志
with tab2:
    st.markdown("""
<div style="background:#fff3e0; padding:10px; border-radius:6px; font-family:monospace; font-size:13px;">
<span style="color:#0d47a1;">FCU → OBC → GCS</span><br>
[15:03:03.08] FCU→OBC→GCS: ACK | Mode: AUTO<br>
[15:03:10] FCU→OBC→GCS: WP_REACHED #1<br>
[15:03:17] FCU→OBC→GCS: WP_REACHED #2<br>
[15:03:19] FCU→OBC→GCS: WP_REACHED #3<br>
[15:03:20] FCU→OBC→GCS: WP_REACHED #4<br>
[15:03:23] FCU→OBC→GCS: WP_REACHED #5<br>
[15:03:44] FCU→OBC→GCS: WP_REACHED #6<br>
[15:03:46] FCU→OBC→GCS: WP_REACHED #7<br>
[15:03:47] FCU→OBC→GCS: WP_REACHED #8<br>
[15:03:51] FCU→OBC→GCS: WP_REACHED #9<br>
[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE<br>
<span style="color:#0d47a1;">OBC → GCS</span>
</div>
""", unsafe_allow_html=True)

# FCU→OBC→GCS 日志
with tab3:
    st.markdown("""
<div style="background:#f3e5f5; padding:10px; border-radius:6px; font-family:monospace; font-size:13px;">
[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE<br>
<span style="color:#0d47a1;">OBC → GCS</span><br>
[15:03:03.08] FCU→OBC→GCS: ACK | Mode: AUTO<br>
[15:03:10] FCU→OBC→GCS: WP_REACHED #1<br>
[15:03:17] FCU→OBC→GCS: WP_REACHED #2<br>
[15:03:19] FCU→OBC→GCS: WP_REACHED #3<br>
[15:03:20] FCU→OBC→GCS: WP_REACHED #4<br>
[15:03:23] FCU→OBC→GCS: WP_REACHED #5<br>
[15:03:44] FCU→OBC→GCS: WP_REACHED #6<br>
[15:03:46] FCU→OBC→GCS: WP_REACHED #7<br>
[15:03:47] FCU→OBC→GCS: WP_REACHED #8<br>
[15:03:51] FCU→OBC→GCS: WP_REACHED #9<br>
[15:03:51] FCU→OBC→GCS: MISSION_COMPLETE
</div>
""", unsafe_allow_html=True)

# ===================== 下方：航线规划+地图（你原有功能保留） =====================
st.divider()
st.subheader("🗺️ 航线规划与飞行监控")

# 固定南京科技职业学院坐标
START_LAT, START_LON = 32.234368, 118.744358
END_LAT, END_LON = 32.236468, 118.744058

# 初始化地图
m = folium.Map(location=[START_LAT, START_LON], zoom_start=16)
Draw(export=True).add_to(m)

# 标记起点终点
folium.Marker([START_LAT, START_LON], tooltip="起点A", icon=folium.Icon(color="green")).add_to(m)
folium.Marker([END_LAT, END_LON], tooltip="终点B", icon=folium.Icon(color="red")).add_to(m)

# 渲染地图
st.components.v1.html(m._repr_html_(), height=500)
