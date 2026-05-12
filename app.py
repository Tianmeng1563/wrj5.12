import streamlit as st
import folium
from streamlit_folium import st_folium
from streamlit_option_menu import option_menu
from datetime import datetime, timedelta
import time
import pandas as pd
import json
import os
import numpy as np
from shapely.geometry import LineString, Polygon

# ===================== 页面配置 =====================
st.set_page_config(layout="wide", page_title="南科院无人机避障航线规划")

# ===================== 数据持久化 =====================
SAVE_FILE = "drone_data.json"
def load_all_data():
    if os.path.exists(SAVE_FILE):
        with open(SAVE_FILE,"r",encoding="utf-8") as f:
            return json.load(f)
    # 直接用你截图里的南科院校内坐标
    return {
        "A": [32.233800, 118.748600],
        "B": [32.235000, 118.750000],
        "A_set": True,
        "B_set": True,
        "obstacles": []
    }

def save_all_data():
    data = {
        "A": list(st.session_state.A),
        "B": list(st.session_state.B),
        "A_set": st.session_state.A_set,
        "B_set": st.session_state.B_set,
        "obstacles": st.session_state.polygon_memory
    }
    with open(SAVE_FILE,"w",encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ===================== 初始化状态 =====================
data = load_all_data()
default_states = {
    "A": tuple(data["A"]),
    "B": tuple(data["B"]),
    "A_set": data["A_set"],
    "B_set": data["B_set"],
    "height": 50,
    "heartbeat_data": [],
    "polygon_memory": data["obstacles"],
    "is_drawing": False,
    "temp_points": [],
    "obs_h": 20,
    "last_click_time": 0,
    "safe_radius": 0.0002,
    "flight_running": False,
    "flight_paused": False,
    "current_wp_idx": 0,
    "flight_speed": 8.5,
    "flight_start_time": None,
    "flight_waypoints": [],
    "battery": 100.0,
    "total_distance": 0.0,
    "elapsed_distance": 0.0,
    "route_side": "auto"
}

for key, val in default_states.items():
    if key not in st.session_state:
        st.session_state[key] = val

# ===================== GCJ-02 转 WGS-84 坐标转换 =====================
def gcj02_to_wgs84(lng:float, lat:float):
    a = 6378245.0
    ee = 0.00669342162296594323
    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * np.sqrt(abs(x))
        ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
        ret += (20.0 * np.sin(y * np.pi) + 40.0 * np.sin(y / 3.0 * np.pi)) * 2.0 / 3.0
        ret += (160.0 * np.sin(y / 12.0 * np.pi) + 320 * np.sin(y / 30.0 * np.pi)) * 2.0 / 3.0
        return ret
    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * np.sqrt(abs(x))
        ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
        ret += (20.0 * np.sin(x * np.pi) + 40.0 * np.sin(x / 3.0 * np.pi)) * 2.0 / 3.0
        ret += (150.0 * np.sin(x / 12.0 * np.pi) + 300.0 * np.sin(x / 30.0 * np.pi)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    return lat - dlat, lng - dlng

# ===================== 航线生成与避障逻辑 =====================
def calc_route_lines(pA, pB, offset=0.0001):
    latA, lonA = pA
    latB, lonB = pB
    dx = lonB - lonA
    dy = latB - latA
    L = np.hypot(dx, dy)
    if L < 1e-8:
        L = 1e-8
    left_off_x = -dy / L * offset
    left_off_y = dx / L * offset
    right_off_x = dy / L * offset
    right_off_y = -dx / L * offset
    left = [
        [latA, lonA],
        [latA + left_off_y, lonA + left_off_x],
        [latB + left_off_y, lonB + left_off_x],
        [latB, lonB]
    ]
    right = [
        [latA, lonA],
        [latA + right_off_y, lonA + right_off_x],
        [latB + right_off_y, lonB + right_off_x],
        [latB, lonB]
    ]
    return left, right

def get_safe_route(pA, pB, obstacles, safe_dist, route_side="auto"):
    base_line = LineString([pA, pB])
    obs_polygons = []
    for obs in obstacles:
        pts = obs["pts"]
        if len(pts) >= 3:
            poly = Polygon(pts).buffer(safe_dist)
            obs_polygons.append(poly)
    conflict = False
    for poly in obs_polygons:
        if base_line.intersects(poly):
            conflict = True
            break
    if not conflict:
        return [pA, pB], False
    left_line, right_line = calc_route_lines(pA, pB, offset=safe_dist)
    if route_side == "auto":
        left_ok = True
        for poly in obs_polygons:
            if LineString(left_line).intersects(poly):
                left_ok = False
                break
        return (left_line if left_ok else right_line), True
    elif route_side == "left":
        return left_line, True
    else:
        return right_line, True

# ===================== 侧边栏 =====================
with st.sidebar:
    st.title("🚁 无人机系统导航")
    page = option_menu("功能页面", ["航线规划", "飞行监控"], default_index=0)
    st.divider()
    st.subheader("坐标系转换")
    coord_type = st.radio("", ["GCJ-02(火星坐标)", "WGS-84(原始坐标)"])
    st.divider()
    st.subheader("系统点位状态")
    st.button("✅ A点已设置" if st.session_state.A_set else "❌ A点未设置", type="primary")
    st.button("✅ B点已设置" if st.session_state.B_set else "❌ B点未设置", type="primary")
    st.divider()
    st.subheader("🛡️ 安全半径配置")
    st.session_state.safe_radius = st.slider("航线与障碍物安全距离", 0.00005, 0.0005, value=st.session_state.safe_radius, step=0.00001, format="%.5f")
    st.session_state.route_side = st.radio("绕飞方向选择", ["left", "right", "auto"], index=2)

# ===================== 航线规划页面 =====================
if page == "航线规划":
    st.title("🚁 南京科技职业学院 无人机避障航线规划")
    col_map, col_ctrl = st.columns([3.2, 1])
    with col_ctrl:
        st.subheader("🎛️ 点位与飞行参数")
        a_lat = st.number_input("起点A 纬度", value=st.session_state.A[0], format="%.6f")
        a_lon = st.number_input("起点A 经度", value=st.session_state.A[1], format="%.6f")
        b_lat = st.number_input("终点B 纬度", value=st.session_state.B[0], format="%.6f")
        b_lon = st.number_input("终点B 经度", value=st.session_state.B[1], format="%.6f")
        st.session_state.height = st.slider("无人机飞行高度 (m)", 0, 200, value=st.session_state.height)
        if st.button("确定设置起点A"):
            st.session_state.A = (a_lat, a_lon)
            st.session_state.A_set = True
            save_all_data()
            st.success("A点已保存！")
        if st.button("确定设置终点B"):
            st.session_state.B = (b_lat, b_lon)
            st.session_state.B_set = True
            save_all_data()
            st.success("B点已保存！")
        st.divider()
        st.subheader("🚧 障碍物圈选")
        st.session_state.obs_h = st.number_input("障碍物高度(m)", 0, 300, value=st.session_state.obs_h)
        if st.session_state.is_drawing:
            st.warning(f"正在绘制，已选点位：{len(st.session_state.temp_points)}")
        else:
            st.info("点开始绘制，在地图上点击圈禁飞区")
        btn1, btn2, btn3 = st.columns(3)
        with btn1:
            if st.button("开始绘制"):
                st.session_state.is_drawing = True
                st.session_state.temp_points = []
        with btn2:
            if st.button("撤销上一点"):
                if st.session_state.temp_points:
                    st.session_state.temp_points.pop()
        with btn3:
            if st.button("取消绘制"):
                st.session_state.is_drawing = False
                st.session_state.temp_points = []
        if st.button("✅ 完成圈选保存"):
            if len(st.session_state.temp_points) >= 3:
                st.session_state.polygon_memory.append({"pts": st.session_state.temp_points.copy(), "h": st.session_state.obs_h})
                save_all_data()
                st.success(f"障碍物保存成功，高度{st.session_state.obs_h}m")
            else:
                st.error("至少3个点位！")
            st.session_state.is_drawing = False
            st.session_state.temp_points = []
            st.rerun()
        if st.button("🗑️ 清空全部障碍物"):
            st.session_state.polygon_memory = []
            st.session_state.temp_points = []
            save_all_data()
            st.rerun()
        st.info(f"已记忆障碍物：{len(st.session_state.polygon_memory)} 个")

    with col_map:
        center_lat = (st.session_state.A[0] + st.session_state.B[0]) / 2
        center_lon = (st.session_state.A[1] + st.session_state.B[1]) / 2
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=19,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="高德卫星地图",
            max_zoom=22
        )
        folium.plugins.Fullscreen(position="topright").add_to(m)
        if coord_type == "GCJ-02(火星坐标)":
            A_wgs = gcj02_to_wgs84(st.session_state.A[1], st.session_state.A[0])
            B_wgs = gcj02_to_wgs84(st.session_state.B[1], st.session_state.B[0])
        else:
            A_wgs = st.session_state.A
            B_wgs = st.session_state.B
        if st.session_state.A_set:
            folium.Marker(A_wgs, icon=folium.Icon(color='red', icon='plane', prefix='fa'), popup="起点A").add_to(m)
        if st.session_state.B_set:
            folium.Marker(B_wgs, icon=folium.Icon(color='green', icon='plane', prefix='fa'), popup="终点B").add_to(m)
        for idx, obs in enumerate(st.session_state.polygon_memory):
            pts = obs["pts"]
            hh = obs["h"]
            if len(pts) >= 3:
                folium.Polygon(locations=pts, color="#dc2626", fill=True, fill_color="#dc2626", fill_opacity=0.45, popup=f"障碍物{idx+1} 高{hh}m").add_to(m)
                poly = Polygon(pts).buffer(st.session_state.safe_radius)
                folium.Polygon(locations=list(poly.exterior.coords), color="#ff9900", fill=False, weight=2, dash_array="5 5").add_to(m)
        if len(st.session_state.temp_points) > 0:
            folium.PolyLine(st.session_state.temp_points, color="#ff7700", weight=3, dash_array="10 5").add_to(m)
        if st.session_state.A_set and st.session_state.B_set:
            safe_waypoints, need_avoid = get_safe_route(A_wgs, B_wgs, st.session_state.polygon_memory, st.session_state.safe_radius, st.session_state.route_side)
            st.session_state.flight_waypoints = safe_waypoints
            folium.PolyLine(safe_waypoints, color="#0066ff", weight=5, popup="避障安全航线").add_to(m)

        output = st_folium(m, width=1150, height=720, key="main_map")
        if st.session_state.is_drawing and output and output.get("last_clicked"):
            now = time.time()
            if now - st.session_state.last_click_time > 0.5:
                pt = output["last_clicked"]
                new_pt = [pt["lat"], pt["lng"]]
                if not st.session_state.temp_points or new_pt != st.session_state.temp_points[-1]:
                    st.session_state.temp_points.append(new_pt)
                    st.session_state.last_click_time = now
                    st.rerun()

# ===================== 飞行监控页面 =====================
else:
    st.title("📡 飞行实时监控 - 任务执行")
    st.success("✅ 无人机链路正常 设备在线")
    col_btn = st.columns(4)
    with col_btn[0]:
        if st.button("🔴 开始任务", type="primary", disabled=st.session_state.flight_running):
            st.session_state.flight_running = True
            st.session_state.flight_paused = False
            st.session_state.flight_start_time = datetime.now()
            st.session_state.current_wp_idx = 0
            st.rerun()
    with col_btn[1]:
        if st.button("⏸️ 暂停", disabled=not st.session_state.flight_running or st.session_state.flight_paused):
            st.session_state.flight_paused = True
            st.rerun()
    with col_btn[2]:
        if st.button("▶️ 继续", disabled=not st.session_state.flight_paused):
            st.session_state.flight_paused = False
            st.rerun()
    with col_btn[3]:
        if st.button("⏹️ 停止重置", type="secondary"):
            st.session_state.flight_running = False
            st.session_state.flight_paused = False
            st.session_state.current_wp_idx = 0
            st.session_state.battery = 100.0
            st.rerun()

    if len(st.session_state.flight_waypoints) < 2:
        st.warning("⚠️ 先去航线规划页面生成航线！")
    else:
        total_dist = 0
        for i in range(len(st.session_state.flight_waypoints)-1):
            p1 = st.session_state.flight_waypoints[i]
            p2 = st.session_state.flight_waypoints[i+1]
            dist = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
            total_dist += dist
        st.session_state.total_distance = round(total_dist * 111000, 2)

        if st.session_state.flight_running and not st.session_state.flight_paused:
            if st.session_state.current_wp_idx < len(st.session_state.flight_waypoints)-1:
                st.session_state.current_wp_idx += 0.01
                st.session_state.battery = max(0, st.session_state.battery - 0.01)
            else:
                st.session_state.flight_running = False
                st.success("🎉 飞行任务完成！")

        progress = st.session_state.current_wp_idx / (len(st.session_state.flight_waypoints)-1)
        st.progress(progress, text=f"任务进度：{round(progress*100,1)}%")

        col_map_flight, col_status = st.columns([2,1])
        with col_map_flight:
            st.subheader("🗺️ 实时飞行地图")
            m_flight = folium.Map(
                location=st.session_state.flight_waypoints[0],
                zoom_start=19,
                tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
                attr="高德卫星地图"
            )
            for idx, obs in enumerate(st.session_state.polygon_memory):
                pts = obs["pts"]
                hh = obs["h"]
                if len(pts)>=3:
                    folium.Polygon(locations=pts,color="#dc2626",fill=True,fill_color="#dc2626",fill_opacity=0.45,popup=f"障碍物{idx+1} 高{hh}m").add_to(m_flight)
            folium.PolyLine(st.session_state.flight_waypoints, color="#0066ff", weight=3, opacity=0.5, dash_array="10 5").add_to(m_flight)
            flown_idx = int(st.session_state.current_wp_idx)
            flown_waypoints = st.session_state.flight_waypoints[:flown_idx+1]
            if len(flown_waypoints)>=2:
                folium.PolyLine(flown_waypoints, color="#22bb22", weight=4, popup="✅ 已飞行路径").add_to(m_flight)
            if len(st.session_state.flight_waypoints) > 0:
                drone_pos = st.session_state.flight_waypoints[min(int(st.session_state.current_wp_idx), len(st.session_state.flight_waypoints)-1)]
                folium.CircleMarker(drone_pos, radius=10, color="orange", fill=True, fill_color="orange", popup="🚁 无人机当前位置").add_to(m_flight)
            st_folium(m_flight, width="100%", height=500, key="flight_map")

        with col_status:
            st.subheader("📡 通信链路状态")
            st.success("✅ GCS 在线")
            st.success("✅ OBC 在线")
            st.success("✅ FCU 在线")
            st.divider()
            st.info(f"📍 当前位置：纬度{drone_pos[0]:.6f}，经度{drone_pos[1]:.6f}")
            st.info(f"🛫 飞行高度：{st.session_state.height} m")
            st.info(f"🛡️ 安全半径：{st.session_state.safe_radius}")
            st.info(f"🚧 障碍物数量：{len(st.session_state.polygon_memory)} 个")

        if st.session_state.flight_running and not st.session_state.flight_paused:
            time.sleep(0.5)
            st.rerun()
