import yaml
import os
import numpy as np

from math import sin, cos, acos, radians
from ament_index_python.packages import get_package_share_path
from rclpy.node import Node
from nav_msgs.msg import Odometry as Odometry_type
from sensor_msgs.msg import NavSatFix as NavSatFix_type


class Transfer(Node):
    def __init__(self):
        self.__package_name = 'bag_to_file'
        self.__node_name = 'topic_to_file_node'
        super().__init__(self.__node_name)
        
        # WGS84坐标系的坐标原点为地球质心
        self.__a = 6378136.49  # WGS84坐标系地球赤道半径（长半轴）
        self.__f = 1 / 298.257223563 # WGS84坐标系扁率
        self.__CONSTANTS_RADIUS_OF_EARTH = 6371001.00 # 地球平均半径
        self.__params = self._load_params()
        self.__topic_names = self.__params['topics']
        self.__file_objs = dict()
        self.__subscriptions = self._create_topic_subscibers()

    def __del__(self):
        for name,f_obj in self.__file_objs.items():
            f_obj.close()
            self.get_logger().info(f'IO stream for {name} has been closed.')
        self.destory_node()

    def _create_topic_subscibers(self) -> dict:
        dict_subscriptions = dict()
        for topic_type in self.__topic_names:
            if topic_type == None:
                self.get_logger().error('Get no topics from app.yaml!')
                break

            print(topic_type)
            if self.__topic_names[topic_type] == None:
                continue
            
            for topic_name in self.__topic_names[topic_type]:
                subcription_name = '_'.join((topic_name.strip('/')).split('/'))
                print('\t', subcription_name)
                tmp_sub = None
                if topic_type == 'odom':
                    tmp_sub = self.create_subscription(
                        Odometry_type, topic_name, 
                        lambda msg, name=subcription_name: self._save_odom_to_file(msg, name), 10)
                elif topic_type == 'gps':
                    tmp_sub = self.create_subscription(
                        NavSatFix_type, topic_name, 
                        lambda msg, name=subcription_name: self._save_gps_to_file(msg, name), 10)
                
                if tmp_sub != None:
                    dict_subscriptions[subcription_name] = tmp_sub
                    file_path = os.path.join(os.environ.get('HOME'), self.__params['odom_file_path'])
                    if not os.path.exists(file_path):
                        os.makedirs(file_path)
                    file_path = os.path.join(file_path, subcription_name + '.txt')
                    if not os.path.exists(file_path):
                        mode = 'a'
                    else:
                        mode = 'w'
                    self.__file_objs[subcription_name] = open(file_path, mode)
                    print('create file_obj:', subcription_name)

        return dict_subscriptions
    
    def _save_odom_to_file(self, msg: Odometry_type, topic_name: str):
        # print(topic_name, '-->')
        data = dict()
        data['x'] = msg.pose.pose.position.x
        data['y'] = msg.pose.pose.position.y
        data['z'] = 0.0
        data['x_'] = 0.0
        data['y_'] = 0.0
        data['z_'] = 0.0
        data['w_'] = 1.0
        data['stamp'] = msg.header.stamp
        self._save_to_file(data, topic_name)

    def _save_gps_to_file(self, msg: NavSatFix_type, topic_name: str):
        # x, y, z = self._gps_to_cartesion(msg.latitude, msg.longitude, msg.altitude)
        data = dict()
        data['x'], data['y'] = self._gps_to_xy(msg.latitude, msg.longitude)
        data['z'] = 0.0
        data['x_'] = 0.0
        data['y_'] = 0.0
        data['z_'] = 0.0
        data['w_'] = 1.0
        data['stamp'] = msg.header.stamp

        self._save_to_file(data, topic_name)
    
    def _format_to_utm(self):
        # 将输入格式化至utm格式
        pass

    def _gps_to_xy(self, lat, lon)-> tuple:
        # gps数据转换至局部坐标系
        lat = radians(lat)
        lon = radians(lon)
        # TODO: 可优化为保存一个变量
        ref_lat = radians(self.__params['ref_lat'])
        ref_lon = radians(self.__params['ref_lon'])
        sin_lat = sin(lat)
        cos_lat = cos(lat)
        sin_ref_lat = sin(ref_lat)
        cos_ref_lat = cos(ref_lat)
        cos_diff_lon = cos(lon - ref_lon)
        arg = np.clip(sin_ref_lat * sin_lat + cos_ref_lat * cos_lat * cos_diff_lon, 
                      -1.0, 1.0)
        c = acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / sin(c))
        
        x = float(k * (cos_ref_lat * sin_lat - sin_ref_lat * cos_lat * cos_diff_lon)
                    * self.__CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * sin(lon - ref_lon) 
                    * self.__CONSTANTS_RADIUS_OF_EARTH)
        x -= self.__params['residual_x']
        y -= self.__params['residual_y']
        return x, y

    def _gps_to_cartesian(self, lat, lon, alt):
        lat = radians(lat)
        lon = radians(lon)
        
        b = self.__a * (1 - self.__f)
        e = (self.__a ** 2 - b ** 2) ** 0.5 / self.__a 
        N = self.__a / (1 - e ** 2 * sin(lat) ** 2) ** 0.5

        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N * (1 - e ** 2) + alt) * sin(lat)

        return x, y, z
    
    def _load_params(self) -> dict:
        cur_path = get_package_share_path(self.__package_name)
        cfg_path = os.path.join(cur_path, 'cfg', 'app.yaml')
        # print("cfg_path-------->", cfg_path)
        # TODO: 异常机制，如果cfg_path实际不存在，则新建文件，并置位为ref_lat=0,ref_lon=0
        with open(cfg_path, encoding='utf-8') as f_obj:
            params = yaml.load(f_obj, Loader=yaml.SafeLoader)

        return params

    def _save_to_file(self, data: dict, topic_name) -> bool:
        stamp_sec = data['stamp'].sec
        stamp_nsec = data['stamp'].nanosec
        time_stamp = stamp_sec + stamp_nsec / 1e9
        tum_line = f"{time_stamp:.9f} {data['x']} {data['y']} {data['z']} {data['x_']} {data['y_']} {data['z_']} {data['w_']}\n"
        self.__file_objs[topic_name].write(tum_line)  
        self.__file_objs[topic_name].flush()      
