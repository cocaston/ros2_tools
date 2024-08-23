# 1.环境配置
## 1.1 安装numpy
pass
## 1.2 安装evo
`pip install evo --upgrade --no-binary evo`

# 2.使用说明
## 2.1 生成所需文件
- 修改配置文件cfg/app.yaml: 
  - 配置需要订阅的话题
    - odom格式的在odom字典中，
    - gps格式的在gps字典中。
- 编译、启动节点
`ros2 run bag_to_file topic_to_file_node`
- 进入文件目录
`cd ~/apps/odom/*.txt`
## 2.2 evo验证
### 2.2.1 轨迹对比
单文件
`evo_traj tum --ref [gps文件名].txt [odom1文件名].txt`
多文件
`evo_traj tum --ref [gps文件名].txt [odom1文件名].txt [odom2文件名].txt [odom3文件名].txt`

note:
- --ref表示参照，通常使用gps数据作为真值。
- [**文件名] 需替换为 实际文件名。
- 目前只用于对比轨迹和x,y变化程度，所以未考虑整体pose（仅用了x,y）。按需修改。
### 2.2.2 误差计算
`evo_ape tum [gps文件名].txt [odom文件名].txt`

rmse
