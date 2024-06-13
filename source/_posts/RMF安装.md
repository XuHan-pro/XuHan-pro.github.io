---
title: RMF安装
date: 2023-8-14 17:49:03
tags:
  - ROS2
  - RMF
categories:
  - RMF
---
- [1. open\_RMF 配置，操作](#1-open_rmf-配置操作)
  - [1.1. RMF安装](#11-rmf安装)
    - [1.1.1. 准备工作](#111-准备工作)
    - [1.1.2. 非ROS依赖安装](#112-非ros依赖安装)
    - [1.1.3. ROS依赖安装](#113-ros依赖安装)
  - [1.2. RMF WEB 安装](#12-rmf-web-安装)
    - [1.2.1. 准备工作](#121-准备工作)
    - [1.2.2. 安装依赖](#122-安装依赖)
  - [1.3. 操作流程：](#13-操作流程)

# 1. open_RMF 配置，操作

## 1.1. RMF安装
 
系统要求：ubuntu22.04
### 1.1.1. 准备工作
 
### 1.1.2. 非ROS依赖安装
```bash  
sudo apt update && sudo apt install \
  python3-pip \
  curl \
  python3-colcon-mixin \
  ros-dev-tools \
  -y
```
 ```bash
 sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'  

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# These pip packages are only used by rmf_demos which are not released as binaries
python3 -m pip install flask-socketio fastapi uvicorn
 ```

### 1.1.3. ROS依赖安装
```bash
sudo rosdep init # run if first time using rosdep.
rosdep update
```

刷新环境
```bash
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

1、创建工作空间，拉取源码
```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import src < rmf.repos
```
 
 
指令：wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos  
问题：[Connecting to raw.githubusercontent.com (raw.githubusercontent.com)|::|:443... failed: Connection refused.](https://blog.csdn.net/m0_52650517/article/details/119831630)
 
指令：vcs import src < rmf.repos  
问题：Command 'vcs' not found  

解决方法：sudo apt-get install python3-vcstool
问题：Failed to connect to github.com port 443 after 21047 ms    网络问题  
[解决方法:](https://blog.csdn.net/Xminyang/article/details/124837086)
```bash
git config --global https.proxy  
git config --global --unset https.proxy  
```
 
2、 安装依赖
```bash
cd ~/rmf_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```
 
3、安装 clang 作为编译器和 lld 作为链接器

```bash
sudo apt update
sudo apt install clang clang-tools lldb lld libstdc++-12-dev
```
 
4、编译安装
```bash
cd ~/rmf_ws
source /opt/ros/humble/setup.bash # replace humble with ROS 2 distro of choice.
 
export CXX=clang++
export CC=clang
colcon build --mixin release lld
```
 
 
## 1.2. RMF WEB 安装
 
### 1.2.1. 准备工作
安装nodejs
```bash
sudo apt update && sudo apt install curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh | bash
#重启终端后继续执行
nvm install 16
```
 
安装pnpm
```bash
curl -fsSL https://get.pnpm.io/install.sh | bash -
#重启终端后继续执行
pnpm env use --global 16
```
安装pipenv
```bash
sudo apt install python3-venv
pip3 install pipenv   
```
问题：pipenv not found: https://blog.csdn.net/FloraCHY/article/details/119790395
 
 
### 1.2.2. 安装依赖
```bash
git clone https://github.com/open-rmf/rmf-web.git
cd  ~/rmf-web
pnpm install
```
查看pnpm源
`pnpm config get registry `  
切换淘宝源
`pnpm config set registry http://registry.npm.taobao.org `  
还原
`pnpm config set registry https://registry.npmjs.org`
 
 
## 1.3. 操作流程：
demo试运行：
 
```bash
cd ~/rmf-web/packages/dashboard
pnpm start
```
AttributeError: module 'lib' has no attribute 'OpenSSL_add_all_algorithms'
  
 
仿真  
```bash
source ~/rmf_ws/install/setup.bash  
ros2 launch rmf_demos_gz office.launch.xml server_uri:="http://localhost:8000/_internal"
```
