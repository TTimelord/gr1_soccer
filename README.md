# RL walking Sim2real

## depends

1. ubuntu 20.04
2. anaconda
3. eigen3
4. rapidjson
5. pybind11
6. items in requirements.txt

## install
1. conda env

```shell
conda create -n thmos-gr1-deploy python=3.10
conda activate thmos-gr1-deploy
pip install -r ./requirements.txt
```

2. copy .so
```shell
sudo cp /thmos_gr1/GR1HW/lib/parallel_ankle/libParallelAnkle.so /usr/lib/libParallelAnkle.so
```

3. sdk build

```shell
cd GR1HW
mkdir build
cd build
cmake ..
make install
```

## demo

1. PrintRobotStates.py : Read robot sensors data.
2. GotoDefaultPos.py : Robot stand with default pos. (Also can check motors)
3. Deploy.py : RL policy deploy.

## else

1. rapid json install:

```shell
sudo apt-get update
sudo apt-get install git
#http: git clone https://github.com/Tencent/rapidjson.git
#ssh:  git clone git@github.com:Tencent/rapidjson.git
cd rapidjson/
mkdir build
cd build
cmake ..
make
sudo make install
```

2. eigen3:

```shell
sudo apt-get install libeigen3-dev
```

3. pybind 11:

```shell
cd GR1HW
git clone git@github.com:pybind/pybind11.git
cd pybind11
mkdir build
cd build
cmake ..
cmake --build . --config Release --target check
```
