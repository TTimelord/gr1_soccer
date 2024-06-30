# RL walking Sim2real

## depends

1. ubuntu 20.04
2. eigen3
3. rapidjson

## install

1. sdk_build

```shell
cd gr1_sdk
mkdir build
cd build
cmake ..
make install
```


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

