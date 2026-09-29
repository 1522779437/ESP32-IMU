1.配置环境变量
. $HOME/esp/esp-idf/export.sh

2.创建工程
cd ~/esp
cp -r $IDF_PATH/examples/get-started/hello_world .

3.配置工程
idf.py set-target esp32

idf.py menuconfig

4.编译
idf.py build

sudo chmod 777 /dev/ttyUSB0

5.烧写
idf.py -p PORT flash
idf.py monitor
idf.py flash monitor

6.监视输出
idf.py -p PORT monitor
