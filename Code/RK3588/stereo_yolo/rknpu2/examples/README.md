# problem
./rknn_yolov5_demo: error while loading shared libraries: libopencv_highgui.so.405: cannot open shared object file: No such file or directory
# solution
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# ldconfig

### 
ln -s /usr/local/lib/libopencv_highgui.so /home/firefly/rknpu2/examples/rknn_yolov5_demo

### run
~/rknpu2/examples/rknn_yolov5_demo$   bash bash ./build-linux_RK3588.sh 
~/rknpu2/examples/rknn_yolov5_demo/install/rknn_yolov5_demo_Linux$  
    ./rknn_yolov5_demo 