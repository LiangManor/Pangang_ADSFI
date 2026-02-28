#!/bin/sh

check_chiptype()
{
cmdline=`cat /proc/cmdline`
ChipType=${cmdline##*'ChipType='}
echo $ChipType
}

cd $(dirname "$0")

if [ -L model/yolov3_symlink.om ]
then
    echo "Link exist"
    rm model/yolov3_symlink.om
else
    echo "Link doesn't exist"
fi

ret=$(check_chiptype)
if [ $ret -eq 1 ];then
    ln -s yolov3.om model/yolov3_symlink.om
elif [ $ret -eq 2 ];then
    ln -s yolov3_hps.om model/yolov3_symlink.om
else
    echo "[error] Not supported type! "
    exit 1
fi

dos2unix Config.yaml
export CM_CONFIG_FILE_PATH=./etc/
export RT_DDS_URI=./etc/dds.xml
if [ ! -d "etc" ];then
    cp -rf /opt/platform/mdc_platform/manual_service/adsfi/camera_det_base/etc ./
fi
chmod +x bin/camera_det
pmupload ./bin/camera_det --allocGroup=default_dm