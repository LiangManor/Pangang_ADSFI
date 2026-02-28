#!/usr/bin/env sh

dos2unix Config.yaml
export CM_CONFIG_FILE_PATH=./etc/
export RT_DDS_URI=./etc/dds.xml
if [ ! -d "etc" ];then
    cp -r /opt/platform/mdc_platform/manual_service/adsfi/lidar_det_base/etc ./
fi
chmod +x bin/lidar_det
pmupload ./bin/lidar_det --allocGroup=default_dm
