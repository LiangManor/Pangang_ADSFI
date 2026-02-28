#!/usr/bin/env sh

dos2unix Config.yaml
export CM_CONFIG_FILE_PATH=./etc/
export RT_DDS_URI=./etc/dds.xml
if [ ! -d "etc" ];then
    cp -r /opt/platform/mdc_platform/manual_service/adsfi/traffic_light_base/etc ./
fi
chmod +x bin/$1
pmupload ./bin/$1 --allocGroup=default_dm

