#!/bin/bash
project_folder=$(cd "$(dirname ""$0"")"; pwd)

sdk_type=gcc

Parmnum=$# #传入参数个数
case $Parmnum in
 0)
        ;;
 1)
        if [ $1 != 'gcc' ] && [ $1 != 'llvm' ]
        then
            echo "example: bash get_target.sh gcc(optional) or bash get_target.sh llvm"
            exit
        fi
        sdk_type=$1
        ;;
 *)
        echo "example: bash get_target.sh gcc(optional) or bash get_target.sh llvm"
        exit
        ;;
esac

echo "***** sdk type is ${sdk_type} *****"

cd modules
for name in `ls`
do
    if [ -d "${name}" ]
    then
        echo "copying $name"
        mkdir -p ../adsf_sample/$name/bin

        if [ -e "./${name}/Config.yaml" ]
        then
            cp ./$name/Config.yaml ../adsf_sample/$name/
        fi

        if [ -e "./${name}/start.sh" ]
        then
            cp ./$name/start.sh ../adsf_sample/$name/
        fi

        if [ -e "./${name}/model/yolov3.om" ]
        then
            mkdir -p ../adsf_sample/$name/model
            cp ./$name/model/yolov3.om ../adsf_sample/$name/model/
        fi

        if [ -e "./${name}/model/yolov3_hps.om" ]
        then
            mkdir -p ../adsf_sample/$name/model
            cp ./$name/model/yolov3_hps.om ../adsf_sample/$name/model/
        fi

        file_folder=${project_folder}/build/cmake-build-debug-default_toolchain/gcc-aos_gea-aarch64/modules/$name

        if [ ${sdk_type} == 'llvm' ]
        then
            file_folder=${project_folder}/build/cmake-build-debug-default_toolchain/clang-aos_gea-aarch64/modules/$name
        fi

        if [ -e "${file_folder}/${name}" ]
        then
            cp $file_folder/$name ../adsf_sample/$name/bin/
        fi
    fi
done
cd ..
echo "finish"
