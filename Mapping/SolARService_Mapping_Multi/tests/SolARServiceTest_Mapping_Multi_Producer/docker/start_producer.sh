#!/bin/sh

## Detect MAPPING_SERVICE_URL var and use its value 
## to set the Mapping service URL in XML configuration file

cd /SolARServiceMappingMultiProducer

if [ -z "$MAPPING_SERVICE_URL" ]
then
    echo "Error: You must define MAPPING_SERVICE_URL env var with the Mapping Service URL"
    exit 1 
else
    echo "MAPPING_SERVICE_URL defined: $MAPPING_SERVICE_URL"
fi

echo "Try to replace the Mapping Service URL in the XML configuration file..."

sed -i -e "s/MAPPING_SERVICE_URL/$MAPPING_SERVICE_URL/g" /.xpcf/SolAServiceTest_Mapping_Multi_Producer_conf.xml

## Detect PATH_TO_IMAGE_DATA var and use its value 
## to set the path to the Hololens image set in XML configuration file

PATH_TO_IMAGE_DATA=".\/data\/data_hololens\/loop_desktop_A"

if [ -n "$HOLOLENS_DATA_SET" ]
then
    echo "HOLOLENS_DATA_SET defined: $HOLOLENS_DATA_SET"

	if [ "$HOLOLENS_DATA_SET" = "B" ]
	then
	    PATH_TO_IMAGE_DATA=".\/data\/data_hololens\/loop_desktop_B"
        echo "Path to image set to: ./data/data_hololens/loop_desktop_B"
	else
        echo "Path to image set to: ./data/data_hololens/loop_desktop_A"
	fi
else
    echo "HOLOLENS_DATA_SET note defined: you can use it to choose Hololens data set A or B"
    echo "Path to image set to default: ./data/data_hololens/loop_desktop_A"
fi

echo "Replace the path to image data in the XML configuration file..."
sed -i -e "s/PATH_TO_IMAGE_DATA/$PATH_TO_IMAGE_DATA/g" /.xpcf/SolARServiceTest_Mapping_Multi_Producer_conf.xml

echo "XML configuration file ready"

export LD_LIBRARY_PATH=.:./modules/

## Start client
./SolARServiceTest_Mapping_Multi_Producer -f /.xpcf/SolARServiceTest_Mapping_Multi_Producer_conf.xml

