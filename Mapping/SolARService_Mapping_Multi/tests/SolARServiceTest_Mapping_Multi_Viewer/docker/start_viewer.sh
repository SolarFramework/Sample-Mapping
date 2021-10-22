#!/bin/sh

## Detect MAPPING_SERVICE_URL var and use its value 
## to set the Mapping service URL in XML configuration file

cd /SolARServiceMappingMultiViewer

if [ -z "$MAPPING_SERVICE_URL" ]
then
    echo "Error: You must define MAPPING_SERVICE_URL env var with the Mapping Service URL"
    exit 1 
else
    echo "MAPPING_SERVICE_URL defined: $MAPPING_SERVICE_URL"
fi

echo "Try to replace the Mapping Service URL in the XML configuration file..."

sed -i -e "s/MAPPING_SERVICE_URL/$MAPPING_SERVICE_URL/g" /.xpcf/SolARServiceTest_Mapping_Multi_Viewer_conf.xml

echo "XML configuration file ready"

export LD_LIBRARY_PATH=.:./modules/

## Start client
./SolARServiceTest_Mapping_Multi_Viewer -f /.xpcf/SolARServiceTest_Mapping_Multi_Viewer_conf.xml

