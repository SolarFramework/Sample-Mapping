#!/bin/sh

## Detect MAPUPDATE_SERVICE_URL var and use its value 
## to set the Map Update service URL in XML configuration file

if [ -z "$MAPUPDATE_SERVICE_URL" ]
then
    echo "Error: You must define MAPUPDATE_SERVICE_URL env var with the MapUpdate Service URL"
    exit 1 
else
    echo "MAPUPDATE_SERVICE_URL defined: $MAPUPDATE_SERVICE_URL"
fi

echo "Try to replace the MapUpdate Service URL in the XML configuration file..."

sed -i -e "s/MAPUPDATE_SERVICE_URL/$MAPUPDATE_SERVICE_URL/g" /.xpcf/SolARPipeline_Mapping_Multi_Remote_properties.xml

echo "XML configuration file ready"

export LD_LIBRARY_PATH=/SolARPipelineMappingMultiRemote:/SolARPipelineMappingMultiRemote/modules/

## Start client
cd /SolARPipelineMappingMultiRemote
./SolARPipeline_Mapping_Multi_Remote -m /.xpcf/SolARPipeline_Mapping_Multi_Remote_modules.xml -p /.xpcf/SolARPipeline_Mapping_Multi_Remote_properties.xml

